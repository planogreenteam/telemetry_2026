#!/usr/bin/env python3
import argparse
import sys
import threading
import time

# Make stdout line-buffered so logs appear immediately on Windows / piped runs.
try:
    sys.stdout.reconfigure(line_buffering=True)
except AttributeError:
    pass

from BMV.bmv_normalizer import normalize_bmv_frame
from BMV.bmv_policy import BMVTransmitPolicy
from BMV.bmv_reader import BMVReader
from LORA.lora_transport import LoRaTransport
from storage.csv_sink import write_telemetry_csv
from telemetry_packet import build_bmv_packet


DEFAULT_DEVICE = "all"
DEFAULT_TRANSPORT = "lora"
DEFAULT_BMV_PORT = "/dev/serial/by-id/usb-VictronEnergy_BV_VE_Direct_cable_VE92RZNP-if00-port0"
DEFAULT_BMV_BAUD = 19200
DEFAULT_CSV_PATH = "bmv_data.csv"
DEFAULT_DEVICE_ID = 1

DEFAULT_LORA_PORT = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
DEFAULT_LORA_BAUD = 9600
DEFAULT_FREQ = "868.100"
DEFAULT_BW = 0
DEFAULT_SF = 7
DEFAULT_POWER = 20
DEFAULT_CR = 1
DEFAULT_CRC = 0
DEFAULT_HEADER = 0
DEFAULT_IQ = 0
DEFAULT_PREAMBLE = 8
DEFAULT_SYNCWORD = 0
DEFAULT_GROUP = 0
DEFAULT_ACK = 0
DEFAULT_RETRIES = 3

DEFAULT_VOLTAGE_DELTA_MV = 1
DEFAULT_CURRENT_DELTA_MA = 10
DEFAULT_POWER_DELTA_W = 1
DEFAULT_HEARTBEAT_SECONDS = 3

# CAN defaults
DEFAULT_CAN_INTERFACE = "can0"
DEFAULT_CAN_BITRATE = 500000
DEFAULT_MPPT_DEVICE_ID = 1
DEFAULT_BMS_DEVICE_ID = 20
DEFAULT_MPPT_HEARTBEAT_SECONDS = 5
DEFAULT_BMS_HEARTBEAT_SECONDS = 60
DEFAULT_CSV_PATH_MPPT = "mppt_data.csv"
DEFAULT_CSV_PATH_BMS = "bms_data.csv"
DEFAULT_NUM_MPPTS = 6

# How often the CAN transmit thread wakes up to check for changes (seconds).
# Pkt0 (power) is checked every tick; pkt1 (status) every STATUS_TICKS ticks
# to prioritize power data over mode/fault/temp on the LoRa link.
CAN_SAMPLE_INTERVAL = 0.0
STATUS_TICKS = 15  # transmit status frames at most once per 5 × 1s = 5s


# ─────────────────────────────────────────────────────────────────────────────
# CORE LOOP — BMV single-stream sender (unchanged)
# ─────────────────────────────────────────────────────────────────────────────

def _process_reading(components, raw_frame, transport, log_prefix):
    normalizer = components["normalizer"]
    policy = components["policy"]
    packet_builder = components["packet_builder"]
    device_id = components["device_id"]
    sink = components.get("sink")
    sub_prefix = components.get("log_prefix", log_prefix)

    try:
        reading = normalizer(raw_frame, device_id)
    except Exception as exc:
        print(f"[{sub_prefix}] Normalize failed: {exc}", flush=True)
        return

    if sink is not None:
        try:
            sink(reading)
        except Exception as exc:
            print(f"[{sub_prefix}] Sink error: {exc}", flush=True)

    event_type = policy.classify(reading)
    if event_type is None:
        return

    seq = policy.mark_sent(reading)
    try:
        packet = packet_builder(reading, event_type, seq)
    except Exception as exc:
        print(f"[{sub_prefix}] Packet build failed: {exc}", flush=True)
        return

    if transport is None:
        print(
            f"[{sub_prefix}] {event_type.name} seq={seq} "
            f"fields={reading['fields']} hex={packet.hex()}",
            flush=True,
        )
        return

    try:
        t0 = time.time()
        transport.send_hex(packet.hex())
        print(
            f"[{sub_prefix}] Sent {event_type.name} seq={seq} "
            f"tx={time.time()-t0:.2f}s fields={reading['fields']}",
            flush=True,
        )
    except Exception as exc:
        print(f"[{sub_prefix}] Transport send failed: {exc}", flush=True)
        return


def run_sender(
    *,
    reader,
    streams=None,
    normalizer=None,
    policy=None,
    packet_builder=None,
    device_id=None,
    log_prefix="telemetry",
    sink=None,
    transport=None,
):
    """Single-stream (BMV) sender loop."""
    if streams is not None:
        raise ValueError(
            "run_sender no longer handles multi-stream CAN — use run_can_sender instead."
        )
    if normalizer is None or policy is None or packet_builder is None:
        raise ValueError(
            "run_sender needs normalizer/policy/packet_builder."
        )

    components = {
        "normalizer": normalizer,
        "policy": policy,
        "packet_builder": packet_builder,
        "device_id": device_id,
        "sink": sink,
        "log_prefix": log_prefix,
    }

    print(f"[{log_prefix}] Starting sender loop", flush=True)
    try:
        while True:
            raw_frame = reader.read_frame()
            _process_reading(components, raw_frame, transport, log_prefix)
    finally:
        reader.close()


# ─────────────────────────────────────────────────────────────────────────────
# CAN SENDER — latest-value cache architecture
#
# Reader thread:   reads CAN frames as fast as they arrive, overwrites a
#                  shared cache dict keyed by (kind, can_id). No queueing —
#                  intermediate frames are naturally discarded.
#
# Transmit thread: wakes every CAN_SAMPLE_INTERVAL seconds, iterates the
#                  cache, runs policy.classify() on each slot, and sends
#                  whatever is due. Power frames (pkt0) are checked every
#                  tick; status frames (pkt1) every STATUS_TICKS ticks.
# ─────────────────────────────────────────────────────────────────────────────

def run_can_sender(*, reader, streams, transport, log_prefix="can"):
    """Multi-stream CAN sender using a latest-value cache + timed TX thread."""

    # cache[(kind, can_id)] = raw_frame dict (overwritten on every new frame)
    cache = {}
    cache_lock = threading.Lock()
    stop_event = threading.Event()

    def _reader_thread():
        try:
            while not stop_event.is_set():
                result = reader.read_frame()
                if result is None:
                    continue
                kind, raw_frame = result
                can_id = raw_frame.get("can_id")
                with cache_lock:
                    cache[(kind, can_id)] = raw_frame
        except Exception as exc:
            print(f"[{log_prefix}] Reader thread crashed: {exc}", flush=True)
        finally:
            reader.close()

    def _transmit_thread():
        tick = 0
        try:
            while not stop_event.is_set():
                time.sleep(CAN_SAMPLE_INTERVAL)
                tick += 1

                with cache_lock:
                    snapshot = dict(cache)

                for (kind, can_id), raw_frame in snapshot.items():
                    comps = streams.get(kind)
                    if comps is None:
                        continue

                    # Deprioritize status frames (pkt1): only process every
                    # STATUS_TICKS ticks so power frames dominate LoRa airtime.
                    packet_id = raw_frame.get("can_id", 0) & 0x0F
                    if packet_id == 1 and (tick % STATUS_TICKS) != 0:
                        continue

                    _process_reading(comps, raw_frame, transport, log_prefix)
                    time.sleep(0.0)  # give modem time to recover between sends

        except Exception as exc:
            print(f"[{log_prefix}] Transmit thread crashed: {exc}", flush=True)

    print(f"[{log_prefix}] Starting CAN sender (cache+timer architecture)", flush=True)
    for kind, comps in streams.items():
        print(
            f"[{log_prefix}]   stream: {kind} -> "
            f"device_id={comps.get('device_id')}, "
            f"policy={type(comps['policy']).__name__}",
            flush=True,
        )

    rt = threading.Thread(target=_reader_thread, name=f"{log_prefix}-reader", daemon=True)
    tt = threading.Thread(target=_transmit_thread, name=f"{log_prefix}-tx", daemon=True)
    rt.start()
    tt.start()

    try:
        while rt.is_alive() and tt.is_alive():
            rt.join(timeout=0.5)
            tt.join(timeout=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()


def build_lora_transport(args):
    return LoRaTransport(
        port=args.lora_port,
        baud=args.lora_baud,
        freq=args.freq,
        bw=args.bw,
        sf=args.sf,
        power=args.power,
        cr=args.cr,
        crc=args.crc,
        header=args.header,
        iq=args.iq,
        preamble=args.preamble,
        syncword=args.syncword,
        group=args.group,
        ack=args.ack,
        retries=args.retries,
    )


# ─────────────────────────────────────────────────────────────────────────────
# COMPONENT BUILDERS
# ─────────────────────────────────────────────────────────────────────────────

def build_bmv_sender_components(args):
    reader = BMVReader(args.bmv_port, args.bmv_baud)
    policy = BMVTransmitPolicy(
        voltage_delta_mv=args.voltage_delta_mv,
        current_delta_ma=args.current_delta_ma,
        power_delta_w=args.power_delta_w,
        heartbeat_seconds=args.heartbeat_seconds,
    )
    sink = lambda reading: write_telemetry_csv(args.csv_path, reading)
    return {
        "reader": reader,
        "normalizer": normalize_bmv_frame,
        "policy": policy,
        "packet_builder": build_bmv_packet,
        "device_id": args.device_id,
        "log_prefix": "bmv",
        "sink": sink,
    }


def build_can_sender_components(args):
    """Reads MPPT and BMS frames off the CAN bus into two packet streams."""
    from CAN.can_reader import CANReader
    from CAN.can_normalizer import (
        normalize_mppt_frame,
        normalize_bms_frame,
        default_id_to_kind,
    )
    from CAN.can_policy import GenericTransmitPolicy
    from telemetry_packet import (
        EventType,
        build_mppt_packet,
        build_bms_packet,
    )

    id_to_kind = default_id_to_kind(num_mppts=args.num_mppts)

    reader = CANReader(
        interface=args.can_interface,
        bitrate=args.can_bitrate,
        id_to_kind=id_to_kind,
    )

    mppt_policy = GenericTransmitPolicy(
        deltas={
            "pv_voltage_v":      args.mppt_pv_voltage_delta_v,
            "pv_current_a":      args.mppt_pv_current_delta_a,
            "pv_power_w":        args.mppt_pv_power_delta_w,
            "battery_voltage_v": args.mppt_batt_voltage_delta_v,
        },
        event_type_change=EventType.DELTA_UPDATE,
        event_type_heartbeat=EventType.HEARTBEAT,
        heartbeat_seconds=args.mppt_heartbeat_seconds,
    )
    bms_policy = GenericTransmitPolicy(
        deltas={
            "battery_voltage_v": args.bms_voltage_delta_v,
            "battery_current_a": args.bms_current_delta_a,
            "soc_pct":           args.bms_soc_delta_pct,
        },
        event_type_change=EventType.DELTA_UPDATE,
        event_type_heartbeat=EventType.HEARTBEAT,
        heartbeat_seconds=args.bms_heartbeat_seconds,
    )

    mppt_sink = lambda r: write_telemetry_csv(args.csv_path_mppt, r)
    bms_sink = lambda r: write_telemetry_csv(args.csv_path_bms, r)

    return {
        "reader": reader,
        "log_prefix": "can",
        "streams": {
            "mppt": {
                "normalizer": normalize_mppt_frame,
                "policy": mppt_policy,
                "packet_builder": build_mppt_packet,
                "device_id": args.mppt_device_id,
                "sink": mppt_sink,
                "log_prefix": "mppt",
            },
            "bms": {
                "normalizer": normalize_bms_frame,
                "policy": bms_policy,
                "packet_builder": build_bms_packet,
                "device_id": args.bms_device_id,
                "sink": bms_sink,
                "log_prefix": "bms",
            },
        },
    }


SENDER_COMPONENT_BUILDERS = {
    "bmv": build_bmv_sender_components,
    "can": build_can_sender_components,
}


# ─────────────────────────────────────────────────────────────────────────────
# 'all' mode: probe hardware, run whatever's available
# ─────────────────────────────────────────────────────────────────────────────

class LockedTransport:
    """Serialize send_hex across threads sharing one LoRa modem."""
    def __init__(self, inner):
        self._inner = inner
        self._lock = threading.Lock()

    def send_hex(self, hex_str):
        with self._lock:
            return self._inner.send_hex(hex_str)

    def __getattr__(self, name):
        return getattr(self._inner, name)


def _try_build_bmv_components(args):
    try:
        return build_bmv_sender_components(args)
    except Exception as exc:
        print(f"[all] BMV unavailable, skipping: {exc}", flush=True)
        return None


def _try_build_can_components(args):
    try:
        return build_can_sender_components(args)
    except Exception as exc:
        print(f"[all] CAN unavailable, skipping: {exc}", flush=True)
        return None


def build_all_sender_components(args):
    return {"_all_mode": True, "log_prefix": "all"}


SENDER_COMPONENT_BUILDERS["all"] = build_all_sender_components


# ─────────────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────────────

def build_parser():
    parser = argparse.ArgumentParser(description="Generic telemetry sender")
    parser.add_argument("--device", choices=tuple(SENDER_COMPONENT_BUILDERS), default=DEFAULT_DEVICE)
    parser.add_argument("--transport", choices=("lora",), default=DEFAULT_TRANSPORT)
    parser.add_argument("--dry-run", action="store_true", help="Build packets but do not send over the transport")
    parser.add_argument("--device-id", type=int, default=DEFAULT_DEVICE_ID, help="Telemetry device id (BMV)")
    parser.add_argument("--csv-path", default=DEFAULT_CSV_PATH, help="CSV output path (BMV)")

    # BMV
    parser.add_argument("--bmv-port", default=DEFAULT_BMV_PORT, help="BMV VE.Direct serial device")
    parser.add_argument("--bmv-baud", type=int, default=DEFAULT_BMV_BAUD, help="BMV serial baud rate")
    parser.add_argument("--voltage-delta-mv", type=int, default=DEFAULT_VOLTAGE_DELTA_MV)
    parser.add_argument("--current-delta-ma", type=int, default=DEFAULT_CURRENT_DELTA_MA)
    parser.add_argument("--power-delta-w", type=int, default=DEFAULT_POWER_DELTA_W)
    parser.add_argument("--heartbeat-seconds", type=int, default=DEFAULT_HEARTBEAT_SECONDS)

    # CAN bus
    parser.add_argument("--can-interface", default=DEFAULT_CAN_INTERFACE,
                        help="SocketCAN interface name (e.g. can0)")
    parser.add_argument("--can-bitrate", type=int, default=DEFAULT_CAN_BITRATE,
                        help="CAN bus bitrate")
    parser.add_argument("--num-mppts", type=int, default=DEFAULT_NUM_MPPTS,
                        help="Number of TPEE MPPTs on the bus")
    parser.add_argument("--csv-path-mppt", default=DEFAULT_CSV_PATH_MPPT,
                        help="CSV output path for MPPT data")
    parser.add_argument("--csv-path-bms", default=DEFAULT_CSV_PATH_BMS,
                        help="CSV output path for BMS data")

    # MPPT
    parser.add_argument("--mppt-device-id", type=int, default=DEFAULT_MPPT_DEVICE_ID,
                        help="Base telemetry device id for MPPT #0; #1 -> base+1, etc.")
    parser.add_argument("--mppt-pv-voltage-delta-v", type=float, default=2.0,
                        help="MPPT: PV voltage change to trigger transmit (V)")
    parser.add_argument("--mppt-pv-current-delta-a", type=float, default=0.5,
                        help="MPPT: PV current change to trigger transmit (A)")
    parser.add_argument("--mppt-pv-power-delta-w", type=float, default=1.0,
                        help="MPPT: PV power change to trigger transmit (W)")
    parser.add_argument("--mppt-batt-voltage-delta-v", type=float, default=1.0,
                        help="MPPT: battery voltage change to trigger transmit (V)")
    parser.add_argument("--mppt-heartbeat-seconds", type=int, default=DEFAULT_MPPT_HEARTBEAT_SECONDS)

    # BMS
    parser.add_argument("--bms-device-id", type=int, default=DEFAULT_BMS_DEVICE_ID,
                        help="Telemetry device id for BMS")
    parser.add_argument("--bms-voltage-delta-v", type=float, default=0.2,
                        help="BMS: battery voltage change to trigger transmit (V)")
    parser.add_argument("--bms-current-delta-a", type=float, default=1.0,
                        help="BMS: battery current change to trigger transmit (A)")
    parser.add_argument("--bms-soc-delta-pct", type=float, default=1.0,
                        help="BMS: SOC change to trigger transmit (percent)")
    parser.add_argument("--bms-heartbeat-seconds", type=int, default=DEFAULT_BMS_HEARTBEAT_SECONDS)

    # LoRa
    parser.add_argument("--lora-port", default=DEFAULT_LORA_PORT, help="LoRa modem serial device")
    parser.add_argument("--lora-baud", type=int, default=DEFAULT_LORA_BAUD, help="LoRa modem baud rate")
    parser.add_argument("--freq", default=DEFAULT_FREQ, help="TX/RX frequency in MHz")
    parser.add_argument("--bw", type=int, default=DEFAULT_BW, help="Bandwidth enum 0-9")
    parser.add_argument("--sf", type=int, default=DEFAULT_SF, help="Spreading factor 5-12")
    parser.add_argument("--power", type=int, default=DEFAULT_POWER, help="TX power 0-22 dBm")
    parser.add_argument("--cr", type=int, default=DEFAULT_CR, help="Coding rate 1-4")
    parser.add_argument("--crc", type=int, default=DEFAULT_CRC, help="CRC 0=off 1=on")
    parser.add_argument("--header", type=int, default=DEFAULT_HEADER, help="Header 0=explicit 1=implicit")
    parser.add_argument("--iq", type=int, default=DEFAULT_IQ, help="IQ invert 0=standard 1=inverted")
    parser.add_argument("--preamble", type=int, default=DEFAULT_PREAMBLE, help="Preamble length")
    parser.add_argument("--syncword", type=int, default=DEFAULT_SYNCWORD, help="Sync word mode 0/1")
    parser.add_argument("--group", type=int, default=DEFAULT_GROUP, help="Group 0-255")
    parser.add_argument("--ack", type=int, choices=(0, 1, 2), default=DEFAULT_ACK, help="ACK mode 0/1/2")
    parser.add_argument("--retries", type=int, choices=range(0, 9), default=DEFAULT_RETRIES, help="Retransmissions 0-8")
    return parser


def _run_bmv_in_thread(components, transport):
    def _target():
        try:
            run_sender(**components, transport=transport)
        except Exception as exc:
            print(f"[bmv] thread crashed: {exc}", flush=True)

    t = threading.Thread(target=_target, name="bmv", daemon=True)
    t.start()
    return t


def _run_all(args, transport):
    components_built = []

    bmv = _try_build_bmv_components(args)
    if bmv is not None:
        components_built.append(("bmv", bmv))
    can = _try_build_can_components(args)
    if can is not None:
        components_built.append(("can", can))

    if not components_built:
        raise RuntimeError(
            "No telemetry sources available. Tried BMV serial port and CAN "
            "interface, neither could be opened. Check --bmv-port and "
            "--can-interface, or run with an explicit --device flag."
        )

    print(f"[all] Starting {len(components_built)} source(s): "
          f"{[name for name, _ in components_built]}", flush=True)

    threads = []
    for name, comps in components_built:
        if name == "bmv":
            threads.append(_run_bmv_in_thread(comps, transport))
        elif name == "can":
            t = threading.Thread(
                target=run_can_sender,
                kwargs={**comps, "transport": transport},
                name="can",
                daemon=True,
            )
            t.start()
            threads.append(t)

    try:
        while any(t.is_alive() for t in threads):
            for t in threads:
                t.join(timeout=0.5)
    except KeyboardInterrupt:
        print("[all] Interrupted, shutting down", flush=True)


def main(argv=None):
    args = build_parser().parse_args(argv)
    components = SENDER_COMPONENT_BUILDERS[args.device](args)

    print(f"[{components['log_prefix']}] Reading from device source", flush=True)

    is_all_mode = components.get("_all_mode", False)

    if args.dry_run:
        if is_all_mode:
            raise ValueError("--dry-run is not supported with --device all "
                             "(use --device bmv or --device can)")
        run_sender(**{k: v for k, v in components.items() if k != "streams"})
        return

    if args.transport != "lora":
        raise ValueError(f"Unsupported transport {args.transport}")

    with build_lora_transport(args) as transport:
        if is_all_mode:
            _run_all(args, LockedTransport(transport))
        elif args.device == "can":
            run_can_sender(**components, transport=transport)
        else:
            run_sender(**components, transport=transport)


if __name__ == "__main__":
    main()
