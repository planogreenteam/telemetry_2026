#!/usr/bin/env python3
import argparse
import heapq
import itertools
import sys
import threading
import time
import traceback

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

# Cap how many CAN packets the transmit thread will push out in a single
# tick. Without this, a correlated event (e.g. sharp acceleration moving
# voltage/current on every MPPT + the BMS at once) can queue up a dozen-plus
# sends back to back, monopolizing the shared LoRa modem for several
# seconds and starving higher-priority BMV sends even with a priority lock
# in place (priority only lets BMV jump the queue *between* sends, not
# preempt one in flight). Capping the burst size spreads it across more
# ticks so the modem is never held for more than a few sends at a time.
# Anything left over just gets re-evaluated (and is still "due") next tick.
MAX_CAN_SENDS_PER_TICK = 3

# send_hex priority values — lower number = served first when multiple
# threads are waiting on PriorityLockedTransport. BMV carries the peak
# current draw we care about most, so it always jumps ahead of queued CAN
# sends.
PRIORITY_BMV = 0
PRIORITY_CAN = 10

# How often the BMV cached-sender's transmit thread wakes to check the
# latest cached VE.Direct reading and decide whether to send.
BMV_SAMPLE_INTERVAL = 0.1

# Discharge current magnitude (amps) that starts the drive elapsed timer.
# Overridable with --drive-start-current-a.
DEFAULT_DRIVE_START_CURRENT_A = 0.5


class DriveStopwatch:
    """Elapsed drive timer. Starts the first time battery *discharge*
    current is observed from either the BMV or the BMS, then keeps counting
    for the life of the process (i.e. until the Pi shuts down or the sender
    restarts).

    Both the Victron BMV (VE.Direct 'I', mA) and the EG4/Pylontech BMS
    (0x356 battery_current_a) report discharge as negative current, so
    "draw" here means current < -start_threshold_a. The threshold exists so
    quiescent electronics load / sensor noise doesn't trip the timer before
    the car actually pulls meaningful current.

    Shared as a single module-level instance across the BMV and CAN threads;
    whichever source sees draw first wins. Thread-safe.
    """

    def __init__(self, start_threshold_a=DEFAULT_DRIVE_START_CURRENT_A):
        self.start_threshold_a = start_threshold_a
        self._start = None
        self._lock = threading.Lock()

    def update(self, current_a, source=""):
        """Feed a battery current reading in amps (negative = discharging)."""
        if current_a is None:
            return
        with self._lock:
            if self._start is None and current_a < -self.start_threshold_a:
                self._start = time.monotonic()
                print(
                    f"[drive-timer] Battery draw detected from {source} "
                    f"({current_a:+.2f} A) — elapsed timer started",
                    flush=True,
                )

    def elapsed_s(self):
        """Whole seconds since the timer started, 0 if it hasn't yet.
        Int, because the wire format packs elapsed_s as a u32 (>I)."""
        with self._lock:
            if self._start is None:
                return 0
            return int(time.monotonic() - self._start)


# Shared across the BMV and CAN(BMS) streams — whichever trips first wins.
_drive_stopwatch = DriveStopwatch()


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
    priority = components.get("priority", PRIORITY_CAN)

    try:
        reading = normalizer(raw_frame, device_id)
    except Exception:
        print(f"[{sub_prefix}] Normalize failed:\n{traceback.format_exc()}", flush=True)
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
    except Exception:
        print(f"[{sub_prefix}] Packet build failed:\n{traceback.format_exc()}", flush=True)
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
        try:
            transport.send_hex(packet.hex(), priority=priority)
        except TypeError:
            # Plain (non-priority-aware) transports don't accept priority.
            transport.send_hex(packet.hex())
        print(
            f"[{sub_prefix}] Sent {event_type.name} seq={seq} "
            f"tx={time.time()-t0:.2f}s fields={reading['fields']}",
            flush=True,
        )
    except Exception:
        print(f"[{sub_prefix}] Transport send failed:\n{traceback.format_exc()}", flush=True)
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
    priority=PRIORITY_BMV,
):
    """Single-stream (BMV) sender loop. Kept for --dry-run (no transport
    contention to worry about there); live runs use run_bmv_cached_sender
    instead — see main()."""
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
        "priority": priority,
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
        except Exception:
            print(f"[{log_prefix}] Reader thread crashed:\n{traceback.format_exc()}", flush=True)
        finally:
            reader.close()

    def _transmit_thread():
        tick = 0
        # Rotating start position into the (stable-order) cache keys, so a
        # fixed cap on sends-per-tick doesn't always favor the same handful
        # of slots. dict() preserves insertion order and updating a key's
        # value never moves it, so a plain "break after N" here always cut
        # off at the same point every tick -- e.g. servicing only the first
        # 3 CAN slots ever inserted (some MPPT boards) forever, while the
        # rest never even got policy.classify() called on them. Rotating
        # the starting index each tick guarantees every slot gets serviced
        # over successive ticks instead of a permanent subset.
        rr_start = 0
        try:
            while not stop_event.is_set():
                time.sleep(CAN_SAMPLE_INTERVAL)
                tick += 1

                with cache_lock:
                    snapshot = dict(cache)

                keys = list(snapshot.keys())
                n = len(keys)
                if n == 0:
                    continue

                sent_this_tick = 0
                checked = 0
                idx = rr_start % n
                while checked < n and sent_this_tick < MAX_CAN_SENDS_PER_TICK:
                    kind, can_id = keys[idx]
                    raw_frame = snapshot[(kind, can_id)]
                    idx = (idx + 1) % n
                    checked += 1

                    comps = streams.get(kind)
                    if comps is None:
                        continue

                    # Deprioritize status frames (pkt1): only process every
                    # STATUS_TICKS ticks so power frames dominate LoRa airtime.
                    packet_id = raw_frame.get("can_id", 0) & 0x0F
                    if packet_id == 1 and (tick % STATUS_TICKS) != 0:
                        continue

                    _process_reading(comps, raw_frame, transport, log_prefix)
                    sent_this_tick += 1
                    # Real pause (not the old no-op time.sleep(0.0)): gives
                    # the modem a beat and, just as importantly, gives the
                    # OS/GIL an actual scheduling window to run the BMV
                    # thread so its higher-priority send can enqueue and
                    # win the shared transport lock between CAN sends,
                    # instead of CAN looping back-to-back with no gap.
                    time.sleep(0.05)

                rr_start = idx  # next tick picks up where this one left off

        except Exception:
            print(f"[{log_prefix}] Transmit thread crashed:\n{traceback.format_exc()}", flush=True)

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


# ─────────────────────────────────────────────────────────────────────────────
# BMV SENDER — latest-value cache architecture (mirrors run_can_sender)
#
# The old single-threaded run_sender() blocks on reader.read_frame() and
# then calls transport.send_hex() inline. If the shared LoRa modem is busy
# (e.g. a correlated CAN burst holding the lock), that call blocks — and
# because reading and sending happen in the same thread, BMV can't advance
# to a fresher VE.Direct frame while it waits. The result: during a sharp
# acceleration event (which tends to move voltage/current across every
# MPPT + the BMS at once, triggering a burst of CAN sends), BMV can go
# dark for as long as the whole burst takes to drain, and the frame it
# finally sends afterward is stale rather than the peak-current sample.
#
# This version decouples reading from sending exactly like run_can_sender:
# a reader thread continuously updates a single-slot cache with the latest
# VE.Direct frame, and a transmit thread wakes on its own schedule, checks
# whatever is currently cached, and sends it if policy.classify() says to.
# Whenever the modem becomes free, BMV always sends the freshest state
# rather than working through a backlog of stale readings.
# ─────────────────────────────────────────────────────────────────────────────

def run_bmv_cached_sender(
    *,
    reader,
    normalizer,
    policy,
    packet_builder,
    device_id,
    transport,
    sink=None,
    log_prefix="bmv",
    sample_interval=BMV_SAMPLE_INTERVAL,
    priority=PRIORITY_BMV,
):
    """BMV sender using a latest-value cache + timed TX thread, so a busy
    shared modem never delays BMV by more than one send, and BMV always
    transmits its freshest reading rather than a stale queued one."""

    cache = {}
    cache_lock = threading.Lock()
    stop_event = threading.Event()

    components = {
        "normalizer": normalizer,
        "policy": policy,
        "packet_builder": packet_builder,
        "device_id": device_id,
        "sink": sink,
        "log_prefix": log_prefix,
        "priority": priority,
    }

    def _reader_thread():
        got_first_frame = False
        last_report = time.monotonic()
        try:
            while not stop_event.is_set():
                raw_frame = reader.read_frame()
                if raw_frame is None:
                    now = time.monotonic()
                    if now - last_report >= 5.0:
                        print(f"[{log_prefix}] Reader alive, still waiting for a "
                              f"complete VE.Direct frame (read_frame() returned None)",
                              flush=True)
                        last_report = now
                    continue
                if not got_first_frame:
                    print(f"[{log_prefix}] First VE.Direct frame received: "
                          f"{raw_frame}", flush=True)
                    got_first_frame = True
                with cache_lock:
                    cache["latest"] = raw_frame
        except Exception:
            print(f"[{log_prefix}] Reader thread crashed:\n{traceback.format_exc()}",
                  flush=True)
        finally:
            reader.close()

    def _transmit_thread():
        checks = 0
        sends = 0
        last_report = time.monotonic()
        try:
            while not stop_event.is_set():
                time.sleep(sample_interval)
                with cache_lock:
                    raw_frame = cache.get("latest")
                checks += 1
                if raw_frame is not None:
                    before = sends
                    _process_reading(components, raw_frame, transport, log_prefix)
                    # _process_reading itself prints "Sent ..." on success; we
                    # can't easily tell from here whether it sent, so just
                    # track that we attempted processing on real data.
                now = time.monotonic()
                if now - last_report >= 5.0:
                    cache_state = "has data" if raw_frame is not None else "EMPTY"
                    print(f"[{log_prefix}] Transmit thread alive: "
                          f"{checks} cache checks in last interval, cache is {cache_state}",
                          flush=True)
                    last_report = now
                    checks = 0
        except Exception:
            print(f"[{log_prefix}] Transmit thread crashed:\n{traceback.format_exc()}",
                  flush=True)

    print(f"[{log_prefix}] Starting BMV sender (cache+timer architecture)", flush=True)

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
    _drive_stopwatch.start_threshold_a = args.drive_start_current_a

    def bmv_sink(reading):
        # The sink runs inside _process_reading() *before* policy.classify()
        # and the packet builder, so stamping elapsed_s here puts it on the
        # wire (BMVField.ELAPSED_S) as well as in the CSV. The BMV policy
        # doesn't watch elapsed_s, so a ticking timer never triggers a
        # transmit by itself — it just rides along on every packet sent.
        current_ma = reading["fields"].get("current_ma")
        _drive_stopwatch.update(
            current_ma / 1000.0 if current_ma is not None else None,
            source="BMV",
        )
        reading["fields"]["elapsed_s"] = _drive_stopwatch.elapsed_s()
        write_telemetry_csv(args.csv_path, reading)

    return {
        "reader": reader,
        "normalizer": normalize_bmv_frame,
        "policy": policy,
        "packet_builder": build_bmv_packet,
        "device_id": args.device_id,
        "log_prefix": "bmv",
        "sink": bmv_sink,
        "priority": PRIORITY_BMV,
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

    _drive_stopwatch.start_threshold_a = args.drive_start_current_a

    mppt_sink = lambda r: write_telemetry_csv(args.csv_path_mppt, r)

    def bms_sink(reading):
        # Only 0x356 frames carry battery_current_a; other BMS frame types
        # simply won't have the key and can't trip the timer. elapsed_s is
        # stamped on every BMS reading so the receiver still gets the timer
        # even if the BMV is offline. The BMS policy doesn't watch
        # elapsed_s, so it never triggers a transmit by itself.
        _drive_stopwatch.update(
            reading["fields"].get("battery_current_a"), source="BMS"
        )
        reading["fields"]["elapsed_s"] = _drive_stopwatch.elapsed_s()
        write_telemetry_csv(args.csv_path_bms, reading)

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
                "priority": PRIORITY_CAN,
            },
            "bms": {
                "normalizer": normalize_bms_frame,
                "policy": bms_policy,
                "packet_builder": build_bms_packet,
                "device_id": args.bms_device_id,
                "sink": bms_sink,
                "log_prefix": "bms",
                "priority": PRIORITY_CAN,
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
    """Serialize send_hex across threads sharing one LoRa modem.

    Plain FIFO — kept for backwards compatibility with any external code
    importing it directly. New code should use PriorityLockedTransport,
    which is what telemetry_sender wires up itself (see main()), so that a
    burst of low-priority CAN sends can't starve high-priority BMV sends.
    """
    def __init__(self, inner):
        self._inner = inner
        self._lock = threading.Lock()

    def send_hex(self, hex_str, priority=None):
        with self._lock:
            return self._inner.send_hex(hex_str)

    def __getattr__(self, name):
        return getattr(self._inner, name)


class PriorityLockedTransport:
    """Serialize send_hex across threads sharing one LoRa modem, but let
    high-priority callers (BMV) jump ahead of already-queued lower-priority
    callers (CAN) that are still waiting for their turn.

    This does NOT preempt a send that's already in flight — the modem can
    only do one AT+SEND at a time — but it guarantees that once the current
    send finishes, the next one to go is the highest-priority one waiting,
    not simply whoever queued up first. Combined with MAX_CAN_SENDS_PER_TICK
    (which limits how many CAN sends can even be queued back-to-back), this
    bounds the worst-case delay for a BMV send to roughly one in-flight
    send's duration, instead of an entire correlated CAN burst.

    Lower `priority` value = served first. Ties broken FIFO via an
    increasing counter.
    """

    def __init__(self, inner):
        self._inner = inner
        self._cv = threading.Condition()
        self._counter = itertools.count()
        self._waiting = []  # heap of [priority, seq] tickets
        self._busy = False

    def send_hex(self, hex_str, priority=PRIORITY_CAN):
        ticket = [priority, next(self._counter)]
        with self._cv:
            heapq.heappush(self._waiting, ticket)
            while self._busy or self._waiting[0] is not ticket:
                self._cv.wait()
            self._busy = True
            self._waiting.remove(ticket)
            heapq.heapify(self._waiting)
        try:
            return self._inner.send_hex(hex_str)
        finally:
            with self._cv:
                self._busy = False
                self._cv.notify_all()

    def __getattr__(self, name):
        return getattr(self._inner, name)


def _try_build_bmv_components(args):
    try:
        return build_bmv_sender_components(args)
    except Exception:
        print(f"[all] BMV unavailable, skipping:\n{traceback.format_exc()}", flush=True)
        return None


def _try_build_can_components(args):
    try:
        return build_can_sender_components(args)
    except Exception:
        print(f"[all] CAN unavailable, skipping:\n{traceback.format_exc()}", flush=True)
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
    parser.add_argument("--drive-start-current-a", type=float,
                        default=DEFAULT_DRIVE_START_CURRENT_A,
                        help="Battery discharge current (A) from BMV or BMS "
                             "that starts the elapsed drive timer")

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
            run_bmv_cached_sender(**components, transport=transport)
        except Exception:
            print(f"[bmv] thread crashed:\n{traceback.format_exc()}", flush=True)

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
            _run_all(args, PriorityLockedTransport(transport))
        elif args.device == "can":
            run_can_sender(**components, transport=transport)
        else:
            # Standalone --device bmv: still use the cache+timer sender so a
            # single slow/blocked send_hex() can't stall reading of the next
            # (potentially peak-current) VE.Direct frame. Priority is a
            # no-op here since BMV is the only thing on the modem, but it
            # keeps the code path identical to 'all' mode.
            run_bmv_cached_sender(**components, transport=transport)


if __name__ == "__main__":
    main()
