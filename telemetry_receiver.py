#!/usr/bin/env python3
import argparse
import os
import queue
import sys
import threading
import time
from datetime import datetime, timezone

try:
    sys.stdout.reconfigure(line_buffering=True)
except AttributeError:
    pass

# Fix Windows console encoding
if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

from BMV.bmv_handler import format_bmv_packet
from CAN.can_handler import format_mppt_packet, format_bms_packet
from LORA.lora_transport import LoRaTransport, extract_hex_payload
from storage.event_csv_sink import write_event_csv
from telemetry_packet import MsgType, decode_packet, is_batch, split_batch


# ─────────────────────────────────────────────────────────────────────────────
# INFLUXDB WRITER
# ─────────────────────────────────────────────────────────────────────────────

def _flatten_event(event: dict, prefix: str = "", out: dict | None = None) -> dict:
    """Flatten a decoded packet so nested dicts become dotted-name fields.
    e.g. {'data': {'voltage': 12.5}} -> {'data_voltage': 12.5}.
    Lists/tuples of primitives become indexed fields: foo_0, foo_1, ..."""
    if out is None:
        out = {}
    for k, v in event.items():
        key = f"{prefix}{k}" if not prefix else f"{prefix}_{k}"
        if isinstance(v, dict):
            _flatten_event(v, prefix=key, out=out)
        elif isinstance(v, (list, tuple)):
            for i, item in enumerate(v):
                if isinstance(item, dict):
                    _flatten_event(item, prefix=f"{key}_{i}", out=out)
                else:
                    out[f"{key}_{i}"] = item
        else:
            out[key] = v
    return out


class InfluxWriter:
    def __init__(self, url, token, org, bucket, measurement, bucket_map=None):
        from influxdb_client import InfluxDBClient
        from influxdb_client.client.write_api import SYNCHRONOUS

        self.default_bucket = bucket
        self.bucket_map = bucket_map or {}
        self.measurement = measurement
        # timeout is in milliseconds. Keep it short: a healthy Influx
        # answers a write in well under a second, and a sick one should
        # fail fast on the background writer thread rather than hold a
        # connection open for the library's ~10s default.
        self.client = InfluxDBClient(url=url, token=token, org=org,
                                     timeout=5_000)
        self.write_api = self.client.write_api(write_options=SYNCHRONOUS)
        print(f"[influx] Connected to {url}  default_bucket={bucket}")
        for msg_type, b in self.bucket_map.items():
            print(f"[influx]   {msg_type.name} -> bucket={b}")

    def write(self, event: dict, tags: dict | None = None):
        if not event:
            return

        flat = _flatten_event(event)

        numeric_fields: dict = {}
        string_fields: dict = {}
        point_tags = dict(tags or {})

        for k, v in flat.items():
            if v is None:
                continue

            # Promote these keys to InfluxDB tags for fast filtering
            if k == "msg_type":
                if isinstance(v, MsgType):
                    point_tags["msg_type"] = v.name
                else:
                    point_tags["msg_type"] = str(v)
                continue

            if k == "device_id":
                point_tags["device_id"] = str(int(v))
                continue

            if k == "mppt_index":
                point_tags["mppt_index"] = f"MPPT_{int(v)}"
                continue

            if isinstance(v, bool):
                numeric_fields[k] = int(v)
            elif isinstance(v, (int, float)):
                numeric_fields[k] = float(v)
            elif isinstance(v, str):
                if k in ("payload_hex",):
                    continue
                string_fields[k] = v
            elif isinstance(v, MsgType):
                string_fields[k] = v.name
            else:
                string_fields[k] = str(v)

        all_fields = {**numeric_fields, **string_fields}
        if not all_fields:
            print(f"[influx] Skipping write: no usable fields in {event}")
            return

        if not numeric_fields:
            all_fields["received"] = 1.0
            print(f"[influx] No numeric fields in event, only strings: "
                  f"{list(string_fields.keys())}")

        record = {
            "measurement": self.measurement,
            "tags": point_tags,
            "fields": all_fields,
            "time": datetime.now(timezone.utc),
        }
        try:
            msg_type = point_tags.get("msg_type")
            bucket = self.default_bucket
            for mt, b in self.bucket_map.items():
                if mt.name == msg_type:
                    bucket = b
                    break
            self.write_api.write(bucket=bucket, record=record)
        except Exception as exc:
            print(f"[influx] Write failed: {exc}")

    def close(self):
        try:
            self.client.close()
        except Exception:
            pass


def make_influx_sink(writer: InfluxWriter, tags: dict):
    # Legacy synchronous sink — writes inline in the caller's thread.
    # The receiver now uses AsyncInfluxSink instead so a slow/dead Influx
    # can never stall the modem polling loop. Kept for any external code
    # importing it directly.
    def _sink(event: dict):
        writer.write(event, tags=tags)
    return _sink


class AsyncInfluxSink:
    """Wraps an InfluxWriter so writes happen on a background thread.

    The receive loop polls the modem with AT+RECV on a tight interval; if
    the modem buffers only the most recent packet, any stall in that loop
    silently drops whatever arrived earlier in the window. A synchronous
    Influx write is exactly such a stall: a healthy write costs tens of ms,
    but a sick Influx (resource-starved Docker/WSL, slow disk) holds the
    connection for up to the client timeout — per packet. Handing events to
    a bounded queue instead makes the receive-loop cost of this sink a
    put_nowait(), i.e. microseconds, regardless of Influx's health.

    If the queue fills (sustained outage or persistent slowness), the
    NEWEST events are dropped for Influx only — the CSV sink runs
    independently and still records everything, so an outage costs
    dashboard points, never data.
    """

    def __init__(self, writer: InfluxWriter, tags: dict, maxsize: int = 500):
        self._writer = writer
        self._tags = tags
        self._q = queue.Queue(maxsize=maxsize)
        self._dropped = 0
        self._thread = threading.Thread(
            target=self._worker, name="influx-writer", daemon=True
        )
        self._thread.start()

    def __call__(self, event: dict):
        try:
            self._q.put_nowait(event)
        except queue.Full:
            self._dropped += 1
            # Log the first drop and then every 100th, so a long outage
            # doesn't flood the console at packet rate.
            if self._dropped % 100 == 1:
                print(
                    f"[influx] queue full ({self._q.maxsize} events "
                    f"waiting); dropped {self._dropped} event(s) so far "
                    f"(Influx too slow or down — CSV still recording)",
                    flush=True,
                )

    def _worker(self):
        while True:
            event = self._q.get()
            try:
                # InfluxWriter.write() already catches and logs write
                # failures; this outer catch is a belt-and-braces guard so
                # nothing can kill the writer thread.
                self._writer.write(event, tags=self._tags)
            except Exception as exc:
                print(f"[influx] Write failed: {exc}", flush=True)
            finally:
                self._q.task_done()


# ─────────────────────────────────────────────────────────────────────────────
# ASCII FALLBACK DECODER
# ─────────────────────────────────────────────────────────────────────────────

import json
import re


def _clean_ascii(text: str) -> str:
    if not text:
        return ""
    return text.replace("\x00", "").replace("\ufffd", "").strip()


def decode_ascii_payload(raw: bytes) -> dict | None:
    if not raw:
        return None

    hex_repr = raw.hex()

    try:
        text = _clean_ascii(raw.decode("utf-8"))
    except UnicodeDecodeError:
        return None

    if not text:
        return None

    event: dict = {
        "msg_type": "ASCII",
        "payload": text,
        "payload_hex": hex_repr,
    }

    if text.startswith("{"):
        try:
            obj = json.loads(text)
            for k, v in obj.items():
                try:
                    event[k.lower()] = float(v)
                except (TypeError, ValueError):
                    if isinstance(v, str):
                        event[k.lower()] = v
            return event
        except json.JSONDecodeError:
            pass

    kv = re.findall(r"(\w+)=([-+]?[\d.]+)", text)
    if kv:
        for k, v in kv:
            try:
                event[k.lower()] = float(v)
            except ValueError:
                pass
        if len(event) > 3:
            return event

    try:
        event["value"] = float(text)
        return event
    except ValueError:
        pass

    parts = [p.strip() for p in text.split(",") if p.strip()]
    if len(parts) >= 2:
        try:
            nums = [float(p) for p in parts]
            for i, v in enumerate(nums[:5]):
                event[f"value_{i}"] = v
            return event
        except ValueError:
            pass

    return event


# ─────────────────────────────────────────────────────────────────────────────
# DECODE / DISPATCH
# ─────────────────────────────────────────────────────────────────────────────

def _looks_decoded(decoded) -> bool:
    if not isinstance(decoded, dict):
        return False
    return isinstance(decoded.get("msg_type"), MsgType)


def decode_line(line, extract_payload, decoder, ascii_fallback=True):
    """Decode one modem RX line into a LIST of events. A line usually
    carries one packet, but a batch frame (see telemetry_packet.build_batch)
    carries several — the sender packs all due CAN readings into one LoRa
    frame to save modem round-trips."""
    payload_hex = extract_payload(line)
    if not payload_hex:
        return []
    packet = bytes.fromhex(payload_hex)
    if not packet:
        return []

    if is_batch(packet):
        events = []
        try:
            sub_packets = split_batch(packet)
        except ValueError as exc:
            print(f"[rx] Bad batch frame: {exc}")
            return []
        for sub in sub_packets:
            try:
                decoded = decoder(sub)
            except ValueError as exc:
                # One corrupted element (each sub-packet has its own CRC)
                # doesn't invalidate its batch-mates.
                print(f"[rx] Failed to decode batch element: {exc}")
                continue
            if _looks_decoded(decoded):
                events.append(decoded)
        return events

    binary_error = None
    if len(packet) > 2:
        try:
            decoded = decoder(packet)
        except ValueError as exc:
            binary_error = exc
            decoded = None

        if _looks_decoded(decoded):
            return [decoded]

    if ascii_fallback:
        ascii_decoded = decode_ascii_payload(packet)
        if ascii_decoded is not None:
            return [ascii_decoded]

    if binary_error is not None:
        raise binary_error
    return []


def route_packet(decoded, handlers=None):
    if not handlers:
        return None
    msg_type = decoded.get("msg_type")
    handler = handlers.get(msg_type)
    if handler is None and isinstance(msg_type, MsgType):
        handler = handlers.get(msg_type.value)
    if handler is None:
        return None
    return handler(decoded)


def dispatch_event(decoded, sinks=None):
    for sink in sinks or ():
        try:
            sink(decoded)
        except Exception as exc:
            print(f"[rx] Sink error: {exc}")


def _now_str():
    return datetime.now().strftime("%H:%M:%S")


# Modem status lines that accompany every streamed frame. Pure chatter —
# skipped silently unless --show-raw is set.
_MODEM_CHATTER_PREFIXES = ("RXDONE", "RSSI", "OK", "NULL", "SNR")


def _is_modem_chatter(line: str) -> bool:
    upper = line.strip().upper()
    return any(upper.startswith(p) for p in _MODEM_CHATTER_PREFIXES)


class SeqGapTracker:
    """Detects missing wire sequence numbers per stream, making packet loss
    a number instead of a feeling — this is how radio/timing changes get
    validated. Keyed by msg_type (not device_id): all six MPPTs share one
    sender-side policy and therefore one seq counter, so per-device seqs
    are intentionally interleaved. Note the sender allocates a seq per
    *built* packet, so a send the modem rejected counts as a gap here too —
    which is correct, the packet was lost either way."""

    def __init__(self):
        self._last = {}
        self._received = 0
        self._lost = 0

    def observe(self, decoded):
        msg_type = decoded.get("msg_type")
        seq = decoded.get("seq")
        if not isinstance(msg_type, MsgType) or seq is None:
            return
        self._received += 1
        last = self._last.get(msg_type)
        self._last[msg_type] = seq
        if last is None:
            return
        gap = (seq - last - 1) & 0xFFFF
        # Ignore huge "gaps" (sender restart / duplicate) — only count
        # plausible runs of loss.
        if 0 < gap < 0x1000:
            self._lost += gap
            total = self._received + self._lost
            print(f"[rx] seq gap: {msg_type.name} missed {gap} packet(s) "
                  f"(last={last}, got={seq}); session loss "
                  f"{self._lost}/{total} ({100.0 * self._lost / total:.1f}%)",
                  flush=True)


def run_receiver(
    *,
    transport,
    decoder,
    extract_payload,
    recv_format=0,
    poll_interval=0.2,
    wait=0.3,
    show_raw=False,
    handlers=None,
    sinks=None,
    log_prefix="receiver",
    ascii_fallback=True,
    rx_mode="stream",
):
    use_stream = rx_mode == "stream" and hasattr(transport, "read_stream_lines")
    if rx_mode == "stream" and not use_stream:
        print(f"[{log_prefix}] Transport has no read_stream_lines(); "
              f"falling back to AT+RECV polling", flush=True)

    if use_stream:
        # Stream mode: the modem prints each received frame on its own —
        # no AT+RECV round-trip, so each frame crosses the modem's
        # 9600-baud UART exactly ONCE (polling makes it print everything
        # twice, doubling UART load and truncating batch dumps under
        # load). Draining the buffer is nearly free, so the loop can spin
        # much faster than the AT+RECV poll cadence.
        poll_interval = min(poll_interval, 0.05)
        print(f"[{log_prefix}] RX mode: stream (unsolicited modem output, "
              f"poll={poll_interval}s)", flush=True)
    else:
        print(f"[{log_prefix}] RX mode: AT+RECV polling "
              f"(poll={poll_interval}s)", flush=True)

    print(f"[{log_prefix}] Starting receiver loop", flush=True)
    print(f"\nListening for LoRa data - press Ctrl-C to stop\n", flush=True)

    consecutive_errors = 0
    MAX_CONSECUTIVE_ERRORS = 10
    seq_gaps = SeqGapTracker()

    while True:
        try:
            if use_stream:
                lines = transport.read_stream_lines()
            else:
                lines = transport.receive_hex_lines(recv_format=recv_format,
                                                    wait=wait)
            consecutive_errors = 0
        except (RuntimeError, OSError) as exc:
            consecutive_errors += 1
            print(f"[rx] Modem poll failed ({consecutive_errors}/"
                  f"{MAX_CONSECUTIVE_ERRORS}): {exc}", flush=True)
            if consecutive_errors >= MAX_CONSECUTIVE_ERRORS:
                print(f"[rx] Modem unresponsive for {MAX_CONSECUTIVE_ERRORS} "
                      f"polls in a row - giving up.", flush=True)
                raise
            time.sleep(min(2.0 * consecutive_errors, 10.0))
            continue

        for line in lines:
            if show_raw:
                print(f"[rx-raw] {line}")

            # Per-frame modem status lines (rxDone, Rssi=, ...) are
            # expected in stream mode — not worth a console line each.
            if _is_modem_chatter(line):
                continue

            try:
                events = decode_line(line, extract_payload, decoder,
                                     ascii_fallback=ascii_fallback)
            except ValueError as exc:
                print(f"[rx] Failed to parse/decode '{line}': {exc}")
                continue

            if not events:
                if not show_raw:
                    print(f"[rx-raw] {line}")
                continue

            for decoded in events:
                msg_type = decoded.get("msg_type")
                type_label = msg_type.name if isinstance(msg_type, MsgType) else str(msg_type)
                print(f"[{_now_str()}] Received packet  type={type_label}", flush=True)

                seq_gaps.observe(decoded)
                dispatch_event(decoded, sinks=sinks)

                routed = route_packet(decoded, handlers=handlers)
                if routed is None:
                    print(f"[rx] {decoded}")
                elif routed != "":
                    print(routed)

        time.sleep(poll_interval)


# ─────────────────────────────────────────────────────────────────────────────
# DEFAULTS
# ─────────────────────────────────────────────────────────────────────────────

DEFAULT_TRANSPORT = "lora"
DEFAULT_PORT = "COM4"
DEFAULT_BAUD = 9600
DEFAULT_FREQ = "868.100"
# BW 2 = 500 kHz — MUST match telemetry_sender.py DEFAULT_BW.
DEFAULT_BW = 2
DEFAULT_SF = 7
DEFAULT_POWER = 20
DEFAULT_CR = 1
DEFAULT_CRC = 0
DEFAULT_HEADER = 0
DEFAULT_IQ = 0
DEFAULT_PREAMBLE = 8
DEFAULT_SYNCWORD = 0
DEFAULT_GROUP = 0
DEFAULT_RX_TIMEOUT = 65535
DEFAULT_RX_ACK = 0
DEFAULT_CSV_PATH = "received_events.csv"

DEFAULT_INFLUX_URL = os.getenv("INFLUX_URL", "http://localhost:8086")
DEFAULT_INFLUX_TOKEN = os.getenv("INFLUX_TOKEN", "C_pCeeM8QagSy6FqWRpYR2ZQeNLr5yxAewGfa0Kdm5jnPBy_Dpf3cD-UMLqQ4A7etWLDCkJi3r_B69EjqHs8AA==")
DEFAULT_INFLUX_ORG = os.getenv("INFLUX_ORG", "my-org")
DEFAULT_INFLUX_BUCKET = os.getenv("INFLUX_BUCKET", "Default-data")
DEFAULT_INFLUX_BUCKET_BMV = os.getenv("INFLUX_BUCKET_BMV", "BMV-data")
DEFAULT_INFLUX_BUCKET_CAN = os.getenv("INFLUX_BUCKET_CAN", "CAN-data")
DEFAULT_INFLUX_MEASUREMENT = os.getenv("INFLUX_MEASUREMENT", "telemetry")


# ─────────────────────────────────────────────────────────────────────────────
# WIRING
# ─────────────────────────────────────────────────────────────────────────────

def build_transport(args):
    return LoRaTransport(
        port=args.port,
        baud=args.baud,
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
        rx_timeout=args.rx_timeout,
        rx_ack=args.rx_ack,
    )


def build_handlers(_args):
    return {
        MsgType.BMV:  format_bmv_packet,
        MsgType.MPPT: format_mppt_packet,
        MsgType.BMS:  format_bms_packet,
    }


def build_sinks(args, influx_writer=None):
    sinks = []

    if args.csv_path:
        sinks.append(
            lambda event: write_event_csv(args.csv_path, event)
            if "timestamp" in event else None
        )

    if influx_writer is not None:
        tags = {"source": "telemetry_receiver", "port": args.port}
        # Async: Influx writes run on their own thread so a slow or dead
        # Influx can never stall modem polling (which drops packets).
        sinks.append(AsyncInfluxSink(influx_writer, tags))

    return tuple(sinks)


def build_influx_writer(args):
    if not args.influx_enable:
        return None
    if not args.influx_token:
        print("[influx] --influx-enable set but no token provided "
              "(use --influx-token or INFLUX_TOKEN env var). Skipping InfluxDB.")
        return None

    bucket_map = {
        MsgType.BMV:  args.influx_bucket_bmv,
        MsgType.MPPT: args.influx_bucket_can,
        MsgType.BMS:  args.influx_bucket_can,
    }

    return InfluxWriter(
        url=args.influx_url,
        token=args.influx_token,
        org=args.influx_org,
        bucket=args.influx_bucket,
        measurement=args.influx_measurement,
        bucket_map=bucket_map,
    )


def build_parser():
    parser = argparse.ArgumentParser(description="Generic telemetry receiver")
    parser.add_argument("--transport", choices=("lora",), default=DEFAULT_TRANSPORT)
    parser.add_argument("--port", default=DEFAULT_PORT, help="Serial device path")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Serial baud rate")
    parser.add_argument("--freq", default=DEFAULT_FREQ, help="TX/RX frequency in MHz, e.g. 868.100")
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
    parser.add_argument("--rx-timeout", type=int, default=DEFAULT_RX_TIMEOUT,
                        help="RX window in seconds or 65535 always open")
    parser.add_argument("--rx-ack", type=int, default=DEFAULT_RX_ACK, help="ACK mode 0/1/2")
    parser.add_argument("--rx-mode", choices=("stream", "poll"), default="stream",
                        help="stream: read the modem's unsolicited RX output "
                             "(each frame crosses the modem UART once — "
                             "required for reliable batch reception). "
                             "poll: legacy AT+RECV polling.")
    parser.add_argument("--recv-format", type=int, choices=(0, 1), default=0,
                        help="AT+RECV format 0=hex 1=text (poll mode only)")
    parser.add_argument("--poll", type=float, default=0.15,
                        help="Seconds between AT+RECV polls. Keep this well "
                             "under the sender's per-frame interval: if the "
                             "modem buffers only the most recent packet, "
                             "slow polling silently drops everything that "
                             "arrived earlier in the window.")
    parser.add_argument("--csv-path", default=DEFAULT_CSV_PATH,
                        help="CSV output path for decoded events")
    parser.add_argument("--show-raw", action="store_true",
                        help="Print raw modem receive lines")
    parser.add_argument("--no-ascii-fallback", action="store_true",
                        help="Disable the ASCII fallback decoder (binary packets only)")

    parser.add_argument("--influx-enable", action=argparse.BooleanOptionalAction,
                        default=True,
                        help="Write decoded events to InfluxDB v2 (on by default; "
                             "use --no-influx-enable to disable)")
    parser.add_argument("--influx-url", default=DEFAULT_INFLUX_URL,
                        help="InfluxDB v2 URL (env: INFLUX_URL)")
    parser.add_argument("--influx-token", default=DEFAULT_INFLUX_TOKEN,
                        help="InfluxDB v2 API token (env: INFLUX_TOKEN)")
    parser.add_argument("--influx-org", default=DEFAULT_INFLUX_ORG,
                        help="InfluxDB v2 organisation (env: INFLUX_ORG)")
    parser.add_argument("--influx-bucket", default=DEFAULT_INFLUX_BUCKET,
                        help="InfluxDB v2 bucket (env: INFLUX_BUCKET)")
    parser.add_argument("--influx-bucket-bmv", default=DEFAULT_INFLUX_BUCKET_BMV,
                        help="InfluxDB bucket for BMV packets (env: INFLUX_BUCKET_BMV)")
    parser.add_argument("--influx-bucket-can", default=DEFAULT_INFLUX_BUCKET_CAN,
                        help="InfluxDB bucket for MPPT and BMS packets (env: INFLUX_BUCKET_CAN)")
    parser.add_argument("--influx-measurement", default=DEFAULT_INFLUX_MEASUREMENT,
                        help="InfluxDB measurement name (env: INFLUX_MEASUREMENT)")
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.transport != "lora":
        raise ValueError(f"Unsupported transport {args.transport}")

    influx_writer = build_influx_writer(args)

    try:
        with build_transport(args) as transport:
            run_receiver(
                transport=transport,
                decoder=decode_packet,
                extract_payload=extract_hex_payload,
                recv_format=args.recv_format,
                poll_interval=args.poll,
                wait=0.15,
                show_raw=args.show_raw,
                handlers=build_handlers(args),
                sinks=build_sinks(args, influx_writer=influx_writer),
                log_prefix="receiver",
                ascii_fallback=not args.no_ascii_fallback,
                rx_mode=args.rx_mode,
            )
    finally:
        if influx_writer is not None:
            influx_writer.close()


if __name__ == "__main__":
    main()
