#!/usr/bin/env python3
"""Pretty-printer for decoded MPPT packets.

Wired in by the receiver's build_handlers() as {MsgType.MPPT: format_mppt_packet}.

Returns a single-line string for the receiver's console output. Influx
ingestion is independent — sinks see the full decoded dict, not this
formatted line.
"""

from datetime import datetime, timezone


def _iso_ts(ts):
    return datetime.fromtimestamp(ts, tz=timezone.utc).isoformat()


def format_mppt_packet(decoded):
    fields = decoded["fields"]
    return (
        f"[rx:mppt] event={decoded['event_type'].name} "
        f"device={decoded['device_id']} "
        f"seq={decoded['seq']} "
        f"timestamp={_iso_ts(decoded['timestamp'])} "
        f"fields={fields}"
    )
