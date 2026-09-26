#!/usr/bin/env python3
"""Pretty-printer for decoded BMS packets (receiver console)."""

from datetime import datetime, timezone


def format_bms_packet(decoded):
    fields = dict(decoded["fields"])
    # SOC first: it's the number the team watches most.
    soc = fields.pop("soc_pct", None)
    timestamp = datetime.fromtimestamp(decoded["timestamp"], tz=timezone.utc).isoformat()
    return (
        f"[rx:bms] SOC={soc}% "
        f"event={decoded['event_type'].name} "
        f"device={decoded['device_id']} "
        f"seq={decoded['seq']} "
        f"timestamp={timestamp} "
        f"fields={fields}"
    )
