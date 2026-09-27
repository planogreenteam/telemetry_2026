#!/usr/bin/env python3
"""Pretty-printer for decoded BMS packets (receiver console)."""

from datetime import datetime, timezone


def format_bms_packet(decoded):
    fields = dict(decoded["fields"])
    # SOC is left off the console line (remaining_ah shows the same thing
    # for the 100 Ah pack); it is still written to CSV and InfluxDB.
    fields.pop("soc_pct", None)
    timestamp = datetime.fromtimestamp(decoded["timestamp"], tz=timezone.utc).isoformat()
    return (
        f"[rx:bms] event={decoded['event_type'].name} "
        f"device={decoded['device_id']} "
        f"seq={decoded['seq']} "
        f"timestamp={timestamp} "
        f"fields={fields}"
    )
