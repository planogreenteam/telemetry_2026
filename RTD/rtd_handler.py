#!/usr/bin/env python3
"""Pretty-printer for decoded RTD (motor PT100 / MAX31865) packets (receiver console)."""

from datetime import datetime, timezone


def format_rtd_packet(decoded):
    timestamp = datetime.fromtimestamp(decoded["timestamp"], tz=timezone.utc).isoformat()
    return (
        f"[rx:rtd] event={decoded['event_type'].name} "
        f"device={decoded['device_id']} "
        f"seq={decoded['seq']} "
        f"timestamp={timestamp} "
        f"fields={decoded['fields']}"
    )
