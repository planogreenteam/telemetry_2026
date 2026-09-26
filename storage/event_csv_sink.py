#!/usr/bin/env python3
"""Ground-station CSV: one row per decoded packet, all message types."""

from datetime import datetime, timezone

from storage.csv_sink import open_csv_writer
from telemetry_packet import MsgType, layout_field_names


_META_HEADERS = (
    "received_at",
    "packet_timestamp",
    "msg_type",
    "event_type",
    "device_id",
    "seq",
)


def _all_field_names():
    names = []
    for msg_type in MsgType:
        for name in layout_field_names(msg_type):
            if name not in names:
                names.append(name)
    return tuple(names)


# Every field any packet type can carry, so BMV, MPPT and BMS rows all
# land in one file with a stable header.
EVENT_CSV_HEADERS = _META_HEADERS + _all_field_names()

_writers = {}


def write_event_csv(csv_path, event):
    csv_file, writer = open_csv_writer(csv_path, EVENT_CSV_HEADERS, cache=_writers)

    row = dict(event.get("fields", {}))
    row.update({
        "received_at": datetime.now(timezone.utc).isoformat(),
        "packet_timestamp": datetime.fromtimestamp(event["timestamp"], tz=timezone.utc).isoformat(),
        "msg_type": event["msg_type"].name,
        "event_type": event["event_type"].name,
        "device_id": event["device_id"],
        "seq": event["seq"],
    })
    writer.writerow(row)
    csv_file.flush()
