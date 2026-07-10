#!/usr/bin/env python3
import csv
from datetime import datetime
from pathlib import Path


CSV_HEADERS = (
    "timestamp",
    "voltage_mv",
    "current_ma",
    "power_w",
    "charge_state",
    "alarm",
)


# One persistent append handle per path for the life of the process.
# Opening + stat-ing + closing the file on every row costs several
# filesystem calls per write on an SD card, and this now runs inside the
# BMV *reader* thread — an SD latency spike there would delay reading the
# next VE.Direct frame. Each path is only ever written by a single thread
# (BMV reader / CAN tx), so no locking is needed.
_open_files = {}


def write_telemetry_csv(csv_path, reading):
    key = str(csv_path)
    csv_file = _open_files.get(key)
    if csv_file is None or csv_file.closed:
        path = Path(csv_path)
        needs_header = not path.exists() or path.stat().st_size == 0
        csv_file = path.open("a", newline="")
        if needs_header:
            csv.writer(csv_file).writerow(CSV_HEADERS)
        _open_files[key] = csv_file

    fields = reading["fields"]
    csv.writer(csv_file).writerow(
        [
            datetime.utcnow().isoformat(),
            fields.get("voltage_mv"),
            fields.get("current_ma"),
            fields.get("power_w"),
            fields.get("charge_state"),
            fields.get("alarm"),
        ]
    )
    csv_file.flush()
