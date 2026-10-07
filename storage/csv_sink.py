#!/usr/bin/env python3
"""On-car CSV logging: one file per source, one row per reading."""

import csv
from datetime import datetime, timezone
from pathlib import Path

from BMS.bms_normalizer import UNIDENTIFIED_REGS


# Columns per device_type. Readings are written by name, so a reading that
# lacks some columns (e.g. an MPPT status frame has no pv_power_w) leaves
# them blank, and extra fields are ignored.
CSV_COLUMNS = {
    "bmv": (
        "voltage_mv", "current_ma", "power_w", "charge_state", "alarm",
        "elapsed_s",
    ),
    "mppt": (
        "mppt_index", "packet_id",
        "pv_voltage_v", "pv_current_a", "pv_power_w",
        "battery_voltage_v", "battery_current_a",
        "mode", "mode_name", "fault", "fault_name", "enabled",
        "ambient_temp_c", "heatsink_temp_c", "raw_hex",
    ),
    "bms": (
        "soc_pct", "soh_pct", "battery_voltage_v", "battery_current_a",
        "cell_sum_v", "cell_v_max_mv", "cell_v_min_mv", "cell_v_delta_mv",
        "cell_max_idx", "cell_min_idx", "cell_count",
        "remaining_ah", "full_capacity_ah", "cycle_count", "temp_max_c",
        "elapsed_s",
        *(f"cell_{i:02d}_mv" for i in range(1, 17)),
        # Raw registers not identified yet (see BMS/bms_normalizer.py).
        *(f"reg_{addr}" for addr in UNIDENTIFIED_REGS),
    ),
    "rtd": (
        "motor_temp_c", "resistance_ohm", "raw_code", "fault", "fault_name",
    ),
}


# One persistent append handle per path for the life of the process.
# Opening + stat-ing + closing the file on every row costs several
# filesystem calls per write on an SD card, and this runs inside reader
# threads — an SD latency spike there would delay reading the next frame.
# Each path is only ever written by a single thread, so no locking.
_writers = {}


def open_csv_writer(csv_path, columns, cache=_writers):
    """Return a cached (file, DictWriter) appending to csv_path.

    If the file already exists with a DIFFERENT header (written by an older
    version of this code), it is renamed to <name>.old-<time>.csv first
    rather than appending rows whose columns don't line up."""
    key = str(csv_path)
    entry = cache.get(key)
    if entry is not None and not entry[0].closed:
        return entry

    path = Path(csv_path)
    if path.exists() and path.stat().st_size > 0:
        with path.open(newline="") as existing:
            header = next(csv.reader(existing), [])
        if tuple(header) != tuple(columns):
            stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
            old = path.with_name(f"{path.stem}.old-{stamp}{path.suffix}")
            path.rename(old)
            print(f"[csv] {path} had an old column layout; moved it to {old}",
                  flush=True)

    needs_header = not path.exists() or path.stat().st_size == 0
    csv_file = path.open("a", newline="")
    writer = csv.DictWriter(csv_file, fieldnames=columns, extrasaction="ignore")
    if needs_header:
        writer.writeheader()
    entry = (csv_file, writer)
    cache[key] = entry
    return entry


def write_telemetry_csv(csv_path, reading):
    device_type = reading.get("device_type")
    data_columns = CSV_COLUMNS.get(device_type) or tuple(reading["fields"])
    columns = ("timestamp", "device_id", *data_columns)
    csv_file, writer = open_csv_writer(csv_path, columns)

    row = dict(reading["fields"])
    for extra in ("mppt_index", "packet_id"):
        if extra in reading:
            row.setdefault(extra, reading[extra])
    row["timestamp"] = datetime.now(timezone.utc).isoformat()
    row["device_id"] = reading.get("device_id")
    writer.writerow(row)
    csv_file.flush()
