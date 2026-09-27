#!/usr/bin/env python3
"""Decode EG4 LL-S Modbus registers into named engineering units.

────────────────────────────────────────────────────────────────────
REGISTER MAP — from a bms_probe dump of the team's battery
────────────────────────────────────────────────────────────────────
Verified against the battery's own display (53.16 V, 0.0 A, 85 %):

    reg 21        SOC, %                       85
    reg 22        pack voltage, 0.01 V         5316  = 53.16 V (display 53.16)
    reg 26        remaining capacity, 0.01 Ah  8500  = 85.00 Ah
    reg 27        full capacity, 0.01 Ah       10000 = 100.00 Ah  (85/100 = SOC)
    reg 37 / 38   highest / lowest cell, mV    3323 / 3322
    reg 41        number of cells              16
    reg 113..128  cells 1..16, mV              3322/3323 — they sum to 53.16 V
    reg 24        hottest cell sensor, °C      36 then 37 in two dumps; the
                                               display's four cell sensors
                                               read 37/36/36/37 (PCB 40)

Probable, not yet proven (were 0 or unambiguous-looking in one dump):

    reg 23        current, 0.01 A signed       0     (display 0.0 A) — confirm
                                                     with the probe while current
                                                     is flowing, and check the sign
    reg 30        cycle count                  7
    reg 32        state of health, %           100

Register 24 could in principle be the average rather than the maximum —
the four sensors are within 1 °C of each other, so it makes no practical
difference; a hot spot after a drive would tell them apart.

The battery answers holding registers 0..128 only (reads starting at 128
or above time out). The individual cell-temperature sensors and the PCB
temperature shown on the display are NOT in that table; they may be in
the input-register table (bms_probe --input-registers).

Not identified yet (logged raw to bms_data.csv as reg_NN so they can be
matched later): 19 (97, constant), 25 (5000), 28 (530), 33 (5600),
35 (10000), 39 / 40 (small numbers that change: probably the highest /
lowest cell numbers). The warning / protection / error flags are
probably among the registers that read 0 while no alarm is active, so
they are NOT sent until identified — a wrong register there is worse than
no value.

To re-check: python3 -m BMS.bms_probe --address <DIP address>
────────────────────────────────────────────────────────────────────
"""

import time


# Holding-register addresses (0-based). All registers are big-endian u16.
REG = {
    "soc":            21,   # %
    "pack_voltage":   22,   # 0.01 V
    "current":        23,   # 0.01 A, signed (see CURRENT_SIGN) — probable
    "remaining_ah":   26,   # 0.01 Ah
    "full_ah":        27,   # 0.01 Ah
    "cycle_count":    30,   # probable
    "soh":            32,   # %, probable
    "temp_max":       24,   # °C, signed; hottest cell sensor
    "cell_count":     41,
    "cell_first":     113,  # cells 1..16 in regs 113..128, mV
}

NUM_CELLS = 16

# Registers read but not yet identified; logged raw for later mapping.
UNIDENTIFIED_REGS = (19, 25, 28, 33, 35, 39, 40)

# (start, count) ranges fetched every poll by BMS/bms_reader.py. The reader
# splits them into requests of at most 32 registers (sizes the probe has
# confirmed the battery answers).
REGISTER_BLOCKS = (
    (0, 42),
    (REG["cell_first"], NUM_CELLS),
)

# The rest of the pipeline (and the BMV) treats NEGATIVE current as
# discharge — the drive timer starts on it. If the probe shows the battery
# reporting discharge as positive, set this to -1.
CURRENT_SIGN = 1

# Cells reading below this are treated as "not fitted" and left out of
# min/max/sum.
MIN_VALID_CELL_MV = 1000

_SANITY_WARN_INTERVAL = 60.0
_last_sanity_warning = 0.0


def _s16(value):
    return value - 0x10000 if value & 0x8000 else value


def _sanity_warn(message):
    global _last_sanity_warning
    now = time.monotonic()
    if now - _last_sanity_warning >= _SANITY_WARN_INTERVAL:
        print(f"[bms] WARNING: {message} — check the register map in "
              f"BMS/bms_normalizer.py with BMS/bms_probe.py", flush=True)
        _last_sanity_warning = now


def normalize_bms_frame(raw_frame, device_id):
    """raw_frame["registers"] is a {register_address: u16} dict."""
    regs = raw_frame["registers"]
    missing = [addr for addr in REG.values() if addr not in regs]
    if missing:
        raise ValueError(f"BMS frame is missing registers {missing}")

    first = REG["cell_first"]
    cells_mv = [regs.get(first + i, 0) for i in range(NUM_CELLS)]
    fitted = [(i + 1, mv) for i, mv in enumerate(cells_mv) if mv >= MIN_VALID_CELL_MV]

    pack_voltage_v = regs[REG["pack_voltage"]] / 100.0
    fields = {
        "battery_voltage_v": pack_voltage_v,
        "battery_current_a": CURRENT_SIGN * _s16(regs[REG["current"]]) / 100.0,
        "soc_pct":           regs[REG["soc"]],
        "soh_pct":           regs[REG["soh"]],
        "remaining_ah":      regs[REG["remaining_ah"]] / 100.0,
        "full_capacity_ah":  regs[REG["full_ah"]] / 100.0,
        "cycle_count":       regs[REG["cycle_count"]],
        "temp_max_c":        _s16(regs[REG["temp_max"]]),
        "cell_count":        regs[REG["cell_count"]],
    }

    if fitted:
        max_idx, max_mv = max(fitted, key=lambda c: c[1])
        min_idx, min_mv = min(fitted, key=lambda c: c[1])
        cell_sum_v = sum(mv for _, mv in fitted) / 1000.0
        fields.update({
            "cell_v_max_mv":   max_mv,
            "cell_v_min_mv":   min_mv,
            "cell_v_delta_mv": max_mv - min_mv,
            "cell_max_idx":    max_idx,
            "cell_min_idx":    min_idx,
            "cell_sum_v":      round(cell_sum_v, 3),
        })
        if abs(cell_sum_v - pack_voltage_v) > 1.0:
            _sanity_warn(f"cell sum {cell_sum_v:.2f} V != pack voltage "
                         f"{pack_voltage_v:.2f} V")
    else:
        _sanity_warn("no cell voltages found")

    if fields["soc_pct"] > 100:
        _sanity_warn(f"SOC reads {fields['soc_pct']} %")

    for i, mv in enumerate(cells_mv, start=1):
        fields[f"cell_{i:02d}_mv"] = mv
    for addr in UNIDENTIFIED_REGS:
        if addr in regs:
            fields[f"reg_{addr}"] = regs[addr]

    return {
        "device_type": "bms",
        "device_id":   device_id,
        "timestamp":   int(raw_frame.get("rx_timestamp") or time.time()),
        "fields":      fields,
    }
