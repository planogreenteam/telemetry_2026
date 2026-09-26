#!/usr/bin/env python3
"""Decode an EG4 LL-S Modbus register block into named engineering units.

────────────────────────────────────────────────────────────────────
REGISTER MAP — VERIFY AGAINST THE REAL BATTERY
────────────────────────────────────────────────────────────────────
EG4 does not publish this map; it comes from community drivers for the
EG4 LL family (the same protocol the EG4 "BMS Test" PC software uses).
Before trusting it, run:

    python -m BMS.bms_probe --port <adapter> --address <DIP address>

and compare SOC, pack voltage and the cell voltages with the battery's
own display / EG4 app. If anything is off, fix the numbers in REG below —
nothing else in the pipeline needs to change.

The normalizer also cross-checks itself at runtime: it warns if SOC is
over 100 % or the sum of the cells disagrees with the pack voltage by
more than 1 V, both of which mean this map is wrong.
────────────────────────────────────────────────────────────────────
"""

import time


# Holding-register addresses (0-based). All registers are big-endian u16.
REG = {
    "pack_voltage":   0,    # 0.01 V
    "current":        1,    # 0.01 A, signed (see CURRENT_SIGN)
    "cell_first":     2,    # cells 1..16 in regs 2..17, mV
    "temp_pcb":       18,   # °C, signed
    "temp_max":       19,   # °C, signed
    "temp_avg":       20,   # °C, signed
    "remaining_ah":   21,   # Ah
    "max_charge_a":   22,   # A (charge current limit)
    "soh":            23,   # %
    "soc":            24,   # %
    "status":         25,   # bitfield / enum
    "warning":        26,   # bitfield
    "protection":     27,   # bitfield
    "error":          28,   # error code
    "cycle_hi":       29,   # cycle count, u32 high word
    "cycle_lo":       30,   # cycle count, u32 low word
    "full_ah":        31,   # full-charge capacity, Ah
}

NUM_CELLS = 16

# Registers fetched in one request by BMS/bms_reader.py.
REGISTER_BLOCK_START = 0
REGISTER_BLOCK_COUNT = 39

# The rest of the pipeline (and the BMV) treats NEGATIVE current as
# discharge — the drive timer starts on it. If the probe shows the battery
# reporting discharge as positive, set this to -1.
CURRENT_SIGN = 1

# Cells reading below this are treated as "not fitted" (e.g. a pack with
# fewer than 16 cells reports zeros) and left out of min/max/sum.
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
    regs = raw_frame["registers"]
    if len(regs) < REGISTER_BLOCK_COUNT:
        raise ValueError(f"BMS frame has {len(regs)} registers, "
                         f"expected {REGISTER_BLOCK_COUNT}")

    first = REG["cell_first"]
    cells_mv = regs[first:first + NUM_CELLS]
    fitted = [(i + 1, mv) for i, mv in enumerate(cells_mv) if mv >= MIN_VALID_CELL_MV]

    pack_voltage_v = regs[REG["pack_voltage"]] / 100.0
    fields = {
        "battery_voltage_v": pack_voltage_v,
        "battery_current_a": CURRENT_SIGN * _s16(regs[REG["current"]]) / 100.0,
        "soc_pct":           regs[REG["soc"]],
        "soh_pct":           regs[REG["soh"]],
        "remaining_ah":      regs[REG["remaining_ah"]],
        "full_capacity_ah":  regs[REG["full_ah"]],
        "max_charge_a":      regs[REG["max_charge_a"]],
        "cycle_count":       (regs[REG["cycle_hi"]] << 16) | regs[REG["cycle_lo"]],
        "temp_pcb_c":        _s16(regs[REG["temp_pcb"]]),
        "temp_max_c":        _s16(regs[REG["temp_max"]]),
        "temp_avg_c":        _s16(regs[REG["temp_avg"]]),
        "status":            regs[REG["status"]],
        "warning_flags":     regs[REG["warning"]],
        "protection_flags":  regs[REG["protection"]],
        "error_code":        regs[REG["error"]],
        "cell_count":        len(fitted),
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

    if fields["soc_pct"] > 100:
        _sanity_warn(f"SOC reads {fields['soc_pct']} %")

    for i, mv in enumerate(cells_mv, start=1):
        fields[f"cell_{i:02d}_mv"] = mv

    return {
        "device_type": "bms",
        "device_id":   device_id,
        "timestamp":   int(raw_frame.get("rx_timestamp") or time.time()),
        "fields":      fields,
    }
