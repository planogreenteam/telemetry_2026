#!/usr/bin/env python3
"""Turn a raw MAX31865 reading into resistance (ohms) and motor temperature (C).

    resistance = raw / 32768 * R_REF

Temperature uses the Callendar-Van Dusen equation (IEC 60751 coefficients)
solved directly for T >= 0 C, and a fifth-order polynomial fit below 0 C,
the same maths as Adafruit's MAX31865 driver.

A reading with a fault has no resistance or temperature, only the fault
byte, so a broken probe shows up as "no temperature, fault=..." rather than
as a plausible-looking wrong number.
"""

import math
import time


R_NOMINAL = 100.0   # PT100: 100 ohm at 0 C
R_REF = 430.0       # reference resistor on the Adafruit #3328 PT100 board

RTD_A = 3.9083e-3
RTD_B = -5.775e-7

# MAX31865 fault status register bits.
FAULT_BITS = (
    (0x80, "rtd_high"),             # RTD above high threshold (open probe)
    (0x40, "rtd_low"),              # RTD below low threshold (shorted probe)
    (0x20, "refin_high"),           # REFIN- > 0.85 x VBIAS
    (0x10, "refin_low"),            # REFIN- < 0.85 x VBIAS, FORCE- open
    (0x08, "rtdin_low"),            # RTDIN- < 0.85 x VBIAS, FORCE- open
    (0x04, "over_under_voltage"),
    # Bits 1-0 are unused by the chip; bit 0 is used here for "the chip
    # reported no fault but the code is 0", which only happens when the
    # board isn't really converting (e.g. SDO not connected).
    (0x01, "no_signal"),
)
FAULT_NO_SIGNAL = 0x01


def fault_name(fault):
    if not fault:
        return ""
    return "|".join(name for bit, name in FAULT_BITS if fault & bit)


def resistance_from_raw(raw, r_ref=R_REF):
    return raw * r_ref / 32768.0


def temperature_from_resistance(resistance, r_nominal=R_NOMINAL):
    # Callendar-Van Dusen for T >= 0: R = R0 (1 + A T + B T^2).
    disc = RTD_A * RTD_A - 4 * RTD_B * (1 - resistance / r_nominal)
    temp = (-RTD_A + math.sqrt(disc)) / (2 * RTD_B)
    if temp >= 0:
        return temp

    # Below 0 C the C coefficient matters; use the polynomial fit instead.
    r = resistance / r_nominal * 100.0
    return (-242.02 + 2.2228 * r + 2.5859e-3 * r ** 2 - 4.8260e-6 * r ** 3
            - 2.8183e-8 * r ** 4 + 1.5243e-10 * r ** 5)


def normalize_rtd_frame(raw_frame, device_id):
    raw = raw_frame["raw"]
    fault = raw_frame.get("fault_status", 0)
    if not fault and raw == 0:
        fault = FAULT_NO_SIGNAL

    fields = {"raw_code": raw, "fault": fault, "fault_name": fault_name(fault)}
    if not fault:
        resistance = resistance_from_raw(raw)
        fields["resistance_ohm"] = round(resistance, 3)
        fields["motor_temp_c"] = round(temperature_from_resistance(resistance), 2)

    return {
        "device_type": "rtd",
        "device_id":   device_id,
        "timestamp":   int(raw_frame.get("rx_timestamp") or time.time()),
        "fields":      fields,
    }
