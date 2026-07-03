#!/usr/bin/env python3
import time


def normalize_bmv_frame(frame, device_id):
    def read_int(*keys, default=None):
        for key in keys:
            value = frame.get(key)
            if value in (None, ""):
                continue
            try:
                return int(value)
            except ValueError:
                continue
        return default

    # VE.Direct reports state of charge under the key "SOC", in per-mille
    # (tenths of a percent) - e.g. "953" means 95.3%. Convert to a whole
    # percent here since the wire packet stores charge_state as a single
    # unsigned byte (0-100).
    raw_soc = read_int("SOC")
    charge_state = raw_soc / 10 if raw_soc is not None else None

    fields = {
        "voltage_mv": read_int("V", default=0),
        "current_ma": read_int("I", default=0),
        "power_w": read_int("P", default=0),
        "charge_state": charge_state,
        "alarm": read_int("Alarm", "AR", default=0),
    }

    return {
        "device_type": "bmv",
        "device_id": device_id,
        "timestamp": int(time.time()),
        "fields": {key: value for key, value in fields.items() if value is not None},
    }
