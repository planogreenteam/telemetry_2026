#!/usr/bin/env python3
"""Binary wire format for the telemetry pipeline.

Header (14 bytes, big-endian):
    u8   protocol version
    u8   msg_type (1=BMV, 2=MPPT, 3=BMS)
    u8   event_type
    u8   device_id
    u16  seq
    u32  timestamp (unix seconds)
    u16  field_mask

Payload: struct-packed values for fields present in field_mask.
CRC: 2 bytes CRC-16 over header+payload.
"""

import binascii
import struct
from enum import IntEnum


# v2: BMV current_ma widened i16 -> i32 (i16 caps at +/-32.767 A and a
# hard acceleration exceeds that, making struct.pack raise and every BMV
# packet drop for as long as the pedal is down), and BMV gained
# PEAK_CURRENT_MA. Sender and receiver must be updated together.
PROTOCOL_VERSION = 2
CRC_FORMAT = ">H"
HEADER_FORMAT = ">BBBBHIH"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
CRC_SIZE = struct.calcsize(CRC_FORMAT)


class MsgType(IntEnum):
    BMV  = 1
    MPPT = 2
    BMS  = 3


class EventType(IntEnum):
    SAMPLE             = 1
    DELTA_UPDATE       = 2
    THRESHOLD_CROSSING = 3
    ALARM              = 4
    HEARTBEAT          = 5
    DEVICE_STATUS      = 6


# ─────────────────────────────────────────────────────────────────────────────
# BMV field layout (unchanged)
# ─────────────────────────────────────────────────────────────────────────────

class BMVField(IntEnum):
    VOLTAGE_MV      = 1 << 0
    CURRENT_MA      = 1 << 1
    POWER_W         = 1 << 2
    CHARGE_STATE    = 1 << 3
    ALARM           = 1 << 4
    ELAPSED_S       = 1 << 5
    PEAK_CURRENT_MA = 1 << 6


# current_ma / peak_current_ma are i32: the BMV reports mA, and i16 tops out
# at 32.767 A — real discharge peaks exceed that.
BMV_FIELD_LAYOUT = (
    (BMVField.VOLTAGE_MV,      "voltage_mv",      ">H"),
    (BMVField.CURRENT_MA,      "current_ma",      ">i"),
    (BMVField.POWER_W,         "power_w",         ">h"),
    (BMVField.CHARGE_STATE,    "charge_state",    ">B"),
    (BMVField.ALARM,           "alarm",           ">B"),
    (BMVField.ELAPSED_S,       "elapsed_s",       ">I"),
    (BMVField.PEAK_CURRENT_MA, "peak_current_ma", ">i"),
)


# ─────────────────────────────────────────────────────────────────────────────
# MPPT field layout — authoritative per OpenSEC Manual V1.9
# Covers both Packet ID 0 (power) and Packet ID 1 (status)
# ─────────────────────────────────────────────────────────────────────────────

class MPPTField(IntEnum):
    PV_VOLTAGE_V      = 1 << 0
    PV_CURRENT_A      = 1 << 1
    PV_POWER_W        = 1 << 2
    BATTERY_VOLTAGE_V = 1 << 3
    BATTERY_CURRENT_A = 1 << 4
    MODE              = 1 << 5
    FAULT             = 1 << 6
    ENABLED           = 1 << 7
    AMBIENT_TEMP_C    = 1 << 8
    HEATSINK_TEMP_C   = 1 << 9
    MPPT_INDEX        = 1 << 10
    PACKET_ID         = 1 << 11
    ELAPSED_S         = 1 << 12


MPPT_FIELD_LAYOUT = (
    (MPPTField.PV_VOLTAGE_V,      "pv_voltage_v",      ">H"),
    (MPPTField.PV_CURRENT_A,      "pv_current_a",      ">h"),
    (MPPTField.PV_POWER_W,        "pv_power_w",        ">h"),
    (MPPTField.BATTERY_VOLTAGE_V, "battery_voltage_v", ">H"),
    (MPPTField.BATTERY_CURRENT_A, "battery_current_a", ">h"),
    (MPPTField.MODE,              "mode",              ">B"),
    (MPPTField.FAULT,             "fault",             ">B"),
    (MPPTField.ENABLED,           "enabled",           ">B"),
    (MPPTField.AMBIENT_TEMP_C,    "ambient_temp_c",    ">b"),
    (MPPTField.HEATSINK_TEMP_C,   "heatsink_temp_c",   ">b"),
    (MPPTField.MPPT_INDEX,        "mppt_index",        ">B"),
    (MPPTField.PACKET_ID,         "packet_id",         ">B"),
    (MPPTField.ELAPSED_S,         "elapsed_s",         ">I"),
)


# ─────────────────────────────────────────────────────────────────────────────
# BMS field layout — EG4 LL-S Pylontech-compatible
# ─────────────────────────────────────────────────────────────────────────────

class BMSField(IntEnum):
    CHARGE_VOLTAGE_V    = 1 << 0
    CHARGE_CURRENT_A    = 1 << 1
    DISCHARGE_CURRENT_A = 1 << 2
    DISCHARGE_VOLTAGE_V = 1 << 3
    SOC_PCT             = 1 << 4
    SOH_PCT             = 1 << 5
    BATTERY_VOLTAGE_V   = 1 << 6
    BATTERY_CURRENT_A   = 1 << 7
    BATTERY_TEMP_C      = 1 << 8
    PROTECTION_FLAGS    = 1 << 9
    ALARM_FLAGS         = 1 << 10
    MODULE_COUNT        = 1 << 11
    CHARGE_ENABLE       = 1 << 12
    DISCHARGE_ENABLE    = 1 << 13
    ELAPSED_S           = 1 << 14


BMS_FIELD_LAYOUT = (
    (BMSField.CHARGE_VOLTAGE_V,    "charge_voltage_v",    ">H"),
    (BMSField.CHARGE_CURRENT_A,    "charge_current_a",    ">h"),
    (BMSField.DISCHARGE_CURRENT_A, "discharge_current_a", ">h"),
    (BMSField.DISCHARGE_VOLTAGE_V, "discharge_voltage_v", ">H"),
    (BMSField.SOC_PCT,             "soc_pct",             ">B"),
    (BMSField.SOH_PCT,             "soh_pct",             ">B"),
    (BMSField.BATTERY_VOLTAGE_V,   "battery_voltage_v",   ">H"),
    (BMSField.BATTERY_CURRENT_A,   "battery_current_a",   ">h"),
    (BMSField.BATTERY_TEMP_C,      "battery_temp_c",      ">h"),
    (BMSField.PROTECTION_FLAGS,    "protection_flags",    ">H"),
    (BMSField.ALARM_FLAGS,         "alarm_flags",         ">H"),
    (BMSField.MODULE_COUNT,        "module_count",        ">B"),
    (BMSField.CHARGE_ENABLE,       "charge_enable",       ">B"),
    (BMSField.DISCHARGE_ENABLE,    "discharge_enable",    ">B"),
    (BMSField.ELAPSED_S,           "elapsed_s",           ">I"),
)


_PACK_SCALES = {
    "pv_voltage_v":         100,
    "pv_current_a":         2000,
    "pv_power_w":           100,
    "battery_voltage_v":    100,
    "battery_current_a":    2000,
    "mode":                 1,
    "fault":                1,
    "enabled":              1,
    "ambient_temp_c":       1,
    "heatsink_temp_c":      1,
    "mppt_index":           1,
    "packet_id":            1,
    "charge_voltage_v":     10,
    "charge_current_a":     10,
    "discharge_current_a":  10,
    "discharge_voltage_v":  10,
    "soc_pct":              1,
    "soh_pct":              1,
    "battery_temp_c":       10,
    "protection_flags":     1,
    "alarm_flags":          1,
    "module_count":         1,
    "charge_enable":        1,
    "discharge_enable":     1,
    "elapsed_s":            1,
}

_LAYOUTS = {
    MsgType.BMV:  BMV_FIELD_LAYOUT,
    MsgType.MPPT: MPPT_FIELD_LAYOUT,
    MsgType.BMS:  BMS_FIELD_LAYOUT,
}

# _RAW_INT_MSG_TYPES controls the PACK path only — BMV fields are stored as
# raw integers on the wire (VE.Direct delivers mV/mA integers directly).
# Decode scaling is handled separately via _DECODE_SCALES.
_RAW_INT_MSG_TYPES = frozenset({MsgType.BMV})

# Applied during decode only. Converts raw wire integers to engineering units
# for BMV fields before they reach the receiver / Influx writer.
_DECODE_SCALES = {
    "voltage_mv":      1000,   # mV -> V
    "current_ma":      1000,   # mA -> A
    "peak_current_ma": 1000,   # mA -> A
    "power_w":         1,
    "charge_state":    1,
    "alarm":           1,
}


def _scale_for(field_name):
    return _PACK_SCALES.get(field_name, 1)


def crc16(data: bytes) -> int:
    return binascii.crc_hqx(data, 0xFFFF)


def _format_range(fmt):
    """(min, max) representable by a struct integer format like '>h'."""
    bits = struct.calcsize(fmt) * 8
    if fmt[-1].islower():  # signed
        return -(1 << (bits - 1)), (1 << (bits - 1)) - 1
    return 0, (1 << bits) - 1


def _pack_header(msg_type, event_type, device_id, seq, timestamp, field_mask):
    return struct.pack(
        HEADER_FORMAT,
        PROTOCOL_VERSION,
        int(msg_type),
        int(event_type),
        int(device_id) & 0xFF,
        int(seq) & 0xFFFF,
        int(timestamp) & 0xFFFFFFFF,
        int(field_mask) & 0xFFFF,
    )


def _wrap_with_crc(data: bytes) -> bytes:
    return data + struct.pack(CRC_FORMAT, crc16(data))


def _build_typed_packet(normalized, event_type, seq, msg_type, layout,
                        device_type_expected):
    if normalized.get("device_type") != device_type_expected:
        raise ValueError(
            f"device_type={normalized.get('device_type')!r}, "
            f"expected {device_type_expected!r}"
        )

    fields = dict(normalized.get("fields", {}))
    use_raw = msg_type in _RAW_INT_MSG_TYPES

    for extra in ("mppt_index", "packet_id"):
        if extra in normalized and extra not in fields:
            fields[extra] = normalized[extra]

    # MPPT identity contract: every MPPT reading must carry an mppt_index so
    # the receiver can separate the five boards. can_normalizer.py is the
    # authoritative source — it sets both mppt_index and the per-board
    # device_id (base + index) from the frame's position in MPPT_EFFECTIVE_IDS.
    # This is only a fail-fast guard against a normalizer regression that would
    # otherwise silently collapse all boards onto one InfluxDB series.
    if msg_type == MsgType.MPPT:
        if fields.get("mppt_index", normalized.get("mppt_index")) is None:
            raise ValueError(
                "MPPT reading has no mppt_index; can_normalizer must set it "
                "from the frame's position in MPPT_EFFECTIVE_IDS so the "
                "receiver can distinguish the boards"
            )

    field_mask = 0
    payload = bytearray()

    for bit, field_name, fmt in layout:
        if field_name not in fields or fields[field_name] is None:
            continue
        if isinstance(fields[field_name], str):
            continue
        value = fields[field_name]
        if use_raw:
            encoded = int(value)
        else:
            encoded = int(round(float(value) * _scale_for(field_name)))
        try:
            payload.extend(struct.pack(fmt, encoded))
        except struct.error:
            # Clamp instead of raising: raising here aborted the WHOLE
            # packet, so one out-of-range field silenced the stream for as
            # long as the value stayed out of range (this is exactly how
            # i16 current_ma made the BMV go dark during acceleration).
            # A clamped reading ("at least 32.767 A") beats no reading.
            lo, hi = _format_range(fmt)
            clamped = min(max(encoded, lo), hi)
            print(
                f"[packet] {field_name}={value} (encoded={encoded}) out of "
                f"range for {fmt}; clamping to {clamped}",
                flush=True,
            )
            payload.extend(struct.pack(fmt, clamped))
        field_mask |= int(bit)

    header = _pack_header(
        msg_type, event_type, normalized["device_id"],
        seq, normalized["timestamp"], field_mask,
    )
    return _wrap_with_crc(header + bytes(payload))


def build_bmv_packet(normalized, event_type, seq):
    return _build_typed_packet(
        normalized, event_type, seq, MsgType.BMV, BMV_FIELD_LAYOUT, "bmv"
    )


def build_mppt_packet(normalized, event_type, seq):
    return _build_typed_packet(
        normalized, event_type, seq, MsgType.MPPT, MPPT_FIELD_LAYOUT, "mppt"
    )


def build_bms_packet(normalized, event_type, seq):
    return _build_typed_packet(
        normalized, event_type, seq, MsgType.BMS, BMS_FIELD_LAYOUT, "bms"
    )


def decode_packet(packet: bytes) -> dict:
    if len(packet) < HEADER_SIZE + CRC_SIZE:
        raise ValueError("Packet too short")

    payload_end = len(packet) - CRC_SIZE
    packet_wo_crc = packet[:payload_end]
    expected_crc = struct.unpack(CRC_FORMAT, packet[payload_end:])[0]
    actual_crc = crc16(packet_wo_crc)
    if actual_crc != expected_crc:
        raise ValueError(
            f"CRC mismatch: expected {expected_crc:#06x}, got {actual_crc:#06x}"
        )

    version, msg_type, event_type, device_id, seq, timestamp, field_mask = struct.unpack(
        HEADER_FORMAT, packet[:HEADER_SIZE]
    )
    if version != PROTOCOL_VERSION:
        raise ValueError(f"Unsupported protocol version {version}")

    payload = packet[HEADER_SIZE:payload_end]
    fields = decode_payload(msg_type, field_mask, payload)

    return {
        "version":    version,
        "msg_type":   MsgType(msg_type),
        "event_type": EventType(event_type),
        "device_id":  device_id,
        "seq":        seq,
        "timestamp":  timestamp,
        "field_mask": field_mask,
        "fields":     fields,
    }


def decode_payload(msg_type, field_mask, payload):
    try:
        mt = MsgType(msg_type)
    except ValueError as exc:
        raise ValueError(f"Unsupported msg_type {msg_type}") from exc

    layout = _LAYOUTS.get(mt)
    if layout is None:
        raise ValueError(f"No layout for msg_type {mt.name}")

    use_raw = mt in _RAW_INT_MSG_TYPES
    fields = {}
    offset = 0

    for bit, field_name, fmt in layout:
        if not (field_mask & int(bit)):
            continue
        size = struct.calcsize(fmt)
        if offset + size > len(payload):
            raise ValueError(
                f"Payload too short for {field_name} "
                f"(need {size}, have {len(payload) - offset})"
            )
        encoded = struct.unpack(fmt, payload[offset:offset + size])[0]
        offset += size
        if use_raw:
            decode_scale = _DECODE_SCALES.get(field_name, 1)
            fields[field_name] = encoded / decode_scale if decode_scale != 1 else encoded
        else:
            scale = _scale_for(field_name)
            fields[field_name] = encoded / scale if scale != 1 else encoded

    if offset != len(payload):
        raise ValueError("Payload has unexpected trailing bytes")
    return fields
