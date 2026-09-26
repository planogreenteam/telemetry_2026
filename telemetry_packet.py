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
from typing import NamedTuple


# v2: BMV current_ma widened i16 -> i32 (i16 caps at +/-32.767 A and a
# hard acceleration exceeds that, making struct.pack raise and every BMV
# packet drop for as long as the pedal is down), and BMV gained
# PEAK_CURRENT_MA.
# v3: BMS moved from CAN (Pylontech frames) to RS485 Modbus with a new
# single-snapshot layout, and wire scales became per-layout (a shared
# by-name scale table gave BMS battery_current_a the MPPT x2000 scale,
# clamping it at +/-16.4 A). Sender and receiver must be updated together.
PROTOCOL_VERSION = 3
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


class WireField(NamedTuple):
    """One field in a layout.

    Pack:   encoded = round(value * scale)
    Decode: value   = encoded / (scale * display_div)

    display_div only exists for BMV, whose fields are named *_mv / *_ma
    and packed as raw integers but have always been decoded to V / A for
    Influx and Grafana."""
    bit: int
    name: str
    fmt: str
    scale: int = 1
    display_div: int = 1


# ─────────────────────────────────────────────────────────────────────────────
# BMV field layout
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
    WireField(BMVField.VOLTAGE_MV,      "voltage_mv",      ">H", display_div=1000),
    WireField(BMVField.CURRENT_MA,      "current_ma",      ">i", display_div=1000),
    WireField(BMVField.POWER_W,         "power_w",         ">h"),
    WireField(BMVField.CHARGE_STATE,    "charge_state",    ">B"),
    WireField(BMVField.ALARM,           "alarm",           ">B"),
    WireField(BMVField.ELAPSED_S,       "elapsed_s",       ">I"),
    WireField(BMVField.PEAK_CURRENT_MA, "peak_current_ma", ">i", display_div=1000),
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
    WireField(MPPTField.PV_VOLTAGE_V,      "pv_voltage_v",      ">H", 100),
    WireField(MPPTField.PV_CURRENT_A,      "pv_current_a",      ">h", 2000),
    WireField(MPPTField.PV_POWER_W,        "pv_power_w",        ">h", 100),
    WireField(MPPTField.BATTERY_VOLTAGE_V, "battery_voltage_v", ">H", 100),
    WireField(MPPTField.BATTERY_CURRENT_A, "battery_current_a", ">h", 2000),
    WireField(MPPTField.MODE,              "mode",              ">B"),
    WireField(MPPTField.FAULT,             "fault",             ">B"),
    WireField(MPPTField.ENABLED,           "enabled",           ">B"),
    WireField(MPPTField.AMBIENT_TEMP_C,    "ambient_temp_c",    ">b"),
    WireField(MPPTField.HEATSINK_TEMP_C,   "heatsink_temp_c",   ">b"),
    WireField(MPPTField.MPPT_INDEX,        "mppt_index",        ">B"),
    WireField(MPPTField.PACKET_ID,         "packet_id",         ">B"),
    WireField(MPPTField.ELAPSED_S,         "elapsed_s",         ">I"),
)


# ─────────────────────────────────────────────────────────────────────────────
# BMS field layout — EG4 LL-S over RS485 Modbus (see BMS/bms_normalizer.py)
#
# One Modbus poll is a complete snapshot, so every packet carries all 16
# fields (~44 bytes). SOC rides on every packet. Individual cell voltages
# stay on the on-car CSV; the radio carries max/min/sum.
# ─────────────────────────────────────────────────────────────────────────────

class BMSField(IntEnum):
    BATTERY_VOLTAGE_V = 1 << 0
    BATTERY_CURRENT_A = 1 << 1
    SOC_PCT           = 1 << 2
    SOH_PCT           = 1 << 3
    CELL_V_MAX_MV     = 1 << 4
    CELL_V_MIN_MV     = 1 << 5
    CELL_MAX_IDX      = 1 << 6
    CELL_MIN_IDX      = 1 << 7
    CELL_SUM_V        = 1 << 8
    TEMP_MAX_C        = 1 << 9
    TEMP_AVG_C        = 1 << 10
    REMAINING_AH      = 1 << 11
    WARNING_FLAGS     = 1 << 12
    PROTECTION_FLAGS  = 1 << 13
    ERROR_CODE        = 1 << 14
    ELAPSED_S         = 1 << 15


BMS_FIELD_LAYOUT = (
    WireField(BMSField.BATTERY_VOLTAGE_V, "battery_voltage_v", ">H", 100),
    WireField(BMSField.BATTERY_CURRENT_A, "battery_current_a", ">h", 100),
    WireField(BMSField.SOC_PCT,           "soc_pct",           ">B"),
    WireField(BMSField.SOH_PCT,           "soh_pct",           ">B"),
    WireField(BMSField.CELL_V_MAX_MV,     "cell_v_max_mv",     ">H"),
    WireField(BMSField.CELL_V_MIN_MV,     "cell_v_min_mv",     ">H"),
    WireField(BMSField.CELL_MAX_IDX,      "cell_max_idx",      ">B"),
    WireField(BMSField.CELL_MIN_IDX,      "cell_min_idx",      ">B"),
    WireField(BMSField.CELL_SUM_V,        "cell_sum_v",        ">H", 100),
    WireField(BMSField.TEMP_MAX_C,        "temp_max_c",        ">b"),
    WireField(BMSField.TEMP_AVG_C,        "temp_avg_c",        ">b"),
    WireField(BMSField.REMAINING_AH,      "remaining_ah",      ">H", 10),
    WireField(BMSField.WARNING_FLAGS,     "warning_flags",     ">H"),
    WireField(BMSField.PROTECTION_FLAGS,  "protection_flags",  ">H"),
    WireField(BMSField.ERROR_CODE,        "error_code",        ">H"),
    WireField(BMSField.ELAPSED_S,         "elapsed_s",         ">I"),
)


_LAYOUTS = {
    MsgType.BMV:  BMV_FIELD_LAYOUT,
    MsgType.MPPT: MPPT_FIELD_LAYOUT,
    MsgType.BMS:  BMS_FIELD_LAYOUT,
}


def layout_field_names(msg_type):
    """Field names carried on the wire for msg_type, in layout order."""
    return tuple(f.name for f in _LAYOUTS[MsgType(msg_type)])


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

    for extra in ("mppt_index", "packet_id"):
        if extra in normalized and extra not in fields:
            fields[extra] = normalized[extra]

    # MPPT identity contract: every MPPT reading must carry an mppt_index so
    # the receiver can separate the boards. can_normalizer.py is the
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

    for field in layout:
        value = fields.get(field.name)
        if value is None or isinstance(value, str):
            continue
        encoded = int(round(float(value) * field.scale))
        try:
            payload.extend(struct.pack(field.fmt, encoded))
        except struct.error:
            # Clamp instead of raising: raising here aborted the WHOLE
            # packet, so one out-of-range field silenced the stream for as
            # long as the value stayed out of range (this is exactly how
            # i16 current_ma made the BMV go dark during acceleration).
            # A clamped reading ("at least 32.767 A") beats no reading.
            lo, hi = _format_range(field.fmt)
            clamped = min(max(encoded, lo), hi)
            print(
                f"[packet] {field.name}={value} (encoded={encoded}) out of "
                f"range for {field.fmt}; clamping to {clamped}",
                flush=True,
            )
            payload.extend(struct.pack(field.fmt, clamped))
        field_mask |= int(field.bit)

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


# ─────────────────────────────────────────────────────────────────────────────
# BATCH CONTAINER
#
# One LoRa frame can carry several ordinary packets. Each AT+SEND costs the
# same command-handling overhead regardless of payload size, so sending a
# 6-MPPT + BMS refresh as one frame instead of seven cuts the modem time for
# a full-car update by ~7x. Format:
#
#     u8 BATCH_MAGIC | u8 count | ( u8 len | packet bytes ) * count
#
# Sub-packets are unmodified packets (header+payload+CRC), so building and
# decoding reuse the existing single-packet code end to end.
# ─────────────────────────────────────────────────────────────────────────────

BATCH_MAGIC = 0xB5  # deliberately far from any PROTOCOL_VERSION value

# Keep whole frames comfortably inside the modem's AT command buffer and a
# single LoRa payload. Hex encoding doubles this on the serial link.
BATCH_MAX_BYTES = 180


def build_batch(packets):
    if not packets:
        raise ValueError("build_batch needs at least one packet")
    if len(packets) > 255:
        raise ValueError("Too many packets for one batch")
    out = bytearray((BATCH_MAGIC, len(packets)))
    for pkt in packets:
        if not 1 <= len(pkt) <= 255:
            raise ValueError(f"Batch element size {len(pkt)} out of range")
        out.append(len(pkt))
        out.extend(pkt)
    if len(out) > BATCH_MAX_BYTES:
        raise ValueError(f"Batch frame {len(out)} bytes exceeds {BATCH_MAX_BYTES}")
    return bytes(out)


def is_batch(data: bytes) -> bool:
    return len(data) >= 2 and data[0] == BATCH_MAGIC


def split_batch(data: bytes):
    """Return the list of sub-packet byte strings inside a batch frame.
    Each still carries its own CRC and goes through decode_packet as usual."""
    if not is_batch(data):
        raise ValueError("Not a batch frame")
    count = data[1]
    packets = []
    offset = 2
    for _ in range(count):
        if offset >= len(data):
            raise ValueError("Batch truncated: missing length byte")
        length = data[offset]
        offset += 1
        if offset + length > len(data):
            raise ValueError("Batch truncated: element shorter than declared")
        packets.append(data[offset:offset + length])
        offset += length
    if offset != len(data):
        raise ValueError("Batch has unexpected trailing bytes")
    return packets


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

    fields = {}
    offset = 0

    for field in layout:
        if not (field_mask & int(field.bit)):
            continue
        size = struct.calcsize(field.fmt)
        if offset + size > len(payload):
            raise ValueError(
                f"Payload too short for {field.name} "
                f"(need {size}, have {len(payload) - offset})"
            )
        encoded = struct.unpack(field.fmt, payload[offset:offset + size])[0]
        offset += size
        divisor = field.scale * field.display_div
        fields[field.name] = encoded / divisor if divisor != 1 else encoded

    if offset != len(payload):
        raise ValueError("Payload has unexpected trailing bytes")
    return fields
