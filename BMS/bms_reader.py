#!/usr/bin/env python3
"""EG4 LL-S BMS reader: Modbus RTU over RS485 (RJ45 port -> USB adapter).

The battery is a Modbus slave. Its slave address is the DIP-switch address
on the front panel (a single battery set to 1 answers to 1). Default line
settings are 9600 8N1. Every read_frame() sends one "read holding
registers" request (function 0x03) for the whole live-data block and
returns the raw register list; BMS/bms_normalizer.py turns it into named
fields.

Only pyserial is needed — RTU framing is small enough to do by hand, and
USB-RS485 adapters on the Pi switch TX/RX direction automatically.

If nothing ever answers, in order of likelihood:
  - A/B wires swapped between the RJ45 and the adapter (no damage, just
    silence — swap them),
  - wrong --bms-address (must match the DIP switches),
  - wrong port (use /dev/serial/by-id/..., not /dev/ttyUSB0, which can
    swap with the BMV and LoRa adapters between boots),
  - the battery's RS485 port is set to an inverter protocol instead of
    Modbus. Run `python -m BMS.bms_probe --scan` to check.
"""

import struct
import time

import serial

from BMS.bms_normalizer import REGISTER_BLOCKS


FUNC_READ_HOLDING = 0x03

# Largest request sent in one go. The battery has answered 32-register
# reads (bms_probe); bigger ones are split.
MAX_REGS_PER_REQUEST = 32

# How often (seconds) to print a diagnostic summary, same cadence as the
# BMV reader so both show up together in the log.
DIAGNOSTIC_INTERVAL = 5.0


class ModbusError(Exception):
    """A request got no usable answer (timeout, bad CRC, exception reply)."""


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def _with_crc(body: bytes) -> bytes:
    # Modbus RTU sends the CRC low byte first.
    return body + struct.pack("<H", crc16_modbus(body))


def build_read_request(address: int, start: int, count: int) -> bytes:
    return _with_crc(struct.pack(">BBHH", address, FUNC_READ_HOLDING, start, count))


def parse_read_response(frame: bytes, address: int, count: int) -> list:
    """Validate a complete 0x03 response frame and return its registers as
    unsigned 16-bit ints. Raises ModbusError on anything wrong."""
    if len(frame) < 5:
        raise ModbusError(f"response too short ({len(frame)} bytes)")
    body, crc_rx = frame[:-2], struct.unpack("<H", frame[-2:])[0]
    if crc16_modbus(body) != crc_rx:
        raise ModbusError("CRC mismatch")
    if body[0] != address:
        raise ModbusError(f"reply from address {body[0]}, expected {address}")
    if body[1] == FUNC_READ_HOLDING | 0x80:
        raise ModbusError(f"exception reply, code {body[2]}")
    if body[1] != FUNC_READ_HOLDING:
        raise ModbusError(f"unexpected function 0x{body[1]:02X}")
    if body[2] != count * 2 or len(body) != 3 + count * 2:
        raise ModbusError(f"byte count {body[2]}, expected {count * 2}")
    return list(struct.unpack(f">{count}H", body[3:]))


class EG4ModbusReader:
    def __init__(self, port, baud=9600, address=1, poll_interval=1.0,
                 timeout=0.5):
        self.address = address
        self.poll_interval = poll_interval
        self.serial = serial.Serial(port=port, baudrate=baud, bytesize=8,
                                    parity="N", stopbits=1, timeout=timeout)
        self._next_poll = time.monotonic()
        # Diagnostic-only counters.
        self._ok = 0
        self._failed = 0
        self._last_error = None
        self._last_report = time.monotonic()
        self._first_ok_logged = False

    def read_registers(self, start, count):
        """One Modbus 0x03 transaction. Returns a list of u16 registers."""
        request = build_read_request(self.address, start, count)
        # Drop any stale bytes (a late reply to a previous request, line
        # noise) so they can't be mistaken for the start of this reply.
        self.serial.reset_input_buffer()
        self.serial.write(request)
        self.serial.flush()

        head = self.serial.read(3)
        if len(head) < 3:
            raise ModbusError(f"no reply from address {self.address} (timeout)")
        if head[1] & 0x80:
            rest = self.serial.read(2)          # exception code already in head[2]
        else:
            rest = self.serial.read(head[2] + 2)  # data + CRC
        return parse_read_response(head + rest, self.address, count)

    def read_blocks(self, blocks):
        """Read every (start, count) range, split into requests of at most
        MAX_REGS_PER_REQUEST. Returns {register_address: u16}."""
        registers = {}
        for start, count in blocks:
            for chunk_start in range(start, start + count, MAX_REGS_PER_REQUEST):
                n = min(MAX_REGS_PER_REQUEST, start + count - chunk_start)
                values = self.read_registers(chunk_start, n)
                registers.update(zip(range(chunk_start, chunk_start + n), values))
        return registers

    def read_frame(self):
        """Poll the live-data registers once per poll_interval. Returns
        {"registers": {address: value}, "rx_timestamp": float} or None if
        this poll failed (the caller just tries again next time)."""
        delay = self._next_poll - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        self._next_poll = max(self._next_poll + self.poll_interval,
                              time.monotonic())

        frame = None
        try:
            registers = self.read_blocks(REGISTER_BLOCKS)
            frame = {"registers": registers, "rx_timestamp": time.time()}
            self._ok += 1
            if not self._first_ok_logged:
                print(f"[bms-reader] First Modbus reply from address "
                      f"{self.address}: {len(registers)} registers", flush=True)
                self._first_ok_logged = True
        except ModbusError as exc:
            self._failed += 1
            self._last_error = str(exc)

        now = time.monotonic()
        if now - self._last_report >= DIAGNOSTIC_INTERVAL:
            detail = f", last error: {self._last_error}" if self._failed else ""
            print(f"[bms-reader] alive: {self._ok} ok, {self._failed} failed "
                  f"polls in last {now - self._last_report:.0f}s{detail}",
                  flush=True)
            self._ok = 0
            self._failed = 0
            self._last_report = now
        return frame

    def close(self):
        if self.serial.is_open:
            self.serial.close()
