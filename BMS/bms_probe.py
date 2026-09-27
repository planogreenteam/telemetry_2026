#!/usr/bin/env python3
"""Bench tool: talk to the EG4 LL-S over RS485 and dump its registers.

Use this BEFORE trusting the telemetry to confirm the wiring, the address
and the register map in BMS/bms_normalizer.py.

    # Which address answers? (tries 1..16)
    python -m BMS.bms_probe --scan

    # Raw register dump + decoded view for one address
    python -m BMS.bms_probe --address 1

--port defaults to the team's adapter (DEFAULT_BMS_PORT in
telemetry_sender.py); pass it for a different adapter or a COM port.

Compare the decoded SOC / pack voltage / cells with the battery's display
or the EG4 app. If a value is in the wrong register, edit REG in
BMS/bms_normalizer.py.
"""

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from BMS.bms_normalizer import (  # noqa: E402
    NUM_CELLS, REG, REGISTER_BLOCKS, UNIDENTIFIED_REGS, normalize_bms_frame,
)
from BMS.bms_reader import (  # noqa: E402
    FUNC_READ_HOLDING, FUNC_READ_INPUT, EG4ModbusReader, ModbusError,
)
from telemetry_sender import DEFAULT_BMS_PORT  # noqa: E402

# Read the wider dump in chunks: some BMS firmware rejects big requests.
DUMP_CHUNK = 32

_LABELS = {addr: name for name, addr in REG.items()}
for _i in range(NUM_CELLS):
    _LABELS[REG["cell_first"] + _i] = f"cell {_i + 1}"
for _addr in UNIDENTIFIED_REGS:
    _LABELS[_addr] = "? (not identified yet)"


def scan(reader, first, last):
    found = []
    for address in range(first, last + 1):
        reader.address = address
        try:
            reader.read_registers(0, 1)
            print(f"  address {address}: ANSWERED")
            found.append(address)
        except ModbusError as exc:
            print(f"  address {address}: {exc}")
    if not found:
        print("\nNo answer on any address. Check A/B wiring (try swapping), "
              "the port, the baud rate, and that the battery is on.")
    return found


def _read_chunk(reader, start, n, func):
    """Read a chunk; if the whole chunk is refused (e.g. it runs past the
    end of the battery's table), fall back to one register at a time and
    stop at the first one that fails. Returns (values, error)."""
    try:
        return reader.read_registers(start, n, func), None
    except ModbusError as exc:
        values = []
        for reg in range(start, start + n):
            try:
                values.extend(reader.read_registers(reg, 1, func))
            except ModbusError:
                break
        return values, exc


def dump(reader, count, func=FUNC_READ_HOLDING):
    table = "input" if func == FUNC_READ_INPUT else "holding"
    labels = _LABELS if func == FUNC_READ_HOLDING else {}
    print(f"\nRaw {table} registers 0..{count - 1} from address {reader.address}:")
    print(f"{'reg':>4}  {'hex':>6}  {'u16':>6}  {'s16':>6}  label")
    for start in range(0, count, DUMP_CHUNK):
        n = min(DUMP_CHUNK, count - start)
        regs, error = _read_chunk(reader, start, n, func)
        for offset, value in enumerate(regs):
            reg = start + offset
            s16 = value - 0x10000 if value & 0x8000 else value
            ascii_pair = bytes([value >> 8, value & 0xFF])
            text = ascii_pair.decode("ascii") if all(32 <= b < 127 for b in ascii_pair) else ""
            label = labels.get(reg, "")
            if text:
                label = f"{label}  '{text}'".strip()
            print(f"{reg:>4}  0x{value:04X}  {value:>6}  {s16:>6}  {label}")
        if error is not None and len(regs) < n:
            print(f"  regs {start + len(regs)}..{start + n - 1}: {error}")


def decoded(reader):
    regs = reader.read_blocks(REGISTER_BLOCKS)
    reading = normalize_bms_frame({"registers": regs}, device_id=0)
    print("\nDecoded with the current register map:")
    for key, value in reading["fields"].items():
        print(f"  {key:<20} {value}")


def main(argv=None):
    parser = argparse.ArgumentParser(description="EG4 LL-S RS485/Modbus probe")
    parser.add_argument("--port", default=DEFAULT_BMS_PORT,
                        help="USB-RS485 adapter serial device (default: the "
                             "team's adapter, same as telemetry_sender.py)")
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--address", type=int, default=1, help="Battery DIP address")
    parser.add_argument("--scan", action="store_true", help="Try addresses 1..16 and stop")
    # 129 = registers 0..128: the battery answers nothing above 128.
    parser.add_argument("--count", type=int, default=129, help="Registers to dump")
    parser.add_argument("--input-registers", action="store_true",
                        help="Dump the input-register table (Modbus 0x04) "
                             "instead of the holding registers")
    args = parser.parse_args(argv)

    reader = EG4ModbusReader(args.port, baud=args.baud, address=args.address)
    try:
        if args.scan:
            scan(reader, 1, 16)
            return
        if args.input_registers:
            dump(reader, args.count, FUNC_READ_INPUT)
            return
        dump(reader, args.count)
        try:
            decoded(reader)
        except ModbusError as exc:
            print(f"\nDecoded view unavailable: {exc}")
    finally:
        reader.close()


if __name__ == "__main__":
    main()
