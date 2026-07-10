#!/usr/bin/env python3
import time

import serial


# VE.Direct protocol sends data in two alternating blocks, each ending in its
# own "Checksum" line:
#   Block A: V, I, P, CE, SOC, TTG, Alarm, Relay, AR, BMV, FW
#   Block B: H1..H17, MON
# We accumulate fields across consecutive blocks and only return once we have
# the fields the caller actually needs (the live measurements from Block A).

REQUIRED_KEYS = ("V", "I", "P")

# How often (seconds) to print a diagnostic line while read_frame() is
# blocked without having produced a usable frame yet. read_frame() is a
# blocking loop with no return until it sees a complete V/I/P block, so
# without this, a caller has zero visibility into whether the serial port
# is silent, producing garbage, or producing valid-but-incomplete blocks.
DIAGNOSTIC_INTERVAL = 5.0


class BMVReader:
    def __init__(self, serial_port, baudrate):
        self.serial = serial.Serial(port=serial_port, baudrate=baudrate, timeout=1)
        self.frame = {}
        # Fields and running byte-sum for the block currently being read.
        # Blocks are only merged into self.frame once their VE.Direct
        # checksum verifies: the sum of every byte in the block (including
        # the "Checksum" line and its checksum byte) must be 0 mod 256.
        # Summing whole readline() outputs is equivalent to summing the
        # frame as specified: the spec frames each field as
        # "\r\nKEY<tab>VALUE" (leading \r\n), while readline() yields
        # "KEY<tab>VALUE\r\n" (trailing \r\n) -- same bytes, same total.
        # Without this check, EMI from the motor controller during
        # acceleration can garble a digit of "I" and we'd record (and
        # transmit) a bogus current as if it were real.
        self._block_fields = {}
        self._block_sum = 0
        # Diagnostic-only state -- doesn't affect parsing behavior.
        self._lines_seen = 0
        self._blocks_seen = 0
        self._checksum_failures = 0
        self._last_report = time.monotonic()
        self._first_line_logged = False

    def read_frame(self, required_keys=REQUIRED_KEYS):
        while True:
            raw = self.serial.readline()
            line = raw.decode(errors="ignore").strip()
            self._block_sum = (self._block_sum + sum(raw)) & 0xFF

            now = time.monotonic()
            if now - self._last_report >= DIAGNOSTIC_INTERVAL:
                print(
                    f"[bmv-reader] alive: {self._lines_seen} lines, "
                    f"{self._blocks_seen} complete blocks, "
                    f"{self._checksum_failures} checksum failures in last "
                    f"{DIAGNOSTIC_INTERVAL:.0f}s, port_open={self.serial.is_open}, "
                    f"pending_fields={sorted(self.frame.keys())}",
                    flush=True,
                )
                self._lines_seen = 0
                self._blocks_seen = 0
                self._checksum_failures = 0
                self._last_report = now

            if not line:
                # readline() timed out (1s) with nothing on the wire, or the
                # line was pure whitespace. Either way, no data right now.
                continue

            self._lines_seen += 1
            if not self._first_line_logged:
                print(f"[bmv-reader] First raw line received: {line!r} "
                      f"(raw bytes: {raw!r})", flush=True)
                self._first_line_logged = True

            if "\t" in line:
                key, value = line.split("\t", 1)
                self._block_fields[key] = value
            else:
                # A non-empty line without a tab isn't valid VE.Direct
                # key\tvalue framing -- likely garbled data (wrong baud
                # rate, noise, or a non-VE.Direct device on this port).
                print(f"[bmv-reader] Non-tab-delimited line (possible baud "
                      f"mismatch or noise): {line!r}", flush=True)

            if line.startswith("Checksum"):
                self._blocks_seen += 1
                block_ok = self._block_sum == 0
                block_fields = self._block_fields
                self._block_fields = {}
                self._block_sum = 0

                if not block_ok:
                    self._checksum_failures += 1
                    print(f"[bmv-reader] Checksum FAILED for block with keys "
                          f"{sorted(block_fields.keys())}; discarding block. "
                          f"(Noise/EMI on the VE.Direct line?)", flush=True)
                    continue

                self.frame.update(block_fields)
                # End of a valid block. Only return if the accumulated frame
                # actually contains the live-measurement keys we need.
                # Otherwise keep accumulating into the next block.
                if all(k in self.frame for k in required_keys):
                    frame = dict(self.frame)
                    self.frame = {}
                    return frame
                else:
                    missing = [k for k in required_keys if k not in self.frame]
                    print(f"[bmv-reader] Block complete but missing required "
                          f"keys {missing}; have {sorted(self.frame.keys())}. "
                          f"Waiting for next block.", flush=True)
                # else: keep going, the next block will fill in the missing fields

    def close(self):
        if self.serial.is_open:
            self.serial.close()
