#!/usr/bin/env python3
"""MAX31865 RTD reader over SPI (Adafruit #3328 amplifier + PT100 probe #3290).

The probe measures the motor temperature.

The MAX31865 measures the PT100's resistance as a 15-bit ratio against the
board's 430 ohm reference resistor. Every read_frame() runs one one-shot
conversion and returns the raw code plus the fault status;
RTD/rtd_normalizer.py turns that into ohms and degrees C.

One-shot (bias on only while converting) is used instead of auto-convert
mode so the bias current doesn't self-heat the probe between polls, the
same approach Adafruit's driver takes.

Only the `spidev` package is needed: the register protocol is a handful of
byte transfers. The MCP2515 CAN hat already owns SPI0 CE0, so the MAX31865
sits on CE1 (/dev/spidev0.1, GPIO7 / pin 26) by default.

If the board never answers (RuntimeError at startup), in order of
likelihood:
  - SPI not enabled (`dtparam=spi=on` in /boot/firmware/config.txt),
  - CS wired to the wrong pin for --rtd-spi-device (CE1 = pin 26),
  - SDI/SDO swapped (SDI is the board's input: Pi MOSI -> SDI, SDO -> MISO),
  - no 3.3 V / GND to the board.
"""

import time


# Register addresses. Reads use the address as-is; writes set bit 7.
REG_CONFIG = 0x00
REG_RTD_MSB = 0x01          # RTD MSB, LSB follows (bit 0 of LSB = fault)
REG_HIGH_FAULT_MSB = 0x03   # high threshold MSB/LSB, low threshold MSB/LSB
REG_FAULT_STATUS = 0x07
WRITE = 0x80

# Configuration register bits.
CFG_VBIAS = 0x80
CFG_ONE_SHOT = 0x20         # self-clears when the conversion finishes
CFG_3WIRE = 0x10
CFG_FAULT_CLEAR = 0x02      # self-clears
CFG_FILTER_50HZ = 0x01      # 0 = 60 Hz notch

# Bias settle time before a conversion, and one-shot conversion time
# (datasheet: 52 ms with the 60 Hz filter, 62.5 ms with 50 Hz), with margin.
BIAS_SETTLE_S = 0.010
CONVERSION_S = {60: 0.065, 50: 0.075}

# Written to the high-fault threshold at startup and read back, to prove a
# MAX31865 is actually on the bus. spidev opens fine with nothing attached,
# and a floating MISO reads 0x00 or 0xFF, so neither of those can be the
# test pattern. (Bit 0 of the LSB is unused, so it is left 0.)
_PROBE_PATTERN = [0xA5, 0x5A]

# Fault thresholds used in operation: the full range, so only the hardware
# open/short detections (and not an arbitrary temperature window) flag.
_THRESHOLDS = [0xFF, 0xFF, 0x00, 0x00]

DIAGNOSTIC_INTERVAL = 5.0


class MAX31865Reader:
    def __init__(self, bus=0, device=1, wires=3, filter_hz=60,
                 poll_interval=1.0, speed_hz=500_000):
        if wires not in (2, 3, 4):
            raise ValueError(f"wires must be 2, 3 or 4, not {wires}")
        if filter_hz not in CONVERSION_S:
            raise ValueError(f"filter_hz must be 50 or 60, not {filter_hz}")
        try:
            import spidev
        except ImportError as exc:
            raise RuntimeError(
                "The spidev package is required for the MAX31865 "
                "(python3 -m pip install spidev; Raspberry Pi / Linux only)"
            ) from exc

        self.bus = bus
        self.device = device
        self.poll_interval = poll_interval
        self._config = ((CFG_3WIRE if wires == 3 else 0)
                        | (CFG_FILTER_50HZ if filter_hz == 50 else 0))
        self._conversion_s = CONVERSION_S[filter_hz]

        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        try:
            self.spi.max_speed_hz = speed_hz
            self.spi.mode = 0b01      # MAX31865 supports SPI modes 1 and 3
            self._check_present()
            self._write(REG_HIGH_FAULT_MSB, _THRESHOLDS)
            self._write(REG_CONFIG, [self._config | CFG_FAULT_CLEAR])
        except Exception:
            self.spi.close()
            raise

        self._next_poll = time.monotonic()
        # Diagnostic-only counters.
        self._ok = 0
        self._failed = 0
        self._faults = 0
        self._last_error = None
        self._last_report = time.monotonic()
        self._first_ok_logged = False

    def _read(self, reg, count):
        return self.spi.xfer2([reg] + [0x00] * count)[1:]

    def _write(self, reg, values):
        self.spi.xfer2([reg | WRITE] + list(values))

    def _check_present(self):
        self._write(REG_HIGH_FAULT_MSB, _PROBE_PATTERN)
        got = self._read(REG_HIGH_FAULT_MSB, 2)
        if got != _PROBE_PATTERN:
            raise RuntimeError(
                f"No MAX31865 answering on /dev/spidev{self.bus}.{self.device} "
                f"(wrote {bytes(_PROBE_PATTERN).hex()}, read back "
                f"{bytes(got).hex()}). Check SPI is enabled and the CS/SDI/SDO "
                f"wiring."
            )

    def read_once(self):
        """One one-shot conversion. Returns (raw_code, fault_status), where
        raw_code is the 15-bit RTD/REF ratio and fault_status is the fault
        register (0 when the conversion had no fault)."""
        self._write(REG_CONFIG, [self._config | CFG_VBIAS])
        time.sleep(BIAS_SETTLE_S)
        self._write(REG_CONFIG, [self._config | CFG_VBIAS | CFG_ONE_SHOT])
        time.sleep(self._conversion_s)

        msb, lsb = self._read(REG_RTD_MSB, 2)
        fault_status = 0
        if lsb & 0x01:
            fault_status = self._read(REG_FAULT_STATUS, 1)[0]
            # Clear the latched fault so the next conversion starts clean.
            self._write(REG_CONFIG, [self._config | CFG_FAULT_CLEAR])
        else:
            self._write(REG_CONFIG, [self._config])  # bias off between polls

        # Re-reading the config catches a board that dropped off the bus
        # mid-run (it would read 0x00 or 0xFF instead of what was written).
        config = self._read(REG_CONFIG, 1)[0]
        if config != self._config:
            raise RuntimeError(
                f"config reads back 0x{config:02X}, expected "
                f"0x{self._config:02X} (board disconnected?)"
            )
        return ((msb << 8) | lsb) >> 1, fault_status

    def read_frame(self):
        """Run one conversion per poll_interval. Returns
        {"raw": int, "fault_status": int, "rx_timestamp": float}, or None if
        this poll failed (the caller just tries again next time)."""
        delay = self._next_poll - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        self._next_poll = max(self._next_poll + self.poll_interval,
                              time.monotonic())

        frame = None
        try:
            raw, fault_status = self.read_once()
            frame = {"raw": raw, "fault_status": fault_status,
                     "rx_timestamp": time.time()}
            self._ok += 1
            if fault_status:
                self._faults += 1
            if not self._first_ok_logged:
                print(f"[rtd-reader] First MAX31865 reading on "
                      f"/dev/spidev{self.bus}.{self.device}: raw={raw} "
                      f"fault=0x{fault_status:02X}", flush=True)
                self._first_ok_logged = True
        except (OSError, RuntimeError) as exc:
            self._failed += 1
            self._last_error = str(exc)

        now = time.monotonic()
        if now - self._last_report >= DIAGNOSTIC_INTERVAL:
            detail = f", last error: {self._last_error}" if self._failed else ""
            print(f"[rtd-reader] alive: {self._ok} ok ({self._faults} with a "
                  f"sensor fault), {self._failed} failed polls in last "
                  f"{now - self._last_report:.0f}s{detail}", flush=True)
            self._ok = 0
            self._failed = 0
            self._faults = 0
            self._last_report = now
        return frame

    def close(self):
        try:
            self._write(REG_CONFIG, [self._config])  # leave the bias off
        except Exception:
            pass
        self.spi.close()
