#!/usr/bin/env python3
import re
import time

import serial


RESET_NOTICE = "TAKE EFFECT AFTER ATZ"
SEND_FAILURE_MARKERS = ("NO ACK", "TIMEOUT", "FAILED", "FAIL")
IGNORED_RX_LINES = {"OK", "NULL"}
HEX_PATTERN = re.compile(r"\b[0-9A-Fa-f]{2}\b")


def is_error_line(line):
    upper = line.upper()
    return "ERROR" in upper or "ERR" == upper or "UNKNOWN" in upper


def needs_reset(lines):
    return any(RESET_NOTICE in line.upper() for line in lines)


def send_failed(lines):
    upper_lines = [line.upper() for line in lines]
    return any(marker in line for line in upper_lines for marker in SEND_FAILURE_MARKERS)


def read_available_lines(ser, settle_time=0.2, quiet_gap=0.05, max_wait=1.5):
    # Poll until the serial buffer goes quiet (no new bytes for `quiet_gap`
    # seconds) or `max_wait` is hit, instead of a single fixed-length sleep.
    # A bursty/slow modem write can otherwise get sliced mid-packet if the
    # second burst arrives just after a one-shot read_all().
    time.sleep(settle_time)
    buf = bytearray()
    start = last_data = time.monotonic()
    while True:
        chunk = ser.read_all()
        if chunk:
            buf.extend(chunk)
            last_data = time.monotonic()
        now = time.monotonic()
        if now - last_data > quiet_gap or now - start > max_wait:
            break
        time.sleep(0.01)
    raw = bytes(buf).decode(errors="ignore")
    if not raw:
        return []
    return [line.strip() for line in raw.replace("\r", "\n").split("\n") if line.strip()]


def send_cmd(ser, cmd, wait=0.25):
    ser.write(f"{cmd.strip()}\r\n".encode())
    return read_available_lines(ser, wait)


def require_ok(ser, cmd, wait=0.25, allow_empty=False):
    lines = send_cmd(ser, cmd, wait)
    if any(is_error_line(line) for line in lines):
        raise RuntimeError(f"Modem rejected command: {cmd}")
    if not lines and not allow_empty:
        raise RuntimeError(f"No modem response for command: {cmd}")
    return lines


def try_probe_modem(ser):
    lines = send_cmd(ser, "AT+CFG", wait=0.4)
    return [] if any(is_error_line(line) for line in lines) else lines


def open_serial(port, baud):
    ser = serial.Serial(port, baud, timeout=0.5)
    time.sleep(2.0)  # LA66 needs ~2s to boot before accepting AT commands
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser


class LoRaTransport:
    def __init__(
        self,
        port,
        baud,
        freq,
        bw,
        sf,
        power,
        cr,
        crc,
        header,
        iq,
        preamble,
        syncword,
        group,
        ack=2,
        retries=3,
        rx_timeout=65535,
        rx_ack=2,
    ):
        self.port = port
        self.baud = baud
        self.freq = freq
        self.bw = bw
        self.sf = sf
        self.power = power
        self.cr = cr
        self.crc = crc
        self.header = header
        self.iq = iq
        self.preamble = preamble
        self.syncword = syncword
        self.group = group
        self.ack = ack
        self.retries = retries
        self.rx_timeout = rx_timeout
        self.rx_ack = rx_ack
        self.serial = None

    def __enter__(self):
        max_attempts = 3
        for attempt in range(1, max_attempts + 1):
            try:
                if self.serial:
                    self.serial.close()
                self.serial = open_serial(self.port, self.baud)
                reset_required = self.configure_modem()

                if reset_required:
                    print("[lora] Applying pending modem settings")
                    send_cmd(self.serial, "ATZ", wait=0.5)
                    self.serial.close()
                    time.sleep(2.0)
                    self.serial = open_serial(self.port, self.baud)
                    try_probe_modem(self.serial)

                return self

            except RuntimeError as exc:
                print(f"[lora] Modem init attempt {attempt}/{max_attempts} failed: {exc}")
                if attempt < max_attempts:
                    time.sleep(3.0)
                else:
                    raise

    def __exit__(self, exc_type, exc, tb):
        if self.serial and self.serial.is_open:
            self.serial.close()
        self.serial = None

    def configure_modem(self):
        ser = self._require_serial()
        reset_required = False
        try_probe_modem(ser)
        time.sleep(0.5)  # let probe response fully clear before sending config commands
        reset_required |= needs_reset(require_ok(ser, f"AT+FRE={self.freq},{self.freq}"))
        reset_required |= needs_reset(require_ok(ser, f"AT+BW={self.bw},{self.bw}"))
        reset_required |= needs_reset(require_ok(ser, f"AT+SF={self.sf},{self.sf}"))
        reset_required |= needs_reset(require_ok(ser, f"AT+POWER={self.power}"))
        reset_required |= needs_reset(require_ok(ser, f"AT+CR={self.cr},{self.cr}"))
        reset_required |= needs_reset(require_ok(ser, f"AT+CRC={self.crc},{self.crc}"))
        reset_required |= needs_reset(require_ok(ser, f"AT+HEADER={self.header},{self.header}"))
        reset_required |= needs_reset(require_ok(ser, f"AT+IQ={self.iq},{self.iq}"))
        reset_required |= needs_reset(require_ok(ser, f"AT+PREAMBLE={self.preamble},{self.preamble}"))
        reset_required |= needs_reset(require_ok(ser, f"AT+SYNCWORD={self.syncword}"))
        reset_required |= needs_reset(require_ok(ser, f"AT+GROUPMOD={self.group},{self.group}"))
        try:
            reset_required |= needs_reset(require_ok(ser, f"AT+RXMOD={self.rx_timeout},{self.rx_ack}"))
        except RuntimeError:
            # Sender-only firmware may not support RX mode configuration.
            pass
        try_probe_modem(ser)
        return reset_required

    def send_hex(self, payload_hex):
        ser = self._require_serial()
        lines = require_ok(
            ser,
            f"AT+SEND=0,{payload_hex},{self.ack},{self.retries}",
            wait=self.send_wait_time(),
        )
        if self.ack != 0 and send_failed(lines):
            raise RuntimeError(f"Receiver acknowledgement failed: {' | '.join(lines)}")
        return lines

    def receive_hex_lines(self, recv_format=0, wait=0.5):
        ser = self._require_serial()
        lines = require_ok(ser, f"AT+RECV={recv_format}", wait=wait, allow_empty=True)
        return [line for line in lines if line.strip().upper() not in IGNORED_RX_LINES]

    def receive_packets(self, recv_format=0, wait=0.5):
        for line in self.receive_hex_lines(recv_format=recv_format, wait=wait):
            payload_hex = extract_hex_payload(line)
            if payload_hex:
                yield line, bytes.fromhex(payload_hex)

    def send_wait_time(self):
        if self.ack == 0:
            return 0.5
        return 1.5 + (self.retries * 5.5)

    def _require_serial(self):
        if not self.serial:
            raise RuntimeError("LoRaTransport is not connected")
        return self.serial


def extract_hex_payload(line):
    # Only look for hex bytes before the first comma. Modems append
    # RSSI/SNR metadata after the payload separated by commas (e.g.
    # "... d1 59 ,-33,13"), and without this the two-char tokens in
    # "-33" and "13" get scooped up as bogus extra payload bytes,
    # corrupting the packet and shifting where the CRC field is read from.
    payload_part = line.split(",", 1)[0]
    hex_bytes = HEX_PATTERN.findall(payload_part)
    if hex_bytes:
        candidate = "".join(hex_bytes)
        if len(candidate) % 2 == 0:
            return candidate
    return None
