#!/usr/bin/env python3
import argparse
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))

from LORA import radio_config as radio
from LORA.lora_transport import LoRaTransport

DEFAULT_PORT = "/dev/tty.usbserial-0002"
DEFAULT_BAUD = 9600
# Radio settings come from the shared config so this test tool can talk to
# telemetry_receiver.py out of the box.
DEFAULT_FREQ = radio.FREQ
DEFAULT_BW = radio.BW
DEFAULT_SF = radio.SF
DEFAULT_POWER = radio.POWER
DEFAULT_CR = radio.CR
DEFAULT_CRC = radio.CRC
DEFAULT_HEADER = radio.HEADER
DEFAULT_IQ = radio.IQ
DEFAULT_PREAMBLE = radio.PREAMBLE
DEFAULT_SYNCWORD = radio.SYNCWORD
DEFAULT_GROUP = radio.GROUP
# The receiver runs with RX ACK off, so waiting for an ACK would always
# report failure.
DEFAULT_ACK = 0
DEFAULT_RETRIES = 3
DEFAULT_SEND_ATTEMPTS = 3
DEFAULT_RETRY_DELAY = 2.0
DEFAULT_TEXT = "Hello-SOLAR"


def send_once(args, send_mode, payload):
    with LoRaTransport(
        port=args.port,
        baud=args.baud,
        freq=args.freq,
        bw=args.bw,
        sf=args.sf,
        power=args.power,
        cr=args.cr,
        crc=args.crc,
        header=args.header,
        iq=args.iq,
        preamble=args.preamble,
        syncword=args.syncword,
        group=args.group,
        ack=args.ack,
        retries=args.retries,
    ) as transport:
        if send_mode == 0:
            transport.send_hex(payload)
        else:
            payload = payload.encode().hex()
            transport.send_hex(payload)


def send_with_retries(args, send_mode, payload):
    last_error = None

    for attempt in range(1, args.send_attempts + 1):
        try:
            send_once(args, send_mode, payload)
            return attempt
        except Exception as exc:
            last_error = exc
            if attempt == args.send_attempts:
                break
            print(
                f"[sender] Send attempt {attempt}/{args.send_attempts} failed: {exc}. "
                f"Retrying in {args.retry_delay:.1f}s..."
            )
            time.sleep(args.retry_delay)

    raise RuntimeError(
        f"Failed to send after {args.send_attempts} attempts: {last_error}"
    ) from last_error


def main():
    parser = argparse.ArgumentParser(description="LoRa sender for AT+SEND firmware")
    parser.add_argument("--port", default=DEFAULT_PORT, help="Serial device path")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Serial baud rate")
    parser.add_argument("--freq", default=DEFAULT_FREQ, help="TX/RX frequency in MHz, e.g. 868.100")
    parser.add_argument("--bw", type=int, default=DEFAULT_BW, help="Bandwidth enum 0-9")
    parser.add_argument("--sf", type=int, default=DEFAULT_SF, help="Spreading factor 5-12")
    parser.add_argument("--power", type=int, default=DEFAULT_POWER, help="TX power 0-22 dBm")
    parser.add_argument("--cr", type=int, default=DEFAULT_CR, help="Coding rate 1-4")
    parser.add_argument("--crc", type=int, default=DEFAULT_CRC, help="CRC 0=off 1=on")
    parser.add_argument("--header", type=int, default=DEFAULT_HEADER, help="Header 0=explicit 1=implicit")
    parser.add_argument("--iq", type=int, default=DEFAULT_IQ, help="IQ invert 0=standard 1=inverted")
    parser.add_argument("--preamble", type=int, default=DEFAULT_PREAMBLE, help="Preamble length")
    parser.add_argument("--syncword", type=int, default=DEFAULT_SYNCWORD, help="Sync word mode 0/1")
    parser.add_argument("--group", type=int, default=DEFAULT_GROUP, help="Group 0-255")
    parser.add_argument("--text", default=DEFAULT_TEXT, help="Text payload to send")
    parser.add_argument("--hex", help="Hex payload to send instead of text")
    parser.add_argument("--ack", type=int, choices=(0, 1, 2), default=DEFAULT_ACK, help="ACK mode 0/1/2")
    parser.add_argument("--retries", type=int, choices=range(0, 9), default=DEFAULT_RETRIES, help="Retransmissions 0-8")
    parser.add_argument(
        "--send-attempts",
        type=int,
        default=DEFAULT_SEND_ATTEMPTS,
        help="Application-level send attempts before failing",
    )
    parser.add_argument(
        "--retry-delay",
        type=float,
        default=DEFAULT_RETRY_DELAY,
        help="Seconds to wait between application-level send attempts",
    )
    args = parser.parse_args()

    if args.hex:
        send_mode = 0
        payload = args.hex
    else:
        send_mode = 1
        payload = args.text

    if args.send_attempts < 1:
        raise ValueError("--send-attempts must be at least 1")
    if args.retry_delay < 0:
        raise ValueError("--retry-delay must be >= 0")

    attempts_used = send_with_retries(args, send_mode, payload)

    label = "hex" if send_mode == 0 else "text"
    if attempts_used == 1:
        print(f"[sender] Sent {label} payload on {args.port}: {payload}")
    else:
        print(
            f"[sender] Sent {label} payload on {args.port} after "
            f"{attempts_used} attempts: {payload}"
        )


if __name__ == "__main__":
    main()
