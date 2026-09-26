#!/usr/bin/env python3
"""LoRa radio settings shared by every sender and receiver.

Both ends MUST use the same values or there is no link, so they live here
once instead of being copied into each script (the copies had already
drifted: the receiver had radio CRC off while the sender had it on, and
the LORA/sender.py test tool used a different BW/SF entirely).
"""

FREQ = "868.100"
# BW 2 = 500 kHz: 4x less airtime per packet than 125 kHz, paid for with
# range we don't need (ground station stays within ~1 km of the car).
BW = 2
SF = 7
POWER = 20
CR = 1
# Radio-level CRC ON: with it off, corrupted frames (RF bit flips) are
# delivered to the receiver instead of being dropped by the radio, and they
# waste the receiver modem's scarce 9600-baud UART time being printed as
# garbage.
CRC = 1
HEADER = 0
IQ = 0
PREAMBLE = 8
SYNCWORD = 0
GROUP = 0
