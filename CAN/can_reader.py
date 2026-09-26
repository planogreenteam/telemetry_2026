#!/usr/bin/env python3
"""SocketCAN reader for the MPPTs.

read_frame() returns (kind, raw_frame_dict) tuples where `kind` is one of
the strings in the id_to_kind mapping passed at construction time (today
only "mppt" — the BMS moved to RS485, see BMS/). The normalizer for that
kind decodes the bytes into named fields.

The reader applies kernel-level CAN filters built from id_to_kind, so frames
with unmapped IDs never reach Python — important on a 125 kbps bus that
might be carrying motor controller chatter we don't care about.

Pi setup recap:
    /boot/firmware/config.txt:
        dtparam=spi=on
        dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25

    At runtime (matches the TPEE 125 kbps default):
        sudo ip link set can0 up type can bitrate 125000
        sudo ifconfig can0 txqueuelen 1000
"""

import can


class CANReader:
    def __init__(self, interface="can0", bitrate=125000, id_to_kind=None,
                 bustype="socketcan"):
        """
        interface:    SocketCAN device name, e.g. "can0".
        bitrate:      informational for SocketCAN (the kernel decides), but
                      python-can records it. 125000 matches the TPEE
                      standard noted in their wiki.
        id_to_kind:   dict mapping CAN ID (int) -> kind string.
                      e.g. {0x20: "mppt", 0x21: "mppt", ...}
                      Frames whose IDs aren't in this dict are dropped at
                      the kernel filter level.
        bustype:      python-can interface name, default "socketcan".
        """
        if not id_to_kind:
            raise ValueError(
                "CANReader needs an id_to_kind mapping. Pass at least one "
                "{can_id: kind} entry — empty mapping would accept every "
                "frame on the bus and overload the LoRa link."
            )

        self.interface = interface
        self.bitrate = bitrate
        self.id_to_kind = dict(id_to_kind)

        filters = self._build_filters(self.id_to_kind.keys())

        # python-can accepts both `interface` (new) and `bustype` (legacy).
        self.bus = can.interface.Bus(
            channel=interface,
            interface=bustype,
            bitrate=bitrate,
            can_filters=filters,
        )

    @staticmethod
    def _build_filters(ids):
        filters = []
        for can_id in ids:
            extended = can_id > 0x7FF
            filters.append({
                "can_id": int(can_id),
                "can_mask": 0x1FFFFFFF if extended else 0x7FF,
                "extended": extended,
            })
        return filters

    def read_frame(self):
        """Block until a relevant frame arrives. Returns (kind, frame_dict)
        or None if the kernel woke us spuriously."""
        while True:
            msg = self.bus.recv(timeout=None)
            if msg is None:
                return None  # surface to caller, sender loop handles it
            if msg.is_error_frame or msg.is_remote_frame:
                continue
            kind = self.id_to_kind.get(msg.arbitration_id)
            if kind is None:
                # Shouldn't happen given the filters, but be defensive
                continue
            frame = {
                "can_id": msg.arbitration_id,
                "extended": bool(msg.is_extended_id),
                "dlc": msg.dlc,
                "data": bytes(msg.data),
                "rx_timestamp": msg.timestamp,
            }
            return kind, frame

    def close(self):
        try:
            self.bus.shutdown()
        except Exception:
            pass
