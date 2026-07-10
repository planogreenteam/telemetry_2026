#!/usr/bin/env python3
import time

from telemetry_packet import EventType


class BMVTransmitPolicy:
    def __init__(
        self,
        voltage_delta_mv,
        current_delta_ma,
        power_delta_w,
        heartbeat_seconds,
    ):
        self.voltage_delta_mv = voltage_delta_mv
        self.current_delta_ma = current_delta_ma
        self.power_delta_w = power_delta_w
        self.heartbeat_seconds = heartbeat_seconds
        self.last_sent = None
        self.last_sent_at = 0.0
        self.seq = 0

    def classify(self, reading):
        now = time.time()
        current_alarm = int(reading["fields"].get("alarm", 0))

        if self.last_sent is None:
            return EventType.SAMPLE

        previous_fields = self.last_sent["fields"]
        previous_alarm = int(previous_fields.get("alarm", 0))

        if current_alarm != previous_alarm and current_alarm != 0:
            return EventType.ALARM

        if self._charge_state_changed(reading):
            return EventType.THRESHOLD_CROSSING

        if self._changed_enough(reading):
            return EventType.DELTA_UPDATE

        if now - self.last_sent_at >= self.heartbeat_seconds:
            return EventType.HEARTBEAT

        return None

    def allocate_seq(self):
        """Take the next wire sequence number. Allocated when a packet is
        BUILT (not when it is confirmed sent), so every packet on the air
        has a unique seq and the receiver's gap counter treats a failed
        send exactly like a lost packet."""
        current_seq = self.seq
        self.seq = (self.seq + 1) & 0xFFFF
        return current_seq

    def commit_sent(self, reading):
        """Record `reading` as the last successfully transmitted state.
        Called only after the transport accepts the packet, so a failed
        send neither resets the heartbeat timer nor suppresses the retry."""
        self.last_sent = reading
        self.last_sent_at = time.time()

    def mark_sent(self, reading):
        # Legacy combined form; prefer allocate_seq() + commit_sent().
        seq = self.allocate_seq()
        self.commit_sent(reading)
        return seq

    def _charge_state_changed(self, reading):
        return (
            reading["fields"].get("charge_state")
            != self.last_sent["fields"].get("charge_state")
        )

    def _changed_enough(self, reading):
        current_fields = reading["fields"]
        previous_fields = self.last_sent["fields"]
        return any(
            (
                abs(current_fields.get("voltage_mv", 0) - previous_fields.get("voltage_mv", 0))
                >= self.voltage_delta_mv,
                abs(current_fields.get("current_ma", 0) - previous_fields.get("current_ma", 0))
                >= self.current_delta_ma,
                abs(current_fields.get("power_w", 0) - previous_fields.get("power_w", 0))
                >= self.power_delta_w,
            )
        )
