#!/usr/bin/env python3
"""Field-delta-based transmit policy for CAN-derived telemetry.

Matches the contract the BMV policy provides (classify / mark_sent), but
generalised:

  - Tracks state per "source key", not just one global last-sent. Multiple
    MPPTs sharing one policy each get their own last-known state. The
    source key defaults to device_id, which is what the MPPT normalizer
    increments per board.

  - Tracks state per "frame kind" for BMSes, because different CAN IDs
    from one BMS pack carry different fields (SOC on 0x355, voltage on
    0x356) — comparing 0x355's SOC against the 0x356 reading we last
    stored would be a category error. Within a single source, each
    *can_id_hex* gets its own slot. If the reading has no can_id_hex
    (MPPT case — already collapsed into one frame per board), it uses
    a default slot per source.

Configurable per-field deltas: when ANY watched field has moved by at
least its delta since the last send (or any field appears that didn't
before, or any field disappears), we emit event_type_change. Otherwise,
once heartbeat_seconds has elapsed, we emit event_type_heartbeat.
"""

import time


class GenericTransmitPolicy:
    def __init__(self, deltas, event_type_change, event_type_heartbeat,
                 heartbeat_seconds=60):
        """
        deltas:                  {field_name: minimum_change_to_transmit}
                                 Fields not in this dict are not watched.
                                 Strings & flags are typically not in here.
        event_type_change:       EventType emitted when a field moved
                                 enough. Required.
        event_type_heartbeat:    EventType emitted when heartbeat fires.
                                 May equal event_type_change.
        heartbeat_seconds:       Force a transmit this long after the
                                 last one even if nothing changed.
        """
        self.deltas = dict(deltas)
        self.event_type_change = event_type_change
        self.event_type_heartbeat = event_type_heartbeat
        self.heartbeat_seconds = heartbeat_seconds

        # Per-(source_key, slot_key) state:
        #   { (src, slot): {"fields": {...}, "sent_at": float} }
        self._state = {}
        self._pending_key = None
        self._pending_reading = None
        self._pending_now = None
        self.seq = 0

    @staticmethod
    def _keys(reading):
        # Default source key is device_id. The slot key must separate frames
        # that carry disjoint field sets within one source, otherwise they
        # overwrite each other's last-sent state and the delta thresholds are
        # never meaningfully compared.
        #
        #   - BMS: different CAN IDs carry different fields (SOC on 0x355,
        #     voltage on 0x356), keyed by can_id_hex.
        #   - MPPT: power (packet_id 0) and status (packet_id 1) frames carry
        #     disjoint fields and arrive at different rates; keyed by packet_id.
        #   - Anything else collapses to a single slot per source.
        src = reading["device_id"]
        fields = reading["fields"]
        if "can_id_hex" in fields:
            slot = fields["can_id_hex"]
        elif reading.get("packet_id") is not None:
            slot = f"pkt{reading['packet_id']}"
        else:
            slot = "_only"
        return src, slot

    def classify(self, reading):
        src, slot = self._keys(reading)
        now = time.time()
        previous = self._state.get((src, slot))

        if previous is None:
            event_type = self.event_type_change
        else:
            since_last = now - previous["sent_at"]
            if self._changed_enough(reading["fields"], previous["fields"]):
                event_type = self.event_type_change
            elif since_last >= self.heartbeat_seconds:
                event_type = self.event_type_heartbeat
            else:
                event_type = None

        if event_type is not None:
            self._pending_key = (src, slot)
            self._pending_reading = reading
            self._pending_now = now
        return event_type

    def allocate_seq(self):
        """Take the next wire sequence number. Allocated when a packet is
        BUILT (not when it is confirmed sent), so every packet on the air
        has a unique seq even when several readings from this policy go
        out inside one batch frame, and the receiver's gap counter treats
        a failed send exactly like a lost packet."""
        current_seq = self.seq
        self.seq = (self.seq + 1) & 0xFFFF
        return current_seq

    def commit_sent(self, reading):
        """Record `reading` as the last successfully transmitted state for
        its (source, slot). Called only after the transport accepts the
        packet, so a failed send neither resets the heartbeat timer nor
        suppresses the retry. Uses the cached pending state from classify()
        when it matches, so the source/slot key is exact."""
        if self._pending_key is not None and self._pending_reading is reading:
            self._state[self._pending_key] = {
                "fields": dict(reading["fields"]),
                "sent_at": self._pending_now,
            }
            self._pending_key = None
            self._pending_reading = None
            self._pending_now = None
        else:
            # Defensive: caller committed a different reading than the last
            # classify(). Record current state anyway so we don't get stuck
            # claiming "nothing changed" forever.
            src, slot = self._keys(reading)
            self._state[(src, slot)] = {
                "fields": dict(reading["fields"]),
                "sent_at": time.time(),
            }

    def mark_sent(self, reading):
        # Legacy combined form; prefer allocate_seq() + commit_sent().
        seq = self.allocate_seq()
        self.commit_sent(reading)
        return seq

    def _changed_enough(self, current_fields, previous_fields):
        # New or removed field counts as a change.
        watched_now = set(self.deltas) & set(current_fields)
        watched_then = set(self.deltas) & set(previous_fields)
        if watched_now != watched_then:
            return True

        for field, threshold in self.deltas.items():
            if field not in current_fields:
                continue
            cur = current_fields[field]
            prev = previous_fields.get(field)
            if prev is None:
                return True
            try:
                cur_f = float(cur)
                prev_f = float(prev)
                # Zero-crossing: any watched field transitioning to or from
                # zero is always treated as a significant change regardless
                # of the configured delta threshold. This ensures a panel
                # disconnect (pv_power_w -> 0) is transmitted immediately
                # rather than waiting for the heartbeat interval.
                if (cur_f == 0.0) != (prev_f == 0.0):
                    return True
                if abs(cur_f - prev_f) >= threshold:
                    return True
            except (TypeError, ValueError):
                # Non-numeric watched field — fall back to equality
                if cur != prev:
                    return True
        return False
