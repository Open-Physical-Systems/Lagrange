"""Events of a run, found in its signals.

A simulation server reports events as its modules raise them. A recording has none stored
yet, so the mock server derives the ones that can be read straight off the data: where the
battery was being charged by the motors, and where the robot turned hard. Every event names
the signal that shows it, so it can be checked against the plot.
"""

from __future__ import annotations

import numpy as np
from lagrange.viewer.v1 import session_pb2

from .recording import Recording

NO_PRIM = 0xFFFFFFFF
REGEN_CURRENT = -0.05  # A: a source current below this is charging, not noise
HARD_TURN_RATE = 1.2  # rad/s
MIN_DURATION_NS = 50_000_000  # shorter spans are joined to noise and dropped
MERGE_GAP_NS = 100_000_000  # spans closer than this are one event


def _spans(mask: np.ndarray, dt_ns: int) -> list[tuple[int, int]]:
    """[begin, end) sample ranges where `mask` holds, merged and filtered by duration."""
    if not mask.any():
        return []
    padded = np.concatenate(([False], mask, [False]))
    edges = np.flatnonzero(padded[1:] != padded[:-1])
    spans = list(zip(edges[::2].tolist(), edges[1::2].tolist(), strict=True))
    merged: list[tuple[int, int]] = []
    for begin, end in spans:
        if merged and (begin - merged[-1][1]) * dt_ns < MERGE_GAP_NS:
            merged[-1] = (merged[-1][0], end)
        else:
            merged.append((begin, end))
    return [(b, e) for b, e in merged if (e - b) * dt_ns >= MIN_DURATION_NS]


def derive_events(recording: Recording) -> list[session_pb2.Event]:
    """Events that the recording's own signals show, ordered by start time."""
    events: list[session_pb2.Event] = []
    info = session_pb2.Event.SEVERITY_INFO
    by_name = {signal.info.name: signal for signal in recording.signals}

    # Regeneration: a source of an electrical net taking current instead of giving it.
    for net in recording.snapshot.nets:
        for terminal in net.terminals:
            signal = by_name.get(terminal.current_signal)
            if signal is None or not terminal.port.endswith("_out"):
                continue
            current = signal.data[:, 0]
            for begin, end in _spans(current < REGEN_CURRENT, signal.dt_ns):
                event = session_pb2.Event(
                    domain="electrical",
                    kind="regen",
                    severity=info,
                    text=f"regenerating into {net.name}, peak {current[begin:end].min():.1f} A",
                    prim=terminal.prim,
                    signal=signal.info.name,
                )
                event.begin.ns = begin * signal.dt_ns
                event.end.ns = end * signal.dt_ns
                events.append(event)

    # Hard turns: what a gyro about the vertical axis measures.
    for signal in recording.signals:
        if not signal.info.name.endswith("/gyro_z"):
            continue
        rate = signal.data[:, 0]
        for begin, end in _spans(np.abs(rate) > HARD_TURN_RATE, signal.dt_ns):
            event = session_pb2.Event(
                domain="rigid_body",
                kind="hard_turn",
                severity=info,
                text=f"turning at up to {np.abs(rate[begin:end]).max():.2f} rad/s",
                prim=signal.info.prim,
                signal=signal.info.name,
            )
            event.begin.ns = begin * signal.dt_ns
            event.end.ns = end * signal.dt_ns
            events.append(event)

    events.sort(key=lambda event: (event.begin.ns, event.domain))
    for index, event in enumerate(events):
        event.id = index + 1
    return events
