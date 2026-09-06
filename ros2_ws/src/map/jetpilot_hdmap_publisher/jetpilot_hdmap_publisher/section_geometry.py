"""Dependency-free station interval rules for open and closed lane sections."""

import math


def is_full_lap(start: float, end: float, length: float, same_gate: bool = False) -> bool:
    if not all(math.isfinite(value) for value in (start, end, length)) or length <= 1e-9:
        return False
    return end - start >= length - 1e-9 or (same_gate and abs(end - start) <= 1e-9)


def contains_station(start: float, end: float, station: float, length: float,
                     closed_loop: bool, same_gate: bool = False) -> bool:
    if not all(math.isfinite(value) for value in (start, end, station, length)) or length <= 1e-9:
        return False
    if closed_loop:
        if is_full_lap(start, end, length, same_gate):
            return True
        start, end, station = start % length, end % length, station % length
        return (station >= start or station < end) if start > end else start <= station < end
    # Internal boundaries belong to the following section; the lane's final point
    # belongs to its final section, including when the entire lane is one section.
    return start <= station < end or (
        start < end and abs(end - length) <= 1e-9 and abs(station - length) <= 1e-9
    )
