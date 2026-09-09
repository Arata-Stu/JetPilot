"""Clock selection shared by all streams used to construct training labels."""

from typing import Any


def stamp_to_ns(msg: Any, fallback_ns: int) -> int:
    stamp = getattr(getattr(msg, "header", None), "stamp", None)
    if stamp is None:
        return int(fallback_ns)
    value = int(getattr(stamp, "sec", 0)) * 1_000_000_000 + int(
        getattr(stamp, "nanosec", 0)
    )
    return value if value > 0 else int(fallback_ns)


def alignment_timestamp_ns(msg: Any, bag_timestamp_ns: int, source: str = "bag") -> int:
    if source == "bag":
        return int(bag_timestamp_ns)
    if source == "header":
        return stamp_to_ns(msg, bag_timestamp_ns)
    raise ValueError("timestamp_source must be bag or header")
