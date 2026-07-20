import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Union

import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

# Universal deserializer fallback across rosbags v0.9.x and v0.10.x+
try:
    from rosbags.serde import deserialize_cdr
except ImportError:
    try:
        from rosbags.serde.cdr import deserialize_cdr
    except ImportError:
        deserialize_cdr = None


def deserialize_message(rawdata: bytes, msgtype: str, typestore: Any) -> Any:
    """Safely deserialize CDR message bytes across any rosbags version."""
    if hasattr(typestore, "deserialize_cdr"):
        return typestore.deserialize_cdr(rawdata, msgtype)
    elif deserialize_cdr is not None:
        return deserialize_cdr(rawdata, msgtype, typestore)
    else:
        raise RuntimeError(
            "Could not find deserialize_cdr method on typestore or rosbags.serde. "
            "Please verify your rosbags installation."
        )


# Definition for dynamic registration if event_camera_msgs is not in typestore
EVENT_PACKET_MSG_DEF = """
std_msgs/Header header
uint32 height
uint32 width
string encoding
bool is_bigendian
uint64 time_base
uint8[] events
"""

@dataclass
class EventPacketRecord:
    arrival_time_ns: int
    stamp_ns: int
    topic: str
    width: int
    height: int
    encoding: str
    payload_bytes: int
    estimated_events: int

@dataclass
class EventImageRecord:
    arrival_time_ns: int
    stamp_ns: int
    frame_id: str
    width: int
    height: int
    encoding: str
    step: int
    active_pixel_ratio: float
    active_pixel_count: int
    mean_brightness: float

@dataclass
class DiagnosticRecord:
    arrival_time_ns: int
    stamp_ns: int
    hardware_id: str
    status_name: str
    level: int
    message: str
    values: Dict[str, Any] = field(default_factory=dict)

@dataclass
class BagData:
    raw_packets: List[EventPacketRecord] = field(default_factory=list)
    processed_packets: List[EventPacketRecord] = field(default_factory=list)
    images: List[EventImageRecord] = field(default_factory=list)
    diagnostics: List[DiagnosticRecord] = field(default_factory=list)
    start_time_ns: int = 0
    end_time_ns: int = 0

    @property
    def duration_sec(self) -> float:
        if self.end_time_ns <= self.start_time_ns:
            return 0.0
        return (self.end_time_ns - self.start_time_ns) / 1e9


class BagReader:
    """
    ROSBag reader for event camera topics (/event_camera/events_raw, /events, /event_image, /diagnostics).
    Uses pure-Python 'rosbags' library to parse CDR messages from .mcap or .db3 storage.
    """

    def __init__(self, bag_path: Union[str, Path], namespace: str = "/event_camera"):
        self.bag_path = Path(bag_path)
        self.namespace = namespace.rstrip("/")
        try:
            self.typestore = get_typestore(Stores.ROS2_JAZZY)
        except (AttributeError, KeyError):
            try:
                self.typestore = get_typestore(Stores.LATEST)
            except Exception:
                self.typestore = get_typestore()
        self._ensure_custom_types()

    def _ensure_custom_types(self) -> None:
        """Register event_camera_msgs/msg/EventPacket if not already in typestore."""
        try:
            self.typestore.types["event_camera_msgs/msg/EventPacket"]
        except KeyError:
            try:
                self.typestore.register_msg(
                    "event_camera_msgs/msg/EventPacket", EVENT_PACKET_MSG_DEF
                )
            except Exception as e:
                # If typestore registration fails, we will handle it gracefully during read
                pass

    def read_bag(self) -> BagData:
        """
        Reads all relevant topics from the bag file and extracts time-series records.
        """
        bag_data = BagData()
        if not self.bag_path.exists():
            raise FileNotFoundError(f"Bag directory or file not found: {self.bag_path}")

        raw_topic = f"{self.namespace}/events_raw"
        events_topic = f"{self.namespace}/events"
        image_topic = f"{self.namespace}/event_image"
        diag_topic = f"{self.namespace}/diagnostics"
        # Also support top-level /diagnostics if bag_manager records it there
        diag_topics = {diag_topic, "/diagnostics"}

        with Reader(self.bag_path) as reader:
            connections = [
                c for c in reader.connections
                if c.topic in {raw_topic, events_topic, image_topic} or c.topic in diag_topics
            ]
            if not connections:
                raise ValueError(
                    f"No matching event camera topics found in bag: {self.bag_path}\n"
                    f"Available topics: {[c.topic for c in reader.connections]}"
                )

            for connection, timestamp, rawdata in reader.messages(connections=connections):
                if bag_data.start_time_ns == 0 or timestamp < bag_data.start_time_ns:
                    bag_data.start_time_ns = timestamp
                if timestamp > bag_data.end_time_ns:
                    bag_data.end_time_ns = timestamp

                msgtype = connection.msgtype
                try:
                    msg = deserialize_message(rawdata, connection.msgtype, self.typestore)
                except Exception:
                    # Try fallback to standard types if custom typestore failed
                    continue

                if connection.topic in {raw_topic, events_topic}:
                    self._parse_event_packet(msg, timestamp, connection.topic, bag_data)
                elif connection.topic == image_topic:
                    self._parse_event_image(msg, timestamp, bag_data)
                elif connection.topic in diag_topics:
                    self._parse_diagnostics(msg, timestamp, bag_data)

        # Sort all records by arrival time
        bag_data.raw_packets.sort(key=lambda r: r.arrival_time_ns)
        bag_data.processed_packets.sort(key=lambda r: r.arrival_time_ns)
        bag_data.images.sort(key=lambda r: r.arrival_time_ns)
        bag_data.diagnostics.sort(key=lambda r: r.arrival_time_ns)

        return bag_data

    def _parse_event_packet(
        self, msg: Any, arrival_time_ns: int, topic: str, bag_data: BagData
    ) -> None:
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        payload_bytes = len(msg.events)
        encoding = getattr(msg, "encoding", "evt3")

        # Estimate event count: EVT3 raw words are typically 16-bit (2 bytes per event word)
        # For decoded or raw packets, payload_bytes / 2 provides a standard estimate of event volume
        estimated_events = payload_bytes // 2 if "evt3" in encoding.lower() else payload_bytes

        record = EventPacketRecord(
            arrival_time_ns=arrival_time_ns,
            stamp_ns=stamp_ns,
            topic=topic,
            width=int(getattr(msg, "width", 0)),
            height=int(getattr(msg, "height", 0)),
            encoding=encoding,
            payload_bytes=payload_bytes,
            estimated_events=estimated_events,
        )
        if "raw" in topic:
            bag_data.raw_packets.append(record)
        else:
            bag_data.processed_packets.append(record)

    def _parse_event_image(
        self, msg: Any, arrival_time_ns: int, bag_data: BagData
    ) -> None:
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        width = int(msg.width)
        height = int(msg.height)
        step = int(msg.step)
        encoding = msg.encoding

        active_pixel_ratio = 0.0
        active_pixel_count = 0
        mean_brightness = 0.0

        if width > 0 and height > 0 and len(msg.data) > 0:
            data_arr = np.frombuffer(msg.data, dtype=np.uint8)
            mean_brightness = float(np.mean(data_arr))
            if encoding == "bgr8":
                # Reshape to (height, width, 3)
                try:
                    img_bgr = data_arr.reshape((height, width, 3))
                    # Background is black (0, 0, 0). Any pixel where b!=0 or g!=0 or r!=0 has events
                    active_mask = np.any(img_bgr != 0, axis=2)
                    active_pixel_count = int(np.sum(active_mask))
                    active_pixel_ratio = float(active_pixel_count) / (width * height)
                except ValueError:
                    pass
            elif encoding == "mono8":
                try:
                    img_mono = data_arr.reshape((height, width))
                    # Background is 127. Any pixel != 127 has events
                    active_mask = img_mono != 127
                    active_pixel_count = int(np.sum(active_mask))
                    active_pixel_ratio = float(active_pixel_count) / (width * height)
                except ValueError:
                    pass

        record = EventImageRecord(
            arrival_time_ns=arrival_time_ns,
            stamp_ns=stamp_ns,
            frame_id=msg.header.frame_id,
            width=width,
            height=height,
            encoding=encoding,
            step=step,
            active_pixel_ratio=active_pixel_ratio,
            active_pixel_count=active_pixel_count,
            mean_brightness=mean_brightness,
        )
        bag_data.images.append(record)

    def _parse_diagnostics(
        self, msg: Any, arrival_time_ns: int, bag_data: BagData
    ) -> None:
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        for status in msg.status:
            # Check if this status belongs to openeb driver or preprocessor
            if "openeb" in status.name.lower() or "preprocessor_stats" in status.name or "driver_stats" in status.name:
                val_dict: Dict[str, Any] = {}
                for kv in status.values:
                    key = kv.key
                    raw_val = kv.value
                    # Parse numerical types gracefully
                    if raw_val.lower() in ("true", "false"):
                        val_dict[key] = raw_val.lower() == "true"
                    else:
                        try:
                            if "." in raw_val:
                                val_dict[key] = float(raw_val)
                            else:
                                val_dict[key] = int(raw_val)
                        except ValueError:
                            val_dict[key] = raw_val

                record = DiagnosticRecord(
                    arrival_time_ns=arrival_time_ns,
                    stamp_ns=stamp_ns,
                    hardware_id=status.hardware_id,
                    status_name=status.name,
                    level=int(status.level),
                    message=status.message,
                    values=val_dict,
                )
                bag_data.diagnostics.append(record)
