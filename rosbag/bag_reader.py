"""
bag_reader.py
"""

from typing import Any
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from tf2_msgs.msg import TFMessage
from tf2_ros.buffer import Buffer


def read_rosbag(filename: str) -> dict[str, list[tuple[Any, int]]]:
    """Read a rosbag"""
    bag_reader = SequentialReader()
    storage_options = StorageOptions(uri=filename, storage_id="sqlite3")
    converter_options = ConverterOptions(
        input_serialization_format="", output_serialization_format="")
    bag_reader.open(storage_options, converter_options)

    topics_dict = {}
    while bag_reader.has_next():
        topic, msg, ts = bag_reader.read_next()
        if topic not in topics_dict:
            topics_dict[topic] = []
        topics_dict[topic].append((msg, ts))
    return topics_dict


def deserialize_tfs(tfs: list[TFMessage], buffer: Buffer) -> Buffer:
    """Deserialize TF messages"""
    for tf in tfs:
        tf_message: TFMessage = deserialize_message(tf, TFMessage)
        for transform in tf_message.transforms:
            buffer.set_transform(transform, 'default_authority')
    return buffer


def deserialize_msgs(msgs_with_ts: list[tuple[Any, int]], msg_type: Any,
                     keep_stamp: bool = False) -> list[Any]:
    """Deserialize messages"""
    deserialized_msgs = []
    for msg, ts in msgs_with_ts:
        if keep_stamp:
            deserialized_msgs.append((deserialize_message(msg, msg_type), ts))
        else:
            deserialized_msgs.append(deserialize_message(msg, msg_type))
    return deserialized_msgs


def deserialize_rosbag(rosbag: dict[str, list[tuple[Any, int]]], msg_types: dict[str, Any],
                       keep_stamp: bool = False) -> dict[str, list[Any]]:
    """Deserialize messages in rosbags"""
    deserialized_msgs = {}
    for topic, msgs_with_ts in rosbag.items():
        try:
            deserialized_msgs[topic] = deserialize_msgs(msgs_with_ts, msg_types[topic], keep_stamp)
        except KeyError:
            deserialized_msgs[topic] = []
    return deserialized_msgs


if __name__ == "__main__":
    from geometry_msgs.msg import PoseStamped

    info = read_rosbag("out1/")
    info = deserialize_rosbag(info, {"/M300/self_localization/pose": PoseStamped}, True)
    print(info)
