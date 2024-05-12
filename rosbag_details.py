#!/usr/bin/env python3
"""

rosbag_details.py

Author:   Johan Musaeus Bruun
Date:     2024-05-09
License:  MIT

"""

import argparse
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Tuple, cast

import numpy as np

from rosbags.highlevel import AnyReader
from rosbags.highlevel.anyreader import AnyReaderError
from rosbags.interfaces import Connection
from rosbags.typesys import Stores, get_typestore
from rosbags.typesys.stores.ros1_noetic import sensor_msgs__msg__CameraInfo, sensor_msgs__msg__Image

# from rosbags.typesys.stores.ros1_noetic import std_msgs__msg__Header


########################################################################################################################


IMAGE_MSGTYPE = "sensor_msgs/msg/Image"
CAMINFO_MSGTYPE = "sensor_msgs/msg/CameraInfo"


########################################################################################################################

@dataclass
class RosbagDetailsParameters:
    rosbag_path: Path
    verbose: bool

    @staticmethod
    def from_cli_args(argv: List[str]) -> "RosbagDetailsParameters":
        description = "Show details about a rosbag, especially about stereo image topics."
        epilog = "Copyright \N{COPYRIGHT SIGN} Johan Musaeus Bruun, 2024. License: MIT."
        parser = argparse.ArgumentParser(description=description, epilog=epilog)
        parser.add_argument("rosbag_path", nargs="?", type=Path, help="Rosbag to show details of.")
        parser.add_argument("-v", "--verbose", action="store_true", help="show verbose output")
        args = parser.parse_args(argv)

        # If no rosbag path is given, print the help message and exit
        if args.rosbag_path is None:
            parser.print_help()
            sys.exit(1)

        # Prepare parameters object
        params = RosbagDetailsParameters(
            rosbag_path=args.rosbag_path,
            verbose=args.verbose,
        )
        return params


########################################################################################################################


@dataclass
class StereoImagePairWithCamInfo:
    left_image_topic: str
    right_image_topic: str
    left_caminfo_topic: str
    right_caminfo_topic: str
    has_caminfo: bool


########################################################################################################################


def main():
    print("##################")
    print("# Rosbag Details #")
    print("##################")
    print("")
    # Obtain parameters from CLI
    params = RosbagDetailsParameters.from_cli_args(sys.argv[1:])
    # Print rosbag details
    print_rosbag_details_and_handle_exceptions(params)
    return


########################################################################################################################


def print_rosbag_details_and_handle_exceptions(params: RosbagDetailsParameters) -> None:
    try:
        print_rosbag_details(params)
    except AnyReaderError as e:
        if str(e) == "File magic is invalid.":
            print(f"ERROR: Invalid rosbag:\n  '{params.rosbag_path}'")
        else:
            print(f"ERROR: {e}")
    except (ValueError, FileNotFoundError) as e:
        print(f"ERROR: {e}")
    print("")


########################################################################################################################


def print_rosbag_details(params: RosbagDetailsParameters) -> None:
    # Expand a potential tilda and resolve the path
    rosbag_path = params.rosbag_path.expanduser().resolve()
    if not rosbag_path.is_file():
        raise FileNotFoundError(f"Rosbag not found: \"{rosbag_path}\"")

    # Print details about the identified stereo image pairs
    indent = "    "
    double_indent = indent * 2
    col_width = 16

    # Obtain rosbag size in bytes
    bag_size_bytes = rosbag_path.stat().st_size

    # Create a type store
    typestore = get_typestore(Stores.ROS1_NOETIC)
    # TODO: Maybe let the typestore depend on the rosbag version? Or make it an argument?

    # create reader instance and open for reading
    with AnyReader([rosbag_path], default_typestore=typestore) as reader:
        # Print bag information
        print("Bag details:")
        print("")
        print_bag_information(reader, bag_size_bytes, col_width, indent=double_indent)
        print_topics_and_types(reader, col_width, indent=double_indent)
        print("")

        # Analyze topics to find stereo pairs
        topics = list(reader.topics.keys())
        stereo_pairs, remaining_topics = find_stereo_pairs(topics)
        msgtypes_by_topic = {topic: topic_info.msgtype for topic, topic_info in reader.topics.items()}
        stereo_image_pairs, unused_stereo_pairs = construct_stereo_image_pairs(stereo_pairs, msgtypes_by_topic)

        # For now, just put non-image stereo pairs into remaining topics.
        for topic_left, topic_right in unused_stereo_pairs:
            remaining_topics.extend([topic_left, topic_right])

        print("Stereo image pairs:")
        if len(stereo_image_pairs) == 0:
            print(f"\n{double_indent}(No stereo pair topics)")
        else:
            for i, stereo_image_pair in enumerate(stereo_image_pairs):
                print("")
                print_stereo_image_pair(stereo_image_pair, reader, col_width, indent)
                if 0 < i < len(stereo_image_pairs) - 1:
                    print(f"{indent}---")
        print("")

        # Print details about the remaining topics
        print("Remaining topics:")
        if len(remaining_topics) == 0:
            print(f"\n{double_indent}(No remaining topics)")
        for remaining_topic in remaining_topics:
            print("")
            print_remaining_topic_details(remaining_topic, reader, col_width, indent=double_indent)
    return


########################################################################################################################


def print_bag_information(reader: AnyReader, bag_size_bytes: int, col_width: int, indent: str = "") -> None:
    assert len(reader.paths) == 1
    bag_path = reader.paths[0]
    duration_ns = reader.duration
    duration_sec = duration_ns * 1e-9
    duration_entry = f"{duration_sec:.1f} sec  ({duration_ns:,} ns)"
    bag_size_mb = bag_size_bytes / 1024 / 1024
    size_entry = f"{bag_size_mb:.1f} MB  ({bag_size_bytes:,} bytes)"
    print(f"{indent}{'Bag path:':<{col_width}}{bag_path}")
    print(f"{indent}{'Bag type:':<{col_width}}{'ROS 2 bag' if reader.is2 else 'ROS 1 bag'}")
    print(f"{indent}{'Bag size:':<{col_width}}{size_entry}")
    print(f"{indent}{'Duration:':<{col_width}}{duration_entry}")
    print(f"{indent}{'Start time:':<{col_width}}{_create_time_entry(reader.start_time)}")
    print(f"{indent}{'End time:':<{col_width}}{_create_time_entry(reader.end_time)}")
    print(f"{indent}{'Num topics:':<{col_width}}{len(reader.topics.keys())}")
    print(f"{indent}{'Num messages:':<{col_width}}{reader.message_count}")
    return


########################################################################################################################


def print_topics_and_types(reader: AnyReader, col_width: int, indent: str = ""):
    # msgtypes_by_topic = {topic: topic_info.msgtype for topic, topic_info in reader.topics.items()}

    topics = list(reader.topics.keys())
    topic_info_list = list(reader.topics.values())

    max_topic_length = max([len(t) for t in topics])
    max_type_length = max([len(ti.msgtype) for ti in topic_info_list])
    tow = max_topic_length
    tyw = max_type_length + 2

    for i, (topic, topic_info) in enumerate(zip(topics, topic_info_list), start=1):
        subheader = "Topics:" if i == 1 else ""
        msg_type = f"[{topic_info.msgtype}]"
        msgs_entry = f"{topic_info.msgcount} msgs"
        hz = compute_hz_for_topic(reader, topic)
        hz_entry = f"{hz:.1f} Hz"
        print(f"{indent}{subheader:<{col_width}}{topic:<{tow}}  {msg_type:<{tyw}}  {msgs_entry} @ {hz_entry}")
    return


########################################################################################################################


def compute_hz_for_topic(reader: AnyReader, topic: str) -> float:
    header_timestamps_ns_list, bag_timestamps_ns_list = extract_timestamps_for_topic(reader, topic)
    sec = float(header_timestamps_ns_list[-1] - header_timestamps_ns_list[0]) / 1_000_000_000
    hz = len(header_timestamps_ns_list) / sec
    return hz


def extract_timestamps_for_topic(reader: AnyReader, topic: str) -> Tuple[List[int], List[int]]:
    topic_info = reader.topics[topic]
    assert len(topic_info.connections) == 1
    connection = topic_info.connections[0]

    header_timestamps_ns_list: List[int] = []
    bag_timestamps_ns_list: List[int] = []

    for connection, timestamp, rawdata in reader.messages(connections=[connection]):
        msg = reader.deserialize(rawdata, connection.msgtype)
        if hasattr(msg, "header"):
            header_timestamps_ns_list.append(msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec)
        bag_timestamps_ns_list.append(timestamp)

    return header_timestamps_ns_list, bag_timestamps_ns_list


########################################################################################################################


def print_stereo_image_pair(stereo_image_pair: StereoImagePairWithCamInfo, reader: AnyReader,
                            col_width: int, indent: str = "") -> None:
    double_indent = indent * 2
    print(f"{indent}Left image:")
    left_image_header_timestamps_ns = \
        print_image_topic_details(stereo_image_pair.left_image_topic, reader, col_width, indent=double_indent)
    print("")
    print(f"{indent}Right image:")
    right_image_header_timestamps_ns = \
        print_image_topic_details(stereo_image_pair.right_image_topic, reader, col_width, indent=double_indent)
    lr_ts_aligned = get_lr_ts_aligned_entry(left_image_header_timestamps_ns, right_image_header_timestamps_ns)
    print(f"{double_indent}{'L/R-alignment:':<{col_width}}{lr_ts_aligned}")
    print("")
    if stereo_image_pair.has_caminfo:
        print(f"{indent}Left camera info:")
        left_cam_info_header_timestamps_ns = \
            print_cam_info_topic_details(stereo_image_pair.left_caminfo_topic, reader, col_width, indent=double_indent)
        print("")
        print(f"{indent}Right camera info:")
        right_cam_info_header_timestamps_ns = \
            print_cam_info_topic_details(stereo_image_pair.right_caminfo_topic, reader, col_width, indent=double_indent)
        lr_ts_aligned = get_lr_ts_aligned_entry(left_cam_info_header_timestamps_ns, right_cam_info_header_timestamps_ns)
        print(f"{double_indent}{'L/R-alignment:':<{col_width}}{lr_ts_aligned}")
        print("")
    return


########################################################################################################################


def print_image_topic_details(topic: str, reader: AnyReader, col_width: int, indent: str = "") -> np.ndarray:
    topic_info = reader.topics[topic]
    assert len(topic_info.connections) == 1, "Only one connection per topic currently supported."
    connection = topic_info.connections[0]

    # Extract image message information
    header_timestamps_ns, bag_timestamps_ns, width, height, encoding = \
        extract_image_message_information(reader, [connection])
    duration = get_duration_entry(header_timestamps_ns)
    start_time = get_start_time_entry(header_timestamps_ns)
    end_time = get_end_time_entry(header_timestamps_ns)
    time_gaps = get_time_gaps_entry(header_timestamps_ns)
    ts_aligned = get_ts_aligned_entry(header_timestamps_ns, bag_timestamps_ns)

    print(f"{indent}{'Topic:':<{col_width}}{connection.topic}")
    print(f"{indent}{'Msg type:':<{col_width}}{connection.msgtype}")
    print(f"{indent}{'Resolution:':<{col_width}}{width}x{height} (WxH)")
    print(f"{indent}{'Encoding:':<{col_width}}{encoding}")
    print(f"{indent}{'Num messages:':<{col_width}}{connection.msgcount}")
    print(f"{indent}{'Duration:':<{col_width}}{duration}")
    print(f"{indent}{'Start time:':<{col_width}}{start_time}")
    print(f"{indent}{'End time:':<{col_width}}{end_time}")
    print(f"{indent}{'Time gaps:':<{col_width}}{time_gaps}")
    print(f"{indent}{'Comment:':<{col_width}}{ts_aligned}")
    return header_timestamps_ns


def get_duration_entry(header_timestamps_ns: np.ndarray) -> str:
    duration_ns = np.max(header_timestamps_ns) - np.min(header_timestamps_ns)
    duration_sec = duration_ns * 1e-9
    duration_entry = f"{duration_sec:.1f} sec  ({duration_ns:,} ns)"
    return duration_entry


def get_start_time_entry(header_timestamps_ns: np.ndarray) -> str:
    start_time_entry = _create_time_entry(np.min(header_timestamps_ns))
    return start_time_entry


def get_end_time_entry(header_timestamps_ns: np.ndarray) -> str:
    end_time_entry = _create_time_entry(np.max(header_timestamps_ns))
    return end_time_entry


def _create_time_entry(timestamp_ns: np.int64) -> str:
    t_sec_only, t_nanosec_only = np.divmod(timestamp_ns, 1_000_000_000)
    t_datetime = datetime.fromtimestamp(int(t_sec_only))
    t_datetime_str = t_datetime.strftime("%Y-%m-%d %H:%M:%S")
    time_entry = f"{t_datetime_str}  ({t_sec_only}.{t_nanosec_only:>09} sec)"
    return time_entry


def get_time_gaps_entry(header_timestamps_ns: np.ndarray) -> str:
    gaps = np.diff(header_timestamps_ns)
    gap_mean_sec = np.mean(gaps) * 1e-9
    gap_min_sec = np.min(gaps) * 1e-9
    gap_max_sec = np.max(gaps) * 1e-9
    gap_stdev_sec = np.std(gaps) * 1e-9
    time_gaps_entry = (f"Mean {gap_mean_sec:.1f} sec, Min {gap_min_sec:.1f} sec, "
                       f"Max {gap_max_sec:.1f} sec, StDev {gap_stdev_sec:.1f} sec")
    return time_gaps_entry


def get_ts_aligned_entry(header_timestamps_ns: np.ndarray, bag_timestamps_ns: np.ndarray) -> str:
    assert header_timestamps_ns.shape[0] == bag_timestamps_ns.shape[0]
    diff_ns = bag_timestamps_ns - header_timestamps_ns
    aligned = diff_ns.min() == 0 & diff_ns.max() == 0
    if aligned:
        ts_aligned_entry = "Bag timestamps are aligned with header timestamps."
    else:
        max_dev_ns = np.max(np.abs(diff_ns))
        ts_aligned_entry = f"Bag timestamps are NOT aligned with header timestamps. Max deviation: {max_dev_ns:,} ns."
    return ts_aligned_entry


def extract_image_message_information(reader: AnyReader, connections: List[Connection]) -> \
        Tuple[np.ndarray, np.ndarray, int, int, str]:
    header_timestamps_ns_list: List[int] = []
    bag_timestamps_ns_list: List[int] = []
    # Start out with dummy values
    width = -1
    height = -1
    encoding = ""
    for i, (connection, timestamp, rawdata) in enumerate(reader.messages(connections=connections)):
        msg = cast(sensor_msgs__msg__Image, reader.deserialize(rawdata, connection.msgtype))
        if i == 0:
            width = msg.width
            height = msg.height
            encoding = msg.encoding
        else:
            assert msg.width == width
            assert msg.height == height
            assert msg.encoding == encoding
        header_timestamps_ns_list.append(msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec)
        bag_timestamps_ns_list.append(timestamp)
    # Check that the dummy values were overwritten
    assert width != -1
    assert height != -1
    assert encoding != ""
    # Convert to numpy arrays
    header_timestamps_ns = np.array(header_timestamps_ns_list, dtype=np.int64)
    bag_timestamps_ns = np.array(bag_timestamps_ns_list, dtype=np.int64)
    return header_timestamps_ns, bag_timestamps_ns, width, height, encoding


########################################################################################################################


def get_lr_ts_aligned_entry(left_header_timestamps_ns: np.ndarray, right_headers_timestamps_ns: np.ndarray) -> str:
    assert left_header_timestamps_ns.shape[0] == right_headers_timestamps_ns.shape[0]
    diff_ns = right_headers_timestamps_ns - left_header_timestamps_ns
    aligned = diff_ns.min() == 0 & diff_ns.max() == 0
    if aligned:
        ts_aligned_entry = "Left and right header timestamps are aligned."
    else:
        max_dev_ns = np.max(np.abs(diff_ns))
        ts_aligned_entry = f"Left and right header timestamps NOT are aligned. Max deviation: {max_dev_ns:,} ns."
    return ts_aligned_entry


########################################################################################################################


def print_cam_info_topic_details(topic: str, reader: AnyReader, col_width: int, indent: str = "") -> np.ndarray:
    topic_info = reader.topics[topic]
    assert len(topic_info.connections) == 1, "Only one connection per topic currently supported."
    connection = topic_info.connections[0]

    # Extract cam info message information
    header_timestamps_ns, bag_timestamps_ns, cam_info_entry = \
        extract_cam_info_message_information(reader, [connection])
    duration = get_duration_entry(header_timestamps_ns)
    start_time = get_start_time_entry(header_timestamps_ns)
    end_time = get_end_time_entry(header_timestamps_ns)
    time_gaps = get_time_gaps_entry(header_timestamps_ns)
    ts_aligned = get_ts_aligned_entry(header_timestamps_ns, bag_timestamps_ns)

    print(f"{indent}{'Topic:':<{col_width}}{connection.topic}")
    print(f"{indent}{'Msg type:':<{col_width}}{connection.msgtype}")
    if cam_info_entry != "":
        print(f"{indent}{'Calibration:':<{col_width}}{cam_info_entry}")
    print(f"{indent}{'Num messages:':<{col_width}}{connection.msgcount}")
    print(f"{indent}{'Duration:':<{col_width}}{duration}")
    print(f"{indent}{'Start time:':<{col_width}}{start_time}")
    print(f"{indent}{'End time:':<{col_width}}{end_time}")
    print(f"{indent}{'Time gaps:':<{col_width}}{time_gaps}")
    print(f"{indent}{'Comment:':<{col_width}}{ts_aligned}")
    return header_timestamps_ns


def extract_cam_info_message_information(reader: AnyReader, connections: List[Connection]) -> \
        Tuple[np.ndarray, np.ndarray, str]:
    header_timestamps_ns_list: List[int] = []
    bag_timestamps_ns_list: List[int] = []
    cam_info_entry = ""
    for i, (connection, timestamp, rawdata) in enumerate(reader.messages(connections=connections)):
        msg = cast(sensor_msgs__msg__CameraInfo, reader.deserialize(rawdata, connection.msgtype))
        # Extract timestamps
        header_timestamps_ns_list.append(msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec)
        bag_timestamps_ns_list.append(timestamp)
        # Extract message information (from first frame only)
        if i == 1:
            cam_info_entry = construct_cam_info_entry(msg)
    # Convert to numpy arrays
    header_timestamps_ns = np.array(header_timestamps_ns_list, dtype=np.int64)
    bag_timestamps_ns = np.array(bag_timestamps_ns_list, dtype=np.int64)
    return header_timestamps_ns, bag_timestamps_ns, cam_info_entry


########################################################################################################################


def construct_cam_info_entry(msg: sensor_msgs__msg__CameraInfo) -> str:
    cam_info_entry = ""
    if hasattr(msg, "distortion_model"):
        cam_info_entry += f"Model '{msg.distortion_model}'"
        assert len(msg.K) == 9
        fx, fy = msg.K[0], msg.K[4]
        cx, cy = msg.K[2], msg.K[5]
        cam_info_entry += f", c = [{cx:.1f}, {cy:.1f}], f = [{fx:.1f}, {fy:.1f}]"
        d_arr = [round(d, 3) for d in msg.D]
        cam_info_entry += f", d = {d_arr}"
        translation_length = np.linalg.norm(msg.P.reshape(3, 4)[:, 3])
        cam_info_entry += f", |t| = {translation_length:.1f}"
    return cam_info_entry


########################################################################################################################


def print_remaining_topic_details(topic, reader: AnyReader, col_width: int, indent: str = "") -> None:
    print_cam_info_topic_details(topic, reader, col_width, indent)
    return


########################################################################################################################


def construct_stereo_image_pairs(stereo_pairs: List[Tuple[str, str]], msgtypes_by_topic: Dict[str, str]) -> \
        Tuple[List[StereoImagePairWithCamInfo], List[Tuple[str, str]]]:
    # Identify image pairs
    image_pair_indices: List[int] = []
    for pair_idx, stereo_pair in enumerate(stereo_pairs):
        if msgtypes_by_topic[stereo_pair[0]] == msgtypes_by_topic[stereo_pair[1]] == IMAGE_MSGTYPE:
            image_pair_indices.append(pair_idx)

    # Identify cam info pairs
    cam_info_pair_indices: List[int] = []
    for pair_idx, stereo_pair in enumerate(stereo_pairs):
        if msgtypes_by_topic[stereo_pair[0]] == msgtypes_by_topic[stereo_pair[1]] == CAMINFO_MSGTYPE:
            cam_info_pair_indices.append(pair_idx)

    # Match them up to construct image pair objects
    stereo_image_pairs: List[StereoImagePairWithCamInfo] = []
    used_pair_indices: List[int] = []
    for image_pair_index in image_pair_indices:
        for cam_info_pair_index in cam_info_pair_indices:
            image_pair = sorted(list(stereo_pairs[image_pair_index]))
            cam_info_pair = sorted(list(stereo_pairs[cam_info_pair_index]))
            if image_pair[0].replace("image", "imcam") == cam_info_pair[0].replace("camera_info", "imcam"):
                # Match, add to list
                stereo_image_pairs.append(
                    StereoImagePairWithCamInfo(
                        left_image_topic=image_pair[0],
                        right_image_topic=image_pair[1],
                        left_caminfo_topic=cam_info_pair[0],
                        right_caminfo_topic=cam_info_pair[1],
                        has_caminfo=True,
                    )
                )
                # Keep track of user pair indices
                used_pair_indices.append(image_pair_index)
                used_pair_indices.append(cam_info_pair_index)
            else:
                # Not a match, do what?
                pass

    # Compute list of unused stereo pairs
    all_indices = list(range(len(stereo_pairs)))
    unused_pair_indices = sorted(set(all_indices) - set(used_pair_indices))

    # Bonus: If there are unused image stereo pairs, add them with has_caminfo=False.
    unused_image_pair_indices = set(unused_pair_indices) & set(image_pair_indices)
    for unused_image_pair_index in unused_image_pair_indices:
        image_pair = sorted(list(stereo_pairs[unused_image_pair_index]))
        stereo_image_pairs.append(
            StereoImagePairWithCamInfo(
                left_image_topic=image_pair[0],
                right_image_topic=image_pair[1],
                left_caminfo_topic="",
                right_caminfo_topic="",
                has_caminfo=False,
            )
        )
        # Keep track of user pair indices
        used_pair_indices.append(unused_image_pair_index)

    # Recompute list of unused stereo pairs
    all_indices = list(range(len(stereo_pairs)))
    unused_pair_indices = sorted(set(all_indices) - set(used_pair_indices))
    unused_stereo_pairs: List[Tuple[str, str]] = [stereo_pairs[i] for i in unused_pair_indices]

    return stereo_image_pairs, unused_stereo_pairs


########################################################################################################################


def find_stereo_pairs(topics):
    # Identify pairs (as two indices)
    topics_lr = [t.replace("left", "lr").replace("right", "lr") for t in topics]
    index_pairs: List[Tuple[int, int]] = []
    for i in range(len(topics_lr)):
        for j in range(i + 1, len(topics_lr)):
            if topics_lr[i] == topics_lr[j]:
                index_pairs.append((i, j))
    # Create list of stereo pairs
    stereo_pairs: List[Tuple[str, str]] = []
    for i, j in index_pairs:
        stereo_pairs.append((topics[i], topics[j]))
    # Create list of remaining topics
    all_indices = list(range(len(topics_lr)))
    used_indices = [item for pair in index_pairs for item in pair]
    remaining_indices = sorted(set(all_indices) - set(used_indices))
    remaining_topics: List[str] = [topics[i] for i in remaining_indices]
    return stereo_pairs, remaining_topics


########################################################################################################################


if __name__ == "__main__":
    main()
