#!/usr/bin/env python3
"""

rosbag_thumbnail.py

Author:   Johan Musaeus Bruun
Date:     2024-05-09
License:  MIT

"""

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, cast

import cv2
import numpy as np

from rosbags.highlevel import AnyReader
from rosbags.highlevel.anyreader import AnyReaderError
from rosbags.image import message_to_cvimage
from rosbags.typesys import Stores, get_typestore
from rosbags.typesys.stores.ros1_noetic import sensor_msgs__msg__Image


########################################################################################################################


IMAGE_MSGTYPE = "sensor_msgs/msg/Image"
CAMINFO_MSGTYPE = "sensor_msgs/msg/CameraInfo"


########################################################################################################################

@dataclass
class RosbagThumbnailParameters:
    rosbag_path: Path
    topic: str
    frame: int
    pct: int
    width: int
    height: int
    format: str
    verbose: bool

    @staticmethod
    def from_cli_args(argv: List[str]) -> "RosbagThumbnailParameters":
        description = "Create a thumbnail of an image topic in a rosbag."
        epilog = "Copyright \N{COPYRIGHT SIGN} Johan Musaeus Bruun, 2024. License: MIT."
        parser = argparse.ArgumentParser(description=description, epilog=epilog)
        parser.add_argument("rosbag_path", nargs="?", type=Path, help="Rosbag to save a thumbnail of.")
        parser.add_argument("-t", "--topic", default="", type=str,
                            help="Image topic to export. Defaults to first image topic found in the bag.")
        parser.add_argument("-f", "--frame", default=1, type=int,
                            help="Image frame to export. Defaults to the first frame in the chosen image topic.")
        parser.add_argument("-p", "--pct", default=-1, type=int,
                            help="Thumbnail percentage of original image size. Alternatively specify width and height.")
        parser.add_argument("--width", default=-1, type=int,
                            help="Width of thumbnail in pixels. Alternatively specify scaling percentage (--pct).")
        parser.add_argument("--height", default=-1, type=int,
                            help="Height of thumbnail in pixels. Alternatively specify scaling percentage (--pct).")
        parser.add_argument("--format", default="jpg", type=str, help="Thumbnail image format, either 'jpg' or 'png'.")
        parser.add_argument("-v", "--verbose", action="store_true", help="show verbose output")
        args = parser.parse_args(argv)

        # If no rosbag path is given, print the help message and exit
        if args.rosbag_path is None:
            parser.print_help()
            sys.exit(1)

        # Prepare parameters object
        params = RosbagThumbnailParameters(
            rosbag_path=args.rosbag_path,
            topic=args.topic,
            frame=args.frame,
            pct=args.pct,
            width=args.width,
            height=args.height,
            format=args.format,
            verbose=args.verbose,
        )
        return params


########################################################################################################################


def main():
    print("####################")
    print("# Rosbag Thumbnail #")
    print("####################")
    print("")
    # Obtain parameters from CLI
    params = RosbagThumbnailParameters.from_cli_args(sys.argv[1:])
    # Export rosbag thumbnail
    export_rosbag_thumbnail_and_handle_exceptions(params)
    return


########################################################################################################################


def export_rosbag_thumbnail_and_handle_exceptions(params: RosbagThumbnailParameters) -> None:
    try:
        export_rosbag_thumbnail(params)
    except AnyReaderError as e:
        if str(e) == "File magic is invalid.":
            print(f"ERROR: Invalid rosbag:\n  '{params.rosbag_path}'")
        else:
            print(f"ERROR: {e}")
    except (ValueError, FileNotFoundError) as e:
        print(f"ERROR: {e}")
    print("")


########################################################################################################################


def export_rosbag_thumbnail(params: RosbagThumbnailParameters) -> None:
    # Expand a potential tilda and resolve the path
    rosbag_path = params.rosbag_path.expanduser().resolve()
    if not rosbag_path.is_file():
        raise FileNotFoundError(f"Rosbag not found: \"{rosbag_path}\"")
    
    # Create a type store
    typestore = get_typestore(Stores.ROS1_NOETIC)

    # Open bag for reading
    with AnyReader([rosbag_path], default_typestore=typestore) as reader:
        
        # Select topic
        selected_topic = select_image_topic(reader, params.topic)
        
        # Extract RGB image
        rgb_image = extract_rgb_image_from_desired_frame(reader, selected_topic, params.frame)
    
    # Downscale RGB image
    rgb_thumbnail = downscale_rgb_image(rgb_image, params.pct, params.width, params.height)
    
    # Determine output path
    thumbnail_path = determine_output_path(params.rosbag_path, params.format)

    # Save image
    save_thumbnail_image(rgb_thumbnail, thumbnail_path, )
    print(f"Thumbnail image ('{selected_topic}', frame {params.frame}) saved to:\n  '{thumbnail_path}'")


########################################################################################################################
# TODO: Consider moving all functions below to utils
########################################################################################################################


def select_image_topic(reader: AnyReader, desired_topic: str = "") -> str:

    # Identify image topics
    image_topics = [topic for topic, topic_info in reader.topics.items() if topic_info.msgtype == IMAGE_MSGTYPE]

    # If no image topics are found in the bag, raise an exception
    if len(image_topics) == 0:
        raise ValueError(f"No image topics found in the given bag:\n  '{reader.paths[0]}'")

    # If desired topic is empty, choose first image topic
    if desired_topic == "":
        return image_topics[0]

    # Else, if the desired topic is not among the image topics, raise an exception
    if desired_topic not in image_topics:
        available_image_topics = "Available image topics: ['{}']".format("', '".join(image_topics))
        raise ValueError(f"No image topic called '{desired_topic}' was found in the bag. {available_image_topics}")

    return desired_topic


########################################################################################################################


def extract_rgb_image_from_desired_frame(reader: AnyReader, selected_topic: str, desired_frame: int) -> np.ndarray:
    
    # Extract topic information
    topic_info = reader.topics[selected_topic]
    assert topic_info.msgtype == IMAGE_MSGTYPE
    
    # Verify that the chosen topic contains messages
    if topic_info.msgcount == 0:
        raise ValueError(f"The selected image topic '{selected_topic}' contains no messages.")

    # Initialize RGB image as a dummy array (with length 0)
    rgb_image = np.array([])

    # Obtain the connection corresponding to the chosen topic
    assert len(topic_info.connections) == 1
    connection = topic_info.connections[0]

    # Loop over frames in the chosen topic
    # Note that the frame number is here defined to start at 1, so that the first frame is denoted "frame 1".
    for frame_number, (connection, timestamp, rawdata) in enumerate(reader.messages(connections=[connection]), start=1):
        if frame_number < desired_frame:
            continue
        # Parse image message
        msg = cast(sensor_msgs__msg__Image, reader.deserialize(rawdata, connection.msgtype))
        # Extract RGB image
        rgb_image = message_to_cvimage(msg, "rgb8")
        break

    if len(rgb_image) == 0:
        num_frames_str = f"Only {topic_info.msgcount} frames available."
        raise ValueError(f"No RGB image found for desired frame number '{desired_frame}'. {num_frames_str}")

    # Verify that a color image was found
    assert rgb_image.ndim == 3
    assert rgb_image.shape[2] == 3

    return rgb_image


########################################################################################################################


def downscale_rgb_image(rgb_image: np.ndarray, pct: int, width: int, height: int) -> np.ndarray:

    # Verify that a color image was provided
    assert rgb_image.ndim == 3
    assert rgb_image.shape[2] == 3
    
    # Verify that the user only specified one type of scaling
    if pct > 0 and (width > 0 or height > 0):
        raise ValueError("Please specify either --pct or --width/--height.")

    # Extract dimensions of the full image
    full_height, full_width, depth = rgb_image.shape
    
    # Handle various scaling cases
    if pct > 0:
        chosen_width = int(round(pct / 100.0 * full_width))
        chosen_height = int(round(pct / 100.0 * full_height))
    elif width > 0 and height > 0:
        chosen_width = width
        chosen_height = height
    elif width > 0:
        # Scale height along with width to maintain aspect ratio
        chosen_width = width
        chosen_height = int(round(width / full_width * full_height))
    elif height > 0:
        # Scale width along with height to maintain aspect ratio
        chosen_width = int(round(height / full_height * full_width))
        chosen_height = height
    else:
        chosen_width = full_width
        chosen_height = full_height

    # Perform actual scaling
    downscaled_rgb_image = cv2.resize(rgb_image, (chosen_width, chosen_height), interpolation=cv2.INTER_AREA)
    
    # TODO: If verbose, write what scaling took place!
    
    return downscaled_rgb_image


########################################################################################################################


def determine_output_path(rosbag_path: Path, image_format: str) -> Path:
    if image_format not in ["jpg", "png"]:
        raise ValueError(f"Image format should be either 'jpg' or 'png', not '{image_format}'.")
    output_path = rosbag_path.with_suffix(f".{image_format}")
    return output_path


########################################################################################################################


def save_thumbnail_image(rgb_thumbnail: np.ndarray, thumbnail_path: Path) -> None:
    bgr_image = cv2.cvtColor(rgb_thumbnail, cv2.COLOR_RGB2BGR)
    if not cv2.imwrite(str(thumbnail_path), bgr_image):
        raise IOError(f"Failed to save thumbnail image to '{thumbnail_path}'. Do you have write permissions?")


########################################################################################################################


if __name__ == "__main__":
    main()
