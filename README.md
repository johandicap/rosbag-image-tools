# rosbag-image-tools

A set of command-line tools for rosbags containing (stereo) images.


## Rosbag Details

Show help:

```bash
python3 rosbag_details.py --help
```

Example usage:

```bash
python3 rosbag_details.py /path/to/my_rosbag.bag
```

Example output:

```
Bag details:

        Bag path:       /path/to/my_rosbag.bag
        Bag type:       ROS 1 bag
        Bag size:       11.9 MB  (12,494,689 bytes)
        Duration:       8.9 sec  (8,897,839,017 ns)
        Start time:     2023-06-08 14:37:59  (1686227879.521662205 sec)
        End time:       2023-06-08 14:38:08  (1686227888.419501213 sec)
        Num topics:     4
        Num messages:   12
        Topics:         /left/camera_info   [sensor_msgs/msg/CameraInfo]  3 msgs @ 0.3 Hz
                        /left/image         [sensor_msgs/msg/Image]       3 msgs @ 0.3 Hz
                        /right/camera_info  [sensor_msgs/msg/CameraInfo]  3 msgs @ 0.3 Hz
                        /right/image        [sensor_msgs/msg/Image]       3 msgs @ 0.3 Hz

Stereo image pairs:

    Left image:
        Topic:          /left/image
        Msg type:       sensor_msgs/msg/Image
        Resolution:     1920x1080 (WxH)
        Encoding:       bayer_rggb8
        Num messages:   3
        Duration:       8.9 sec  (8,900,753,150 ns)
        Start time:     2023-06-08 14:37:59  (1686227879.498537056 sec)
        End time:       2023-06-08 14:38:08  (1686227888.399290206 sec)
        Time gaps:      Mean 4.5 sec, Min 1.8 sec, Max 7.1 sec, StDev 2.7 sec
        Comment:        Bag timestamps are NOT aligned with header timestamps. Max deviation: 27,909,430 ns.

    Right image:
        Topic:          /right/image
        Msg type:       sensor_msgs/msg/Image
        Resolution:     1920x1080 (WxH)
        Encoding:       bayer_bggr8
        Num messages:   3
        Duration:       8.9 sec  (8,900,435,477 ns)
        Start time:     2023-06-08 14:37:59  (1686227879.498726295 sec)
        End time:       2023-06-08 14:38:08  (1686227888.399161772 sec)
        Time gaps:      Mean 4.5 sec, Min 1.8 sec, Max 7.1 sec, StDev 2.7 sec
        Comment:        Bag timestamps are NOT aligned with header timestamps. Max deviation: 30,200,369 ns.
        L/R-alignment:  Left and right header timestamps NOT are aligned. Max deviation: 222,099 ns.

    Left camera info:
        Topic:          /left/camera_info
        Msg type:       sensor_msgs/msg/CameraInfo
        Calibration:    Model 'plumb_bob', c = [808.9, 571.3], f = [1618.3, 1618.9], d = [-0.209, 0.097, 0.007, -0.002], |t| = 0.0
        Num messages:   3
        Duration:       8.9 sec  (8,900,753,150 ns)
        Start time:     2023-06-08 14:37:59  (1686227879.498537056 sec)
        End time:       2023-06-08 14:38:08  (1686227888.399290206 sec)
        Time gaps:      Mean 4.5 sec, Min 1.8 sec, Max 7.1 sec, StDev 2.7 sec
        Comment:        Bag timestamps are NOT aligned with header timestamps. Max deviation: 24,361,353 ns.

    Right camera info:
        Topic:          /right/camera_info
        Msg type:       sensor_msgs/msg/CameraInfo
        Calibration:    Model 'plumb_bob', c = [964.6, 515.0], f = [1616.0, 1615.2], d = [-0.199, 0.078, -0.001, 0.007], |t| = 8634.5
        Num messages:   3
        Duration:       8.9 sec  (8,900,435,477 ns)
        Start time:     2023-06-08 14:37:59  (1686227879.498726295 sec)
        End time:       2023-06-08 14:38:08  (1686227888.399161772 sec)
        Time gaps:      Mean 4.5 sec, Min 1.8 sec, Max 7.1 sec, StDev 2.7 sec
        Comment:        Bag timestamps are NOT aligned with header timestamps. Max deviation: 24,971,035 ns.
        L/R-alignment:  Left and right header timestamps NOT are aligned. Max deviation: 222,099 ns.


Remaining topics:

        (No remaining topics)

```
