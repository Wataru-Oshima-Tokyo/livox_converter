# Livox Message Converter Node

This ROS package provides a node that subscribes to LiDAR data published by `livox_ros_driver` on the `/livox/lidar` topic, converts the incoming messages from `livox_ros_driver/CustomMsg` format to `livox_ros_driver2/CustomMsg` format, and publishes the converted messages to `/livox/lidar_custom`. It utilizes OpenMP for efficient parallel processing, speeding up the conversion of large numbers of data points within each message.

## Dependencies

- ROS Melodic (other versions may work but are not tested)
- OpenMP for parallel processing
- `livox_ros_driver` package
- `livox_ros_driver2` package

Ensure these dependencies are properly installed and configured in your ROS environment before proceeding.

## Installation

1. Navigate to your catkin workspace's `src` directory (e.g., `~/catkin_ws/src`):

```bash
cd ~/catkin_ws/src
```

2. Clone this package into your workspace (assuming this package is available in a Git repository; adjust commands if downloading or copying from another source):

```bash
git clone <repository_url>
```

3. Navigate back to the root of your catkin workspace and compile:

```bash
cd ~/catkin_ws
catkin_make
```

Don't forget to source your workspace's setup script:

```bash
source devel/setup.bash
```

## Usage

To run the `livox_converter_node`, use the following command:

```bash
rosrun <package_name> livox_converter_node
```

Replace `<package_name>` with the name of this package in your workspace.

## Node Information

### Subscriptions

- `/livox/lidar`: The node subscribes to this topic expecting messages of type `livox_ros_driver/CustomMsg`.

### Publications

- `/livox/lidar_custom`: The node publishes converted messages of type `livox_ros_driver2/CustomMsg` to this topic.

### Parameters

This node does not use ROS parameters for configuration.

## Known Issues & Limitations

- This package has been tested with ROS Melodic on Ubuntu 18.04. Compatibility with other ROS versions or operating systems may require adjustments.
- The efficiency gain from parallel processing using OpenMP may vary depending on the specific hardware configuration and the size of the data being processed.

## Contributing

Contributions to improve this package are welcome. Please submit pull requests or report issues through the project's GitHub repository.

