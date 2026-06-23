#!/usr/bin/env python3

# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: Apache-2.0

# Launches static transforms from the CARLA world origin to supported UTM frames.

import launch
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the carla converter launch description."""

    transform_utm_31N = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["-166021.443", "0", "0", "0", "0", "0", "world_origin", "utm_31N"],
        parameters=[{"use_sim_time": True}],
    )
    transform_utm_31S = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["-166021.443", "-10000000.000", "0", "0", "0", "0", "world_origin", "utm_31S"],
        parameters=[{"use_sim_time": True}],
    )
    transform_utm_30N = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["-833978.557", "0", "0", "0", "0", "0", "world_origin", "utm_30N"],
        parameters=[{"use_sim_time": True}],
    )
    transform_utm_30S = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["-833978.557", "-10000000.000", "0", "0", "0", "0", "world_origin", "utm_30S"],
        parameters=[{"use_sim_time": True}],
    )

    return launch.LaunchDescription([transform_utm_31N, transform_utm_31S, transform_utm_30N, transform_utm_30S])


if __name__ == "__main__":
    generate_launch_description()
