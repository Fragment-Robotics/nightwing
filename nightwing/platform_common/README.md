# Nightwing Platform Common

This package manages the overall configuration and system launch for the nightwing project.

## Launch Files

**base_launch.py**: The base launch file for spinning up the system. This file will include all necessary sub-component base launch files to spin up the system. This file can be use to produce delta configurations from this base. The base launch setup:
  - Realsense 435i camera component
  - Aligned RGBD topic published in the same message ``RGBD.msg``.

## Configurations

**realsense.yaml**: Realsense camera parameters. Default setup configures synced RGB+D messages as `1280x720` size and `30` fps.