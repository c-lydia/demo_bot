# can_drive_c

High-performance CAN driver package implemented in C++ for ROS2.

Creation date: 2026-02-28  
Last updated: 2026-02-28

## Overview

`can_drive_c` is a compiled equivalent of `can_driver`, designed to reduce Python runtime overhead and improve CAN I/O performance using SocketCAN directly from C++ (`rclcpp`).

The node executable is:

- `can_drive_c_node`

## Features

- SocketCAN read/write on `can0` at 1 Mbps
- 1 Hz CAN counters:
  - `/can_transmit_count`
  - `/can_receive_count`
  - `/can_error_count`
- Command subscriptions:
  - `/publish_motor`
  - `/publish_servo`
  - `/publish_pwm`
  - `/publish_digital_solenoid`
  - `/publish_robomaster_current`
  - `/publish_vesc`
  - `/publish_swerve`
  - `/publish_damiao`
- Feedback publishers:
  - `/encoder_feedback`
  - `/digital_analog_feedback`
  - `/robomaster_feedback`
  - `/vesc_status1`
  - `/vesc_status2`
  - `/vesc_status3`
  - `/vesc_status4`
  - `/imu/quaternion`
  - `/swerve_feedback`
  - `/brt_feedback`

## Build

From the workspace root:

```bash
colcon build --packages-select can_drive_c
```

## Run

```bash
source install/setup.bash
ros2 run can_drive_c can_drive_c_node
```

## Notes

- This package depends on `custom_messages`, `rclcpp`, `std_msgs`, and `geometry_msgs`.
- Bringing up `can0` uses:
  - `ip link show can0`
  - `sudo ip link set can0 up type can bitrate 1000000`
