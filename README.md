# Platooning control in a STM32H7 board with micro-ROS

This code makes the STM32H7 a participant in a vehicle platoon, only in longitudinal control, no lane changes or bends.
It can act both as a follower or a leader.

## Requirements

- ARM GNU Cross Toolchain (>= v11)
- Docker
- ROS (tested on Humble)

## Install

1. From the project's root dir, clone the micro-ROS STM32 utils from [here](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils.git).

```
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_stm32cubemx_utils.git # Must have ROS sourced

```
