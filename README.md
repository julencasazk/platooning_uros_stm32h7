# Platooning control in a STM32H7 board with micro-ROS

This code makes the STM32H7 a participant in a vehicle platoon, only in longitudinal control, no lane changes or bends.
It can act both as a follower or a leader.

## Requirements

- Tested Ubuntu 22.04 through WSL
- ARM GNU Cross Toolchain (>= v11)
- Docker
- ROS Humble (for other versions .gitmodules has to be changed, and `humble` replaced by `$ROS_DISTRO` in instructions)

## Usage

1. Clone this repo and it's submodules recursively:
```
git clone --recurse-submodules https://github.com/julencasazk/platooning_uros_stm32h7.git
```
```
```
```
```
```
```
```
```

1. From the project's root dir, clone the PID message repo from [here](https://github.com/julencasazk/pid_msg)
```
git clone https://github.com/julencasazk/pid_gains_msg.git micro_ros_stm32cubemx_utils/microros_static_library/library_generation/extra_packages/pid_gains_message
```

2. Build the micro-ROS libraries with the official Docker lib builder.
```
docker pull microros/micro_ros_static_library_builder:humble
docker run -it --rm -v $(pwd):/project --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library microros/micro_ros_static_library_builder:humble
```

You might need to use `sudo` depending on how your Docker is set up.

You should get to a user input with something like this:
```
Found CFLAGS:
-------------
-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H753xx -DPID_USE_FLOAT64 -ICore/Inc -IUSB_DEVICE/App -IUSB_DEVICE/Target -IDrivers/STM32H7xx_HAL_Driver/Inc -IDrivers/STM32H7xx_HAL_Driver/Inc/Legacy -IMiddlewares/Third_Party/FreeRTOS/Source/include -IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc -IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -IDrivers/CMSIS/Device/ST/STM32H7xx/Include -IDrivers/CMSIS/Include -ILib/pid_stm32/inc -Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include -Og -Wall -fdata-sections -ffunction-sections -g -gdwarf-2 -MMD -MP -MFprint_cflags
-------------
Do you want to continue with them? (y/n)
```

3. Build the project from the project's root dir with:
```
make all j$(nproc)
```

4. Flash the project to the STM32H7 board. I've used the STM32CubeIDE GUI programmer to do this.
