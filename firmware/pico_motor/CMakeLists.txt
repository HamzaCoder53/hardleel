# Pico motor + servo firmware (RP2040). Build on the laptop (WSL):
#   export PICO_SDK_PATH=~/pico-sdk
#   cmake -S firmware/pico_motor -B /tmp/fw_build && cmake --build /tmp/fw_build -j4
# -> /tmp/fw_build/motor_servo.uf2: hold BOOTSEL, plug the Pico in, copy the .uf2 onto the RPI-RP2 drive.
cmake_minimum_required(VERSION 3.13)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
project(hardleel_motor C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

add_executable(motor_servo motor_servo.c)
target_link_libraries(motor_servo pico_stdlib hardware_pwm hardware_clocks hardware_sync hardware_watchdog)
pico_enable_stdio_usb(motor_servo 1)
pico_enable_stdio_uart(motor_servo 0)
pico_add_extra_outputs(motor_servo)

# the firmware exactly as it ran on 25 Oct (reference; no watchdog/MIN/ZERO)
add_executable(motor_servo_orig motor_servo_orig.c)
target_link_libraries(motor_servo_orig pico_stdlib hardware_pwm hardware_clocks hardware_sync)
pico_enable_stdio_usb(motor_servo_orig 1)
pico_enable_stdio_uart(motor_servo_orig 0)
pico_add_extra_outputs(motor_servo_orig)
