cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME FlyByWire)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Application/Src/application.c
    Application/DataAcquisition/Src/data_acquisition.c
    Application/DataAcquisition/Src/sensor_supervision.c
    Drivers/BMP390/Src/bmp390.c
    Drivers/GNSS/Src/nmea_parser.c
    Drivers/ICM20948/Src/icm20948.c
    Platform/STM32/Src/stm32_i2c_bus.c
    Platform/STM32/Src/stm32_uart_stream.c
    Platform/STM32/Src/uart_console.c
    Services/Health/Src/sensor_health.c
    Services/Airspeed/Src/airspeed.c
    Services/Barometer/Src/barometer.c
    Services/GPS/Src/gps_service.c
    Services/IMU/Src/imu.c
    Services/Telemetry/Src/mavlink_encoder.c
    Services/Telemetry/Src/mavlink_telemetry.c
    Utilities/Src/byte_ring_buffer.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Application/Inc
    Application/DataAcquisition/Inc
    Drivers/BMP390/Inc
    Drivers/GNSS/Inc
    Drivers/ICM20948/Inc
    Platform/Interfaces/Inc
    Platform/STM32/Inc
    Services/Health/Inc
    Services/Airspeed/Inc
    Services/Barometer/Inc
    Services/GPS/Inc
    Services/IMU/Inc
    Services/Telemetry/Inc
    Utilities/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Newlib-nano omits floating-point formatting unless it is requested explicitly.
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -u _printf_float
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
