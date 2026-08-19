cmake_minimum_required(VERSION 3.22)
project(FlyByWireTests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

enable_testing()

add_executable(flybywire_tests
    test_main.c
    ../Drivers/GNSS/Src/nmea_parser.c
    ../Drivers/ICM20948/Src/icm20948.c
    ../Services/Health/Src/sensor_health.c
    ../Services/GPS/Src/gps_service.c
    ../Services/IMU/Src/imu.c
    ../Utilities/Src/byte_ring_buffer.c
)
target_include_directories(flybywire_tests PRIVATE
    ../Drivers/GNSS/Inc
    ../Drivers/ICM20948/Inc
    ../Platform/Interfaces/Inc
    ../Services/Health/Inc
    ../Services/GPS/Inc
    ../Services/IMU/Inc
    ../Utilities/Inc
)
if(MSVC)
  target_compile_options(flybywire_tests PRIVATE /W4)
else()
  target_compile_options(flybywire_tests PRIVATE -Wall -Wextra -Werror)
  target_link_libraries(flybywire_tests PRIVATE m)
endif()

add_test(NAME flybywire_tests COMMAND flybywire_tests)

add_executable(mavlink_encoder_tests
    test_mavlink_encoder.c
    ../Services/Telemetry/Src/mavlink_encoder.c
)
target_include_directories(mavlink_encoder_tests PRIVATE
    ../Services/Telemetry/Inc
)
if(MSVC)
  target_compile_options(mavlink_encoder_tests PRIVATE /W4 /WX)
else()
  target_compile_options(mavlink_encoder_tests PRIVATE -Wall -Wextra -Werror)
endif()
add_test(NAME mavlink_encoder_tests COMMAND mavlink_encoder_tests)
