cmake_minimum_required(VERSION 3.22)
project(FlyByWireTests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

enable_testing()

add_executable(flybywire_tests
    test_main.c
    ../Core/Src/byte_ring_buffer.c
    ../Core/Src/icm20948.c
    ../Core/Src/imu.c
    ../Core/Src/neo6m.c
    ../Core/Src/sensor_health.c
)
target_include_directories(flybywire_tests PRIVATE ../Core/Inc)
if(MSVC)
  target_compile_options(flybywire_tests PRIVATE /W4)
else()
  target_compile_options(flybywire_tests PRIVATE -Wall -Wextra -Werror)
  target_link_libraries(flybywire_tests PRIVATE m)
endif()

add_test(NAME flybywire_tests COMMAND flybywire_tests)
