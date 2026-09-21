cmake_minimum_required(VERSION 3.16)
project(open_sprite_runtime_native LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(sprite_rt_timing src/sprite_rt_timing.cpp)
target_compile_options(sprite_rt_timing PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(sprite_rt_timing PRIVATE pthread)

add_executable(sprite_can_shadow src/sprite_can_shadow.cpp)
target_compile_options(sprite_can_shadow PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(sprite_can_shadow PRIVATE pthread)
