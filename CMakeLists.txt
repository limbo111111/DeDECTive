cmake_minimum_required(VERSION 3.16)

project(DeDECTive VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(PkgConfig REQUIRED)
pkg_check_modules(HACKRF REQUIRED IMPORTED_TARGET libhackrf)

add_executable(dedective
    src/main.cpp
    src/hackrf_source.cpp
    src/packet_receiver.cpp
    src/packet_decoder.cpp
)

target_link_libraries(dedective PRIVATE PkgConfig::HACKRF)

target_include_directories(dedective PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)
