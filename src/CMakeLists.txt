cmake_minimum_required(VERSION 3.22)
project(kvm-for-mcu C CXX ASM)

add_subdirectory(engine)
add_subdirectory(runner)
add_subdirectory(fuzzer)