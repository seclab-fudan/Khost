cmake_minimum_required(VERSION 3.22)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/./include
    ${CMAKE_CURRENT_SOURCE_DIR}/../common
    ${CMAKE_CURRENT_SOURCE_DIR}/../engine/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../engine/runtime/host/include
)

add_executable(
    khost 
    khost.cpp
    stacktrace.cpp
    gdbstub.cpp
)

add_executable(
    khost-debug 
    khost.cpp
    stacktrace.cpp
    gdbstub.cpp
)
target_link_libraries(khost PRIVATE engine)
target_compile_definitions(khost PRIVATE KVM_OPEN_DEBUG=0)
target_link_libraries(khost-debug PRIVATE engine-debug)
target_compile_definitions(khost-debug PRIVATE KVM_OPEN_DEBUG=1)
