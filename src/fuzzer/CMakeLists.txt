cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 99)

set(HASH "\#")
set(CMAKE_INSTALL_PREFIX "/usr/local" CACHE PATH "Installation prefix")
set(BIN_PATH "${CMAKE_INSTALL_PREFIX}/bin")
set(HELPER_PATH "${CMAKE_INSTALL_PREFIX}/lib/afl")
set(DOC_PATH "${CMAKE_INSTALL_PREFIX}/share/doc/afl")

# get system information and show to user
execute_process(
    COMMAND uname -s
    OUTPUT_VARIABLE SYS
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
execute_process(
    COMMAND uname -m
    OUTPUT_VARIABLE ARCH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
message(STATUS "Compiling AFL++ for OS ${SYS} on ARCH ${ARCH}")

# set compile options
add_compile_options(
    -g -O2 -Wall -Wextra -Wno-pointer-sign -Wno-pointer-arith -Wno-variadic-macros
    -Wno-format-truncation -Wno-format-truncation -fPIC 
    -DKVM_HACK=1
    -DAFL_PATH="${HELPER_PATH}" 
    -DBIN_PATH="${BIN_PATH}" 
    -DDOC_PATH="${DOC_PATH}"
    -DPYTHON_OK=0
    -DPYFLAGS=""
    -DPYTHON_INCLUDE="/"
    -DIN_REPO=0
    -DSHMAT_OK=1
    -DAFL_NO_X86=1
)

# set include dir and common files
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../common
    ${CMAKE_CURRENT_SOURCE_DIR}/../engine/include
)
set(AFL_FUZZ_FILES
    afl-fuzz-bitmap.c
    afl-fuzz-cmplog.c
    afl-fuzz-extras.c
    afl-fuzz-init.c
    afl-fuzz-mutators.c
    afl-fuzz-one.c
    afl-fuzz-python.c
    afl-fuzz-queue.c
    afl-fuzz-redqueen.c
    afl-fuzz-run.c
    afl-fuzz-sanfuzz.c
    afl-fuzz-skipdet.c
    afl-fuzz-state.c
    afl-fuzz-stats.c
    afl-fuzz-statsd.c
    afl-fuzz.c
)
set(COMM_HDR
    ${CMAKE_CURRENT_SOURCE_DIR}/include/alloc-inl.h 
    ${CMAKE_CURRENT_SOURCE_DIR}/include/config.h 
    ${CMAKE_CURRENT_SOURCE_DIR}/include/debug.h 
    ${CMAKE_CURRENT_SOURCE_DIR}/include/types.h
)

# add objects of AFL++ and set link libraries
add_library(afl-performance OBJECT 
    ${COMM_HDR} 
    afl-performance.c 
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hash.h
)
add_library(afl-common OBJECT 
    ${COMM_HDR} 
    afl-common.c 
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common.h
)
add_library(afl-forkserver OBJECT 
    ${COMM_HDR} 
    afl-forkserver.c 
    ${CMAKE_CURRENT_SOURCE_DIR}/include/forkserver.h
)
target_link_libraries(afl-forkserver PUBLIC fuzz_engine)
add_library(afl-sharedmem OBJECT
    ${COMM_HDR} 
    afl-sharedmem.c 
    ${CMAKE_CURRENT_SOURCE_DIR}/include/sharedmem.h
)
add_executable(khost-afl-fuzz 
    ${COMM_HDR} 
    ${CMAKE_CURRENT_SOURCE_DIR}/include/afl-fuzz.h 
    ${AFL_FUZZ_FILES}
    $<TARGET_OBJECTS:afl-performance>
    $<TARGET_OBJECTS:afl-common>
    $<TARGET_OBJECTS:afl-forkserver>
    $<TARGET_OBJECTS:afl-sharedmem>
)
# add_executable(kvm-afl-showmap 
#     ${COMM_HDR}
#     afl-showmap.c 
#     afl-fuzz-mutators.c 
#     afl-fuzz-python.c 
#     $<TARGET_OBJECTS:afl-performance>
#     $<TARGET_OBJECTS:afl-common>
#     $<TARGET_OBJECTS:afl-forkserver>
#     $<TARGET_OBJECTS:afl-sharedmem>
# )
# add_executable(kvm-afl-tmin 
#     ${COMM_HDR}
#     afl-tmin.c 
#     $<TARGET_OBJECTS:afl-performance>
#     $<TARGET_OBJECTS:afl-common>
#     $<TARGET_OBJECTS:afl-forkserver>
#     $<TARGET_OBJECTS:afl-sharedmem>
# )
# add_executable(kvm-afl-analyze 
#     ${COMM_HDR}
#     afl-analyze.c 
#     $<TARGET_OBJECTS:afl-performance>
#     $<TARGET_OBJECTS:afl-common>
#     $<TARGET_OBJECTS:afl-forkserver>
#     $<TARGET_OBJECTS:afl-sharedmem>
# )
# add_executable(kvm-afl-gotcpu
#     ${COMM_HDR}
#     afl-gotcpu.c
#     $<TARGET_OBJECTS:afl-common>
# )
set(COMM_LIBS dl rt m engine)
target_link_libraries(khost-afl-fuzz ${COMM_LIBS})
# target_link_libraries(khost-afl-showmap ${COMM_LIBS})
# target_link_libraries(khost-afl-tmin ${COMM_LIBS})
# target_link_libraries(khost-afl-analyze ${COMM_LIBS})
# target_link_libraries(khost-afl-gotcpu ${COMM_LIBS})
