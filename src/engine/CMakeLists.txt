# CMakeList for engine of khost project

cmake_minimum_required(VERSION 3.22)

option(USE_APT "usage of the APT" ON)
if(NOT USE_APT)
    message(WARNING "APT is disabled. This may cause alignment abort.")
endif()

# always use release build type to build engine
set(CMAKE_BUILD_TYPE "Release")

# engine is a pure C++ project
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

# enable -O2 for engine
add_compile_options("-O2" "-Wall")

# runtime code is auto generated and placed in autogen_runtime.cpp
set(AUTOGEN_PARA_RT ${CMAKE_CURRENT_SOURCE_DIR}/runtime/host/autogen_rt_para_normal.cpp)
set(AUTOGEN_FULL_RT ${CMAKE_CURRENT_SOURCE_DIR}/runtime/host/autogen_rt_full_normal.cpp)
set(AUTOGEN_PARA_RT_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/runtime/host/autogen_rt_para_debug.cpp)
set(AUTOGEN_FULL_RT_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/runtime/host/autogen_rt_full_debug.cpp)

# engine need runtime to be compiled first
# we use python helper script
set(RUNTIME_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/runtime/guest/CMakeLists.txt
    ${CMAKE_CURRENT_SOURCE_DIR}/runtime/rt_compile.py
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/global.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/rt_global.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/data_def.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/data_offset.h
    ${CMAKE_CURRENT_SOURCE_DIR}/runtime/guest/include/rt_cpu.h
    ${CMAKE_CURRENT_SOURCE_DIR}/runtime/guest/include/rt_data.h
    ${CMAKE_CURRENT_SOURCE_DIR}/runtime/guest/include/rt_debug.h
    ${CMAKE_CURRENT_SOURCE_DIR}/runtime/guest/include/rt_fuzzware.h
    ${CMAKE_CURRENT_SOURCE_DIR}/runtime/guest/include/rt_handler.h
    ${CMAKE_CURRENT_SOURCE_DIR}/runtime/guest/include/rt_nvic.h
    ${CMAKE_CURRENT_SOURCE_DIR}/runtime/guest/include/rt_sysctl.h
    ${CMAKE_CURRENT_SOURCE_DIR}/runtime/guest/include/rt_systick.h
    ${CMAKE_CURRENT_SOURCE_DIR}/runtime/guest/include/rt_emu.h
    ${CMAKE_CURRENT_SOURCE_DIR}/runtime/guest/include/rt_trigger.h
    ${CMAKE_CURRENT_SOURCE_DIR}/runtime/guest/include/rt_fault.h
    ${CMAKE_CURRENT_SOURCE_DIR}/runtime/guest/rt_link.lds
    ${CMAKE_CURRENT_SOURCE_DIR}/runtime/guest/rt_data.c
    ${CMAKE_CURRENT_SOURCE_DIR}/runtime/guest/rt_debug.c
    ${CMAKE_CURRENT_SOURCE_DIR}/runtime/guest/rt_handler.c
    ${CMAKE_CURRENT_SOURCE_DIR}/runtime/guest/rt_handler_asm.S
    ${CMAKE_CURRENT_SOURCE_DIR}/runtime/guest/rt_startup_asm.S
    ${CMAKE_CURRENT_SOURCE_DIR}/runtime/guest/rt_fuzzware.c
    ${CMAKE_CURRENT_SOURCE_DIR}/runtime/guest/rt_cpu.c
    ${CMAKE_CURRENT_SOURCE_DIR}/runtime/guest/rt_nvic.c
    ${CMAKE_CURRENT_SOURCE_DIR}/runtime/guest/rt_systick.c
    ${CMAKE_CURRENT_SOURCE_DIR}/runtime/guest/rt_sysctl.c
    ${CMAKE_CURRENT_SOURCE_DIR}/runtime/guest/rt_emu.c
    ${CMAKE_CURRENT_SOURCE_DIR}/runtime/guest/rt_trigger.c
    ${CMAKE_CURRENT_SOURCE_DIR}/runtime/guest/rt_fault.c
    ${CMAKE_CURRENT_SOURCE_DIR}/runtime/guest/rt_harness.c
)
add_custom_command(
    OUTPUT ${AUTOGEN_FULL_RT}
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/runtime/rt_compile.py full normal ${USE_APT} 
	COMMENT "Running runtime compiler for PROFILE=FULL VERSION=normal APT=${USE_APT}"
    DEPENDS ${RUNTIME_FILES}
)
add_custom_target(full_runtime DEPENDS ${AUTOGEN_FULL_RT})

add_custom_command(
    OUTPUT ${AUTOGEN_FULL_RT_DEBUG}
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/runtime/rt_compile.py full debug ${USE_APT}
	COMMENT "Running runtime compiler for PROFILE=FULL VERSION=debug APT=${USE_APT}"
    DEPENDS ${RUNTIME_FILES}
)
add_custom_target(full_runtime_debug DEPENDS ${AUTOGEN_FULL_RT_DEBUG})

add_custom_command(
    OUTPUT ${AUTOGEN_PARA_RT}
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/runtime/rt_compile.py para normal ${USE_APT}
	COMMENT "Running runtime compiler for PROFILE=PARA VERSION=normal APT=${USE_APT}"
    DEPENDS ${RUNTIME_FILES}
)
add_custom_target(para_runtime DEPENDS ${AUTOGEN_PARA_RT})

add_custom_command(
	OUTPUT ${AUTOGEN_PARA_RT_DEBUG}
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/runtime/rt_compile.py para debug ${USE_APT}
	COMMENT "Running runtime compiler for PROFILE=PARA VERSION=debug APT=${USE_APT}"
    DEPENDS ${RUNTIME_FILES}
)
add_custom_target(para_runtime_debug DEPENDS ${AUTOGEN_PARA_RT_DEBUG})

# export source codes of engine
add_library(engine STATIC
    # global
    en_config.cpp
    # engine
    lib.cpp
    en_global.cpp
    en_board.cpp
    en_board_full.cpp
    en_board_para.cpp
    en_cpu.cpp
    en_cpu_full.cpp
    en_cpu_para.cpp
    # device
    device/dev_nvic.cpp
    device/dev_mmio.cpp
    device/dev_sysctl.cpp
    device/dev_systick.cpp
    device/dev_fuzzware_model.cpp
    # loader
    loader/loader.cpp
    loader/trampoline.cpp
    loader/disasm.cpp
    loader/coverage.cpp
    # runtime
    ${AUTOGEN_FULL_RT}
    ${AUTOGEN_PARA_RT}
)

# export source codes of engine
add_library(engine-debug STATIC
    # global
    en_config.cpp
    # engine
    lib.cpp
    en_global.cpp
    en_board.cpp
    en_board_full.cpp
    en_board_para.cpp
    en_cpu.cpp
    en_cpu_full.cpp
    en_cpu_para.cpp
    # device
    device/dev_nvic.cpp
    device/dev_mmio.cpp
    device/dev_sysctl.cpp
    device/dev_systick.cpp
    device/dev_fuzzware_model.cpp
    # loader
    loader/loader.cpp
    loader/trampoline.cpp
    loader/disasm.cpp
    loader/coverage.cpp
    # runtime
    ${AUTOGEN_FULL_RT_DEBUG}
    ${AUTOGEN_PARA_RT_DEBUG}
)

# engine object needs runtime with same profile
add_dependencies(engine full_runtime para_runtime)
add_dependencies(engine-debug full_runtime_debug para_runtime_debug)

# add compile defines
if(NOT USE_APT)
    target_compile_definitions(engine PRIVATE "DISABLE_APT")
    target_compile_definitions(engine-debug PRIVATE "DISABLE_APT")
endif()

# export headers of engine
target_include_directories(engine PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../common
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/loader/include
    ${CMAKE_CURRENT_SOURCE_DIR}/device/include
    ${CMAKE_CURRENT_SOURCE_DIR}/runtime/guest/include
    ${CMAKE_CURRENT_SOURCE_DIR}/runtime/host/include
)

target_include_directories(engine-debug PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../common
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/loader/include
    ${CMAKE_CURRENT_SOURCE_DIR}/device/include
    ${CMAKE_CURRENT_SOURCE_DIR}/runtime/guest/include
    ${CMAKE_CURRENT_SOURCE_DIR}/runtime/host/include
)

target_link_libraries(engine stdc++ keystone capstone yaml-cpp)
target_compile_definitions(engine PRIVATE KVM_OPEN_DEBUG=0)
target_link_libraries(engine-debug stdc++ keystone capstone yaml-cpp)
target_compile_definitions(engine-debug PRIVATE KVM_OPEN_DEBUG=1)

install(TARGETS engine DESTINATION .)
install(TARGETS engine-debug DESTINATION .)