# CMakeLists for runtime of kvm-for-mcu project
# note: runtime will be load into the guest memory address space and execute inside kvm

cmake_minimum_required(VERSION 3.22)
project(khost-runtime C)

# option to disable the page table, only for test
option(USE_APT "usage of the APT" ON)
if(NOT USE_APT)
    message(WARNING "APT is disabled. This may cause alignment abort.")
endif()

# always use release profile
set(CMAKE_BUILD_TYPE "Release")

# set compiler
# runtime will be compiled baremetally
enable_language(ASM)
set(CMAKE_C_COMPILER "/usr/bin/arm-none-eabi-gcc")
set(CMAKE_ASM_COMPILER "/usr/bin/arm-none-eabi-gcc")
set(CMAKE_OBJCOPY "/usr/bin/arm-none-eabi-objcopy")

# must set a sepecific profile for the runtime
if(NOT DEFINED PROFILE)
    message(FATAL_ERROR "no profile set")
endif()

# must set a sepecific profile for the runtime
if(NOT DEFINED VERSION)
    message(FATAL_ERROR "no version set")
endif()

# change cmake variable to compile defines
# [1] FULLVIRT: FullVirt Mode
# [2] PARAVIRT: ParaVirt Mode 
if("${PROFILE}" STREQUAL "full")
    set(PROFILE_DEFINE "FULLVIRT")
elseif("${PROFILE}" STREQUAL "para")
    set(PROFILE_DEFINE "PARAVIRT")
else()
    message(FATAL_ERROR "PROFILE of the runtime not known!")
endif()

# change cmake variable to version
# [1] NORMAL: NORMAL Version
# [2] DEBUG: DEBUG Version
if("${VERSION}" STREQUAL "normal")
	set(VERSION_DEFINE "KVM_OPEN_DEBUG=0")
elseif("${VERSION}" STREQUAL "debug")
	set(VERSION_DEFINE "KVM_OPEN_DEBUG=1")
else()
	message(FATAL_ERROR "VERSION of the runtime not known!")
endif()

# set runtime linker script
set(LINKER_SCRIPT "${PROJECT_SOURCE_DIR}/rt_link.lds")

# change compile options
# hack: we use -mfpu=vfp in order to compile the fp-context saving code in 
# runtime while use -mfloat-abi=softfp to avoid using of fp-regs in runtime
add_compile_options("-O2" "-Wall" "-march=armv7-a" "-mfloat-abi=softfp" "-mfpu=vfp")

# set specs to nosys
add_link_options("--specs=nosys.specs")

# generate map file of the runtime
add_link_options("-Wl,-Map=${PROFILE}_${VERSION}_runtime.map")

# set include dir
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/../../include
    ${PROJECT_SOURCE_DIR}/../../../common
)

# set source files
add_executable(${PROFILE}_${VERSION}_runtime
    rt_data.c
    rt_debug.c
    rt_handler.c
    rt_handler_asm.S
    rt_startup_asm.S
    rt_fuzzware.c
    rt_cpu.c
    rt_nvic.c
    rt_systick.c
    rt_sysctl.c
    rt_emu.c
    rt_trigger.c
    rt_fault.c
    rt_harness.c
)

# define profile
target_compile_definitions(${PROFILE}_${VERSION}_runtime PRIVATE ${PROFILE_DEFINE} ${VERSION_DEFINE})
if(NOT USE_APT)
	target_compile_definitions(${PROFILE}_${VERSION}_runtime PRIVATE "DISABLE_APT")
endif()

# link files staticly
target_link_libraries(
	${PROFILE}_${VERSION}_runtime -T${LINKER_SCRIPT} -static
)

# use objcopy to generate binary file to reduce size
add_custom_command(
	TARGET ${PROFILE}_${VERSION}_runtime
    POST_BUILD
	COMMAND ${CMAKE_OBJCOPY} ARGS -O binary ${PROFILE}_${VERSION}_runtime ${PROFILE}_${VERSION}_runtime.bin
)
