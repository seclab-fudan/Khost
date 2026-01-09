#include "rt_data.h"

uint32_t RUNTIME_firmware_entry;

/// @brief global firmware info
CodeRange RUNTIME_code_range;

/// @brief global firmware state
FirmwareState RUNTIME_firmware_state __attribute__((section(".rt_dmz_data")));

/// @brief CPU context of the firmware MCU
FirmwareContext RUNTIME_firmware_context;

/// @brief state of the NVIC controller
NvicState RUNTIME_nvic_state __attribute__((section(".rt_dmz_data")));

/// @brief state of the sysctl
SysctlState RUNTIME_sysctl_state __attribute__((section(".rt_dmz_data")));

/// @brief state of the systick
EmuSystickState RUNTIME_emu_systick_state __attribute__((section(".rt_dmz_data")));

/// @brief state of the mcu
McuState RUNTIME_mcu_state;

/// @brief count of the incompatible instruction
uint32_t RUNTIME_incompatible_insn_cnt;

/// @brief list of the incompatible instruction
IncompatibleInsn RUNTIME_incompatible_insn_list[RUNTIME_INCOMPATIBLE_INSN_CNT];

/// @brief count of the raw fuzzware model
uint32_t RUNTIME_fuzzware_model_cnt;

/// @brief start position of the raw model which is not load into LUT
uint32_t RUNTIME_fuzzware_model_new_start;

/// @brief raw list of fuzzware model
FuzzwareModelInfo RUNTIME_fuzzware_model_list[RUNTIME_FUZZWARE_MODEL_CNT];

/// @brief current input position
uint32_t RUNTIME_input_cur;

/// @brief input length from the fuzzer
uint32_t RUNTIME_input_len;

/// @brief input from the fuzzer
uint8_t RUNTIME_input_buffer[RUNTIME_INPUT_BUFFER_SIZE];

/// @brief if the runtime is first run
uint32_t RUNTIME_first_run;

/// @brief load base of the runtime
uint32_t RUNTIME_load_base;

/// @brief initial stack pointer
uint32_t RUNTIME_initial_sp;

/// @brief number of total irq
uint32_t RUNTIME_num_irq __attribute__((section(".rt_dmz_data")));

/// @brief address range list
AddressRange RUNTIME_address_range[RUNTIME_ADDRESS_RANGE_SIZE];

/// @brief the init time of the firmware
uint32_t RUNTIME_init_period __attribute__((section(".rt_dmz_data")));

/// @brief control the output of the firmware
uint32_t RUNTIME_output_option __attribute__((section(".rt_dmz_data")));

/// @brief current running task in firmware
uint32_t RUNTIME_current_task;

/// @brief inital sp for tasks
uint32_t RUNTIME_per_task_initial_stack[RUNTIME_MAX_TASK_CNT];

/// @brief firmware context valid
uint32_t RUNTIME_first_crash_context_valid;

/// @brief firmware context of the first crash
uint32_t RUNTIME_first_crash_excp_id;
uint32_t RUNTIME_first_crash_task_id;
FirmwareContext RUNTIME_first_crash_context;

/// @brief if we use fuzzware model
uint32_t RUNTIME_use_fuzzware;