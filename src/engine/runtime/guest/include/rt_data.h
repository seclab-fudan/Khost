#ifndef RT_DATA_H
#define RT_DATA_H

#include "data_def.h"

extern uint32_t RUNTIME_firmware_entry;

extern CodeRange RUNTIME_code_range;

extern FirmwareState RUNTIME_firmware_state;

extern FirmwareContext RUNTIME_firmware_context;

extern NvicState RUNTIME_nvic_state;

extern SysctlState RUNTIME_sysctl_state;

extern EmuSystickState RUNTIME_emu_systick_state;

extern McuState RUNTIME_mcu_state;

extern uint32_t RUNTIME_incompatible_insn_cnt;

extern IncompatibleInsn RUNTIME_incompatible_insn_list[RUNTIME_INCOMPATIBLE_INSN_CNT];

extern uint32_t RUNTIME_fuzzware_model_cnt;

extern uint32_t RUNTIME_fuzzware_model_new_start;

extern FuzzwareModelInfo RUNTIME_fuzzware_model_list[RUNTIME_FUZZWARE_MODEL_CNT];

extern uint32_t RUNTIME_input_cur;

extern uint32_t RUNTIME_input_len;

extern uint8_t RUNTIME_input_buffer[RUNTIME_INPUT_BUFFER_SIZE];

extern uint32_t RUNTIME_first_run;

extern uint32_t RUNTIME_load_base;

extern uint32_t RUNTIME_initial_sp;

extern uint32_t RUNTIME_num_irq;

extern AddressRange RUNTIME_address_range[RUNTIME_ADDRESS_RANGE_SIZE];

extern uint32_t RUNTIME_init_period;

extern uint32_t RUNTIME_output_option;

extern uint32_t RUNTIME_current_task;

extern uint32_t RUNTIME_per_task_initial_stack[RUNTIME_MAX_TASK_CNT];

extern uint32_t RUNTIME_first_crash_context_valid;

extern uint32_t RUNTIME_first_crash_excp_id;
extern uint32_t RUNTIME_first_crash_task_id;
extern FirmwareContext RUNTIME_first_crash_context;

extern uint32_t RUNTIME_use_fuzzware;

#endif  // RT_DATA_H