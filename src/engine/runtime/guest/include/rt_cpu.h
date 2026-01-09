#ifndef RT_CPU_H
#define RT_CPU_H

#include "rt_global.h"
#include "rt_data.h"
#include "rt_debug.h"
#include "rt_sysctl.h"
#include "rt_nvic.h"

void cpu_reset();

/// @brief gets value of IPSR
/// @return value of the IPSR
inline uint32_t cpu_get_ipsr() {
    return RUNTIME_mcu_state.ipsr;
}
inline uint32_t cpu_get_ipsr() __attribute__((always_inline));

/// @brief gets value of CONTROL
/// @return value of CONTROL
inline uint32_t cpu_get_control() {
    return RUNTIME_mcu_state.control;
}
inline uint32_t cpu_get_control() __attribute__((always_inline));

/// @brief gets value of PRIMASK
/// @return value of PRIMASK
inline uint32_t cpu_get_primask() {
    return RUNTIME_mcu_state.primask;
}
inline uint32_t cpu_get_primask() __attribute__((always_inline));

/// @brief gets value of BASEPRI
/// @return value of BASEPRI
inline uint32_t cpu_get_basepri() {
    return RUNTIME_mcu_state.basepri;
}
inline uint32_t cpu_get_basepri() __attribute__((always_inline));

/// @brief gets value of FAULTMASK
/// @return value of FAULTMASK
inline uint32_t cpu_get_faultmask() {
    return RUNTIME_mcu_state.faultmask;
}
inline uint32_t cpu_get_faultmask() __attribute__((always_inline));

void cpu_set_control(uint32_t value);
void cpu_exception_enter(uint32_t exc_number);
void cpu_exception_return(uint32_t exc_return);
void cpu_check_irq();
void cpu_handle_incompatible_insn(uint32_t pc);

void runtime_abort();
void guest_abort();
void guest_exit();
void guest_timeout();

#endif  // RT_CPU_H