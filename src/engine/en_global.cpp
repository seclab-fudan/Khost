#include "en_global.h"

namespace khost {

////////////////////////////////////////////////////////////////////////////////
// Registers
////////////////////////////////////////////////////////////////////////////////

char ARMv7M_UNKNOWN_name[]     = "UNKNOWN";
char ARMv7M_APSR_name[]        = "APSR";
char ARMv7M_IAPSR_name[]       = "IAPSR";
char ARMv7M_EAPSR_name[]       = "EAPSR";
char ARMv7M_XPSR_name[]        = "XPSR";
char ARMv7M_IPSR_name[]        = "IPSR";
char ARMv7M_EPSR_name[]        = "EPSR";
char ARMv7M_IEPSR_name[]       = "IEPSR";
char ARMv7M_MSP_name[]         = "MSP";
char ARMv7M_PSP_name[]         = "PSP";
char ARMv7M_PRIMASK_name[]     = "PRIMASK";
char ARMv7M_BASERPI_name[]     = "BASERPI";
char ARMv7M_BASERPI_MAX_name[] = "BASERPI_MAX";
char ARMv7M_FAULTMASK_name[]   = "FAULTMASK";
char ARMv7M_CONTROL_name[]     = "CONTROL";

char *sysreg_name(int id) {
    switch (id)
    {
    case ARMv7M_APSR:
        return ARMv7M_APSR_name;
    case ARMv7M_IAPSR:
        return ARMv7M_IAPSR_name;
    case ARMv7M_EAPSR:
        return ARMv7M_EAPSR_name;
    case ARMv7M_XPSR:
        return ARMv7M_XPSR_name;
    case ARMv7M_IPSR:
        return ARMv7M_IPSR_name;
    case ARMv7M_EPSR:
        return ARMv7M_EPSR_name;
    case ARMv7M_IEPSR:
        return ARMv7M_IEPSR_name;
    case ARMv7M_MSP:
        return ARMv7M_MSP_name;
    case ARMv7M_PSP:
        return ARMv7M_PSP_name;
    case ARMv7M_PRIMASK:
        return ARMv7M_PRIMASK_name;
    case ARMv7M_BASERPI:
        return ARMv7M_BASERPI_name;
    case ARMv7M_BASERPI_MAX:
        return ARMv7M_BASERPI_MAX_name;
    case ARMv7M_FAULTMASK:
        return ARMv7M_FAULTMASK_name;
    case ARMv7M_CONTROL:
        return ARMv7M_CONTROL_name;
    default:
        return ARMv7M_UNKNOWN_name;
    }
}

char ARMv7M_APSR_invalid_name[] = "invalid";
char ARMv7M_APSR_nzcvq_name[]   = "nzcvq";
char ARMv7M_APSR_g_name[]       = "g";
char ARMv7M_APSR_nzcvqg_name[]  = "nzcvqg";

char *mask_name(int id) {
    switch (id)
    {
    case ARMv7M_APSR_nzcvq:
        return ARMv7M_APSR_nzcvq_name;
    case ARMv7M_APSR_nzcvqg:
        return ARMv7M_APSR_nzcvqg_name;
    case ARMv7M_APSR_g:
        return ARMv7M_APSR_g_name;
    default:
        return ARMv7M_APSR_invalid_name;
    }
}

uint64_t general_reg_id(int id) {
    if (id >= 0 && id <= 14) {
        return 0x6030000000100000ll + id * 0x2ll;
    }
    if (id == 15) {
        return ARMv8A_PC;
    }
    return 0;
}

std::mutex print_mutex;

}  // namespace khost