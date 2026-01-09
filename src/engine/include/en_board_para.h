#ifndef EN_BOARD_PARA_H
#define EN_BOARD_PARA_H

#if KVM_OPEN_DEBUG
    #include "autogen_rt_para_debug.h"
#else
    #include "autogen_rt_para_normal.h"
#endif

#include "en_board.h"

#include "en_cpu_para.h"
#include "dev_mmio.h"
#include "dev_nvic.h"
#include "dev_systick.h"
#include "dev_sysctl.h"
#include "dev_fuzzware_model.h"

namespace khost {

class BoardPara : public Board {
public:
    BoardPara();
    virtual ~BoardPara();

    // virtual devices
    inline CPUPara *cpu() { return _cpu; }
    inline NVIC *nvic() { return _nvic; }
    inline SysCtl *sysctl() { return _sysctl; }
    inline EmuSysTick *systick() { return _systick; }
    
    // firmware state
    virtual FirmwareState *firmware_state() override;
    virtual void reset_firmware_state() override;

    // mmio handle
    MMIORegion *find_mmio_handler(uint64_t phy_addr);

    // reset and run
    virtual bool reset(uint8_t *buffer, uint32_t size) override;
    virtual int run() override;
	bool reset_hooks();
	uint8_t *rt_buffer = 0;
	uint32_t *rt_size = 0;
	uint32_t *rt_cur = 0;

protected:
    // init operation
    virtual bool init_devices() override;
    virtual bool init_runtime_state() override;
    virtual bool init_vcpu() override;

private:
    // descriptors
    int  _gic_fd = -1;
    CPUPara *_cpu = nullptr;

    // devices
    NVIC          *_nvic           = nullptr;
    SysCtl        *_sysctl         = nullptr;
    MMIOModel     *_mpu            = nullptr;
    EmuSysTick    *_systick        = nullptr;
    FuzzwareModel *_fuzzware_model = nullptr;
    std::map<uint32_t, MMIORegionDes> _mmio_map;

    void init_page_table();
    void load_input(uint8_t *buffer, uint32_t size);
};

}  // namespace khost

#endif  // EN_BOARD_PARA_H
