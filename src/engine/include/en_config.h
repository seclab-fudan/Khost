#ifndef CONFIG_H
#define CONFIG_H

#include "global.h"
#include "data_def.h"
#include <string>
#include <map>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace khost {

// firmware information
struct FirmwareInfo {
    // load base of the firmware
    uint32_t load_base;
    // number of irq
    uint32_t num_irq;
    // initial SP
    uint32_t initial_sp;
    // initial PC
    uint32_t initial_pc;
    // entry of the kvm (not-used)
    uint32_t kvm_entry;
    // size of the code
    uint32_t firmware_code_size;
    // start of the addition
    uint32_t addition_start;
    // size of the addition
    uint32_t addition_size;
    // guest physical address of page table (not-used)
    uint32_t addr_page_table_start;
    // guest physical address of vector table (not-used)
    uint32_t addr_vector_table_start;
    // guest physical address of exception handlers (not-used)
    uint32_t addr_excp_handler_start[ARMv7A_VEC_MAX];
};

// basic block
struct BasicBlock {
    uint32_t start;
    uint32_t end;
};

IncompatibleInsn* new_mrs(uint16_t _rd, uint16_t _sysm);
IncompatibleInsn* new_msr(uint16_t _rn, uint16_t _sysm, uint16_t _mask);
IncompatibleInsn* new_cps(uint16_t _im, uint16_t _i, uint16_t _f);

// load information
struct LoadInfo {
    void *source_buffer;
    uint32_t source_size;
    uint32_t phys_addr;

    LoadInfo(void *s_buffer, uint32_t s_size, uint32_t s_addr)
        : source_buffer(s_buffer), source_size(s_size), phys_addr(s_addr) {}
};

/// @brief Exit execution after reach max hit times
struct HitCountExitPoint {
    uint32_t addr;
    uint32_t hit_count;
};

/// @brief Address area and permission
struct AddressInfo {
    uint32_t start;
    uint32_t end;
    bool r;
    bool w;
    bool x;

    AddressInfo(uint32_t _start = 0x0, uint32_t _end = 0x0, 
                bool _r = false, bool _w = false, bool _x = false)
        : start(_start), end(_end), r(_r), w(_w), x(_x) {}
};

// config provider for the whole system
class Config {
public:
    inline int mode() {
        return _mode;
    }
    inline std::string config_path() {
        return _config_path;
    }
    inline std::string original_config_path() {
        return _original_config_path;
    }
    inline std::string working_dir_path() {
        return _working_dir_path;
    }
    inline std::string frimware_path() { 
        return _firmware_path; 
    }
    inline uint32_t time_measure_begin_addr() { 
        return _time_measure_begin_addr; 
    }
    inline uint32_t time_measure_end_addr() { 
        return _time_measure_end_addr; 
    }
    inline std::vector<BasicBlock> *basic_blocks() {
        return &_basic_blocks;
    }
    inline std::vector<uint32_t> *hook_exit_addrs() {
        return &_hook_exit_addrs;
    }
	inline std::vector<uint32_t> *hook_nop_addrs() {
		return &_hook_nop_addrs;
	}
	inline std::vector<uint32_t> *return_zero_addrs() {
		return &_return_zero_addrs;
	}
    inline std::vector<std::pair<uint32_t, std::string>> *hal_hooks() {
        return &_hal_hooks;
    }
    inline bool enable_fuzzware() {
        return _enable_fuzzware;
    }
    inline FirmwareInfo *firmware_info() {
        return &_firmware_info;
    }
    void add_incompatible_insn(uint32_t pc, IncompatibleInsn *insn) { 
        insn->addr = pc;
        _incompatible_insn_map[pc] = insn;
    }
    inline bool is_incompatible_insn(uint32_t pc) {
        return _incompatible_insn_map.find(pc) != _incompatible_insn_map.end();
    }
    inline IncompatibleInsn *get_incompatible_insn(uint32_t pc) {
        return _incompatible_insn_map[pc];
    }
    inline std::map<uint32_t, IncompatibleInsn*> *get_incompatible_insn_map() {
        return &_incompatible_insn_map;
    }
    inline void add_load_info(LoadInfo *info) { 
        _load_info.push_back(info); 
    }
    inline std::vector<LoadInfo*> *load_info() {
        return &_load_info;
    } 
    std::string belonging_symbol(uint32_t addr);
    std::map<uint32_t, std::string>* symbol_table() { return &_symbol_table; };
    FuzzwareModelInfo* get_model(uint32_t addr, uint32_t pc);

    std::map<uint32_t, std::map<uint32_t, FuzzwareModelInfo *>> *fuzzware_model() {
        return &_fuzzware_model;
    }

    void set_reloc_insn(uint32_t before, uint32_t after) {
        _reloc_map[after] = before;
    }
    void set_additional_insn_map(uint32_t before, uint32_t after) {
        _additional_reloc_map[after] = before;
    }
    std::map<uint32_t, uint32_t> *reloc_map() { return &_reloc_map; }
    std::map<uint32_t, uint32_t> *additional_reloc_map() { return &_additional_reloc_map; }
    uint32_t reloc_pc(uint32_t pc) {
        if (_reloc_map.find(pc) != _reloc_map.end()) {
            return _reloc_map[pc];
        }
        if (_additional_reloc_map.find(pc) != _additional_reloc_map.end()) {
            return _additional_reloc_map[pc];
        }  
        return pc;
    }
    // interrupt settings ======================================================
    uint32_t init_period() { return _init_period; }
    uint32_t fire_after() { return _fire_after; }

    // hit count exit point ====================================================
    void hit_count_point_load(YAML::Node *yaml_node);
    void hit_count_value_reset_all();
    void hit_count_value_increase(uint32_t addr);
    bool hit_count_point_exisit(uint32_t addr);
    bool hit_count_exit_check(uint32_t addr);

    // memory region ===========================================================
    void address_info_load(YAML::Node *yaml_node);
    AddressInfo address_info(uint32_t addr);
    std::vector<AddressInfo> *address_info_list();

    bool load_from_file(int mode, std::string config_yml_path, std::string working_dir);

    // options =================================================================

    // level of the output
    void option_set_output_level(int level) { _option_output_level = level; }
    int option_output_level() { return _option_output_level; }

    // targets of the output
    void option_set_output_target(int target, bool enable) {
        if (enable) {
            _option_output_target |= target;
        } else {
            _option_output_target &= (~target);
        }
    }
    uint32_t option_output_target() { return _option_output_target; }
    bool option_should_target_output(int target) { return _option_output_target & target; }

    // if we should save the load result
    void option_set_save_rewrite_result(bool save, std::string path) {
        _option_save_rewrite_result = save;
        _option_rewrite_result_path = path;
    }
    bool option_save_rewrite_result() { return _option_save_rewrite_result; }
    std::string option_save_rewrite_result_path() { return _option_rewrite_result_path; }

    // if we should use the hugepage
    void option_set_use_hugepage(bool use) { _option_use_huge_page = use; }
    bool option_use_hugepage() { return _option_use_huge_page; }

    // if we should use auto model
    void option_set_auto_model(bool model) { _option_auto_model = model; }
    bool option_auto_model() { return _option_auto_model; }

    // get global instance of Config
    inline static Config &instance() {
        static Config gconfig;
        return gconfig;
    }
    ~Config();

private:
    int _mode;

    // path to the original config file 
    std::string _config_path;

    // path to the config file in fuzz (may update)
    std::string _original_config_path;

    // current working dir
    std::string _working_dir_path;

    // path to the firmware binary 
    std::string _firmware_path;

    // entry point for time measuring
    uint32_t _time_measure_begin_addr;

    // exit point for time measuring
    uint32_t _time_measure_end_addr;

    // basic block of the firmware
    std::vector<BasicBlock> _basic_blocks;

    // symbol table generate from ghidra or ida
    std::map<uint32_t, std::string> _symbol_table;

    // hook addrs to exit from the firmware
    std::vector<uint32_t> _hook_exit_addrs;

    // init period with no interrupt
    uint32_t _init_period;
    uint32_t _fire_after;

    // fuzzware model
    std::map<uint32_t, std::map<uint32_t, FuzzwareModelInfo *>> _fuzzware_model;

    // firmware infomation (calculate by loader)
    FirmwareInfo _firmware_info;

    // memory load information
    std::vector<LoadInfo*> _load_info;

    // incompatible instructions
    std::map<uint32_t, IncompatibleInsn*> _incompatible_insn_map;

    std::map<uint32_t, uint32_t> _reloc_map;
    std::map<uint32_t, uint32_t> _additional_reloc_map;

    // hit count exit points
    std::map<uint32_t, HitCountExitPoint> _hit_count_exit_points;
    std::map<uint32_t, uint32_t> _hit_count_values;

	// hook addrs to skip from long delay
	std::vector<uint32_t> _hook_nop_addrs;

	// hook addrs to return 0
	std::vector<uint32_t> _return_zero_addrs;

    // vector for mmio memory region
    std::vector<AddressInfo> _address_info;

    // enable the use of fuzzware
    bool _enable_fuzzware;

    // hal hooks
    std::vector<std::pair<uint32_t, std::string>> _hal_hooks;

    Config() {
        _option_save_rewrite_result = false;
        _option_auto_model = false;
        _option_use_huge_page = true;
        _option_output_level = OUTPUT_LEVEL_ERROR;
        _option_output_target = 0;
    }

    bool load_symbol_table(YAML::Node *symbol_table);
    bool load_fuzzware_model(YAML::Node *fuzzware_model);

    std::string adjust_path(std::string yaml_path, std::string raw_path);

    bool _option_save_rewrite_result;
    std::string _option_rewrite_result_path;
    bool _option_auto_model;
    bool _option_use_huge_page;
    int _option_output_level;
    uint32_t _option_output_target;

    void dump_config();
};

#define CONFIG (Config::instance())

}  // namespace khost

#endif  // CONFIG_H
