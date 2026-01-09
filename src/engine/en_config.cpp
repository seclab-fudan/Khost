#include "en_config.h"
#include "loader.h"
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>

namespace khost {

// new_mrs
// MRS<c> <Rd>,<spec_reg>
// Creates information of MRS instructions
IncompatibleInsn* new_mrs(uint16_t _rd, uint16_t _sysm) {
    IncompatibleInsn *insn = new IncompatibleInsn;
    insn->type = ARMv7M_MRS;
    insn->rd = _rd;
    insn->sysm = _sysm;
    return insn;
}

// new_msr
// MSR<c><q>  <spec_reg>, <Rn>
// Creates information of MSR instructions
IncompatibleInsn* new_msr(uint16_t _rn, uint16_t _sysm, uint16_t _mask) {
    IncompatibleInsn *insn = new IncompatibleInsn;
    insn->type = ARMv7M_MSR;
    insn->rn = _rn;
    insn->sysm = _sysm;
    insn->mask = _mask;
    return insn;
}

// new_cps
// CPS<effect> <iflags>
// Creates information of CPS instructions
IncompatibleInsn* new_cps(uint16_t _im, uint16_t _i, uint16_t _f) {
    IncompatibleInsn *insn = new IncompatibleInsn;
    insn->type = ARMv7M_CPS;
    insn->im = _im;
    insn->i = _i;
    insn->f = _f;
    return insn;
}

// load_from_file: load config from yaml file
// config_yml_path: path to the yaml file
// return: true on success, false on fail
bool Config::load_from_file(int mode, std::string config_yml_path, std::string working_dir) {
    _mode = mode;
    _original_config_path = config_yml_path;
    _working_dir_path = working_dir;

    // if working dir is set, let's copy the config to working dir
    if (working_dir.length()) {
        _config_path = working_dir + "/config.yml";
        std::ifstream src(config_yml_path, std::ios::binary);
	    std::ofstream dst(_config_path, std::ios::binary);
	    dst << src.rdbuf();
    } else {
        _config_path = _original_config_path;
    }

    // load and parse yaml file
    YAML::Node config;
    try {
        config = YAML::LoadFile(config_yml_path);
    } catch (...) {
        std::cerr << "failed to load config from file " << config_yml_path 
                  << std::endl;
        return false;
    }

    // "firmware" contains path to the target firmware
    YAML::Node path_node = config["firmware"];
    try {
        std::string raw_path = path_node.as<std::string>();
        _firmware_path = adjust_path(config_yml_path, raw_path);
        // make sure the firmware is copy to working dir
        if (working_dir.length()) {
            std::string target_path = working_dir + "/" + raw_path;
            std::ifstream src(_firmware_path, std::ios::binary);
	        std::ofstream dst(target_path, std::ios::binary);
	        dst << src.rdbuf();
        }
    } catch(...) {
        std::cerr << "failed to get firmware path" << std::endl;
        return false;
    }

    // "time_measure_begin" contains entery point for time measuring
    _time_measure_begin_addr = (uint32_t)-1;
    YAML::Node begin_addr_node = config["time_measure_begin"];
    if (begin_addr_node.IsDefined() && !begin_addr_node.IsNull()) {
        _time_measure_begin_addr = begin_addr_node.as<uint32_t>();
    }

    // "time_measure_end" contains exit point for time measuring
    _time_measure_end_addr = (uint32_t)-1;
    YAML::Node end_addr_node = config["time_measure_end"];
    if (end_addr_node.IsDefined() && !end_addr_node.IsNull()) {
        _time_measure_end_addr = end_addr_node.as<uint32_t>();
    }

    // "nop_addrs" contains hook address for nop
	_hook_nop_addrs.clear();
    YAML::Node hook_nop_node = config["nop_addrs"];
    if (hook_nop_node.IsDefined() && !hook_nop_node.IsNull()) {
        _hook_nop_addrs = hook_nop_node.as<std::vector<uint32_t>>();
    }

    // "return 0" contains hook address for nop
	_return_zero_addrs.clear();
    YAML::Node return_zero_node = config["return_zero_addrs"];
    if (return_zero_node.IsDefined() && !return_zero_node.IsNull()) {
        _return_zero_addrs = return_zero_node.as<std::vector<uint32_t>>();
    }

    // "fuzzware_enable" sets if we use fuzzware mmio model
    _enable_fuzzware = true;
    YAML::Node enable_fuzzware_node = config["fuzzware_enable"];
    if (enable_fuzzware_node.IsDefined() && !enable_fuzzware_node.IsNull()) {
        _enable_fuzzware = enable_fuzzware_node.as<bool>();
    }

    // "hal_hooks contains hook for hal"
    _hal_hooks.clear();
    YAML::Node hal_hook_node = config["hal_hooks"];
    if (hal_hook_node.IsDefined() && !hal_hook_node.IsNull()) {
        for (auto hal_hook_item: hal_hook_node) {
            uint32_t addr = hal_hook_item.first.as<uint32_t>();
            std::string name = hal_hook_item.second.as<std::string>();
            _hal_hooks.push_back(std::make_pair(addr, name));
        }
    }

    // "basic_blocks" contains basic blocks of the firmware
    _basic_blocks.clear();
    YAML::Node basic_blocks_node = config["basic_blocks"];
    if (basic_blocks_node.IsDefined() && !basic_blocks_node.IsNull()) {
        for (auto basic_block_node: basic_blocks_node) {
			// skip nop addrs
            uint32_t begin = basic_block_node.first.as<uint32_t>();
            uint32_t end = basic_block_node.second.as<uint32_t>();
			if (std::find(_hook_nop_addrs.begin(), _hook_nop_addrs.end(), end - 0x4) != _hook_nop_addrs.end()) {
				continue;
			}
			if (std::find(_return_zero_addrs.begin(), _return_zero_addrs.end(), begin) != _return_zero_addrs.end()) {
				continue;
			}
            _basic_blocks.push_back(BasicBlock {begin, end});
        }
    }

    // "exit_addrs" contains hook address for exit
    _hook_exit_addrs.clear();
    YAML::Node hook_exit_node = config["exit_addrs"];
    if (hook_exit_node.IsDefined() && !hook_exit_node.IsNull()) {
        _hook_exit_addrs = hook_exit_node.as<std::vector<uint32_t>>();
    }

    // "symbols" contains symbol table
    _symbol_table.clear();
    YAML::Node symbol_table = config["symbols"];
    if (symbol_table.IsDefined() && !symbol_table.IsNull()) {
        load_symbol_table(&symbol_table);
    }

    // "mmio_models" contains fuzzware model
    _fuzzware_model.clear();
    YAML::Node fuzzware_model = config["mmio_models"];
    if (fuzzware_model.IsDefined() && !fuzzware_model.IsNull()) {
        load_fuzzware_model(&fuzzware_model);
    }

    // "hit_count_exit_points" contains hit count exit points
    _hit_count_exit_points.clear();
    _hit_count_values.clear();
    YAML::Node hit_count_points = config["hit_count_exit_points"];
    if (hit_count_points.IsDefined() && !hit_count_points.IsNull()) {
        hit_count_point_load(&hit_count_points);
    }

    // "memory_map" contains memory area and permissions
    YAML::Node memory_maps = config["memory_map"];
    if (memory_maps.IsDefined() && !memory_maps.IsNull()) {
        address_info_load(&memory_maps);
    }

    // "interrupt" contains interrupt configure
    _init_period = 0;
    _fire_after = (uint32_t)-1;
    YAML::Node interrupt = config["interrupt"];
    if (interrupt.IsDefined() && !interrupt.IsNull()) {
        try {
            _init_period = interrupt["init_period"].as<uint32_t>();
        } catch (...) {};
        try {
            _fire_after = interrupt["fire_after"].as<uint32_t>();
        } catch (...) {};
    }

    _reloc_map.clear();
    Loader loader;
    loader.load();

    if (CONFIG.option_should_target_output(EN_OUTPUT_LOADER)) {   
        dump_config();
    }
    return true;
}

// adjust_path: change relative path by yaml file to relative path by working 
// directory
// yaml_path: path to yaml file
// raw_path: raw path to firmware
std::string Config::adjust_path(std::string yaml_path, std::string raw_path) {
    // return abs path
    if (raw_path.length() && raw_path[0] == '/') {
        return raw_path;
    }
    // get yaml dir
    int last_pos = yaml_path.length();
    while (last_pos-- > 0) {
        if (yaml_path[last_pos] == '/') {
            break;
        }
    }
    if (last_pos == 0) {
        return raw_path;
    }
    return yaml_path.substr(0, last_pos) + "/" + raw_path;
}

// load_symbol_table: read symbols from yaml node
// symbol_table: yaml node "symbols"
// return: true for success
bool Config::load_symbol_table(YAML::Node *symbol_table) {
    for (auto symbol: *symbol_table) {
        uint32_t addr = symbol.first.as<uint32_t>();
        std::string symbol_name = symbol.second.as<std::string>();
        _symbol_table[addr] = symbol_name;
    }
    return true;
}

// load_fuzzware_model: read model from yaml node
// fuzzware_model: yaml node "mmio_models"
// return: true for success
bool Config::load_fuzzware_model(YAML::Node *fuzzware_model) {
    for (auto model_by_type: *fuzzware_model) {
        std::string name_region = model_by_type.first.as<std::string>();
        for (auto region: (*fuzzware_model)[name_region]) {
            // set basic info of model
            struct FuzzwareModelInfo *model = new FuzzwareModelInfo;
            model->pc          = region.second["pc"].as<unsigned>();
            model->addr        = region.second["addr"].as<unsigned>();
            model->access_size = region.second["access_size"].as<int>();
            // bitextract
            if ("bitextract" == name_region) {
                model->type       = FUZZWARE_BITEXTRACT;
                model->mask       = region.second["mask"].as<unsigned>();
                model->size       = region.second["size"].as<int>();
                model->left_shift = region.second["left_shift"].as<unsigned>();
            }
            // constant
            else if ("constant" == name_region) {
                model->type = FUZZWARE_CONSTANT;
                model->val  = region.second["val"].as<unsigned>();
            }
            // passthrough
            else if ("passthrough" == name_region) {
                model->type     = FUZZWARE_PASSTHROUGH;
                model->init_val = region.second["init_val"].as<unsigned>();
            }
            // set
            else if ("set" == name_region) {
                model->type = FUZZWARE_SET;
                auto value_vec = region.second["vals"].as<std::vector<unsigned>>();
                for (uint32_t i = 0; i < value_vec.size(); i++) {
                    if (i == RUNTIME_FUZZWARE_SET_SIZE) {
                        printf("fuzzware set size too small\n");
                        exit(-1);
                    }
                    model->vals[i] = value_vec[i];
                }
                model->val_cnt = value_vec.size();
            }
            // unmodeled region is fall back to identity
            else {
                model->type = FUZZWARE_IDENTITY;
            }
            _fuzzware_model[model->addr][model->pc] = model;
        }
    }
    return true;
}

// ~Config: clear fuzzware model memory
Config::~Config() {
    // clear model
    for (auto i: _fuzzware_model) {
        for (auto j: i.second) {
            delete j.second;
        }
    }
    // clear incompatible instructions
    for (auto i: _incompatible_insn_map)
        delete i.second;
    // clear load info
    for (auto i: _load_info) {
        munmap(i->source_buffer, i->source_size);
        delete i;
    }
}

// belonging_symbol: get symbol of the addr
// addr: address in firmware
std::string Config::belonging_symbol(uint32_t addr) {
    auto iter = _symbol_table.upper_bound(addr);
    if (iter == _symbol_table.begin()) {
        return std::string("unknown");
    }
    iter--;
    return iter->second;
}

// get_model: check if the addr and pc has been modeled by fuzzware and return 
// pointer into model info
// addr: mmio addr
// pc: mmio access pc
// return: pointer to model, nullptr when not exisits 
FuzzwareModelInfo *Config::get_model(uint32_t addr, uint32_t pc) {
    auto pc_iter = _fuzzware_model.find(addr);
    if (pc_iter == _fuzzware_model.end()) {
        return nullptr;
    }

    auto info_iter = pc_iter->second.find(pc);
    if (info_iter == pc_iter->second.end()) {
        return nullptr;
    }

    return info_iter->second;
}

void Config::dump_config() {
    printf("INFO  | loader: firmware path: %s\n", _firmware_path.c_str());
    printf("INFO  | loader: time measure begin: 0x%08x\n", _time_measure_begin_addr);
    printf("INFO  | loader: time measure end: 0x%08x\n", _time_measure_end_addr);
    printf("INFO  | loader: basic blocks:\n");
    // for (auto i: _basic_blocks) {
    //     printf("INFO  | loader:     0x%08x: 0x%08x\n", i.start, i.end);
    // }
    printf("INFO  | loader: hook exit addrs:\n");
    for (auto i: _hook_exit_addrs) {
        printf("INFO  | loader:     0x%08x\n", i);
    }
    printf("INFO  | loader: hook nop addrs:\n");
    for (auto i: _hook_nop_addrs) {
        printf("INFO  | loader:     0x%08x\n", i);
    }
    printf("INFO  | loader: return zero addrs:\n");
    for (auto i: _return_zero_addrs) {
        printf("INFO  | loader:     0x%08x\n", i);
    }
    // printf("INFO  | loader: symbol table:\n");
    // for (auto i: _symbol_table) {
    //     printf("INFO  | loader:     0x%08x: %s\n", i.first, i.second.c_str());
    // }
    // printf("INFO  | loader: mmio models:\n");
    // for (auto i: _fuzzware_model) {
    //     for (auto j: i.second) {
    //         printf("INFO  | loader:     pc=0x%08x, addr=0x%08x\n", j.second->pc, j.second->addr);
    //     }
    // }
    printf("INFO  | loader: firmware info:\n");
    printf("INFO  | loader:   load_base: 0x%08x\n", _firmware_info.load_base);
    printf("INFO  | loader:   num_irq: %d\n", _firmware_info.num_irq);
    printf("INFO  | loader:   initial_sp: 0x%08x\n", _firmware_info.initial_sp);
    printf("INFO  | loader:   initial_pc: 0x%08x\n", _firmware_info.initial_pc);
    printf("INFO  | loader:   firmware_code_size: 0x%08x\n", _firmware_info.firmware_code_size);
    printf("INFO  | loader:   addition_start: 0x%08x\n", _firmware_info.addition_start);
}

//==============================================================================
// Hit Count Exit Point

/// @brief load hit count exit points from yaml
/// @param yaml_node yaml node
void Config::hit_count_point_load(YAML::Node *yaml_node) {
    for (auto hit_count_point: *yaml_node) {
        uint32_t addr = hit_count_point.first.as<uint32_t>();
        uint32_t hit_count = hit_count_point.second.as<uint32_t>();
        _hit_count_exit_points[addr] = (HitCountExitPoint){ addr, hit_count };
        _hit_count_values[addr] = 0;
    }
}

/// @brief reset hit count value of all exit points
void Config::hit_count_value_reset_all() {
    for (auto i: _hit_count_exit_points) {
        _hit_count_values[i.first] = 0x0;
    }
}

/// @brief increase the hit count value of the exit point
/// @param addr address of the exit point
void Config::hit_count_value_increase(uint32_t addr) {
    if (!hit_count_point_exisit(addr)) {
        return;
    }
    _hit_count_values[addr] += 1;
}

/// @brief check if there's a exit point at address
/// @param addr firmware physical address
/// @return true for exists
bool Config::hit_count_point_exisit(uint32_t addr) {
    if (_hit_count_exit_points.find(addr) != _hit_count_exit_points.end()) {
        return true;
    }
    return false;
}

/// @brief check if we reach the limit of the hit count
/// @param addr address of the exit point
/// @return true for reach the limit
bool Config::hit_count_exit_check(uint32_t addr) {
    if (!hit_count_point_exisit(addr)) {
        return false;
    }
    int hit_limit = _hit_count_exit_points[addr].hit_count;
    int hit = _hit_count_values[addr];
    return hit >= hit_limit;
}

//==============================================================================
// Address Info

/// @brief load address info from yaml
/// @param yaml_node yaml node
void Config::address_info_load(YAML::Node *yaml_node) {
    _address_info.clear();
    for (auto address_info: *yaml_node) {
        std::string name = address_info.first.as<std::string>();
        uint32_t start = address_info.second["base_addr"].as<uint32_t>();
        uint32_t end = address_info.second["size"].as<uint32_t>() + start;
        std::string per = address_info.second["permissions"].as<std::string>();
        if (per.length() != 3) {
            printf("invalid memory permission\n");
            exit(1);
        }
        _address_info.push_back(AddressInfo(
            start, end,
            per[0] == 'r', per[1] == 'w', per[2] == 'x'
        ));
    }
}

/// @brief get the address info containing addr
/// @param addr address
/// @return address info
AddressInfo Config::address_info(uint32_t addr) {
    for (auto info: _address_info) {
        if (info.start <= addr && addr < info.end) {
            return info;
        }
    }
    return AddressInfo();
} 

/// @brief return the address info list
/// @return pointer to the address infp vector
std::vector<AddressInfo> *Config::address_info_list() {
    return &_address_info;
}

}  // namespace khost
