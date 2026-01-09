#include "stacktrace.h"
#include "gdbstub.h"
#include "en_config.h"
#include "en_board.h"
#include "en_board_full.h"
#include "en_board_para.h"
#if KVM_OPEN_DEBUG
	#include "autogen_rt_full_debug.h"
#else
	#include "autogen_rt_full_normal.h"
#endif

#include "stdio.h"
#include "unistd.h"
#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include <fstream>
#include <regex>
#include <ranges>
#include <chrono>

struct CmdOption {
    std::string mode;
    bool huge_page;
    bool auto_model;
    bool check_coverage;
    std::string ground_truth_path;
    uint32_t timeout;
    int output_level;
    uint32_t output_target;
    std::string config_path;
    bool single_input;
    std::string input_path;
    std::map<int, std::filesystem::directory_entry> input_map;
    bool save_rewrite_result;
    std::string rewrite_result_path;
    bool debug;
	bool libafl_replay;

    CmdOption() {
        mode = "full";
        huge_page = false;
        auto_model = false;
        check_coverage = false;
        ground_truth_path = "";
        timeout = 0;
        output_level = OUTPUT_LEVEL_ERROR;
        output_target = 0x0;
        config_path = "";
        input_path = "";
        single_input = true;
        input_map.clear();
        save_rewrite_result = false;
        rewrite_result_path = "";
        debug = false;
		libafl_replay = false;
    }
} option;
 
void print_usage() {
    printf("khost, speed up MCU firmware rehosting with KVM native running\n");
    printf("\nUSAGE:\n");
    printf("\tkhost [args] /path/to/the/config /path/to/the/input\n");
    printf("ARGS:\n");
    printf("\t-m <mode>\t\trunning mode if khost\n");
    printf("\t\t\t\t- options: full, para\n");
    printf("\t\t\t\t- default: full\n");
    printf("\t-p \t\t\tenable the usage of hugepage\n");
    printf("\t-f \t\t\tenable auto fuzzware modeling (only under full mode)\n");
    printf("\t-t <seconds>\t\tset timeout seconds\n");
    printf("\t-c <ground-truth>\tcheck basic block coverage with ground-truth\n");
    printf("\t-l <output-level>\tlevel of the output\n");
    printf("\t\t\t\t- options: error, info, debug\n");
    printf("\t\t\t\t- default: error\n");
    printf("\t-o <output-target>\tselect which module to output\n");
    printf("\t\t\t\t- options: all, board, cpu, emu, fault, fuzzware, handler, nvic, sysctl, systick, trigger, loader\n");
    printf("\t\t\t\t- default: none\n");
    printf("\t-r <rewrite-result>\tsave the result of the rewriter to file\n");
    printf("\t-d \t\t\tenable debug on 0.0.0.0:1234\n");
    printf("\t-b \t\t\treplay according to the output of libafl\n");
    printf("\t-h \t\t\tprint this message\n");
}

void parse_args(int argc, char *argv[]) {
    int short_opt = 0;
    while ((short_opt = getopt(argc, argv, "m:pfc:t:l:o:r:dbh::")) != -1) {
        switch (short_opt)
        {
        case 'm': // mode
            option.mode = optarg;
            if (option.mode != "full" && option.mode != "para") {
                printf("%s: invalid -m option given\n", argv[0]);
                exit(-1);
            }
            printf("INFO  | khost: set mode to %s\n", option.mode.c_str());
            break;
        case 'p': // huge page
            option.huge_page = true;
            printf("INFO  | khost: enable huge page\n");
            break;
        case 'f': // auto fuzzware model
            option.auto_model = true;
            printf("INFO  | khost: enable auto fuzzware modeling\n");
            break;
        case 'c': // check coverage and ground truth
            option.ground_truth_path = optarg;
            if (!std::filesystem::exists(option.ground_truth_path)) {
                printf("%s: invalid ground truth path: %s\n", argv[0], option.ground_truth_path.c_str());
                exit(-1);
            }
            option.check_coverage = true;
            break;
        case 'r':
            option.save_rewrite_result = true;
            option.rewrite_result_path = optarg;
            if (std::filesystem::exists(option.rewrite_result_path)) {
                printf("%s: rewrite result file not empty: %s\n", argv[0], option.rewrite_result_path.c_str());
                exit(-1);
            }
            printf("INFO  | khost: rewrite result will save to %s\n", option.rewrite_result_path.c_str());
            break;
        case 't': // timeout value
            try {
                option.timeout = std::stoi(optarg);
                printf("INFO  | khost: set timeout to %d seconds\n", option.timeout);
                option.timeout *= 1000000;
            } catch (...) {
                printf("%s: invalid -t option given\n", argv[0]);
                exit(-1);
            }
            break;
        case 'l': { // output level
            std::string level = optarg;
            if (level == "debug") {
                option.output_level = OUTPUT_LEVEL_DEBUG;
                printf("INFO  | khost: set output level to error debug\n");
            } else if (level == "info") {
                option.output_level = OUTPUT_LEVEL_INFO;
                printf("INFO  | khost: set output level to info\n");
            } else if (level == "error") {
                option.output_level = OUTPUT_LEVEL_ERROR;
                printf("INFO  | khost: set output level to error\n");
            } else {
                printf("%s: invalid -l option given\n", argv[0]);
                exit(-1);
            }
            break;
        }
        case 'o': { // output target
            std::string target_name = optarg;
            for (auto &&part : std::views::split(target_name, ',')) {
                std::string name;
                for (auto c: part) name += c;
                if (name == "cpu") {
                    option.output_target |= RT_OUTPUT_CPU;
                    printf("INFO  | khost: enable output for module cpu\n");
                } else if (name == "emu") {
                    option.output_target |= RT_OUTPUT_EMU;
                    printf("INFO  | khost: enable output for module emu\n");
                } else if (name == "fault") {
                    option.output_target |= RT_OUTPUT_FAULT;
                    printf("INFO  | khost: enable output for module fault\n");
                } else if (name == "fuzzware") {
                    option.output_target |= RT_OUTPUT_FUZZWARE;
                    printf("INFO  | khost: enable output for module fuzzware\n");
                } else if (name == "handler") {
                    option.output_target |= RT_OUTPUT_HANDLER;
                    printf("INFO  | khost: enable output for module handler\n");
                } else if (name == "nvic") {
                    option.output_target |= RT_OUTPUT_NVIC;
                    printf("INFO  | khost: enable output for module nvic\n");
                } else if (name == "sysctl") {
                    option.output_target |= RT_OUTPUT_SYSCTL;
                    printf("INFO  | khost: enable output for module sysctl\n");
                } else if (name == "systick") {
                    option.output_target |= RT_OUTPUT_SYSTICK;
                    printf("INFO  | khost: enable output for module systick\n");
                } else if (name == "trigger") {
                    option.output_target |= RT_OUTPUT_TRIGGER;
                    printf("INFO  | khost: enable output for module trigger\n");
                } else if (name == "board") {
                    option.output_target |= EN_OUTPUT_BOARD;
                    printf("INFO  | khost: enable output for module board\n");
                } else if (name == "loader") {
                    option.output_target |= EN_OUTPUT_LOADER;
                    printf("INFO  | khost: enable output for loader\n");
                } else if (name == "all") {
                    option.output_target = 0xffffffff;
                    printf("INFO  | khost: enable output for all modules\n");
                }
            }
            break;
        }
        case 'd':
            option.debug = true;
            break;
		case 'b':
			option.libafl_replay = true;
			break;
        case 'h': // help
            print_usage();
            exit(0);
        default:
            printf("%s: invalid usage, use -h to get command line options\n", argv[0]);
            exit(-1);
        }
    }
    if (option.mode == "para" && option.auto_model) {
        printf("ERROR | auto model is not support under para mode\n");
        exit(-1);
    }

    // check positional args
    std::vector<std::string> pos_args;
    for (int i = optind; i < argc; i++) {
        pos_args.push_back(argv[i]);
    }
    if (pos_args.size() < 2) {
        printf("%s: no config file and input file given\n", argv[0]);
        printf("%s: invalid usage, use -h to get command line options\n", argv[0]);
        exit(-1);
    }

    // get and check config path
    std::string config_path = pos_args[0];
    if (!std::filesystem::exists(config_path)) {
        printf("config path invalid: %s\n", config_path.c_str());
        exit(-1);
    }
    printf("INFO  | khost: load config from %s\n", config_path.c_str());
    option.config_path = config_path;

    // get and check input path
    std::string input_file_or_dir_path = pos_args[1];
    if (!std::filesystem::exists(input_file_or_dir_path)) {
        printf("input path invalid: %s\n", input_file_or_dir_path.c_str());
        exit(-1);
    }
    if (!std::filesystem::is_directory(input_file_or_dir_path)) {
        option.single_input = true;
        option.input_path = input_file_or_dir_path;
        printf("INFO  | khost: load single input form %s\n", option.input_path.c_str());
    } else {
        option.single_input = false;
        for (int i = 1; i < pos_args.size(); i++) {
            std::string dir_path = pos_args[i];
			if (!option.libafl_replay) {
            	for (const auto& entry: std::filesystem::directory_iterator(dir_path)) {
            	    std::string file_name = entry.path().filename();
            	    if (file_name.length() > 0 && file_name[0] == '.') {
            	        continue;
            	    }
            	    if (!entry.is_regular_file()) {
            	        continue;
            	    }
            	    std::regex pattern1(R"(id:(\d+))");
            	    std::smatch match1;
            	    if (!std::regex_search(file_name, match1, pattern1)) {
            	        continue;
            	    }
            	    std::regex pattern2(R"(time:(\d+))");
            	    std::smatch match2;
            	    if (!std::regex_search(file_name, match2, pattern2)) {
            	        continue;
            	    }
            	    int t = std::stoi(match2[1].str());
            	    option.input_map[t] = entry;
            	}
			} else {
            	std::string dir_path = pos_args[i];
				int t = 0;
            	for (const auto& entry: std::filesystem::directory_iterator(dir_path)) {
            	    std::string file_name = entry.path().filename();
            	    if (file_name.length() > 0 && file_name[0] == '.') {
            	        continue;
            	    }
            	    if (!entry.is_regular_file()) {
            	        continue;
            	    }
            	    option.input_map[t++] = entry;
            	}

			}
            if (option.input_map.size() == 0) {
                printf("input directory is not valid fuzz queue, crash or hang: %s\n", dir_path.c_str());
                exit(-1);
            }
        }
        printf("INFO  | khost: load %ld inputs\n", option.input_map.size());
    }
}

void init_config() {
    khost::Config::instance().option_set_use_hugepage(option.huge_page);
    khost::Config::instance().option_set_output_level(option.output_level);
    for (uint32_t i = 0; i < 32; i++) {
        khost::Config::instance().option_set_output_target(1 << i, option.output_target & (1 << i));
    }
    if (option.save_rewrite_result) {
        khost::Config::instance().option_set_save_rewrite_result(true, option.rewrite_result_path);
    } else {
        khost::Config::instance().option_set_save_rewrite_result(false, "");
    }

    if (!khost::Config::instance().load_from_file(
        option.mode == "full" ? FULL_MODE : PARA_MODE, option.config_path, std::string("")
    )) {
        printf("ERROR | khost: faild to load config\n");
        exit(-1);
    }
}

static khost::Board *board = nullptr;

void init_board() {
    if (option.mode == "full") {
        board = new khost::BoardFull();
    } else {
        board = new khost::BoardPara();
    }
    if (board == nullptr) {
        printf("ERROR | khost: failed to alloc virtual board\n");
        exit(-1);
    }
    if (!board->init()) {
        printf("ERROR | khost: failed to init virtual board\n");
        delete board;
        exit(-1);
    }
    if (option.check_coverage) {
        std::ifstream ground_truth_file(option.ground_truth_path);
        std::vector<uint32_t> basic_block_start;
        std::string line;

        while (std::getline(ground_truth_file, line)) {
            if (line.empty()) continue;
            try {
                uint32_t addr = std::stoul(line, nullptr, 16);
                basic_block_start.push_back(addr);
            } catch (const std::exception& e) {
                continue;
            }
        }
        printf("INFO  | khost: load %ld basic blocks from ground truth\n", basic_block_start.size());
        board->set_coverage_ground_truth(basic_block_start);
    }
}

void check_coverage() {
    auto coverage_map = board->hit_coverage_map();
    printf("INFO  | khost: hit %ld basic blocks\n", coverage_map->size());

    std::string json_dir = std::filesystem::path(option.config_path).parent_path().string();
    std::string json_path = json_dir + "/" + "basic_blocks.json";
    std::string json_str = "{\n";
    bool first_line = true;
    for (auto block: *coverage_map) {
        if (!first_line) {
            json_str += ",\n";
        }
        first_line = false;
        char line[32];
        sprintf(line, "    \"0x%08x\": 1", block.first);
        json_str += std::string(line);
    }
    if (!first_line) {
        json_str += "\n";
    }
    json_str += "}\n";

    std::ofstream ofs(json_path, std::ios::out | std::ios::trunc);
    if (!ofs) {
        printf("ERROR | failed to write basic block json result\n");
        delete board;
        exit(-1);
    }
    ofs << json_str;
    ofs.close();
    printf("INFO  | covered basic block is written to %s\n", json_path.c_str());
}

int exec(int timeout) {
    while(true) {
        if (timeout) {
            if (option.mode == "full") {
                dynamic_cast<khost::BoardFull *>(board)->cpu()->set_alarm(timeout);
            } else {
                dynamic_cast<khost::BoardPara *>(board)->cpu()->set_alarm(timeout);
            }
        }
        int ret = board->run();
        if (timeout) {
            if (option.mode == "full") {
                dynamic_cast<khost::BoardFull *>(board)->cpu()->set_alarm(0);
            } else {
                dynamic_cast<khost::BoardPara *>(board)->cpu()->set_alarm(0);
            }
        }
        if (ret == EXIT_MODEL && option.auto_model) {
            if (!dynamic_cast<khost::BoardFull *>(board)->update_fuzzware_model()) {
                return EXIT_OK;
            }
            continue;
        }
        if (ret == EXIT_MODEL) {
            ret == EXIT_FIRMWARE_CRASH;
        }
        return ret;
    }
}

void single_run() {
    std::ifstream file(option.input_path, std::ios::binary | std::ios::ate);
    if (!file) {
        printf("ERROR | khost: failed to open input file: %s\n", option.input_path.c_str());
        delete board;
        exit(-1);
    }
    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);
    uint8_t* buffer = new uint8_t[size];
    if (!file.read(reinterpret_cast<char*>(buffer), size)) {        
        printf("ERROR | khost: failed to read input file: %s\n", option.input_path.c_str());
        delete[] buffer;
        delete board;
        return;
    }
    file.close();

    if (!board->reset(buffer, size)) {
        printf("ERROR | khost: failed to reset virtual board\n");
        delete[] buffer;
        delete board;
        return;
    }

    if (option.debug) {
        debug_run(board);
        delete[] buffer;
        return;
    } 

    int ret = exec(option.timeout);
    switch (ret)
    {
    case EXIT_OK:
        printf("INFO  | khost: firmware EXIT_OK\n");
        break;
    case EXIT_FIRMWARE_CRASH: {
        printf("INFO  | khost: firmware EXIT_CRASH\n");
        StackTrace trace = stack_trace(board);
        // print stack trace
        printf("========== Crash Trace ==========\n");
        print_trace(&trace);
        printf("=================================\n");
        break;
    }
    case EXIT_INTERNAL_CRASH: {
        printf("ERROR | khost: khost internal crash\n");
        StackTrace trace = stack_trace(board);
        // print stack trace
        printf("========== Crash Trace ==========\n");
        print_trace(&trace);
        printf("=================================\n");
        break;
    }
    case EXIT_TIMEOUT:
        printf("INFO  | khost: firmware EXIT_TIMEOUT\n");
        break;
    default: {
        printf("ERROR | khost: unknown exit reason %d\n", ret);
        StackTrace trace = stack_trace(board);
        // print stack trace
        printf("========== Crash Trace ==========\n");
        print_trace(&trace);
        printf("=================================\n");
        break;
    }
    }
    delete[] buffer;
}

std::vector<std::pair<std::string, StackTrace>> unique_trace;

void add_crash_trace(std::string input) {
    StackTrace trace = stack_trace(board);
    for (int i = 0; i < unique_trace.size(); i++) {
        if (same_trace(trace, unique_trace[i].second)) {
            return;
        }
    }
    unique_trace.push_back(std::make_pair(input, trace));
}

void multi_run() {
    int total_crash = 0;
    int64_t max_time = 0;
    std::string slow_input;
    unique_trace.clear();
    for (const auto& entry: option.input_map) {
        std::string file_path = entry.second.path().string();
        std::ifstream file(file_path, std::ios::binary | std::ios::ate);
        if (!file) {
            printf("ERROR | khost: failed to open file: %s\n", file_path.c_str());
            continue;
        }

        std::streamsize size = file.tellg();
        file.seekg(0, std::ios::beg);
        uint8_t* buffer = new uint8_t[size];
        if (!file.read(reinterpret_cast<char*>(buffer), size)) {
            printf("ERROR | khost: failed to read file: %s\n", file_path.c_str());
            delete[] buffer;
            continue;
        }
        file.close();

        if (!board->reset(buffer, size)) {
            printf("ERROR | khost: failed to reset virtual board\n");
            delete[] buffer;
            continue;
        }
        auto start_time = std::chrono::high_resolution_clock::now();
        int ret = exec(option.timeout);
        auto end_time = std::chrono::high_resolution_clock::now();
        int64_t duration = duration_cast<std::chrono::microseconds>(end_time - start_time).count();
        if (duration > max_time) {
            max_time = duration;
            slow_input = file_path;
        }
        if (ret == EXIT_FIRMWARE_CRASH || ret == EXIT_INTERNAL_CRASH) {
            total_crash++;
            add_crash_trace(file_path);
        }
        delete[] buffer;

        if (option.check_coverage) {
            auto new_hit_coverage = board->new_hit_coverage_map();
            if (new_hit_coverage->size() > 0) {
                printf(
                    "INFO  | khost: new basic block found: time = %d, cov = %ld(+%ld), file: %s\n",
                    entry.first, 
                    board->hit_coverage_map()->size(), new_hit_coverage->size(), file_path.c_str()
                );
            }
        }
    }
    printf("INFO  | khost: the slowest input is %s\n", slow_input.c_str());
    printf("INFO  | khost: total crashes %d\n", total_crash);
    printf("INFO  | khost: find %ld unique crashes\n", unique_trace.size());
    for (int i = 0; i < unique_trace.size(); i++) {
        printf("[Unique Crash %02d] ==========\n", i+1);
        printf("file: %s\n", unique_trace[i].first.c_str());
        print_trace(&unique_trace[i].second);
    }
    if (unique_trace.size() > 0) {
        printf("============================\n");
    }
}

int main(int argc, char *argv[]) {
    parse_args(argc, argv);
    init_config();
    init_board();
    if (option.single_input) {
        single_run();
    } else {
        if (option.debug) {
            printf("ERROR | khost: debug is not support with multi inputs\n");
        }
        multi_run();
    }
    if (option.check_coverage) {
        check_coverage();
    }
    delete board;
    return 0;
}
