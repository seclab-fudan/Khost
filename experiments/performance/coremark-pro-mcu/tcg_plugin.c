#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/time.h>
#include <qemu-plugin.h>

QEMU_PLUGIN_EXPORT int qemu_plugin_version = QEMU_PLUGIN_VERSION;

static uint64_t watch_addr1 = 0;
static uint64_t watch_addr2 = 0;

static int hit1 = 0;
static int hit2 = 0;

static long start_time = 0;
static long finish_time = 0;

static long get_current_time() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    long time_ms = tv.tv_sec * 1000 + tv.tv_usec / 1000;
	return time_ms;
}

static void insn_exec_cb(qemu_plugin_id_t id, void *userdata) {
    uint64_t addr = (uintptr_t)userdata;
    if (addr == watch_addr1 && !hit1) {
        hit1 = 1;
		start_time = get_current_time();
    	fprintf(stdout, "[Start ] First exec at 0x%lx | Time: %ld ms\n", addr, start_time);
    } else if (addr == watch_addr2 && !hit2) {
        hit2 = 1;
		finish_time = get_current_time();
    	fprintf(stdout, "[Finish] First exec at 0x%lx | Time: %ld ms\n", addr, finish_time);
    	fprintf(stdout, "[Time Interval]: %ld ms\n", finish_time - start_time);
		exit(0);
    }
}

static void tb_trans_cb(qemu_plugin_id_t id, struct qemu_plugin_tb *tb) {
    size_t n = qemu_plugin_tb_n_insns(tb);
    for (size_t i = 0; i < n; i++) {
        struct qemu_plugin_insn *insn = qemu_plugin_tb_get_insn(tb, i);
        uint64_t addr = qemu_plugin_insn_vaddr(insn);

        if (addr == watch_addr1 || addr == watch_addr2) {
            qemu_plugin_register_vcpu_insn_exec_cb(insn, insn_exec_cb,
                                                   QEMU_PLUGIN_CB_NO_REGS,
                                                   (void *)(uintptr_t)addr);
        }
    }
}

static void plugin_init(qemu_plugin_id_t id,
                        const qemu_info_t *info,
                        int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "Error: You must specify two addresses using arg=0xADDR format.\n");
        exit(1);
    }

    watch_addr1 = strtoull(argv[0], NULL, 0);
    watch_addr2 = strtoull(argv[1], NULL, 0);

    fprintf(stderr, "Plugin watching ADDR1 = 0x%lx, ADDR2 = 0x%lx\n", watch_addr1, watch_addr2);

    qemu_plugin_register_vcpu_tb_trans_cb(id, tb_trans_cb);
}

QEMU_PLUGIN_EXPORT int qemu_plugin_install(qemu_plugin_id_t id,
                                           const qemu_info_t *info,
                                           int argc, char **argv) {
    plugin_init(id, info, argc, argv);
    return 0;
}

