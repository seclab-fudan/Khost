#! /bin/bash

# build qemu tcg watcher
KHOST_ROOT=$(realpath ".")
QEMU_ROOT=$KHOST_ROOT"/qemu-8.2.10"
PATH_SIMBENCH=$KHOST_ROOT"/experiments/performance/simbench-mcu"
TCG_PLUGIN_SRC=$PATH_SIMBENCH"/tcg_plugin.c"
TCG_PLUGIN="/tmp/tcg_plugin.so"
if [ -e $TCG_PLUGIN ]
then
	rm $TCG_PLUGIN
fi

gcc -Wno-incompatible-pointer-types -fPIC -shared -o $TCG_PLUGIN $TCG_PLUGIN_SRC  \
-I $QEMU_ROOT/include/qemu \
-I $QEMU_ROOT \
-DQEMU_PLUGIN


# firmname
testcases=("mem-hot" "mem-cold" "coprocessor" "device-access" "dfault" "ifault" "syscall" "undef" "inter-page-direct" "inter-page-indirect" "same-page-direct" "same-page-indirect" "large-blocks" "small-blocks")

ROUND=1
for ((j=0; j<ROUND; j++))
do
    echo "################### Round ${j} ########################"
    for ((i=0; i<14; i++))
    do
        firmware=$PATH_SIMBENCH"/"${testcases[$i]}"/simbench.elf"
        echo "Analyzing $firmware"
        start_addr="0x"$(arm-none-eabi-objdump -d $firmware  | grep -E "arch_priv_enter>:" | awk -F ' ' '{print $1}' | sed 's/^0*//')
        stop_addr="0x"$(arm-none-eabi-objdump -d $firmware  | grep -E "arch_priv_leave>:" | awk -F ' ' '{print $1}' | sed 's/^0*//')
        time $QEMU_ROOT/build/qemu-system-arm -nographic -monitor tcp:127.0.0.1:5888,server,nowait -machine netduinoplus2 -serial stdio -kernel $firmware -plugin $TCG_PLUGIN,$start_addr=on,$stop_addr=on
    done
done

