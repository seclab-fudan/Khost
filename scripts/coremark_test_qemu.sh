#! /bin/bash

# build qemu tcg watcher
KHOST_ROOT=$(realpath ".")
QEMU_ROOT=$KHOST_ROOT"/qemu-8.2.10" # path to QEMU root dir
PATH_COREMARK=$KHOST_ROOT"/experiments/performance/coremark-pro-mcu"
TCG_PLUGIN_SRC=$PATH_COREMARK"/tcg_plugin.c"
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
testcases=("cjpeg" "core" "linear" "nnet" "parser" "radix" "sha" "zip")

for ((j=0; j<3; j++)) 
do
	echo "################## Round ${j} ###################"
	for ((i=0; i<8; i++))
	do
		firmware=$PATH_COREMARK"/"${testcases[$i]}"/coremark-pro.elf"
		echo "Ananlyzing $firmware"
		start_addr="0x"$(arm-none-eabi-objdump -d $firmware  | grep -E "start_clock>:" | awk -F ' ' '{print $1}' | sed 's/^0*//')
		stop_addr="0x"$(arm-none-eabi-objdump -d $firmware  | grep -E "stop_clock>:" | awk -F ' ' '{print $1}' | sed 's/^0*//')
		time $QEMU_ROOT/build/qemu-system-arm -nographic -monitor tcp:127.0.0.1:5888,server,nowait -machine netduinoplus2 -serial stdio -kernel $firmware -plugin $TCG_PLUGIN,$start_addr=on,$stop_addr=on
		sleep 1
	done

done
