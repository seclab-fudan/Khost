#! /bin/bash

# firmname
# "cjpeg" "core" "linear_alg-mid-100x100-sp" "nnet" "parser" "radix" "sha" "zip"
ROOT="./experiments/performance/coremark-pro-linux"
testcases=(
		"./$ROOT/cjpeg-rose7-preset.elf -v0 -i100"
		"./$ROOT/core.elf -v0 -i1"
		"./$ROOT/linear_alg-mid-100x100-sp.elf -v0 -i100"
		"./$ROOT/nnet_test.elf -v0 -i1"         
		"./$ROOT/parser-125k.elf -v0 -i100"
		"./$ROOT/radix2-big-64k.elf -v0 -i100"
		"./$ROOT/sha-test.elf -v0 -i1000"
		"./$ROOT/zip-test.elf -v0 -i10000"
)

for ((j=0; j<3; j++)) 
do
	echo "================== Round ${j} ==================="
	for ((i=0; i<8; i++))
	do
		echo "--------------- ${testcases[$i]} -------------"
		eval "${testcases[$i]}"
		sleep 1
	done
done
