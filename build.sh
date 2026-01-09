#! /bin/bash

if [ -d "./build" ]; then
    echo "use existing build dir"
else
    mkdir build
fi
cd build
cmake -DCMAKE_BUILD_TYPE=Release ../src 
# cmake -DCMAKE_BUILD_TYPE=Release ../src -DBUILD_SHARED_LIBS=OFF
maxcores=`echo $(nproc)-1|bc`
make -j$maxcores

if [ -f "./fuzzer/khost-afl-fuzz" -a -f "./runner/khost" ]; then
	echo "Build successfully."
else
	echo "Fail to build!"
fi
