#!/bin/bash
PROJDIR=/home/halfuzz/hal-fuzz
FUZZDIR="$(dirname "$(readlink -f "$0")")"
FIRMNAME=$(echo $FUZZDIR | awk -F'/' '{print $NF}')
CONFIG="${FIRMNAME:3}".yml
BINARY=$FUZZDIR/$CONFIG
INPUTS=$FUZZDIR/corpus
OUTPUTS=$FUZZDIR/output/
FUZZ=$PROJDIR/AFLplusplus/afl-fuzz
HOOK=$PROJDIR/hal_fuzz/native/native_hooks.so
HARNESS="python3 -m hal_fuzz.harness -t -c $BINARY"
# HARNESS="python3 -m hal_fuzz.harness -t -c $BINARY --native-lib=$HOOK"

echo "AFL_I_DONT_CARE_ABOUT_MISSING_CRASHES=1 AFL_SKIP_BIN_CHECK=1 $FUZZ -U -T $FIRMNAME -t 1000 -m none -i $INPUTS -o $OUTPUTS -- $HARNESS @@"
AFL_I_DONT_CARE_ABOUT_MISSING_CRASHES=1 AFL_SKIP_BIN_CHECK=1 $FUZZ -U -T $FIRMNAME -t 1000 -m none -i $INPUTS -o $OUTPUTS -- $HARNESS @@
