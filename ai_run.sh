#!/usr/bin/env sh


cd ./ai/build/
nice -n -18 ./detection > ../../ai.txt
# gdb --args ./new_ai/build/detection