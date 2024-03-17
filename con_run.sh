#!/usr/bin/env sh

nice -n -19 ./control/build/main_node > result.txt
# gdb --args  ./control/build/main_node
# nice -n -19 ./control/build/main_node
