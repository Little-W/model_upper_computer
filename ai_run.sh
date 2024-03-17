#!/usr/bin/env sh


cd ./ai/build/
nice -n -18 ./detection > /home/edgeboard/ftp_share/code/405/ai.txt
# gdb --args ./new_ai/build/detection