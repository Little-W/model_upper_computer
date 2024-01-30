#!/usr/bin/env sh
cd ai
rm -r build
mkdir build
cd build
cmake ..
make -j2 -l2
cd ../..
