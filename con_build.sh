#!/usr/bin/env sh
#export CXX=/usr/bin/g++-8
#export CC=/usr/bin/gcc-8
cd control
rm -r build
mkdir build
cd build
cmake ..
make -j8 -l8
cd ../..
