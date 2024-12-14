#!/bin/sh

if [ "$1" = "-f" ]; then
    rm -rf build
fi

mkdir build
cd build
cmake ..
cmake --build .