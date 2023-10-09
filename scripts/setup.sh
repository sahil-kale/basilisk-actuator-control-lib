#!/bin/bash

set -euxo pipefail

sudo apt install git clang-format clang-tidy 

git submodule update --init --recursive

mkdir -p build/ > /dev/null

cd build

cmake .. -DCMAKE_BUILD_TYPE=Debug -G "Unix Makefiles"