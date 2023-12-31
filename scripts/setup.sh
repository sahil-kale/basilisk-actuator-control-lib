#!/bin/bash

set -euxo pipefail

sudo apt install -y git clang-format clang-tidy build-essential gdb doxygen

git submodule update --init --recursive

mkdir -p build/ > /dev/null
