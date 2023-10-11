#!/bin/bash

set -euxo pipefail

sudo apt install git clang-format clang-tidy build-essential

git submodule update --init --recursive

mkdir -p build/ > /dev/null
