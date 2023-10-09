#!/bin/bash

# find all C, C++, and header files in the current directory and subdirectories
files=$(find . -name "*.c" -o -name "*.cpp" -o -name "*.h" -o -name "*.hpp")

# Remove any files located in build/
files=$(echo "$files" | grep -v "build/")
# Remove any files located in libs/
files=$(echo "$files" | grep -v "libs/")

# Loop through the files and format them with clang-format
for file in $files
do
  clang-format -i "$file"
done