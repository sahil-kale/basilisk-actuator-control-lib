#!/bin/bash

set -exo pipefail

# make a build directory if it doesn't exist
mkdir -p build

# Change to the build directory
cd build

# Check if the first argument is set
if [ -n "$1" ]
then
  # Check if the first argument is "build" or "test"
  if [ "$1" == "build" ]
  then
    # Build the code
    make all
  elif [ "$1" == "test" ]
  then
    # Run the tests
    ./test/tests
  else
    # If the argument is not "build" or "test", print an error message
    echo "Error: Invalid argument. Valid arguments are 'build', 'test', 'regen-cmake'."
    exit 1
  fi
else
  # If no argument is specified, build and run the tests
  cmake .. -DCMAKE_BUILD_TYPE=Debug -G "Unix Makefiles"
    make all && ./tests
fi
