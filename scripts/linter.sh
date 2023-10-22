mkdir -p cppcheckbuild

# Find all C, C++, and header files in the current directory
files=$(find . -name "*.c" -o -name "*.cpp" -o -name "*.h" -o -name "*.hpp")

# Remove any files located in build/
files=$(echo "$files" | grep -v "build/")
# Remove any files located in libs/
files=$(echo "$files" | grep -v "libs/")
# Remove any files located in tests/
files=$(echo "$files" | grep -v "test/")

# Store a variable to hold the suppressions
suppressions="--suppress=missingIncludeSystem --suppress=missingInclude --suppress=unusedFunction --suppress=unusedStructMember --suppress=unmatchedSuppression"

# Of the files glob, find only the hpp files
hpp_files=$(echo "$files" | grep -E "\.hpp$")

# Find the directories of the hpp files. Prepend the -I flag to each directory
hpp_dirs=$(echo "$hpp_files" | xargs -n1 dirname | sort | uniq | sed -e 's/^/-I/')

cppcheck $files --enable=all $suppressions $hpp_dirs --template=gcc --error-exitcode=1 --cppcheck-build-dir=cppcheckbuild