#!/bin/bash
# Usage:
#   ./swap_main_for_test.sh <test_name.cpp>
#   ./swap_main_for_test.sh restore
#
# Swaps 'src/main.cpp' with a test file from the 'tests/' directory.
# The <test_name.cpp> is the name of the cpp file in 'tests/'.
# Example: ./swap_main_for_test.sh test_motors.cpp
#
# Use 'restore' to bring back the original main.cpp.

SRC_DIR="src"
TESTS_DIR="tests"
BACKUP_FILE="$SRC_DIR/main.cpp.bak"
MAIN_FILE="$SRC_DIR/main.cpp"

# Restore operation
if [ "$1" = "restore" ]; then
    if [ -f "$BACKUP_FILE" ]; then
        mv "$BACKUP_FILE" "$MAIN_FILE"
        echo "Restored original main.cpp."
    else
        echo "No backup found to restore."
    fi
    exit 0
fi

# Swap operation
if [ -z "$1" ]; then
    echo "Error: No test file specified."
    echo "Usage: $0 <test_file_name.cpp>"
    echo "   or: $0 restore"
    exit 1
fi

TEST_FILE="$TESTS_DIR/$1"

if [ ! -f "$TEST_FILE" ]; then
    echo "Error: Test file not found at '$TEST_FILE'"
    exit 1
fi

# Backup the original main.cpp if it exists and isn't already a backup
if [ -f "$MAIN_FILE" ] && [ ! -f "$BACKUP_FILE" ]; then
    mv "$MAIN_FILE" "$BACKUP_FILE"
    echo "Backed up original main.cpp to main.cpp.bak"
fi

cp "$TEST_FILE" "$MAIN_FILE"
echo "Swapped '$1' in as main.cpp."
