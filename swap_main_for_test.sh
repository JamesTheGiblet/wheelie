#!/bin/bash
# Usage: ./swap_main_for_test.sh [restore]
# If called with 'restore', it restores the original main.cpp. Otherwise, it swaps in the test file.

SRC_DIR="src"
TESTS_DIR="tests"
BACKUP_FILE="$SRC_DIR/main.cpp.bak"
MAIN_FILE="$SRC_DIR/main.cpp"
TEST_FILE="$TESTS_DIR/test_ultrasonic_led.cpp"

if [ "$1" == "restore" ]; then
    if [ -f "$BACKUP_FILE" ]; then
        mv "$BACKUP_FILE" "$MAIN_FILE"
        echo "Restored original main.cpp."
    else
        echo "No backup found to restore."
    fi
else
    if [ -f "$MAIN_FILE" ]; then
        mv "$MAIN_FILE" "$BACKUP_FILE"
        echo "Backed up original main.cpp."
    fi
    cp "$TEST_FILE" "$MAIN_FILE"
    echo "Test file copied to src/main.cpp."
fi
