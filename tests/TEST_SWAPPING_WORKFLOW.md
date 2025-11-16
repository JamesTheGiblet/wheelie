# Test Swapping Workflow

This project uses a script to easily swap the main application file (`src/main.cpp`) with any test file from the `tests/` directory. This allows you to run targeted tests on the hardware without manually renaming or moving files.

## Script: `swap_main_for_test.sh`

### Usage

```sh
./swap_main_for_test.sh <test_file_name.cpp>

```

- Replaces `src/main.cpp` with the specified test file from the `tests/` directory.
- Backs up the original `main.cpp` to `src/main.cpp.bak` (if not already backed up).
- Example:

  ```sh

  ./swap_main_for_test.sh test_motors_only.cpp
  ```

### Restore the Original `main.cpp`

```sh
./swap_main_for_test.sh restore

```

- Restores the original `main.cpp` from the backup (`src/main.cpp.bak`).
- If no backup exists, prints an error message.

### Notes

- The script must be run from the project root or with correct relative paths.
- If you swap in a test file, always restore the original `main.cpp` before committing or deploying production code.
- The script checks for the existence of the test file and backup before proceeding.

### Example Workflow

1. Swap in a test:

   ```sh
   ./swap_main_for_test.sh test_encoder_only.cpp

   ```

2. Build and upload the firmware as usual.
3. When finished, restore the original main:

   ```sh
   ./swap_main_for_test.sh restore
   ```

---

For more details, see the script source: [`swap_main_for_test.sh`](../swap_main_for_test.sh)
