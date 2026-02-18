#!/bin/bash

# solid chunk of this code was written by AI but it mostly finds where files
# are located in the Bazel build output and feeds them to OpenOCD. I had a lot
# of issues finding the correct folder paths but I can make this more robust if 
# needed -Jacob Likins

# Check if openocd installed
if ! command -v openocd &> /dev/null; then
    echo "ERROR: 'openocd' is not installed."
    echo "Please run: sudo apt install openocd"
    exit 1
fi

# this should fix the issue with new ubuntu
if command -v gdb-multiarch &> /dev/null; then
    GDB_CMD="gdb-multiarch"
elif command -v arm-none-eabi-gdb &> /dev/null; then
    GDB_CMD="arm-none-eabi-gdb"
else
    echo "Please run: sudo apt install gdb-multiarch"
    exit 1
fi


ELF_FILE="$1"

# the Path of the ELF
ELF_ABS=$(realpath "$ELF_FILE")

# Switch to the correct Project Root
if [ -n "$BUILD_WORKING_DIRECTORY" ]; then
    cd "$BUILD_WORKING_DIRECTORY"
fi
REPO_ROOT=$(pwd)

# Calculates the Local Source Directory
SOURCE_DIR=$(dirname "$1" | sed 's|bazel-bin/||' | sed 's|bazel-out/[^/]*/bin/||') 

WORKSPACE_NAME=$(basename "$REPO_ROOT")
BAZEL_SYMLINK="bazel-$WORKSPACE_NAME"

if [ -L "$BAZEL_SYMLINK" ]; then
    EXTERNAL_LIBS_PATH="$REPO_ROOT/$BAZEL_SYMLINK/external"
    echo "Found External Libs: $EXTERNAL_LIBS_PATH"
    HAL_MAP_1="-ex \"set substitute-path external $EXTERNAL_LIBS_PATH\""
    HAL_MAP_2="-ex \"set substitute-path /proc/self/cwd/external $EXTERNAL_LIBS_PATH\""
else
    echo "Warning: Could not find bazel symlink. HAL debugging might fail."
    HAL_MAP_1=""
    HAL_MAP_2=""
fi

echo "--------------------------------------------------"
echo " Debugging: $ELF_FILE"
echo " Repo Root: $REPO_ROOT"
echo " Using GDB: $GDB_CMD"
echo "--------------------------------------------------"

# Kill Zombies
pkill -x openocd 

# Cleanup Trap
cleanup() {
    if [ -n "$OPENOCD_PID" ]; then
        kill $OPENOCD_PID 2>/dev/null
    fi
}
trap cleanup EXIT

# Start OpenOCD
openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c "init; reset halt" > /dev/null 2>&1 &
OPENOCD_PID=$!
sleep 2

# Start GDB using the detected command
eval $GDB_CMD -tui "$ELF_ABS" \
    -ex \"target remote localhost:3333\" \
    -ex \"monitor reset halt\" \
    -ex \"load\" \
    -ex \"dir .\" \
    -ex \"dir $SOURCE_DIR\" \
    $HAL_MAP_1 \
    $HAL_MAP_2 \
    -ex \"break main\" \
    -ex \"continue\" \
    -ex \"layout src\"