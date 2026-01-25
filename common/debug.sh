#!/bin/bash

# solid chunk of this code was written by AI but it mostly finds where files
# are located in the Bazel build output and feeds them to OpenOCD. I had a lot
# of issues finding the correct folder paths but I can make this more robust if 
# needed -Jacob Likins

ELF_FILE="$1"

# the Absolute Path of the ELF
ELF_ABS=$(realpath "$ELF_FILE")

# Switch to the correct Project Root
if [ -n "$BUILD_WORKING_DIRECTORY" ]; then # bazel gives us this
    cd "$BUILD_WORKING_DIRECTORY"
fi
REPO_ROOT=$(pwd)

# Calculates the Local Source Directory
SOURCE_DIR=$(dirname "$1" | sed 's|bazel-bin/||' | sed 's|bazel-out/[^/]*/bin/||') 


# Bazel creates a symlink named "bazel-<folder_name>" that points to the cache.
# find it to locate where the HAL driver sources are hiding.
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

# Start GDB with External Mapping
eval arm-none-eabi-gdb -tui "$ELF_ABS" \
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