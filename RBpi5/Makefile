# Variables
BUILD_DIR = build
TARGET = robot_exec

.PHONY: all Env-Build env build run clean

# Default: build and then run
all: build run

# 1. Install System-level compilers and tools
env:
	sudo apt update
	sudo apt install -y --no-install-recommends \
		build-essential cmake git pkg-config clinfo \
		libcamera-dev libopencv-dev python3-opencv
	xargs -a requirements.txt sudo apt install -y
	$(MAKE) lidar-sdk
	

# 3. Standard CMake Build process
build: clean
	mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && cmake .. && make -j$(shell nproc)

# 4. Run the compiled binary
run:
	./$(BUILD_DIR)/$(TARGET)

# Clean up build files
clean:
	rm -rf $(BUILD_DIR)

lidar-sdk:
	@if [ ! -d "YDLidar-SDK" ]; then \
		git clone https://github.com/YDLIDAR/YDLidar-SDK.git; \
	fi
	mkdir -p YDLidar-SDK/build
	cd YDLidar-SDK/build && cmake .. && make -j$(shell nproc) && sudo make install
