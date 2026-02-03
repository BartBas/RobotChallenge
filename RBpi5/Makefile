# Variables
BUILD_DIR = build
TARGET = robot_exec

.PHONY: all Env-Build env build run clean

# Default: build and then run
all: build run

# 1. Install System-level compilers and tools
Env-Build:
	sudo apt update
	sudo apt install -y build-essential cmake git libcamera-dev pkg-config
	$(MAKE) lidar-sdk

# 2. Install library dependencies (assuming they are apt packages or python tools)
env:
	@echo "Installing libraries from requirements.txt..."
	@# This assumes your requirements.txt contains system package names
	@# If they are python-based, use: pip install -r requirements.txt --break-system-packages
	xargs -a requirements.txt sudo apt install -y

# 3. Standard CMake Build process
build:
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