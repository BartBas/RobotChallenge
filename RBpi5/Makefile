BUILD_DIR = build
TARGET    = robot_exec
SETUP     = setup_tool

.PHONY: all env build run setup clean lidar-sdk micro

# Default: build then run
all: build run

# Install system-level dependencies
env:
	sudo apt update
	xargs -a requirements.txt sudo apt install -y --no-install-recommends
	$(MAKE) lidar-sdk

# Build YDLidar SDK (only needed once)
lidar-sdk:
	@if [ ! -d "YDLidar-SDK" ]; then \
		git clone https://github.com/YDLIDAR/YDLidar-SDK.git; \
	fi
	mkdir -p YDLidar-SDK/build
	cd YDLidar-SDK/build && cmake .. && make -j$(shell nproc) && sudo make install

# Build everything
build: clean
	mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && cmake .. && make -j$(shell nproc)

# Run main robot
run:
	./$(BUILD_DIR)/$(TARGET)

# Run setup tool (lidar deadzones + colour picker)
setup:
	./$(BUILD_DIR)/$(SETUP)

# Clean
clean:
	rm -rf $(BUILD_DIR)

# Quick edit shortcut
micro:
	micro src/main.cpp src/Config.cpp src/Config.h \
	      src/LidarController.* src/CamController.* \
	      src/MapController.*   src/MotorController.* \
	      config.txt
