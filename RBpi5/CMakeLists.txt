cmake_minimum_required(VERSION 3.16)
project(RobotProject)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-write-strings -Wno-terminate")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# YDLidar SDK
add_subdirectory(YDLidar-SDK)

# ── Shared source files (used by both targets) ──────────────────────
set(SHARED_SOURCES
    src/CamController.cpp
    src/LidarController.cpp
    src/MotorController.cpp
    src/MapController.cpp
    src/Config.cpp
)

set(SHARED_INCLUDES
    include
    /usr/include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/YDLidar-SDK/src
    ${CMAKE_CURRENT_SOURCE_DIR}/YDLidar-SDK/include
    ${CMAKE_CURRENT_SOURCE_DIR}/YDLidar-SDK/core
    ${CMAKE_CURRENT_SOURCE_DIR}/YDLidar-SDK/core/common
)

# ── Main robot binary ────────────────────────────────────────────────
add_executable(robot_exec
    src/main.cpp
    ${SHARED_SOURCES}
)
target_include_directories(robot_exec PRIVATE ${SHARED_INCLUDES})
target_link_libraries(robot_exec
    ${OpenCV_LIBS}
    ydlidar_sdk
    Eigen3::Eigen
    pthread
)

# ── Setup tool ───────────────────────────────────────────────────────
add_executable(setup_tool
    setup/main.cpp
    ${SHARED_SOURCES}
)
target_include_directories(setup_tool PRIVATE
    ${SHARED_INCLUDES}
    ${CMAKE_CURRENT_SOURCE_DIR}/src   # so setup/main.cpp can #include "../src/..."
)
target_link_libraries(setup_tool
    ${OpenCV_LIBS}
    ydlidar_sdk
    Eigen3::Eigen
    pthread
)
