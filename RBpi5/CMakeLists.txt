cmake_minimum_required(VERSION 3.10)
project(RobotProject)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-write-strings -Wno-terminate")

find_package(OpenCV REQUIRED)

# Add the SDK folder
add_subdirectory(YDLidar-SDK)

file(GLOB SOURCES "src/*.cpp")
add_executable(robot_exec ${SOURCES})

# Updated include paths to match the YDLidar-SDK structure
target_include_directories(robot_exec PRIVATE 
    include
    /usr/include
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/YDLidar-SDK/src
    ${CMAKE_CURRENT_SOURCE_DIR}/YDLidar-SDK/include
    ${CMAKE_CURRENT_SOURCE_DIR}/YDLidar-SDK/core
    ${CMAKE_CURRENT_SOURCE_DIR}/YDLidar-SDK/core/common  # <--- Add this line
)

target_link_libraries(robot_exec 
    ${OpenCV_LIBS} 
    ydlidar_sdk 
    pthread
)
