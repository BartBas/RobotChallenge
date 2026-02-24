cmake_minimum_required(VERSION 3.10)

# 1. This must be here!
project(RobotProject)

# 2. Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 3. Find OpenCV package
find_package(OpenCV REQUIRED)

# 4. Tell CMake where your header files are
include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})

# 5. List your source files
file(GLOB SOURCES "src/*.cpp")

# 6. Create the executable (The name MUST match TARGET in your Makefile)
add_executable(robot_exec ${SOURCES})

# 7. Link libraries - USE robot_exec NOT ${PROJECT_NAME}
target_link_libraries(robot_exec ${OpenCV_LIBS})
# target_link_libraries(robot_exec YDLidar-SDK)  # Uncomment when needed
