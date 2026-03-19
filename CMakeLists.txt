cmake_minimum_required(VERSION 3.10)
project(Autonomous_RC_Car)

set(OpenCV_DIR /opt/opencv/lib/cmake/opencv4)
find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS})
add_executable(main Main.cpp)
target_link_libraries(main ${OpenCV_LIBS})

# Set C++ standard
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

# 1. Include directories (No more ../../ mess!)
include_directories(rpi/headers)
include_directories(config)
include_directories(utils)
include_directories(interfacing/headers)
include_directories(controls_experiments/headers)

# 2. Define the source files for the control logic
set(CONTROL_SOURCES 
    rpi/src/PurePursuit.cpp 
    utils/CustomUtils.cpp
    rpi/src/Perception.cpp
    rpi/src/AutonomyFSM.cpp
    rpi/src/RescueController.cpp
    interfacing/serial/Serial.cpp
)

# 3. Create the test executable
# This combines your test file with the actual logic implementation
add_executable(run_tests tests/main.cpp ${CONTROL_SOURCES})
target_link_libraries(run_tests ${OpenCV_LIBS})

add_executable(rpi_main rpi/src/main.cpp ${CONTROL_SOURCES})
target_link_libraries(rpi_main ${OpenCV_LIBS})

add_executable(perception_demo tests/perception_test.cpp ${CONTROL_SOURCES})
target_link_libraries(perception_demo ${OpenCV_LIBS})