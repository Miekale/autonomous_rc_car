cmake_minimum_required(VERSION 3.10)
project(Autonomous_RC_Car)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. Include directories (No more ../../ mess!)
include_directories(rpi/headers)
include_directories(arduino/headers)
include_directories(config)

# 2. Define the source files for the control logic
set(CONTROL_SOURCES 
    rpi/src/PurePursuit.cpp 
    arduino/src/PurePursuitControls.cpp
)

# 3. Create the test executable
# This combines your test file with the actual logic implementation
add_executable(run_tests tests/main.cpp ${CONTROL_SOURCES})