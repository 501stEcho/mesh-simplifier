cmake_minimum_required(VERSION 3.12)
project(mesh-simplifier)

set(CMAKE_CXX_FLAGS -g3)

# Paths
set(INCLUDE_DIR ./include)
set(SRC_DIR ./src)

# Source files

set(SOURCES
    ./main.cpp
    ${SRC_DIR}/Parser.cpp
)

# Executables
set(EXECUTABLE mesh-simplifier)

add_executable(${EXECUTABLE} ${SOURCES})

# Include Directories
target_include_directories(${EXECUTABLE} PRIVATE ${INCLUDE_DIR})


# Output
set_target_properties(${EXECUTABLE} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)
