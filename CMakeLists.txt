cmake_minimum_required(VERSION 3.12)
project(mesh-simplifier)

set(CMAKE_CXX_FLAGS -g3)

# Packages

find_package(Eigen3 3.3 REQUIRED)

# Paths
set(INCLUDE_DIR ./include)
set(SRC_DIR ./src)

# Source files

set(SOURCES
${INCLUDE_DIR}/Types.cpp
    ${SRC_DIR}/Parser.cpp
    ${SRC_DIR}/QEMSimplifier.cpp
    ${SRC_DIR}/Writer.cpp
    ${SRC_DIR}/Tools.cpp
)

# Executables
set(EXECUTABLE mesh-simplifier)
set(TEST_EXECUTABLE unit_tests)

add_executable(${EXECUTABLE} ./main.cpp ${SOURCES})
add_executable(${TEST_EXECUTABLE} test.cpp ${SOURCES})

# Include Directories
target_include_directories(${EXECUTABLE} PRIVATE ${INCLUDE_DIR})
target_include_directories(${TEST_EXECUTABLE} PRIVATE ${INCLUDE_DIR})

# Linked Libraries
target_link_libraries(${EXECUTABLE} PRIVATE Eigen3::Eigen)
target_link_libraries(${TEST_EXECUTABLE} PRIVATE Eigen3::Eigen)

# Output
set_target_properties(${EXECUTABLE} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

set_target_properties(${TEST_EXECUTABLE} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)
