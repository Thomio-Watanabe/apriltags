cmake_minimum_required(VERSION 2.6)
project(apriltags)

## Locate external packages Eigen3 and OpenCV
find_package(PkgConfig)
pkg_check_modules(PC_Eigen3 REQUIRED eigen3)
include_directories(${PC_Eigen3_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Look for header files in src/ directory
include_directories(src)

# Get all source files
file(GLOB SOURCE_FILES "src/*.cc")

# Create shared library
add_library(apriltags SHARED ${SOURCE_FILES})
target_link_libraries(apriltags ${OpenCV_LIBS})

#add_subdirectory(example)

# Install headers and shared library.
install(DIRECTORY src/
        DESTINATION include/apriltags
        FILES_MATCHING PATTERN "*.h")

install(TARGETS apriltags
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)


# uninstall target
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
