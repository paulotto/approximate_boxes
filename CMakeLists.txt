cmake_minimum_required(VERSION 3.10)

project(
        ApproximateBoxes
        VERSION 1.0.0
        DESCRIPTION "A way to approximate a volume with cubes using octrees."
        HOMEPAGE_URL "https://github.com/paulotto/approximate_boxes"
        LANGUAGES CXX)
message(STATUS "Project: ${PROJECT_NAME} ${PROJECT_VERSION}")
message(STATUS "${PROJECT_DESCRIPTION}")
message(STATUS "${PROJECT_HOMEPAGE_URL}")

if (${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message(FATAL_ERROR "In-source build is not possible and not recommended. Choose an empty directory for build output.")
endif (${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})

if (CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)

    # Optionally set things like CMAKE_CXX_STANDARD, CMAKE_POSITION_INDEPENDENT_CODE here
    set(CMAKE_CXX_STANDARD 17)

    # Let's ensure -std=c++xx instead of -std=g++xx
    set(CMAKE_CXX_EXTENSIONS OFF)

    # Let's nicely support folders in IDEs
    set_property(GLOBAL PROPERTY USE_FOLDERS ON)
endif ()

# Find CGAL
find_package(CGAL REQUIRED)

if (CGAL_FOUND)
    include(${CGAL_USE_FILE})
else ()
    message(FATAL_ERROR "This program requires the CGAL library, and will not be compiled.")
endif ()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

# Include directories for header files
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Add all the .cpp files in the src/ directory to the build
file(GLOB SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/include/approximate_boxes/*.h")

# Make an automatic library - will be static or dynamic based on user setting
add_library(libApproximateBoxes STATIC ${SOURCES} ${HEADERS})
set_target_properties(libApproximateBoxes PROPERTIES OUTPUT_NAME "ApproximateBoxes")

target_include_directories(libApproximateBoxes PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/approximate_boxes>"
        "$<INSTALL_INTERFACE:include/approximate_boxes>")
target_link_libraries(libApproximateBoxes CGAL::CGAL)

# Add an executable
add_executable(ApproximateBoxesExample example/example.cpp)

target_link_libraries(ApproximateBoxesExample libApproximateBoxes)
