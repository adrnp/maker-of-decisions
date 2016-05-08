cmake_minimum_required (VERSION 2.8)
project ("Maker of Decisions")

find_package (Threads)
add_definitions (-std=c++11)
include_directories ("${PROJECT_SOURCE_DIR}/mavlink")

# include all the directories containing the libraries
include_directories ("${PROJECT_SOURCE_DIR}/libs/*")
add_subdirectory ("libs/bearing")
add_subdirectory ("libs/serial")
add_subdirectory ("libs/udp")
add_subdirectory ("libs/planners")

# include the original planner directory
include_directories ("${PROJECT_SOURCE_DIR}/planner")
add_subdirectory ("planner")

# include the sensors directory
include_directories ("${PROJECT_SOURCE_DIR}/sensors")

# get all the files that make up mod
file (GLOB SOURCES "*.cpp")
file (GLOB SENSOR_SOURCES "sensors/*.cpp")

# set the output path for the resulting executable
set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

# make the executable
add_executable(mod ${SOURCES} ${SENSOR_SOURCES})
target_link_libraries (mod BearingLib)
target_link_libraries (mod SerialLib)
target_link_libraries (mod UDPLib)
target_link_libraries (mod PomdpLib)
target_link_libraries (mod PlannerLib)
target_link_libraries (mod ${CMAKE_THREAD_LIBS_INIT})