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

# include the planners directory
include_directories ("${PROJECT_SOURCE_DIR}/planners")
include_directories ("${PROJECT_SOURCE_DIR}/planner")
add_subdirectory ("planner")

# set the output path for the resulting executable
set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

# get all the files that make up mod
file (GLOB SOURCES "*.cpp")
file (GLOB PLANNER_SOURCES "planners/*.cpp")

# make the executable
add_executable(mod ${SOURCES}
					${PLANNER_SOURCES})
target_link_libraries (mod BearingLib)
target_link_libraries (mod SerialLib)
target_link_libraries (mod UDPLib)
target_link_libraries (mod PomdpLib)
target_link_libraries (mod ${CMAKE_THREAD_LIBS_INIT})