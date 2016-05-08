# get all the files that make up the pomdp planner lib
file(GLOB SOURCES "*.cpp")

add_library(PomdpLib ${SOURCES})