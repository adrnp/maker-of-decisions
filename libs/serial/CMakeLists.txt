# get all the files that make up serial lib
file(GLOB SOURCES "*.cpp")

add_library(SerialLib ${SOURCES})