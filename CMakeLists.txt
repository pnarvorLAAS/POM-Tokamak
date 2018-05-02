cmake_minimum_required(VERSION 3.0)
project(tokamak VERSION 0.1)

SET(SOURCES
    src/Tokamak.cpp
 )

add_library(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${ENVIRE_LIBRARIES})


SET(LIBRARY_NAME ${PROJECT_NAME})
SET(DEPENDCIES "envire_core pom-base")
SET(HEADERS_DIR "include")
configure_file(${CMAKE_SOURCE_DIR}/template.pc.in ${CMAKE_BINARY_DIR}/pkgconfig/${LIBRARY_NAME}.pc @ONLY)
