cmake_minimum_required(VERSION 3.0)
project(tokamak VERSION 0.1)

SET(SOURCES
    src/Tokamak.cpp
    src/TokamakASN1.cpp
)

SET(HEADERS
    include/Tokamak.hpp
    include/TokamakASN1.hpp
    include/circularMap.hpp
    include/exception.hpp
    include/StateOfTransform.hpp
)


add_library(${PROJECT_NAME} SHARED ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${ENVIRE_LIBRARIES} asn1_pom_conversions cdff_types pom-base) 

add_dependencies( ${PROJECT_NAME}
    asn1_pom_conversions
    cdff_types
    pom-base
)


SET(LIBRARY_NAME ${PROJECT_NAME})
SET(DEPENDENCIES "envire_core pom-base cdff_types asn1_pom_conversions")
SET(HEADERS_DIR "include")
configure_file(${CMAKE_SOURCE_DIR}/template.pc.in ${CMAKE_BINARY_DIR}/pkgconfig/${LIBRARY_NAME}.pc @ONLY)

# Install script
set(HEADERS_INSTALL_DIR include/tokamak)
set(LIB_INSTALL_DIR lib)
include(${CMAKE_SOURCE_DIR}/CMakeModules/CDFF_at_LAAS-install.cmake)
