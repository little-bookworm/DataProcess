cmake_minimum_required(VERSION 3.0.2)
project(DataProcess)

add_definitions("-Wall -g")
add_compile_options(-std=c++14)

set(MODULE_NAME "DataProcess")

set(CMAKE_BUILD_TYPE  Debug)
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -O0 -Wfatal-errors -pthread -w -g")

find_package(OpenCV 3 REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  ${CMAKE_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_INCLUDE_DIRS}
)

link_directories(
)

##########################################################
##################### DataProcess lib ###################
##########################################################

file(GLOB_RECURSE cpp_srcs ${CMAKE_SOURCE_DIR}/src/*.cpp)

add_library(dataprocess SHARED ${cpp_srcs})

target_link_libraries(dataprocess
  ${OpenCV_LIBRARIES} 
  ${YAML_CPP_LIBRARIES} 
)

##########################################################
########################   Test   ########################
##########################################################

file(GLOB_RECURSE test_srcs ${CMAKE_SOURCE_DIR}/test/*.cpp)

add_executable(test ${test_srcs})

target_link_libraries(test
dataprocess
  ${OpenCV_LIBRARIES} 
)

##########################################################
#######################   Install   ######################
##########################################################
file(GLOB_RECURSE dataprocess_header ${CMAKE_SOURCE_DIR}/*.h ${CMAKE_SOURCE_DIR}/*.cuh)

set(CMAKE_INSTALL_MODULE_ROOT ${CMAKE_INSTALL_PREFIX}/${MODULE_NAME})
set(CMAKE_INSTALL_BINDIR ${CMAKE_INSTALL_MODULE_ROOT}/bin)
set(CMAKE_INSTALL_LIBDIR ${CMAKE_INSTALL_MODULE_ROOT}/lib)
set(CMAKE_INSTALL_HEADER ${CMAKE_INSTALL_MODULE_ROOT}/include)
message(STATUS "INSTALL bin ${CMAKE_INSTALL_BINDIR}")
message(STATUS "INSTALL lib ${CMAKE_INSTALL_LIBDIR}")
message(STATUS "INSTALL include ${CMAKE_INSTALL_HEADER}")

install(
  TARGETS test
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(
  TARGETS dataprocess
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(
  FILES ${dataprocess_header}
  DESTINATION ${CMAKE_INSTALL_HEADER}
)
