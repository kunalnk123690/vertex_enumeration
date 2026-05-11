cmake_minimum_required(VERSION 3.16)
project(vertex_enumeration)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
add_compile_options(-march=native -O3 -w)

find_package(Eigen3 REQUIRED)

include_directories( 
  include
)

file(GLOB LIB_FILES lib/libqhull_r/*.c lib/libqhullcpp/*.cpp)
add_library(LIBQHULL ${LIB_FILES})

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} 
  LIBQHULL
  Eigen3::Eigen
)