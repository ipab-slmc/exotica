cmake_minimum_required(VERSION 3.0.2)

message(STATUS "Compiling using C++ 14 (required by EXOTica)")
set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wfloat-conversion -Wall -Wno-maybe-uninitialized)
# add_compile_options(-Werror)

# MoveIt Core Robot Model isnt aligned :'(
#add_definitions(-DEIGEN_MAX_ALIGN_BYTES=0 -DEIGEN_DONT_VECTORIZE)
