cmake_minimum_required(VERSION 2.8.3)

message(STATUS "Compiling using C++ 14 (required by EXOTica)")
add_compile_options(-std=c++14)

# MoveIt Core Robot Model isnt aligned :'(
#add_definitions(-DEIGEN_MAX_ALIGN_BYTES=0 -DEIGEN_DONT_VECTORIZE)
