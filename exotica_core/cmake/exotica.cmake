cmake_minimum_required(VERSION 3.0.2)

add_compile_options(-Wfloat-conversion -Wall)
# gcc only: -Wno-maybe-uninitialized
# add_compile_options(-Werror)

# MoveIt Core Robot Model isnt aligned :'(
#add_definitions(-DEIGEN_MAX_ALIGN_BYTES=0 -DEIGEN_DONT_VECTORIZE)
