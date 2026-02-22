cmake_minimum_required(VERSION 3.22)
project(exotica_collision_scene_fcl_latest)

find_package(ament_cmake REQUIRED)
find_package(exotica_core REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(fcl REQUIRED)

AddInitializer(collision_scene_fcl_latest)
GenInitializers()

add_library(${PROJECT_NAME} SHARED src/collision_scene_fcl_latest.cpp)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME} exotica_core geometric_shapes)
target_link_libraries(${PROJECT_NAME} fcl)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_initializers)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES exotica_plugins.xml DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(exotica_core geometric_shapes fcl)

pluginlib_export_plugin_description_file(exotica_core exotica_plugins.xml)

ament_package()
