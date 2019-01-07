cmake_minimum_required(VERSION 2.8.3)
project(exotica)

find_package(catkin REQUIRED)

# include(ExternalProject)
#  ExternalProject_Add(exotica_doc
#   DOWNLOAD_COMMAND ""
#   UPDATE_COMMAND ""
#   PATCH_COMMAND ""
#   TEST_COMMAND ""
#   CONFIGURE_COMMAND ""
#   INSTALL_COMMAND ""

#   BUILD_COMMAND sphinx-build -d ${PROJECT_BINARY_DIR}/doc/doctrees -b html ${CMAKE_CURRENT_SOURCE_DIR}/doc ${PROJECT_BINARY_DIR}/doc/html
#   #BUILD_ALWAYS Yes
# )

# ExternalProject_Add_Step(exotica_doc
#   MOVEToShared
#   COMMENT "Install docs to share directory."
#   COMMAND ${CMAKE_COMMAND} -E copy_directory ${PROJECT_BINARY_DIR}/doc/html/ ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/doc
#   DEPENDEES install
# )

# catkin_package()

catkin_metapackage()

# add_executable(dummy src/dummy.cpp)
# add_dependencies(dummy exotica_doc)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )
