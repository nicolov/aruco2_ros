cmake_minimum_required(VERSION 2.8.3)
project(aruco2_catkin)

find_package(catkin_simple REQUIRED)
catkin_simple()

set(ARUCO_VERSION 2.0.7)

include(ExternalProject)
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)
ExternalProject_Add(aruco2_src
  URL http://downloads.sourceforge.net/project/aruco/${ARUCO_VERSION}/aruco-${ARUCO_VERSION}.zip
  CONFIGURE_COMMAND cmake -DUSE_OWN_EIGEN3=OFF -DCMAKE_INSTALL_PREFIX:PATH=${CATKIN_DEVEL_PREFIX} -DCMAKE_BUILD_TYPE=Release ../aruco2_src
  BUILD_COMMAND $(MAKE)
  INSTALL_COMMAND make install
  )

cs_add_library(${PROJECT_NAME} src/lib_wrap.cc)
add_dependencies(${PROJECT_NAME} aruco2_src)
target_link_libraries(${PROJECT_NAME}
  ${CATKIN_DEVEL_PREFIX}/lib/libaruco${CMAKE_SHARED_LIBRARY_SUFFIX})

cs_install()
cs_export(
  INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
  CFG_EXTRAS aruco2-extras.cmake
  )
