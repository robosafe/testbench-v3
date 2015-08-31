cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(OGRE OGRE)
  pkg_check_modules(OGRE-Terrain OGRE-Terrain)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS} ${OGRE_INCLUDE_DIRS} ${OGRE-Terrain_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS} ${OGRE_LIBRARY_DIRS})

#add_library(sysPlugTest SHARED sysPlugTest.cc)
#target_link_libraries(sysPlugTest ${GAZEBO_libraries} ${OGRE_LIBRARIES})

include(FindBoost)
find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)

find_package(Protobuf REQUIRED)
find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(sim_step_monitoring SHARED sim_step_monitoring.cc)
target_link_libraries(sim_step_monitoring ${GAZEBO_libraries} ${OGRE_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} pthread)

find_package(PythonLibs REQUIRED)
message(STATUS ${PYTHON_LIBRARIES})
message(STATUS ${PYTHON_INCLUDE_DIRS})
