cmake_minimum_required(VERSION 2.8.3)
project(moveit_totg_planning_request_adapter)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  pluginlib
  roscpp
)

find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

# enable C++11 if needed for MoveIt on Kinetic
if ("${moveit_core_VERSION}" VERSION_GREATER "0.9.0")
  add_definitions(-std=c++11)
endif()

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning
    pluginlib
    roscpp
  DEPENDS
    EIGEN3
  LIBRARIES
    ${PROJECT_NAME}
)

include_directories(external ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  external/trajectories/Path.cpp
  external/trajectories/Trajectory.cpp
  src/add_totg.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(FILES planning_request_adapters_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
