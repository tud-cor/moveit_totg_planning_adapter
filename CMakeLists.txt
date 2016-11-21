cmake_minimum_required(VERSION 2.8.3)
project(moveit_totg)

# C++ 11
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  roslint
  rosparam_shortcuts
  cmake_modules
  moveit_msgs
  moveit_ros_planning
  joint_trajectory_controller
)

find_package(Boost REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rosparam_shortcuts
    moveit_ros_planning
  DEPENDS
    # Eigen
  INCLUDE_DIRS
    # include
    external
  LIBRARIES
    totg_planning_adapter
)

###########
## Build ##
###########

include_directories(
  # include
  external
  ${catkin_INCLUDE_DIRS}
)

# --------------------------------------------
# TOTG planning adapter
add_library(totg_planning_adapter
  src/add_totg.cpp
  external/trajectories/Path.cpp
  external/trajectories/Trajectory.cpp
)
target_link_libraries(totg_planning_adapter
  ${catkin_LIBRARIES}
)

# Mark adapter for installation
install(TARGETS totg_planning_adapter
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES planning_request_adapters_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
