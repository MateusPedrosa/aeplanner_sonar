cmake_minimum_required(VERSION 2.8.3)
project(aeplanner)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  visualization_msgs
  message_generation
  tf
  tf2
  genmsg 
  actionlib_msgs 
  actionlib

  kdtree
  pigain
  dynamic_reconfigure
  la3dm
  pcl_conversions
  pcl_ros
)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES Node.msg Target.msg TargetList.msg
)
add_action_files(
  DIRECTORY action 
  FILES aeplanner.action
)
add_service_files(
  DIRECTORY srv
  FILES Reevaluate.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  visualization_msgs
  std_msgs
  actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
  LIBRARIES aeplanner ${catkin_LIBRARIES}
  CATKIN_DEPENDS message_runtime roscpp geometry_msgs visualization_msgs tf kdtree
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

set(AEPLANNER_SOURCES
  src/aeplanner_nodelet.cpp
  src/aeplanner.cpp
  src/aeplanner_viz.cpp
  src/param.cpp
  src/voxel_classifier.cpp
  src/frontier_detector.cpp
  src/priority_scorer.cpp
  src/blacklist.cpp
  src/target_priority_map.cpp
  src/hemisphere_sampler.cpp
  src/frontier_sampler.cpp
  src/pose_scorer.cpp
  src/planner_state_machine.cpp
  src/dwell_controller.cpp
  src/global_target_selector.cpp
  src/reachability_checker.cpp
)

add_library(aeplanner ${AEPLANNER_SOURCES})
target_link_libraries(aeplanner
  ${catkin_LIBRARIES}
)
add_dependencies(aeplanner
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  aeplanner_ref_generate_messages_cpp
  ${PROJECT_NAME}_gencfg
)

set(AEPLANNER_NODE_SOURCES
  src/aeplanner_node.cpp
  src/aeplanner.cpp
  src/aeplanner_viz.cpp
  src/param.cpp
  src/voxel_classifier.cpp
  src/frontier_detector.cpp
  src/priority_scorer.cpp
  src/blacklist.cpp
  src/target_priority_map.cpp
  src/hemisphere_sampler.cpp
  src/frontier_sampler.cpp
  src/pose_scorer.cpp
  src/planner_state_machine.cpp
  src/dwell_controller.cpp
  src/global_target_selector.cpp
  src/reachability_checker.cpp
)

add_executable(aeplanner_node ${AEPLANNER_NODE_SOURCES})
target_link_libraries(aeplanner_node
  ${catkin_LIBRARIES}
)
add_dependencies(aeplanner_node
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  aeplanner_ref_generate_messages_cpp
  ${PROJECT_NAME}_gencfg
)
