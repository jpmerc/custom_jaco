cmake_minimum_required(VERSION 2.8.3)
project(custom_jaco)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  pcl_msgs
)

find_package( PCL REQUIRED )
include_directories( ${catkin_INCLUDE_DIRS}  ${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include #${PCL_INCLUDE_DIRS}
 #  LIBRARIES ${PCL_LIBRARY_DIRS}
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


add_executable(grasp src/grasp.cpp)
target_link_libraries(grasp ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(grasp custom_jaco_gencpp)

add_executable(jaco_joint_states src/jaco_joint_states.cpp)
target_link_libraries(jaco_joint_states ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(jaco_joint_states custom_jaco_gencpp)

add_executable(calibration src/calibration.cpp)
target_link_libraries(calibration ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(calibration custom_jaco_gencpp)

add_executable(teach_repeat src/teach_repeat.cpp)
target_link_libraries(teach_repeat ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(teach_repeat custom_jaco_gencpp)

#add_executable(perception
#  src/nodes/perception.cpp
#  src/objectExtractor.cpp
#  src/jaco.cpp
#  src/teacher.cpp
#)
#target_link_libraries(perception ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#add_dependencies(perception custom_jaco_gencpp)
