cmake_minimum_required(VERSION 2.8.3)
project(moveit_check)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  COMPONENTS
    interactive_markers
    moveit_core
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    unr_object_manipulation
    geometry_msgs
    baxter_interface
)

find_package(Boost REQUIRED COMPONENTS 
  date_time
  thread
)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   object_position.msg
   object_position_list.msg
   colorPos2d.msg
   color_pos_2d_array.msg
#   Message2.msg
 )

## Generate services in the 'srv' folder
add_service_files(
  FILES
    object_position_ar_tag.srv
    object_position_ar_tag_list.srv
    object_position_ar_tag_hand.srv
    object_position_color.srv

)
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
  
)
## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here


## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES table_setting_demo
  CATKIN_DEPENDS roscpp rospy message_runtime geometry_msgs active_vision_msgs moveit_ros_planning_interface tf 
#  DEPENDS system_lib
)


include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})


