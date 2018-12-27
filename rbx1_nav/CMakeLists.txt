cmake_minimum_required(VERSION 2.8.3)

project(rbx1_nav)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure roscpp rospy message_generation std_msgs geometry_msgs nav_msgs)

add_message_files(
  FILES
  myPath.msg
  # Message2.msg
)



catkin_python_setup()

generate_messages(
 DEPENDENCIES
 geometry_msgs
 nav_msgs
)

generate_dynamic_reconfigure_options(cfg/CalibrateAngular.cfg cfg/CalibrateLinear.cfg)

catkin_package(CATKIN_DEPENDS dynamic_reconfigure message_runtime)
