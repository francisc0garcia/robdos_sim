cmake_minimum_required(VERSION 2.8.3)
project(robdos_sim)

# C++14:
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        mavros_msgs
        gazebo_ros
        message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   state.msg
 )

# Actually generate the language-specific message and service files
generate_messages(DEPENDENCIES std_msgs)

# Depend on system install of Gazebo
find_package(
  gazebo REQUIRED
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES robdos_sim
        CATKIN_DEPENDS roscpp rospy std_msgs mavros_msgs interactive_markers visualization_msgs message_runtime
        DEPENDS  roscpp gazebo_ros gazebo
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(robot_ros_utils src/gazebo_plugins/robot_ros_utils.cpp)
target_link_libraries(robot_ros_utils ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(robot_sub_drive src/gazebo_plugins/robot_sub_drive.cpp)
target_link_libraries(robot_sub_drive robot_ros_utils ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_executable(robdos_control src/robdos_control.cpp)
add_dependencies(robdos_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robdos_control ${Boost_LIBRARIES} ${catkin_LIBRARIES} )

# test app
#add_executable(waypoint_publisher src/test/waypoint_publisher.cpp)
#target_link_libraries(waypoint_publisher ${Boost_LIBRARIES}  ${catkin_LIBRARIES})
