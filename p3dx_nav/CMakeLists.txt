# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(p3dx_nav)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg geometry_msgs rosaria)

## Declare ROS messages and services
add_message_files(FILES Sensors.msg)
add_service_files(FILES ServerClientPelea.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})

add_executable(enable_motors src/enable_motors.cpp)
target_link_libraries(enable_motors ${catkin_LIBRARIES})

## Build service client and server
# %Tag(SRVCLIENT)%
add_executable(p3dx_server src/Nav_waypoints.cpp src/Movimiento_basico.cpp src/p3dx_server.cpp src/Move_base_mov.cpp)
target_link_libraries(p3dx_server ${catkin_LIBRARIES})
add_dependencies(p3dx_server p3dx_nav_gencpp)

add_executable(p3dx_client src/p3dx_client.cpp)
target_link_libraries(p3dx_client ${catkin_LIBRARIES})
add_dependencies(p3dx_client p3dx_nav_gencpp)

add_executable(p3dx_sensors src/p3dx_sensors.cpp)
target_link_libraries(p3dx_sensors ${catkin_LIBRARIES})
add_dependencies(p3dx_sensors p3dx_nav_gencpp)

add_executable(teleop src/teleop.cpp)
target_link_libraries(teleop ${catkin_LIBRARIES})
# %EndTag(SRVCLIENT)%

# %EndTag(FULLTEXT)%
