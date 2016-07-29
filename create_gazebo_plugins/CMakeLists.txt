cmake_minimum_required(VERSION 2.8.3)
project(create_gazebo_plugins)

# Find gazebo
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS create_node
                                        gazebo_ros
                                        geometry_msgs
                                        nav_msgs
                                        roscpp
                                        sensor_msgs)

catkin_package(INCLUDE_DIRS include
               LIBRARIES gazebo_ros_create
               CATKIN_DEPENDS create_node
                              gazebo_ros
                              geometry_msgs
                              nav_msgs
                              roscpp
                              sensor_msgs)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${GAZEBO_INCLUDE_DIRS})

add_library(gazebo_ros_create src/gazebo_ros_create.cpp)

add_dependencies(gazebo_ros_create create_node_gencpp
                                   geometry_msgs_gencpp
                                   nav_msgs_gencpp
                                   sensor_msgs_gencpp)

target_link_libraries(gazebo_ros_create
                      ${catkin_LIBRARIES}
                      ${GAZEBO_LIBRARIES})

install(TARGETS gazebo_ros_create
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
