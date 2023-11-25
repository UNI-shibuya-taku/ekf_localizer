cmake_minimum_required(VERSION 3.0.2)
project(ekf_localizer)

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")
add_compile_options(-std=c++14)
# add_compile_options(-o2 -g -Wall)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  eigen_conversions
  geometry_msgs
  image_transport
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
)

# OpenCV
find_package(OpenCV 4.0 QUIET)
if(NOT OpenCV_FOUND)
    find_package(OpenCV 3.0 QUIET)
    if(NOT OpenCV_FOUND)
        find_package(OpenCV 3.4.3 QUIET)
        if(NOT OpenCV_FOUND)
            message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
        endif()
    endif()
endif()

# PCL
find_package(PCL REQUIRED)

# Eigen
find_package(Eigen3 REQUIRED)



catkin_package(
    INCLUDE_DIRS include
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS cv_bridge eigen_conversions geometry_msgs image_transport nav_msgs pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs tf2_geometry_msgs tf2_ros
    # DEPENDS system_lib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
)

# ekf
add_executable(ekf_node 
    src/ekf.cpp
    src/ekf_node.cpp
)
target_link_libraries(ekf_node ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})

# map_matcher
add_executable(map_matcher_node 
    src/map_matcher.cpp
    src/map_matcher_node.cpp
)
target_link_libraries(map_matcher_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Eigen3_LIBRARIES})

# tf
add_executable(tf_broadcaster_node
    src/tf_broadcaster.cpp
    src/tf_broadcaster_node.cpp
)
target_link_libraries(tf_broadcaster_node ${catkin_LIBRARIES})

add_executable(dynamic_tf_cub src/dynamic_tf_cub.cpp)
target_link_libraries(dynamic_tf_cub ${catkin_LIBRARIES})

add_executable(odom_filter src/odom_filter.cpp)
target_link_libraries(odom_filter ${catkin_LIBRARIES})