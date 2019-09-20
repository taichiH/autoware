cmake_minimum_required(VERSION 2.8.3)
project(trafficlight_recognizer)
execute_process(
  COMMAND rosversion -d
  OUTPUT_VARIABLE ROS_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
  )

set(ROS_VERSION $ENV{ROS_DISTRO})


find_package(autoware_build_flags REQUIRED)

find_package(autoware_msgs REQUIRED)

find_package(vector_map REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  cv_bridge
  geometry_msgs
  libvectormap
  roscpp
  sensor_msgs
  std_msgs
  tf
  vector_map_server
  visualization_msgs
  waypoint_follower
  autoware_msgs
  vector_map
  image_transport
  nodelet
  eigen_conversions
  )

find_package(OpenCV REQUIRED)
find_package(Eigen3 QUIET)

if (NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else ()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif ()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)

find_package(OpenGL REQUIRED)

###################################
## catkin specific configuration ##
###################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${vector_map_INCLUDE_DIRS}
  ${autoware_msgs_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${Opencv_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/nodes/tlr_tracker/libs/src/
  ${CMAKE_SOURCE_DIR}/nodes/tlr_tracker/libs/src/piotr_fhog
  ${CMAKE_SOURCE_DIR}/nodes/tlr_tracker/libs/src/cn
  )

catkin_package(
  INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS} include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  autoware_msgs
  cmake_modules
  cv_bridge
  geometry_msgs
  libvectormap
  roscpp
  sensor_msgs
  std_msgs
  tf
  vector_map
  vector_map_server
  visualization_msgs
  waypoint_follower
  image_transport
  LIBRARIES
  libutils
  libtrafficlightkcf
  )

set(CMAKE_CXX_FLAGS "-O2 -Wall -Wunused-variable ${CMAKE_CXX_FLAGS}")

### utils ###
add_library(
  libutils lib/utils.cpp
  )
target_link_libraries(libutils
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  )
add_dependencies(libutils
  ${catkin_EXPORTED_TARGETS}
  )

### tlr_classifier ###
add_executable(tlr_classifier
  nodes/tlr_classifier/traffic_light_classifier_node.cpp
  nodes/tlr_classifier/traffic_light_classifier.cpp
  )
target_link_libraries(tlr_classifier
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  libutils
  )


### feat_proj ###
add_executable(feat_proj
  nodes/feat_proj/feat_proj.cpp
  )

target_link_libraries(feat_proj
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  )

add_dependencies(feat_proj
  ${catkin_EXPORTED_TARGETS}
  )


### tlr_tracker ###
add_subdirectory(nodes/tlr_tracker/libs/src)

add_library(libtrafficlightkcf
  nodes/tlr_tracker/kcf_tracker.cpp
  )

target_link_libraries(libtrafficlightkcf
  kcf ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
  )

add_dependencies(libtrafficlightkcf
  ${catkin_EXPORTED_TARGETS}
  )

### nodelet ###
add_library(${PROJECT_NAME} SHARED
  nodes/feat_proj/feat_proj_bbox.cpp
  nodes/tlr_tracker/kcf_tracker_node.cpp)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${EIGEN_LIBRARIES}
  libutils
  libtrafficlightkcf)


if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test-feat_proj_bbox test/test_feat_proj_bbox.test
    test/test_feat_proj_bbox.cpp nodes/feat_proj/feat_proj_bbox.cpp)
  target_link_libraries(test-feat_proj_bbox libutils ${catkin_LIBRARIES} ${OpenCV_LIBS})
endif()
