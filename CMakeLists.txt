cmake_minimum_required(VERSION 2.8.3)
project(cvusb_cam)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        roslib
        cv_bridge
        message_generation
        sensor_msgs
        std_msgs
        image_transport
        pcl_conversions
        pcl_ros
)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)


catkin_package(
  CATKIN_DEPENDS   roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL OpenCV
  #INCLUDE_DIRS include
)
include_directories(
  INCLUDE_DIRS include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    )


add_executable(cv_cam_node src/cv_cam.cpp)
target_link_libraries(cv_cam_node ${catkin_LIBRARIES})
add_dependencies(cv_cam_node cv_cam_node_generate_messages_cpp)

add_executable(point2img_node src/cloud2img.cpp)
target_link_libraries(point2img_node ${catkin_LIBRARIES})
add_dependencies(point2img_node point2img_node_generate_messages_cpp)
