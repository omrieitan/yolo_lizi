cmake_minimum_required(VERSION 2.8.3)
project(yolo_lizi)

add_definitions(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  cv_bridge
  image_transport
  message_generation
)
add_message_files(
  FILES
  lizi_imu.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES yolo_lizi
 CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)



SET(OpenCV_DIR /home/omri/Desktop/iot_project/installation/OpenCV-3.4.4/share/OpenCV)
find_package(OpenCV REQUIRED)

add_library(yolo_opencv include/object_detection_yolo.cpp include/object_detection_yolo.h)
target_link_libraries(yolo_opencv
        ${OpenCV_LIBS}
        )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(camera_driver src/camera_driver.cpp)
target_link_libraries(camera_driver  ${OpenCV_LIBS} ${catkin_LIBRARIES} )

add_executable(yolo_node src/yolo_node.cpp)
target_link_libraries(yolo_node  ${OpenCV_LIBS} ${catkin_LIBRARIES} yolo_opencv)
