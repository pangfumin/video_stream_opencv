cmake_minimum_required(VERSION 2.8.3)
project(video_stream_opencv)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  image_transport
  camera_info_manager
  sensor_msgs
)


if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-std=c++11 -O3 -Wall -Wno-unused-variable -Wno-unused-but-set-variable -g ${CMAKE_CXX_FLAGS}")
    message(STATUS "optional:-std=c++11 -O3 -Wall -g")
endif(CMAKE_COMPILER_IS_GNUCXX)


#link_directories("/usr/local/lib")
link_libraries(usb-1.0 rt)



find_package(OpenCV)

catkin_package()

include_directories(
        include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(CqUsbCam SHARED
        src/sensors/AR0144.cpp
        src/sensors/AR0135.cpp
        src/sensors/MT9M001.cpp
        src/sensors/MT9V034.cpp
        src/CqUsbCam.cpp
        src/DataProcess.cpp
        src/DataCapture.cpp
        src/libcyusb.cpp
        )

add_executable(capture src/main.cpp)
target_link_libraries(capture CqUsbCam ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


add_executable(video_stream src/video_stream.cpp)
target_link_libraries(video_stream ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS video_stream
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS
  scripts/test_video_resource.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
