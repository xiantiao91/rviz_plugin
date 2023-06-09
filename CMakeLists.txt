cmake_minimum_required(VERSION 3.0.2)
project(my_rviz_plugin)
set(cartographer_ros_msgs_DIR "/home/jie/ros_ws/cartographer_ros/devel/share/cartographer_ros_msgs/cmake")
set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/install)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
find_package(Qt5 REQUIRED COMPONENTS Widgets Core)
set(QT_LIBRARIES Qt5::Widgets)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rviz
  cartographer_ros_msgs
)

catkin_package(
    LIBRARIES my_rviz_plugin
      CATKIN_DEPENDS roscpp rviz
)

find_package(rviz REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(
 include/my_rviz_plugin
  ${catkin_INCLUDE_DIRS}
)
link_directories(${catkin_LIBRARY_DIRS})

qt5_wrap_cpp(MOC_FILES include/my_rviz_plugin/auto_subscriber_display.h OPTIONS -I${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(${PROJECT_NAME}
    src/auto_subscriber_display.cpp
    src/custom_point_cloud_common.cpp
    ${MOC_FILES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${rviz_LIBRARIES} ${QT_LIBRARIES} ${PCL_LIBRARY_DIRS})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}
)
install(FILES plugin.xml
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}
)
