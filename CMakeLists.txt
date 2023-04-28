cmake_minimum_required(VERSION 3.0.2)
project(my_rviz_plugin)
set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/install)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rviz
)

catkin_package(
    LIBRARIES my_rviz_plugin
      CATKIN_DEPENDS roscpp rviz
)

find_package(rviz REQUIRED)
find_package(pluginlib REQUIRED)



## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include/my_rviz_plugin
  ${catkin_INCLUDE_DIRS}
)
link_directories(${catkin_LIBRARY_DIRS})

add_library(${PROJECT_NAME} src/auto_subscriber_display.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${rviz_LIBRARIES})
set_target_properties(${PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}
)
install(FILES plugin_description.xml
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}
)
#install(FILES plugin_description.xml
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
#install(TARGETS auto_subscriber_display
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#install(FILES plugin_description.xml
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
