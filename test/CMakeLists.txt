find_package(catkin REQUIRED COMPONENTS
  message_filters
  pcl_conversions
  pcl_msgs
  pcl_ros
  sensor_msgs)

catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    message_filters
    pcl_conversions
    pcl_msgs
    pcl_ros
    sensor_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

# Add binary for inspecting subscription latency.
#add_executable(sample_subscribe sample_subscribe.cpp)
#target_link_libraries(sample_subscribe openni2_driver_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
#add_dependencies(sample_subscribe ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)

#add_library(openni2_subscriber_nodelet subscriber_nodelet.cpp)
#target_link_libraries(openni2_subscriber_nodelet openni2_driver_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
#add_dependencies(openni2_subscriber_nodelet ${PROJECT_NAME}_gencfg)

add_executable(subscribe_node subscriber_node.cpp)
target_link_libraries(subscribe_node openni2_driver_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
add_dependencies(subscribe_node ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)

add_executable(subscribe_node2 subscriber_node2.cpp)
target_link_libraries(subscribe_node2 openni2_driver_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
add_dependencies(subscribe_node2 ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)
