cmake_minimum_required(VERSION 2.8.3)
project(agents_layer)
set_directory_properties(PROPERTIES COMPILE_OPTIONS "-Wall;-Werror")

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  angles
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  cohan_msgs
  pluginlib
  roscpp
  rospy
)

generate_dynamic_reconfigure_options(
  cfg/AgentsLayer.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES  agents_layer
  CATKIN_DEPENDS angles costmap_2d dynamic_reconfigure geometry_msgs cohan_msgs pluginlib roscpp rospy
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(agents_layer
  src/base_agents_layer.cpp
  src/agents_layer.cpp
)


add_dependencies(agents_layer  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(agents_layer  ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslint)
  roslint_cpp()
  roslint_add_test()
endif()

install(FILES costmap_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


install(TARGETS agents_layer
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
