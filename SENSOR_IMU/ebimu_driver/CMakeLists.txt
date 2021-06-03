cmake_minimum_required(VERSION 2.8.3)
project(ebimu_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
  pharos_msgs #이걸 추가 하니까 pharos_msgs를 사용할 수 있구나
#  message_generation
)

generate_messages(
        DEPENDENCIES
        std_msgs

#        pharos_msgs
)

catkin_package(
  CATKIN_DEPENDS
    serial
    std_msgs

    pharos_msgs
#    geometry_msgs
)


#add_message_files(
#
#
#)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ebimu_driver_node src/ebimu_driver_node.cpp)
add_executable(serial_example_node src/serial_example_node.cpp)

target_link_libraries(ebimu_driver_node
  ${catkin_LIBRARIES}
)
target_link_libraries(serial_example_node
  ${catkin_LIBRARIES}
)

add_dependencies(ebimu_driver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(serial_example_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
