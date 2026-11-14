cmake_minimum_required(VERSION 3.8)
project(eleven_serial_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial_driver REQUIRED)
find_package(Threads REQUIRED)

add_library(eleven_serial_node SHARED src/ros2_mcu.cpp)
target_include_directories(eleven_serial_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${serial_driver_INCLUDE_DIRS}
)
target_link_libraries(eleven_serial_node PUBLIC
  rclcpp::rclcpp
  ${serial_driver_LIBRARIES}
  Threads::Threads
)

install(DIRECTORY include/
  DESTINATION include
)
install(TARGETS eleven_serial_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(eleven_serial_node)
ament_export_dependencies(rclcpp)
ament_package()
