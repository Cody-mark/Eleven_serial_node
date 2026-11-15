cmake_minimum_required(VERSION 3.8)
project(eleven_serial_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(serial_driver REQUIRED)
find_package(Threads REQUIRED)

# header-only：实现全部在 include/eleven_serial_node/serial_mcu.hpp，不编译 .so
add_library(eleven_serial_node INTERFACE)
target_include_directories(eleven_serial_node INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${serial_driver_INCLUDE_DIRS})   # serial_driver 未导出 imported target，只能透传 include 路径
target_link_libraries(eleven_serial_node INTERFACE
  ${serial_driver_LIBRARIES}
  Threads::Threads)

# 链路测试程序：固定频率发送测试帧
add_executable(serial_link_test src/serial_link_test.cpp)
target_link_libraries(serial_link_test eleven_serial_node)

install(DIRECTORY include/
  DESTINATION include)
install(TARGETS serial_link_test
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS eleven_serial_node
  EXPORT eleven_serial_node_targets)
install(EXPORT eleven_serial_node_targets
  FILE eleven_serial_nodeTargets.cmake
  NAMESPACE eleven_serial_node::
  DESTINATION share/eleven_serial_node/cmake)

ament_export_targets(eleven_serial_node_targets)
ament_export_dependencies(serial_driver Threads)
ament_package()
