cmake_minimum_required(VERSION 3.8)
project(bts7960_hw_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# pigpio daemon interface library
# On Raspberry Pi: sudo apt install pigpio python3-pigpio
# The pigpiod_if2 library provides the C API for remote GPIO access
find_library(PIGPIOD_IF2 pigpiod_if2)

add_library(${PROJECT_NAME} SHARED
  src/bts7960_system.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

if(PIGPIOD_IF2)
  target_link_libraries(${PROJECT_NAME} ${PIGPIOD_IF2})
else()
  message(WARNING "pigpiod_if2 not found — BTS7960 hardware interface will not link. "
                  "Install pigpio on Raspberry Pi: sudo apt install pigpio")
endif()

pluginlib_export_plugin_description_file(hardware_interface bts7960_hw_interface.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
