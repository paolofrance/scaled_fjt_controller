cmake_minimum_required(VERSION 3.8)
project(scaled_fjt_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(joint_trajectory_controller  REQUIRED)
find_package(controller_interface         REQUIRED)
find_package(hardware_interface           REQUIRED)
find_package(pluginlib                    REQUIRED)
find_package(rclcpp                       REQUIRED)
find_package(rclcpp_lifecycle             REQUIRED)


set(THIS_PACKAGE_INCLUDE_DEPENDS
  joint_trajectory_controller
  controller_interface  
  hardware_interface  
  pluginlib            
  rclcpp            
  rclcpp_lifecycle  
)

include_directories(include)

add_library(scaled_fjt_controller SHARED
  src/scaled_fjt_controller.cpp
  src/microinterpolator.cpp
)
target_include_directories(scaled_fjt_controller PRIVATE
  include
)

ament_target_dependencies(scaled_fjt_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(controller_interface scaled_fjt_controller.xml)


install(
  DIRECTORY include/
  DESTINATION include/scaled_fjt_controller
)
install(
  TARGETS
    scaled_fjt_controller
  EXPORT export_scaled_fjt_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_package()
