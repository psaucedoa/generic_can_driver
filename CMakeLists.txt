cmake_minimum_required(VERSION 3.5)
project(generic_can_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(
  ${PROJECT_NAME} SHARED
  src/generic_can_driver_node.cpp
)

target_compile_options(${PROJECT_NAME} PRIVATE -Wno-unused-function)

rclcpp_components_register_node(generic_can_driver
  PLUGIN "generic_can_driver::GenericCanDriverNode"
  EXECUTABLE generic_can_driver_exe
)

ament_auto_package(INSTALL_TO_SHARE

)