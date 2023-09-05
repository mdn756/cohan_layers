cmake_minimum_required(VERSION 3.8)
project(cohan_layers)

set(lib_name ${PROJECT_NAME}_core)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99) 
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14) 
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cohan_msgs REQUIRED)

set(dep_pkgs
    rclcpp
    nav2_costmap_2d
    geometry_msgs
    cohan_msgs
    pluginlib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(${lib_name} SHARED
            src/human_layer.cpp
           )
include_directories(include)

install(TARGETS ${lib_name}
       # DESTINATION lib/${lib_name})
        DESTINATION lib)

pluginlib_export_plugin_description_file(nav2_costmap_2d human_layer.xml)
ament_target_dependencies(${lib_name} ${dep_pkgs})
ament_package()
