cmake_minimum_required(VERSION 3.8)
project(vrx_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vrx_gz REQUIRED)
find_package(vrx_ros REQUIRED)
find_package(vrx_gazebo REQUIRED)
find_package(wamv_description REQUIRED)
find_package(wamv_gazebo REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)  
find_package(nav_msgs REQUIRED)

ament_export_include_directories(include)
ament_export_libraries(vrx_control)
ament_export_dependencies(rclcpp rclpy std_msgs geometry_msgs gazebo_msgs tf2 tf2_ros rcl_interfaces tf2_geometry_msgs sensor_msgs)  # Add tf2_geometry_msgs here

set(acados_include "/home/yang/acados/include")
set(acados_lib "/home/yang/acados/lib")
set(wamv_model ${CMAKE_CURRENT_SOURCE_DIR}/scripts/c_generated_code)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${gazebo_msgs_INCLUDE_DIRS}
  ${wamv_model}
  ${acados_include}
  ${acados_include}/blasfeo/include/
  ${acados_include}/hpipm/include/
  ${acados_include}/acados/
  ${acados_include}/qpOASES_e/
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
)

link_directories(${wamv_model})

add_executable(wamv_mpc_node
  src/wamv_mpc_node.cpp
  src/wamv_mpc.cpp
)

ament_target_dependencies(wamv_mpc_node
  rclcpp
  std_msgs
  geometry_msgs
  gazebo_msgs
  tf2
  tf2_ros
  rcl_interfaces
  nav_msgs
  sensor_msgs
  tf2_geometry_msgs  # Add tf2_geometry_msgs as a dependency
)

target_link_libraries(wamv_mpc_node
  ${wamv_model}/libacados_ocp_solver_wamv.so
  ${acados_lib}/libacados.so
)

install(TARGETS wamv_mpc_node
  DESTINATION lib/${PROJECT_NAME}
)

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

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
