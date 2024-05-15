cmake_minimum_required(VERSION 3.8)
project(discovery_server_discovery)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS On)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(fastrtps REQUIRED)
find_package(rclcpp REQUIRED)
find_package(discovery_interfaces REQUIRED)

add_executable(discovery_server_listener src/discovery_server_main.cpp src/listener.cpp)
target_include_directories(discovery_server_listener PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(discovery_server_listener PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  discovery_server_listener
  "rclcpp"
  "discovery_interfaces"
)
target_link_libraries(discovery_server_listener fastrtps)

install(TARGETS discovery_server_listener
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()
