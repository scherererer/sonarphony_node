cmake_minimum_required(VERSION 3.8)
project(sonarphony_node)

add_subdirectory(ext/SonarPhony/library/sonarphony)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(marine_acoustic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Core Network SerialPort)

add_executable(sonarphony_node
	src/main.cc
	src/SonarNode.cc
	src/SonarNode.hh
  src/SonarApplication.cc
	src/SonarApplication.hh
)

ament_target_dependencies(sonarphony_node rclcpp marine_acoustic_msgs std_msgs)
target_link_libraries(sonarphony_node Qt6::Core libsonarphony)

install(TARGETS
  sonarphony_node
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
