cmake_minimum_required(VERSION 3.5)
project(initial_test)

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
find_package(std_msgs REQUIRED)
find_package(test_messages REQUIRED)
find_package(rcutils)
find_package(rmw REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(control_publisher src/dummy_publisher.cpp)
ament_target_dependencies(control_publisher rclcpp std_msgs)

add_executable(control_subscriber src/dummy_subscriber.cpp)
ament_target_dependencies(control_subscriber rclcpp std_msgs)

add_executable(mip_controller src/controller_node.cpp src/ringbuf.h src/ringbuf.cpp)
ament_target_dependencies(mip_controller rclcpp std_msgs test_messages)

add_executable(mip_plant src/plant_node.cpp src/ringbuf.h src/ringbuf.cpp)
ament_target_dependencies(mip_plant rclcpp std_msgs test_messages)

install(TARGETS
  control_publisher
  control_subscriber
  mip_controller
  mip_plant
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
