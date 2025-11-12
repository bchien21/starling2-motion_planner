cmake_minimum_required(VERSION 3.8)
project(starling2-planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(basic-plan src/basic-plan.cpp)
target_include_directories(basic-plan PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(basic-plan PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  basic-plan
  "moveit_ros_planning_interface"
  "rclcpp"
)

install(TARGETS basic-plan
  DESTINATION lib/${PROJECT_NAME})


add_executable(multi-obs-planner src/multi_obs_planner.cpp)
target_include_directories(multi-obs-planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(multi-obs-planner PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  multi-obs-planner
  "moveit_ros_planning_interface"
  "rclcpp"
  "nlohmann_json"
)
install(TARGETS multi-obs-planner
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
