cmake_minimum_required(VERSION 3.8)
project(manipulator_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

include_directories(include)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# make cpp executable
# add_executable(cpp_executable src/cpp_node.cpp) # need to edit with the appropriate names later
# ament_target_dependencies(cpp_executable rclcpp)

# install cpp executables
# install(TARGETS 
#   cpp_executables
#   DESTINATION lib/${PROJECT_NAME}
# )

# install python modules
ament_python_install_package(${PROJECT_NAME})

# install python executables 
install(PROGRAMS
  scripts/jparse_cls.py
  scripts/xarm_vel_main_experiment.py
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

ament_package()
