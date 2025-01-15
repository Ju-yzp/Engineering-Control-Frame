cmake_minimum_required(VERSION 3.10)
project(robotic_arm_control)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

###########
## Build ##
###########

add_executable(control_example
${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm/kinematic.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm/planning.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/data_types/limits_struct.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/data_types/trajectory_struct.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/data_types/solutions_struct.cpp
)

ament_target_dependencies(
  control_example
  "rclcpp"
  "Eigen3"
)

target_include_directories(control_example
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)

install(
  TARGETS control_example
  DESTINATION lib
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

ament_package()