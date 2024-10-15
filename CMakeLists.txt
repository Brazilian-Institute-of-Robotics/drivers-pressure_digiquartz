cmake_minimum_required(VERSION 3.8)
project(pressure_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros_driver_base REQUIRED)
# find_package(Boost REQUIRED COMPONENTS system)

add_library(pressure_driver src/pressure_driver.cpp)
target_compile_features(pressure_driver PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++20
target_include_directories(pressure_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  pressure_driver
  "rclcpp"
  "std_msgs"
  "ros_driver_base"
)

# add_executable(node src/pressure_node.cpp)
# target_include_directories(node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_link_libraries(node pressure_driver)
# ament_target_dependencies(node
#   "rclcpp"
#   "std_msgs"
#   "ros_driver_base"
# )

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(pressure_driver PRIVATE "PRESSURE_PKG_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS pressure_driver
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(main src/main.cpp)
target_include_directories(main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(main pressure_driver)

# install(TARGETS main node
#   DESTINATION lib/${PROJECT_NAME})

install(TARGETS main
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  pressure_driver
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_export_dependencies(rclcpp std_msgs ros_driver_base)

ament_package()
