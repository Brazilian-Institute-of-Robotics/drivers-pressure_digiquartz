# Copyright (c) 2024, SENAI Cimatec
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.8)
project(pressure_pkg)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros_driver_base REQUIRED)

# Add library
add_library(pressure_driver src/pressure_driver.cpp)
target_compile_features(pressure_driver PUBLIC c_std_99 cxx_std_20)

# Include directories for the library
target_include_directories(pressure_driver PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link dependencies for the library
ament_target_dependencies(
    pressure_driver
    "rclcpp"
    "std_msgs"
    "ros_driver_base"
)

# Compile definitions for the library
target_compile_definitions(pressure_driver PRIVATE "PRESSURE_PKG_BUILDING_LIBRARY")

# Installation rules for header files
install(
    DIRECTORY include/
    DESTINATION include
)

# Installation rules for the library
install(
    TARGETS pressure_driver
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Add executable
add_executable(main src/main.cpp)
target_include_directories(main PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(main pressure_driver)

# Installation rules for the executable
install(TARGETS main
    DESTINATION lib/${PROJECT_NAME}
)

# Installation rules for launch and config files
install(DIRECTORY
    launch config
    DESTINATION share/${PROJECT_NAME}/
)

# Testing setup
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

# Export information for downstream packages
ament_export_include_directories(include)
ament_export_libraries(pressure_driver)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rclcpp std_msgs ros_driver_base)

ament_package()
