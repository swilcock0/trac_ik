cmake_minimum_required(VERSION 3.10)
project(trac_ik_kinematics_plugin)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang"))
    set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(trac_ik_lib REQUIRED)

# Find nlopt using PkgConfig
find_package(PkgConfig REQUIRED)
pkg_check_modules(nlopt REQUIRED nlopt)

# Also try to find the actual library
find_library(NLOPT_LIBRARY NAMES nlopt PATHS /lib /lib/x86_64-linux-gnu /usr/lib /usr/lib/x86-64-linux-gnu)

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED src/trac_ik_kinematics_plugin.cpp)

ament_target_dependencies(${PROJECT_NAME}
  moveit_core
  pluginlib
  tf2
  tf2_kdl
  trac_ik_lib
)

target_link_libraries(${PROJECT_NAME} ${nlopt_LIBRARIES})
if(NLOPT_LIBRARY)
  target_link_libraries(${PROJECT_NAME} ${NLOPT_LIBRARY})
endif()
target_include_directories(${PROJECT_NAME} PRIVATE ${nlopt_INCLUDE_DIRS})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

pluginlib_export_plugin_description_file(moveit_core trac_ik_kinematics_description.xml)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  FILES
    trac_ik_kinematics_description.xml
  DESTINATION
    share/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  moveit_core
  pluginlib
  tf2
  tf2_kdl
  trac_ik_lib
)

ament_package()
