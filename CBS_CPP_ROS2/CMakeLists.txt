cmake_minimum_required(VERSION 3.8)
project(mapf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(dependencies
  std_msgs
  std_srvs
  geometry_msgs
  rclcpp_components
  nav_msgs
  visualization_msgs
  nav2_util
  OpenCV
  # Boost
  # Eigen3
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(OpenCV REQUIRED)
# find_package(Boost 1.67.0 COMPONENTS filesystem system iostreams program_options REQUIRED)
# find_package(Eigen3 REQUIRED)
# find_package(nanoflann REQUIRED)

include_directories(
  include
  # ${Eigen3_INCLUDE_DIRS}
)

#############
##  Build  ##
#############

add_library(mapf_lib SHARED
  src/mapf.cpp
  src/warehouse_grid.cpp
  src/HighLevelSolver.cpp
  src/LowLevelSolver.cpp
  src/TreeNode.cpp
)

ament_target_dependencies(mapf_lib
  ${dependencies}
)

##########################
## Building executables ##
##########################

add_executable(mapf src/mapf.cpp
  src/warehouse_grid.cpp
  src/HighLevelSolver.cpp
  src/LowLevelSolver.cpp
  src/TreeNode.cpp
)

target_link_libraries(mapf mapf_lib)
ament_target_dependencies(mapf ${dependencies})

install(
  TARGETS mapf
  DESTINATION lib/${PROJECT_NAME}
)


install(TARGETS mapf
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME} 
  FILES_MATCHING PATTERN "*.py"
)

install(DIRECTORY maps DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshs DESTINATION share/${PROJECT_NAME})

#############
##  Test   ##
#############

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
