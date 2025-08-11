cmake_minimum_required(VERSION 3.8)
project(yolocpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

#eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# ============= cuda ============
include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)

#yolo and track library ###serial
include_directories(
  ~/yolocpp/libs/cppyolo/detect/include
  ~/yolocpp/libs/cppyolo/bytetrack/include
  ~/yolocpp/libs/serial/include###
  include
)
link_directories(
  ~/yolocpp/libs/cppyolo/build
  ~/yolocpp/libs/serial/build###
)

add_executable(yolocpp 
  src/yolocpp.cpp 
  src/yolocppnode.cpp
  src/PID.cpp
)

target_link_libraries(yolocpp
  bytetrack
  yolo_infer
  serial###
  ${OpenCV_LIBS}
)

ament_target_dependencies(yolocpp
  rclcpp
  std_msgs
)

install(TARGETS yolocpp
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
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
