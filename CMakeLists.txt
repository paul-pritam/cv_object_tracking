cmake_minimum_required(VERSION 3.8)
project(cv_object_tracker)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)
#detector component
add_library(detector_lib SHARED
  src/detector.cpp
)

target_link_libraries(detector_lib ${OpenCV_LIBRARIES}) 
target_include_directories(detector_lib PUBLIC ${OpenCV_INCLUDE_DIRS})

ament_target_dependencies(detector_lib
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  image_transport
  cv_bridge
  sensor_msgs
  geometry_msgs
  vision_msgs
)
rclcpp_components_register_nodes(detector_lib
"cv_tracker::Detector"
)
#tracker component
add_library(tracker_lib SHARED
  src/tracker.cpp
)
ament_target_dependencies(tracker_lib
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  image_transport
  cv_bridge
  sensor_msgs
  geometry_msgs
  vision_msgs
)
rclcpp_components_register_nodes(tracker_lib
"cv_tracker::Tracker" 
)

#unified executable
add_executable(cv_node src/main.cpp)
target_link_libraries(
  cv_node
  detector_lib
  tracker_lib
)
ament_target_dependencies(
  cv_node
  rclcpp
  rclcpp_lifecycle
)
install(TARGETS
  detector_lib
  tracker_lib
  cv_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS cv_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
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

ament_export_include_directories(include)
ament_export_libraries(detector_lib tracker_lib)
ament_package()
