cmake_minimum_required(VERSION 3.8)
project(spot_node_label_config_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(${PROJECT_NAME}_node
  src/main.cpp
  src/domain/label_parser.cpp
  src/application/publish_servo_map_uc.cpp
  src/adapters/ros_string_publisher.cpp
  src/adapters/k8s_labels_source.cpp
)

target_include_directories(${PROJECT_NAME}_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs)

find_package(CURL REQUIRED)
target_link_libraries(${PROJECT_NAME}_node pthread CURL::libcurl)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
