cmake_minimum_required(VERSION 3.8)
project(spot_hcsr04_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# libgpiod for GPIO access
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBGPIOD REQUIRED gpiod)

add_executable(${PROJECT_NAME}_node
  src/main.cpp
)

target_include_directories(${PROJECT_NAME}_node PUBLIC
  ${LIBGPIOD_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_node
  ${LIBGPIOD_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  sensor_msgs
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
