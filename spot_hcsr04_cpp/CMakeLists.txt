cmake_minimum_required(VERSION 3.8)
project(spot_hcsr04_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# libgpiod for GPIO access
find_package(PkgConfig REQUIRED)

pkg_check_modules(LIBGPIOD gpiod QUIET)
if(NOT LIBGPIOD_FOUND)
  pkg_check_modules(LIBGPIOD libgpiod QUIET)
endif()
if(NOT LIBGPIOD_FOUND)
  pkg_check_modules(LIBGPIOD gpiod-dev QUIET)
endif()

if(NOT LIBGPIOD_FOUND)
  message(FATAL_ERROR "libgpiod not found via pkg-config")
endif()

add_executable(${PROJECT_NAME}_node
  src/main.cpp
)

target_include_directories(${PROJECT_NAME}_node PRIVATE
  ${LIBGPIOD_INCLUDE_DIRS}
)

target_link_directories(${PROJECT_NAME}_node PRIVATE
  ${LIBGPIOD_LIBRARY_DIRS}
)

target_compile_options(${PROJECT_NAME}_node PRIVATE
  ${LIBGPIOD_CFLAGS_OTHER}
)

target_link_libraries(${PROJECT_NAME}_node
  ${LIBGPIOD_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
