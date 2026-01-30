cmake_minimum_required(VERSION 3.8)
project(spot_lcd_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# Options for features
option(ENABLE_ROS2 "Enable ROS2 integration" OFF)
option(ENABLE_GPIO "Enable GPIO support for LCD" OFF)

# Include directories
include_directories(include)

# Add executable
add_executable(${PROJECT_NAME}_node
    src/main.cpp
    src/config.cpp
    src/lcd_core.cpp
    src/lcd_display_service.cpp
    src/temp_monitor.cpp
    src/uptime_monitor.cpp
)

# Link libraries
target_link_libraries(${PROJECT_NAME}_node pthread)

# Conditional compilation for GPIO
if(ENABLE_GPIO)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(WIRINGPI wiringPi QUIET)
    if(WIRINGPI_FOUND)
        target_compile_definitions(${PROJECT_NAME}_node PRIVATE USE_GPIO=1)
        target_link_libraries(${PROJECT_NAME}_node ${WIRINGPI_LIBRARIES})
        target_include_directories(${PROJECT_NAME}_node PRIVATE ${WIRINGPI_INCLUDE_DIRS})
    else()
        message(WARNING "wiringPi not found, building without GPIO support")
        target_compile_definitions(${PROJECT_NAME}_node PRIVATE USE_GPIO=0)
        set(ENABLE_GPIO OFF)
    endif()
else()
    target_compile_definitions(${PROJECT_NAME}_node PRIVATE USE_GPIO=0)
    # Ensure basic libraries are linked even without GPIO
    target_link_libraries(${PROJECT_NAME}_node pthread)
endif()

# Conditional compilation for ROS2
if(ENABLE_ROS2)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(builtin_interfaces REQUIRED)
    
    target_compile_definitions(${PROJECT_NAME}_node PRIVATE USE_ROS2=1)
    target_link_libraries(${PROJECT_NAME}_node rclcpp std_msgs builtin_interfaces)
    ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs builtin_interfaces)
    
    install(TARGETS ${PROJECT_NAME}_node
      DESTINATION lib/${PROJECT_NAME}
    )
    
    ament_package()
else()
    # Для локальной сборки без ROS2
    set_target_properties(${PROJECT_NAME}_node PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
    )
endif()