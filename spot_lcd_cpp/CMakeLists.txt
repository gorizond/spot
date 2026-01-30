cmake_minimum_required(VERSION 3.10)
project(spot_lcd_node)

set(CMAKE_CXX_STANDARD 17)

# Find required packages
find_package(PkgConfig REQUIRED)

# Options for features
option(ENABLE_ROS2 "Enable ROS2 integration" OFF)
option(ENABLE_GPIO "Enable GPIO support for LCD" ON)

# Include directories
include_directories(include)

# Add executable
add_executable(${PROJECT_NAME} 
    src/main.cpp
    src/config.cpp
    src/lcd_core.cpp
    src/lcd_display_service.cpp
    src/temp_monitor.cpp
    src/uptime_monitor.cpp
)

# Link libraries
target_link_libraries(${PROJECT_NAME} pthread)

# Conditional compilation for ROS2
if(ENABLE_ROS2)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    
    target_compile_definitions(${PROJECT_NAME} PRIVATE USE_ROS2=1)
    target_link_libraries(${PROJECT_NAME} rclcpp std_msgs)
    ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs)
endif()

# Conditional compilation for GPIO
if(ENABLE_GPIO)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(WIRINGPI wiringPi QUIET)
    if(WIRINGPI_FOUND)
        target_compile_definitions(${PROJECT_NAME} PRIVATE USE_GPIO=1)
        target_link_libraries(${PROJECT_NAME} ${WIRINGPI_LIBRARIES})
        target_include_directories(${PROJECT_NAME} PRIVATE ${WIRINGPI_INCLUDE_DIRS})
    else()
        message(WARNING "wiringPi not found, building without GPIO support")
        target_compile_definitions(${PROJECT_NAME} PRIVATE USE_GPIO=0)
        set(ENABLE_GPIO OFF)
    endif()
else()
    target_compile_definitions(${PROJECT_NAME} PRIVATE USE_GPIO=0)
    # Ensure basic libraries are linked even without GPIO
    target_link_libraries(${PROJECT_NAME} pthread)
endif()