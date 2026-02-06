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

# Add executables for different nodes
add_executable(${PROJECT_NAME}_lcd_node
    src/lcd_node.cpp
    src/config.cpp
    src/lcd_core.cpp
    src/lcd_display_service.cpp
)

add_executable(${PROJECT_NAME}_temp_node
    src/temperature_node.cpp
    src/config.cpp
    src/temp_monitor.cpp
)

add_executable(${PROJECT_NAME}_uptime_node
    src/uptime_node.cpp
    src/config.cpp
    src/uptime_monitor.cpp
)

# For backward compatibility, also add the original combined node
add_executable(${PROJECT_NAME}_node
    src/main.cpp
    src/config.cpp
    src/lcd_core.cpp
    src/lcd_display_service.cpp
    src/temp_monitor.cpp
    src/uptime_monitor.cpp
)

# Link libraries for all nodes
target_link_libraries(${PROJECT_NAME}_lcd_node pthread)
target_link_libraries(${PROJECT_NAME}_temp_node pthread)
target_link_libraries(${PROJECT_NAME}_uptime_node pthread)
target_link_libraries(${PROJECT_NAME}_node pthread)

# Conditional compilation for GPIO
if(ENABLE_GPIO)
    find_package(PkgConfig REQUIRED)
    # Try multiple possible names for gpiod pkg-config file
    pkg_check_modules(LIBGPIOD gpiod QUIET)
    if(NOT LIBGPIOD_FOUND)
        pkg_check_modules(LIBGPIOD libgpiod QUIET)
    endif()
    if(NOT LIBGPIOD_FOUND)
        pkg_check_modules(LIBGPIOD gpiod-dev QUIET)
    endif()
    
    if(LIBGPIOD_FOUND)
        target_compile_definitions(${PROJECT_NAME}_lcd_node PRIVATE USE_GPIO=1)
        target_compile_definitions(${PROJECT_NAME}_temp_node PRIVATE USE_GPIO=1)
        target_compile_definitions(${PROJECT_NAME}_uptime_node PRIVATE USE_GPIO=1)
        target_compile_definitions(${PROJECT_NAME}_node PRIVATE USE_GPIO=1)
        target_link_libraries(${PROJECT_NAME}_lcd_node ${LIBGPIOD_LIBRARIES})
        target_link_libraries(${PROJECT_NAME}_temp_node ${LIBGPIOD_LIBRARIES})
        target_link_libraries(${PROJECT_NAME}_uptime_node ${LIBGPIOD_LIBRARIES})
        target_link_libraries(${PROJECT_NAME}_node ${LIBGPIOD_LIBRARIES})
        target_include_directories(${PROJECT_NAME}_lcd_node PRIVATE ${LIBGPIOD_INCLUDE_DIRS})
        target_include_directories(${PROJECT_NAME}_temp_node PRIVATE ${LIBGPIOD_INCLUDE_DIRS})
        target_include_directories(${PROJECT_NAME}_uptime_node PRIVATE ${LIBGPIOD_INCLUDE_DIRS})
        target_include_directories(${PROJECT_NAME}_node PRIVATE ${LIBGPIOD_INCLUDE_DIRS})
    else()
        message(WARNING "libgpiod not found via pkg-config, trying direct linking")
        # Try to find libgpiod library directly
        find_library(LIBGPIOD_LIB NAMES gpiod libgpiod)
        if(LIBGPIOD_LIB)
            target_compile_definitions(${PROJECT_NAME}_lcd_node PRIVATE USE_GPIO=1)
            target_compile_definitions(${PROJECT_NAME}_temp_node PRIVATE USE_GPIO=1)
            target_compile_definitions(${PROJECT_NAME}_uptime_node PRIVATE USE_GPIO=1)
            target_compile_definitions(${PROJECT_NAME}_node PRIVATE USE_GPIO=1)
            target_link_libraries(${PROJECT_NAME}_lcd_node ${LIBGPIOD_LIB} pthread)
            target_link_libraries(${PROJECT_NAME}_temp_node ${LIBGPIOD_LIB} pthread)
            target_link_libraries(${PROJECT_NAME}_uptime_node ${LIBGPIOD_LIB} pthread)
            target_link_libraries(${PROJECT_NAME}_node ${LIBGPIOD_LIB} pthread)
            message(STATUS "libgpiod library found directly: ${LIBGPIOD_LIB}")
        else()
            message(WARNING "libgpiod not found, building without GPIO support")
            target_compile_definitions(${PROJECT_NAME}_lcd_node PRIVATE USE_GPIO=0)
            target_compile_definitions(${PROJECT_NAME}_temp_node PRIVATE USE_GPIO=0)
            target_compile_definitions(${PROJECT_NAME}_uptime_node PRIVATE USE_GPIO=0)
            target_compile_definitions(${PROJECT_NAME}_node PRIVATE USE_GPIO=0)
            set(ENABLE_GPIO OFF)
        endif()
    endif()
else()
    target_compile_definitions(${PROJECT_NAME}_lcd_node PRIVATE USE_GPIO=0)
    target_compile_definitions(${PROJECT_NAME}_temp_node PRIVATE USE_GPIO=0)
    target_compile_definitions(${PROJECT_NAME}_uptime_node PRIVATE USE_GPIO=0)
    target_compile_definitions(${PROJECT_NAME}_node PRIVATE USE_GPIO=0)
    # Even when GPIO is disabled, still link against libgpiod if available to avoid linker errors
    # This ensures that the #ifdef USE_GPIO conditionals work correctly without linker errors
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        # Try multiple possible names for gpiod pkg-config file
        pkg_check_modules(LIBGPIOD gpiod QUIET)
        if(NOT LIBGPIOD_FOUND)
            pkg_check_modules(LIBGPIOD libgpiod QUIET)
        endif()
        if(NOT LIBGPIOD_FOUND)
            pkg_check_modules(LIBGPIOD gpiod-dev QUIET)
        endif()
        if(LIBGPIOD_FOUND)
            target_link_libraries(${PROJECT_NAME}_lcd_node ${LIBGPIOD_LIBRARIES})
            target_link_libraries(${PROJECT_NAME}_temp_node ${LIBGPIOD_LIBRARIES})
            target_link_libraries(${PROJECT_NAME}_uptime_node ${LIBGPIOD_LIBRARIES})
            target_link_libraries(${PROJECT_NAME}_node ${LIBGPIOD_LIBRARIES})
        else()
            find_library(LIBGPIOD_LIB NAMES gpiod libgpiod)
            if(LIBGPIOD_LIB)
                target_link_libraries(${PROJECT_NAME}_lcd_node ${LIBGPIOD_LIB})
                target_link_libraries(${PROJECT_NAME}_temp_node ${LIBGPIOD_LIB})
                target_link_libraries(${PROJECT_NAME}_uptime_node ${LIBGPIOD_LIB})
                target_link_libraries(${PROJECT_NAME}_node ${LIBGPIOD_LIB})
            endif()
        endif()
    endif()
    # Ensure basic libraries are linked
    target_link_libraries(${PROJECT_NAME}_lcd_node pthread)
    target_link_libraries(${PROJECT_NAME}_temp_node pthread)
    target_link_libraries(${PROJECT_NAME}_uptime_node pthread)
    target_link_libraries(${PROJECT_NAME}_node pthread)
endif()

# Conditional compilation for ROS2
if(ENABLE_ROS2)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(builtin_interfaces REQUIRED)
    
    target_compile_definitions(${PROJECT_NAME}_lcd_node PRIVATE "USE_ROS2=1")
    target_compile_definitions(${PROJECT_NAME}_temp_node PRIVATE "USE_ROS2=1")
    target_compile_definitions(${PROJECT_NAME}_uptime_node PRIVATE "USE_ROS2=1")
    target_compile_definitions(${PROJECT_NAME}_node PRIVATE "USE_ROS2=1")
    
    target_link_libraries(${PROJECT_NAME}_lcd_node rclcpp std_msgs builtin_interfaces)
    target_link_libraries(${PROJECT_NAME}_temp_node rclcpp std_msgs builtin_interfaces)
    target_link_libraries(${PROJECT_NAME}_uptime_node rclcpp std_msgs builtin_interfaces)
    target_link_libraries(${PROJECT_NAME}_node rclcpp std_msgs builtin_interfaces)
    
    ament_target_dependencies(${PROJECT_NAME}_lcd_node rclcpp std_msgs builtin_interfaces)
    ament_target_dependencies(${PROJECT_NAME}_temp_node rclcpp std_msgs builtin_interfaces)
    ament_target_dependencies(${PROJECT_NAME}_uptime_node rclcpp std_msgs builtin_interfaces)
    ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs builtin_interfaces)
    
    install(TARGETS ${PROJECT_NAME}_lcd_node ${PROJECT_NAME}_temp_node ${PROJECT_NAME}_uptime_node ${PROJECT_NAME}_node
      DESTINATION lib/${PROJECT_NAME}
    )
    
    ament_package()
else()
    # Для локальной сборки без ROS2
    set_target_properties(${PROJECT_NAME}_lcd_node ${PROJECT_NAME}_temp_node ${PROJECT_NAME}_uptime_node ${PROJECT_NAME}_node PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
    )
    
    # Include directories for ROS2 headers when ENABLE_ROS2=OFF but files still need to compile
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(ROS2_DEPS QUIET rclcpp std_msgs builtin_interfaces)
        if(ROS2_DEPS_FOUND)
            target_include_directories(${PROJECT_NAME}_lcd_node PRIVATE ${ROS2_DEPS_INCLUDE_DIRS})
            target_include_directories(${PROJECT_NAME}_temp_node PRIVATE ${ROS2_DEPS_INCLUDE_DIRS})
            target_include_directories(${PROJECT_NAME}_uptime_node PRIVATE ${ROS2_DEPS_INCLUDE_DIRS})
            target_include_directories(${PROJECT_NAME}_node PRIVATE ${ROS2_DEPS_INCLUDE_DIRS})
        endif()
    endif()
    
    # Alternative approach: define dummy macros when ROS2 is disabled to prevent compilation errors
    target_compile_definitions(${PROJECT_NAME}_lcd_node PRIVATE "USE_ROS2=0")
    target_compile_definitions(${PROJECT_NAME}_temp_node PRIVATE "USE_ROS2=0")
    target_compile_definitions(${PROJECT_NAME}_uptime_node PRIVATE "USE_ROS2=0")
    target_compile_definitions(${PROJECT_NAME}_node PRIVATE "USE_ROS2=0")
endif()