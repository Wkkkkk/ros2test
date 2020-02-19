cmake_minimum_required(VERSION 3.5)
project(cpp_intra)

# Default to C99
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tutorials_msgs REQUIRED)

include_directories(include)

add_library(composition_nodes SHARED
        src/publisher_node.cpp
        src/subscriber_node.cpp)
target_compile_definitions(composition_nodes
        PRIVATE "MINIMAL_COMPOSITION_DLL")
ament_target_dependencies(composition_nodes rclcpp rclcpp_components tutorials_msgs)

# This package installs libraries without exporting them.
# Export the library path to ensure that the installed libraries are available.
if (NOT WIN32)
    ament_environment_hooks(
            "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
    )
endif ()

add_executable(composition_publisher src/standalone_publisher.cpp)
target_link_libraries(composition_publisher composition_nodes)
ament_target_dependencies(composition_publisher
        rclcpp)

add_executable(composition_subscriber src/standalone_subscriber.cpp)
target_link_libraries(composition_subscriber composition_nodes)
ament_target_dependencies(composition_subscriber
        rclcpp)

add_executable(composition_composed src/composed.cpp)
target_link_libraries(composition_composed composition_nodes)
ament_target_dependencies(composition_composed rclcpp)

install(TARGETS
        composition_nodes
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(TARGETS
        composition_publisher
        composition_subscriber
        composition_composed
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}/
        )

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
