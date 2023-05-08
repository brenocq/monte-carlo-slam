cmake_minimum_required(VERSION 3.14)
project(monte-carlo-slam LANGUAGES CXX)
find_package(atta 0.4.0.0 REQUIRED)

# Components
atta_add_target(robot_component "src/robotComponent.cpp")
atta_add_target(map_component "src/mapComponent.cpp")

# Scripts
atta_add_target(robot_script "src/robotScript.cpp")
target_link_libraries(robot_script PRIVATE robot_component map_component)

# Project script
atta_add_target(project_script "src/projectScript.cpp")
target_link_libraries(project_script PRIVATE robot_component map_component)
