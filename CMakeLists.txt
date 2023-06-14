cmake_minimum_required(VERSION 3.14)
project(monte-carlo-slam LANGUAGES CXX)
find_package(atta 0.4.1 REQUIRED EXACT)

# Components
atta_add_target(robot_component "src/robotComponent.cpp")
atta_add_target(map_component "src/mapComponent.cpp")

# Scripts
atta_add_target(controller "src/controller.cpp")
target_link_libraries(controller PRIVATE robot_component map_component)
atta_add_target(world "src/world.cpp")
target_link_libraries(world PRIVATE robot_component map_component)
atta_add_target(system "src/system.cpp")
target_link_libraries(system PRIVATE robot_component map_component)
