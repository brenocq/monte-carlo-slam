cmake_minimum_required(VERSION 3.14)
project(monte-carlo-slam LANGUAGES CXX)
find_package(atta 0.4.0.0 REQUIRED)

atta_add_target(project_script "src/projectScript.cpp")
