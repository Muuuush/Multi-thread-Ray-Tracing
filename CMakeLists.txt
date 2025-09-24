cmake_minimum_required(VERSION 3.20)

set(CMAKE_CXX_STANDARD 20)

project(rc
    LANGUAGES CXX
)

add_executable(main main.cpp)
target_include_directories(main
    PUBLIC ./dep
    PUBLIC ./include)
