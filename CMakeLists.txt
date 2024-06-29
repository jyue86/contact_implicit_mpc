cmake_minimum_required(VERSION 3.10)
project(contact_implicit_mpc)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 REQUIRED PATHS Eigen)
include_directories(Eigen/)

# find_package(Threads MODULE REQUIRED)
find_package(drake CONFIG REQUIRED PATHS /opt/drake)

add_executable(main src/examples/allegro_setup_main.cpp)
target_link_libraries(allegro_ex drake::drake Eigen3::Eigen)
