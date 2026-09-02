cmake_minimum_required(VERSION 3.16)

project(SimRobotFleet LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

add_library(sim_robot_fleet
    src/Display.cpp
    src/FleetManager.cpp
    src/Pose.cpp
    src/RandomPlanner.cpp
    src/Robot.cpp
    src/RobotBuilder.cpp
    src/RobotFleet.cpp
)

target_include_directories(sim_robot_fleet
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include/SimRobotFleet
)

target_link_libraries(sim_robot_fleet
    PUBLIC
        ${OpenCV_LIBS}
        Threads::Threads
)

add_executable(sim_robot_fleet_app
    src/main.cpp
)

target_link_libraries(sim_robot_fleet_app
    PRIVATE
        sim_robot_fleet
)
