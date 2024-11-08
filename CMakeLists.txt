
# CMake minimum version
cmake_minimum_required(VERSION 3.5)

# Project name
project(New_robot_controller)

# Specify required Qt modules
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Specify Qt version
find_package(Qt5 COMPONENTS Core Gui Network Widgets REQUIRED)

# Define the executable target
add_executable(New_robot_controller
    main.cpp
    mainwindow.cpp
    udp.cpp
    yrc1000micro_com.cpp
    yrc1000micro_command.cpp
    controller.cpp
    mainwindow.ui  # Include UI files directly in the target
)

# Add headers to the target (necessary if you want to expose them in IDEs)
target_include_directories(New_robot_controller PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link Qt libraries to the target
target_link_libraries(New_robot_controller Qt5::Core Qt5::Gui Qt5::Network Qt5::Widgets)

# Define the deprecated warnings flag
target_compile_definitions(New_robot_controller PRIVATE QT_DEPRECATED_WARNINGS)
