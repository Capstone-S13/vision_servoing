cmake_minimum_required(VERSION 3.0.2)
project(dock_tf)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package()

catkin_python_setup()

file(GLOB python_scripts_DIR
    "scripts/*.py")

install(
    DIRECTORY
    DESTINATION share/${PROJECT_NAME}
)

catkin_install_python(PROGRAMS
    scripts/dock_tf.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

