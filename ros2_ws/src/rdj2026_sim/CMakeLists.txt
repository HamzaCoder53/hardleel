cmake_minimum_required(VERSION 3.8)
project(rdj2026_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY arena config launch maps urdf worlds
        DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/gate_controller.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()
