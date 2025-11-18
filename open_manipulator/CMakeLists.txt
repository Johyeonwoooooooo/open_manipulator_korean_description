################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5) # CMake (빌드 시스템) 3.5이상을 필요로 함
project(open_manipulator) # 프로젝트 이름 -> package.xml과 같아야함

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
# C++ 패키지 → ament_cmake
# Python 패키지 → ament_python
find_package(ament_cmake REQUIRED) # ament_cmake가 반드시 있어야함

################################################################################
# Macro for ament package
################################################################################
ament_package()
