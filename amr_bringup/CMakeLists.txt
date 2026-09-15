# ament_python packages do not use CMakeLists.txt for building
# This file is intentionally minimal - all installation is handled by setup.py

cmake_minimum_required(VERSION 3.8)
project(amr_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

ament_package()
