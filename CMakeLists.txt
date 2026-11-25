# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# 显式设置版本号，避免 ESP-IDF 在构建时用 git describe 读取
# .git/refs/heads/动作组最终 —— ninja 无法识别含中文的路径，
# 会把它当成缺失文件，导致每次构建都无限 "Re-running CMake"。
set(PROJECT_VER "1.0.0")
project(Spider_Robot)