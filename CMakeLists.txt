# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(esp-idf-bmi160-spi)


# set_target_properties(esp-idf-bmi160-spi PROPERTIES
#     CXX_STANDARD 23
#     CXX_STANDARD_REQUIRED ON
#     CXX_EXTENSIONS ON
# )