cmake_minimum_required(VERSION 3.0.0)

#set(CMAKE_CXX_STANDARD 17)

project(RTK-X VERSION 0.1.0 LANGUAGES C CXX)

include_directories(${PROJECT_SOURCE_DIR}/include/)
aux_source_directory(${PROJECT_SOURCE_DIR}/src/ spprtk)
add_library(script STATIC ${spprtk})

add_executable(main ${PROJECT_SOURCE_DIR}/app/main.cpp)

target_link_libraries(main script)
