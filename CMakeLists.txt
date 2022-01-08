cmake_minimum_required(VERSION 3.20)
project(CG)

add_subdirectory(src/tgaimage)

add_executable(CG src/linear/Vec.cpp src/linear/Vec.h src/linear/Mat.cpp src/linear/Mat.h src/main.cpp src/Number.cpp src/Number.h)
set_target_properties(CG PROPERTIES CXX_STANDARD 20)
target_include_directories(CG PUBLIC include)
target_link_libraries(CG PRIVATE tgaimage)
