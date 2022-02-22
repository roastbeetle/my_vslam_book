cmake_minimum_required(VERSION 2.8)
project(ch8)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
add_definitions("-DENABLE_SSE")
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(OpenCV 3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(G2O REQUIRED)

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
        "/usr/include/eigen3/"
        ${Pangolin_INCLUDE_DIRS}
)

#add_executable(optical_flow optical_flow.cpp)
#target_link_libraries(optical_flow ${OpenCV_LIBS})

add_executable(direct_method direct_method.cpp)
target_link_libraries(direct_method Sophus::Sophus ${OpenCV_LIBS} ${Pangolin_LIBRARIES} g2o_core g2o_stuff)

add_executable(semi_dense semi_dense.cpp)
target_link_libraries(semi_dense Sophus::Sophus ${OpenCV_LIBS} ${Pangolin_LIBRARIES} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${G2O_TYPES_SBA} g2o_core g2o_stuff)
