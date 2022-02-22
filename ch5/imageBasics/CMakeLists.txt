project(imageBasics)

find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(imageBasics imageBasics.cpp)

target_link_libraries(imageBasics ${OpenCV_LIBS})

add_executable(undistortImage undistortImage.cpp)

target_link_libraries(undistortImage ${OpenCV_LIBS})
