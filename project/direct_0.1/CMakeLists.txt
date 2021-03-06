cmake_minimum_required( VERSION 2.8 )
project ( myslam )

set(CMAKE_BUILD_TYPE Release)
set( CMAKE_CXX_COMPILER "g++" )
set(CMAKE_CXX_STANDARD 14)

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

############### dependencies ######################
# Eigen
include_directories( "/usr/include/eigen3" )
# OpenCV
find_package( OpenCV 3.1 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
# Sophus 
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )
# VTK
find_package( VTK REQUIRED )
include_directories( ${VTK_INCLUDE_DIRS} )

find_package( G2O REQUIRED )
include_directories( ${G2O_INCLUDE_DIRS} )

set( THIRD_PARTY_LIBS 
    ${OpenCV_LIBS}
    Sophus::Sophus
    ${VTK_LIBRARIES}
    ${G2O_CORE_LIBRARIY}
    ${G2O_TYPES_SBA}
    ${G2O_STUFF_LIBRARY}
)

############### dependencies ######################
include_directories( ${PROJECT_SOURCE_DIR}/include )
add_subdirectory( src )
add_subdirectory( test )
