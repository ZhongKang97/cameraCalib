cmake_minimum_required( VERSION 2.8 )
project ( CameraCalib )

#set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_BUILD_TYPE "Debug" )
set( CMAKE_CXX_FLAGS "-std=c++11" )

set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

############### dependencies ######################
#Boost
#find_package(Boost REQUIRED COMPONENTS system filesystem  thread)
#include_directories(${Boost_INCLUDE_DIR})
# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
#Threads
find_package(Threads)
set( THIRD_PARTY_LIBS 
    ${OpenCV_LIBS}
    ${CMAKE_THREAD_LIBS_INIT}
  #  ${Boost_LIBRARIES}
)
############### source and test ######################
include_directories(${PROJECT_SOURCE_DIR}/include )
#添加含有CMakeLists文件的子文件夹
add_subdirectory( src )
