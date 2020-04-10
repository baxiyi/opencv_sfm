cmake_minimum_required(VERSION 3.0)
project(opencv_sfm)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Eigen
include_directories("/usr/include/eigen3")
# OpenCV
find_package(OpenCV 3.1 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
# Sophus 
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})
# Ceres
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS} )
# Boost
find_package(Boost      REQUIRED COMPONENTS system chrono filesystem unit_test_framework program_options)
include_directories(${Boost_INCLUDE_DIR})
# PCL
find_package(PCL REQUIRED COMPONENTS common io visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
set(THIRD_PARTY_LIBS
    ${OpenCV_LIBS}
    ${Sophus_LIBRARIES}
    ${CERES_LIBRARIES}
    ${Boost_LIBRARIES}
    opencv_xfeatures2d
    ${PCL_LIBRARIES}
    )

include_directories("${PROJECT_SOURCE_DIR}/camera")
include_directories("${PROJECT_SOURCE_DIR}/frame")
include_directories("${PROJECT_SOURCE_DIR}/map")
include_directories("${PROJECT_SOURCE_DIR}/mapPoint")
include_directories("${PROJECT_SOURCE_DIR}/bundleAdj")


add_executable(sfm_test main.cpp includes.h sfm.h sfm.cpp camera/camera.h camera/camera.cpp frame/frame.h frame/frame.cpp map/map.h map/map.cpp mapPoint/mapPoint.h mapPoint/mapPoint.cpp bundleAdj/bundleAdj.h bundleAdj/bundleAdj.cpp)
target_link_libraries(sfm_test ${THIRD_PARTY_LIBS})