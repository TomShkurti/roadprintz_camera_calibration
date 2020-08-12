cmake_minimum_required(VERSION 2.8.3)
project(roadprintz_camera_calibration)

find_package(catkin_simple REQUIRED)

#uncomment next line to use OpenCV library
find_package(OpenCV REQUIRED)

#uncomment the next 2 lines to use the point-cloud library
#find_package(PCL 1.7 REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})


#uncomment the following 4 lines to use the Eigen library
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(Ceres REQUIRED)
message(STATUS "-- Found Ceres version ${CERES_VERSION}: ${CERES_INCLUDE_DIRS}")

include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# use carefully;  can interfere with point-cloud library
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

#uncomment the following 4 lines to use the Eigen library
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

# Libraries: uncomment the following and edit arguments to create a new library
cs_add_library(circle_detector src/circle_detector.cpp)   

# Executables: uncomment the following and edit arguments to compile new nodes
# may add more of these lines for more nodes from the same package
#cs_add_executable(find_red_pixels src/find_red_pixels.cpp)
#cs_add_executable(find_features src/find_features.cpp)
#cs_add_executable(draw_polylines src/draw_polylines.cpp)
cs_add_executable(ipcam src/ipcam.cpp)
cs_add_executable(ipcam_svc src/ipcam_svc.cpp)
cs_add_executable(test_find_contours src/test_find_contours.cpp)
cs_add_executable(test_find_patterns src/test_find_patterns.cpp)

cs_add_executable(optimize src/wsn_cam_calib_ceres.cpp)
cs_add_executable(optimize_randnoise src/wsn_cam_calib_ceres_randnoise.cpp)
cs_add_executable(extrinsic src/wsn_cam_calib_ceres_EXTRINSIC.cpp)
cs_add_executable(camera src/wsn_cam_calib_ceres_CAMERA.cpp)
cs_add_executable(head src/wsn_cam_calib_ceres_HEAD.cpp)

cs_add_executable(detector_hough src/detector_hough.cpp)
cs_add_executable(detector_corners src/detector_corners.cpp)
cs_add_executable(detector_integral src/detector_integral.cpp)

cs_add_executable(intrinsic src/INTRINSIC_CAL.cpp)
cs_add_executable(intrinsic_old ceres_code/wsn_cam_calib_ceres.cpp)
cs_add_executable(intrinsic_detect src/INTRINSIC_DETECT.cpp)


cs_add_executable(lester_shapematch src/lester_shapematch.cpp)
cs_add_executable(extrinsic_cam src/CAMERA_MONOTARGET_CAL.cpp)

#cs_add_executable(harris_corners src/harris_corners.cpp)
#the following is required, if desire to link a node in this package with a library created in this same package
# edit the arguments to reference the named node and named library within this package
# target_link_libraries(example my_lib)
#target_link_libraries( find_red_pixels ${OpenCV_LIBS})
#target_link_libraries( find_features ${OpenCV_LIBS})
#target_link_libraries( draw_polylines ${OpenCV_LIBS})
target_link_libraries( ipcam ${OpenCV_LIBS})
target_link_libraries( ipcam_svc ${OpenCV_LIBS})
target_link_libraries( test_find_contours circle_detector ${OpenCV_LIBS})
target_link_libraries( test_find_patterns circle_detector ${OpenCV_LIBS})

target_link_libraries(optimize ${OpenCV_LIBS} ${CERES_LIBRARIES})
target_link_libraries(extrinsic ${OpenCV_LIBS} ${CERES_LIBRARIES})
target_link_libraries(camera ${OpenCV_LIBS} ${CERES_LIBRARIES})
target_link_libraries(head ${OpenCV_LIBS} ${CERES_LIBRARIES})
target_link_libraries(optimize_randnoise ${OpenCV_LIBS} ${CERES_LIBRARIES})

target_link_libraries( detector_hough ${OpenCV_LIBS})
target_link_libraries( detector_corners ${OpenCV_LIBS})
target_link_libraries( detector_integral ${OpenCV_LIBS})


target_link_libraries(intrinsic ${OpenCV_LIBS} ${CERES_LIBRARIES})
target_link_libraries(intrinsic_old ${OpenCV_LIBS} ${CERES_LIBRARIES})
target_link_libraries(intrinsic_detect circle_detector ${OpenCV_LIBS} ${CERES_LIBRARIES})


target_link_libraries(lester_shapematch ${OpenCV_LIBS})
target_link_libraries(extrinsic_cam ${OpenCV_LIBS} ${CERES_LIBRARIES})

cs_install()
cs_export()
    
