set(PROJECT_NAME vins_mono_noros)
cmake_minimum_required(VERSION 2.8.3)
project(${PROJECT_NAME})

add_definitions(-DNO_ROS)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -fPIC")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -fPIC")

find_package(OpenCV 3.4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(Ceres REQUIRED)
find_package(Pangolin REQUIRED)
find_package(CUDA REQUIRED)
find_package(PCL REQUIRED)

get_directory_property(dir_defs DIRECTORY ${CMAKE_SOURCE_DIR} COMPILE_DEFINITIONS)
set(vtk_flags)
foreach(it ${dir_defs})
    if(it MATCHES "vtk*")
    list(APPEND vtk_flags ${it})
    endif()
endforeach()

foreach(d ${vtk_flags})
    remove_definitions(-D${d})
endforeach()
set(CUDA_NVCC_FLAGS
    ${CUDA_NVCC_FLAGS};
    #    -std=c++11 -O3 -gencode arch=compute_30,code=sm_30  # T440s
    #    -std=c++11 -O3 -gencode arch=compute_53,code=sm_53  # tx1
    #    -std=c++11 -O3 -gencode arch=compute_61,code=sm_61  # gtx 980 Ti / 1080
    -O3 -gencode arch=compute_30,code=sm_30  # gtx 760
)

include_directories(
  ${PROJECT_SOURCE_DIR}/src/
  ${PROJECT_SOURCE_DIR}/../camera_model/include/
  ${PROJECT_SOURCE_DIR}/../feature_tracker/src/
  ${PROJECT_SOURCE_DIR}/../vins_estimator/src/
  ${PROJECT_SOURCE_DIR}/../pose_graph/src/
  ${PROJECT_SOURCE_DIR}/../stereo_mapper/src/
  ${PROJECT_SOURCE_DIR}/../OpenChisel/open_chisel/include/
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

set(MAPPING_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/../stereo_mapper/src/stereo_mapper.cpp
  ${PROJECT_SOURCE_DIR}/../stereo_mapper/src/calc_cost.cu
  ${PROJECT_SOURCE_DIR}/src/mapper.cpp
)

file(GLOB OPEN_CHISEL_SOURCE_FILES
  #${PROJECT_SOURCE_DIR}/../OpenChisel/open_chisel/include/*.h*
  ${PROJECT_SOURCE_DIR}/../OpenChisel/open_chisel/src/camera/*.cpp
  ${PROJECT_SOURCE_DIR}/../OpenChisel/open_chisel/src/geometry/*.cpp
  ${PROJECT_SOURCE_DIR}/../OpenChisel/open_chisel/src/io/*.cpp
  ${PROJECT_SOURCE_DIR}/../OpenChisel/open_chisel/src/marching_cubes/*.cpp
  ${PROJECT_SOURCE_DIR}/../OpenChisel/open_chisel/src/mesh/*.cpp
  ${PROJECT_SOURCE_DIR}/../OpenChisel/open_chisel/src/pointcloud/*.cpp
  ${PROJECT_SOURCE_DIR}/../OpenChisel/open_chisel/src/*.cpp
  #${PROJECT_SOURCE_DIR}/src/chisel_server.cpp
  #${PROJECT_SOURCE_DIR}/src/chisel_node.cpp
)

set(VINS_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/../feature_tracker/src/feature_tracker.cpp
  ${PROJECT_SOURCE_DIR}/../vins_estimator/src/estimator.cpp
  ${PROJECT_SOURCE_DIR}/../vins_estimator/src/feature_manager.cpp
  ${PROJECT_SOURCE_DIR}/../vins_estimator/src/factor/pose_local_parameterization.cpp
  ${PROJECT_SOURCE_DIR}/../vins_estimator/src/factor/projection_factor.cpp
  ${PROJECT_SOURCE_DIR}/../vins_estimator/src/factor/projection_td_factor.cpp
  ${PROJECT_SOURCE_DIR}/../vins_estimator/src/factor/marginalization_factor.cpp
  ${PROJECT_SOURCE_DIR}/../vins_estimator/src/initial/solve_5pts.cpp
  ${PROJECT_SOURCE_DIR}/../vins_estimator/src/initial/initial_aligment.cpp
  ${PROJECT_SOURCE_DIR}/../vins_estimator/src/initial/initial_sfm.cpp
  ${PROJECT_SOURCE_DIR}/../vins_estimator/src/initial/initial_ex_rotation.cpp
  ${PROJECT_SOURCE_DIR}/../pose_graph/src/pose_graph.cpp
  ${PROJECT_SOURCE_DIR}/../pose_graph/src/keyframe.cpp
  ${PROJECT_SOURCE_DIR}/../pose_graph/src/ThirdParty/DBoW/BowVector.cpp
  ${PROJECT_SOURCE_DIR}/../pose_graph/src/ThirdParty/DBoW/FBrief.cpp
  ${PROJECT_SOURCE_DIR}/../pose_graph/src/ThirdParty/DBoW/FeatureVector.cpp
  ${PROJECT_SOURCE_DIR}/../pose_graph/src/ThirdParty/DBoW/QueryResults.cpp
  ${PROJECT_SOURCE_DIR}/../pose_graph/src/ThirdParty/DBoW/ScoringObject.cpp
  ${PROJECT_SOURCE_DIR}/../pose_graph/src/ThirdParty/DUtils/Random.cpp
  ${PROJECT_SOURCE_DIR}/../pose_graph/src/ThirdParty/DUtils/Timestamp.cpp
  ${PROJECT_SOURCE_DIR}/../pose_graph/src/ThirdParty/DVision/BRIEF.cpp
  ${PROJECT_SOURCE_DIR}/../pose_graph/src/ThirdParty/VocabularyBinary.cpp
  ${PROJECT_SOURCE_DIR}/../camera_model/src/chessboard/Chessboard.cc
  ${PROJECT_SOURCE_DIR}/../camera_model/src/calib/CameraCalibration.cc
  ${PROJECT_SOURCE_DIR}/../camera_model/src/camera_models/Camera.cc
  ${PROJECT_SOURCE_DIR}/../camera_model/src/camera_models/CameraFactory.cc
  ${PROJECT_SOURCE_DIR}/../camera_model/src/camera_models/CostFunctionFactory.cc
  ${PROJECT_SOURCE_DIR}/../camera_model/src/camera_models/PinholeCamera.cc
  ${PROJECT_SOURCE_DIR}/../camera_model/src/camera_models/CataCamera.cc
  ${PROJECT_SOURCE_DIR}/../camera_model/src/camera_models/EquidistantCamera.cc
  ${PROJECT_SOURCE_DIR}/../camera_model/src/camera_models/ScaramuzzaCamera.cc
  ${PROJECT_SOURCE_DIR}/../camera_model/src/sparse_graph/Transform.cc
  ${PROJECT_SOURCE_DIR}/../camera_model/src/gpl/gpl.cc
  ${PROJECT_SOURCE_DIR}/../camera_model/src/gpl/EigenQuaternionParameterization.cc
  #${PROJECT_SOURCE_DIR}/../stereo_mapper/src/stereo_mapper.cpp
  #${PROJECT_SOURCE_DIR}/../stereo_mapper/src/calc_cost.cu
  ${PROJECT_SOURCE_DIR}/src/global_parameters.cpp
  ${PROJECT_SOURCE_DIR}/src/common/utility.cpp
  ${PROJECT_SOURCE_DIR}/src/vi_estimator.cpp
  ${PROJECT_SOURCE_DIR}/src/relocalizer.cpp
  ${PROJECT_SOURCE_DIR}/src/visualizer.cpp
  ${PROJECT_SOURCE_DIR}/src/vi_system.cpp
  ${PROJECT_SOURCE_DIR}/src/tracker.cpp
  #${PROJECT_SOURCE_DIR}/src/mapper.cpp
  ${PROJECT_SOURCE_DIR}/src/chisel_server.cpp
  ${PROJECT_SOURCE_DIR}/src/chisel_node.cpp
)

Message("PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR}")
Message("Ceres: ${CERES_INCLUDE_DIRS}")
Message("Eigen: ${EIGEN3_INCLUDE_DIR}")
Message("OpenCV: Include: ${OpenCV_INCLUDE_DIRS} Libraries: ${OpenCV_LIBS}")
Message("Boost: ${Boost_INCLUDE_DIR}")
Message("Pangolin: ${Pangolin_INCLUDE_DIR}")
Message("PCL: ${PCL_INCLUDE_DIRS}")

cuda_add_library(stereo_mapper SHARED ${MAPPING_SOURCE_FILES})
target_link_libraries(stereo_mapper ${CUDA_LIBRARIES} ${OpenCV_LIBS})

add_library(open_chisel SHARED ${OPEN_CHISEL_SOURCE_FILES})
#target_link_libraries(open_chisel ${PCL_LIBRARIES})

add_library(${PROJECT_NAME} SHARED ${VINS_SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} stereo_mapper open_chisel ${PCL_LIBRARIES} 
${CERES_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${Pangolin_LIBRARIES})

add_executable(vi_executor src/main.cpp)
target_link_libraries(vi_executor ${PROJECT_NAME} stereo_mapper open_chisel ${OpenCV_LIBS})