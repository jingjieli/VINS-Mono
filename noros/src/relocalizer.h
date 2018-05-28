#pragma once 

#include <vector>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include "common/tic_toc.h"
#include "global_parameters.h"
// #include "keyframe.h"
// #include "pose_graph.h"
//#include "vi_estimator.h"

// struct POSE_MSG
// {
//     double timestamp;
//     double position_x;
//     double position_y;
//     double position_z;
//     double orientation_w;
//     double orientation_x;
//     double orientation_y;
//     double orientation_z;
//     double linear_x;
//     int linear_y;
// };

class KeyFrame;
class PoseGraph;
class VIEstimator;
class Visualizer;

class Relocalizer
{
public:
    Relocalizer();
    ~Relocalizer();
    void init();
    void setVIEstimator(VIEstimator* estimator_ptr);
    void setVisualizer(Visualizer* vis_ptr);
    void initLoopClosure();
    void readParameters(std::string file_path);
    void prepareFeatures(const IMG_MSG &img_msg);
    void prepareFrame(const FRAME_MSG &frame_msg);
    void preparePose(const POSE_MSG &pose_msg); 
    void updateExtrinsic(const POSE_MSG &pose_msg);
    void updateRelativePose(const POSE_MSG &pose_msg);
    void processVIO(const POSE_MSG &pose_msg); // process pose output by VIO
    void process();
    void quit();

private:
    std::mutex m_buf;
    std::mutex m_process;

    Eigen::Vector3d reloc_tic;
    Eigen::Matrix3d reloc_qic;

    std::queue<FRAME_MSG> image_buf;
    std::queue<IMG_MSG> point_buf;
    std::queue<POSE_MSG> pose_buf;
    std::queue<Eigen::Vector3d> odometry_buf;

    PoseGraph *pose_graph = nullptr;

    double last_image_time = -1.0;

    int skip_first_cnt = 0;
    int skip_cnt = 0;

    Eigen::Vector3d last_t = { -100, -100, -100 };

    int frame_index = 0;

    bool start_flag = false;
    bool load_flag = false; 

    int sequence = 1;

    std::thread measurement_process;
    //std::thread keyboard_command_process; // FIX ME

    VIEstimator* vi_estimator_ptr = nullptr;

    Visualizer* visualizer_ptr = nullptr;

    bool is_running = false;
};