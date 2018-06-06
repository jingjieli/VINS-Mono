#pragma once 

#include <map>
#include <set>
#include "global_parameters.h"
//#include <Eigen/Dense>
#include <opencv/cxeigen.hpp>

class StereoMapper;
class ChiselNode;

class Mapper 
{
public:
    Mapper();
    ~Mapper();
    void init();
    void setChiselNode(ChiselNode* chisel_node);
    void prepareFrame(const FRAME_MSG &frame_msg);
    void updateCurrentAndRefPose(const POSE_MSG &curr_pose, const POSE_MSG &ref_pose);
private:
    StereoMapper* stereo_mapper = nullptr;

    ChiselNode* chisel_ptr = nullptr;

    std::map<std::string, cv::Mat> img_pool;
    std::map<std::string, cv::Mat> img_pool_r;

    Eigen::Matrix3d K_eigen;
    Eigen::Matrix3d R_bs;
    Eigen::Vector3d P_bs;
    cv::Mat T_BS;

    cv::Mat K1, D1;
    cv::Mat K2, D2;
    cv::Mat R21, T21;
    cv::Mat R1, P1, R2, P2, Q;
    cv::Mat map11, map12;
    cv::Mat map21, map22;
    std::set<int> id_set_l;

    std::string last_time;
    cv::Mat result;

    cv::Mat K;

    cv::Mat img1, img2;
    Eigen::Matrix3d R1_eigen, R2_eigen;
    Eigen::Vector3d T1_eigen, T2_eigen;

    bool start = false;

    void sendCloud(const cv::Mat &dense_points_, const cv::Mat &un_img_l0);
};