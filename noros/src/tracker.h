#pragma once 

#include <cstdio>
#include <vector>
#include <eigen3/Eigen/Dense>

#include "global_parameters.h"
#include "feature_tracker.h"
// #include "vi_estimator.h"
// #include "relocalizer.h"

using namespace std;

// struct FEATURE_POINT
// {
//     float id_of_point;
//     float u_of_point;
//     float v_of_point;
//     float velocity_x_of_point;
//     float velocity_y_of_point;
//     float undistorted_x;
//     float undistorted_y;
//     float undistorted_z;
// };

// struct IMG_MSG
// {
//     double timestamp;
//     std::vector<FEATURE_POINT> features;
// }; 

// struct FRAME_MSG
// {
//     double timestamp;
//     cv::Mat frame;
// };

class VIEstimator;
class Relocalizer;

class Tracker 
{
public:
    Tracker();
    ~Tracker();
    void init();
    void setVIEstimator(VIEstimator* estimator_ptr);
    void setRelocalizer(Relocalizer* relocalizer);
    void processFrame(double img_timestamp, cv::Mat &input_frame);
private:
    bool first_image_flag = true;
    double first_image_time = 0.0, last_image_time = 0.0;
    int pub_count = 1;
    bool pub_init = false;
    FeatureTracker* trackerData[NUM_OF_CAM] = { nullptr };
    VIEstimator* vi_estimator_ptr = nullptr;
    Relocalizer* relocalizer_ptr = nullptr;

    void publishFrame(double img_timestamp,  cv::Mat &input_frame);
    void visualizeTracking(cv::Mat &input_frame);
};