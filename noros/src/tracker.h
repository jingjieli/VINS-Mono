#pragma once 

#include <cstdio>
#include <vector>
#include <eigen3/Eigen/Dense>

#include "global_parameters.h"
#include "feature_tracker.h"

using namespace std;

class VIEstimator;
class Relocalizer;
class Mapper;

class Tracker 
{
public:
    Tracker();
    ~Tracker();
    void init();
    void setVIEstimator(VIEstimator* estimator_ptr);
    void setMapper(Mapper* mapper);
    void setRelocalizer(Relocalizer* relocalizer);
    void processFrame(double img_timestamp, std::vector<cv::Mat> &input_frames);
private:
    bool first_image_flag = true;
    double first_image_time = 0.0, last_image_time = 0.0;
    int pub_count = 1;
    bool pub_init = false;
    FeatureTracker* trackerData[NUM_OF_CAM] = { nullptr };
    VIEstimator* vi_estimator_ptr = nullptr;
    Relocalizer* relocalizer_ptr = nullptr;
    Mapper* mapper_ptr = nullptr;

    void publishFrame(double img_timestamp, std::vector<cv::Mat> &input_frames);
    void visualizeTracking(std::vector<cv::Mat> &input_frames);
};