#pragma once 

#include <thread>

#include "global_parameters.h"
class Tracker;
class VIEstimator;
class Relocalizer;
class Visualizer;
class Mapper;
class ChiselNode;

class VISystem 
{
public:
    VISystem();
    ~VISystem();
    void init(const std::string &config_file);
    void quit();
    bool readParameters(std::string config_file);
    void processFrame(double img_timestamp, std::vector<cv::Mat> &input_frames);
    void processImu(const IMU_MSG &imu_msg);
private:
    Tracker* tracker = nullptr;
    VIEstimator* vi_estimator = nullptr;
    Relocalizer* relocalizer = nullptr;
    Visualizer* visualizer = nullptr;
    Mapper* mapper = nullptr;
    ChiselNode* chisel_node = nullptr;

    std::thread tracker_thread;
    std::thread estimator_thread;
    std::thread reloc_thread;
};