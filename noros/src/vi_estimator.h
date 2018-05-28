#pragma once

#include <stdio.h>
#include <queue>
#include <vector>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <opencv2/opencv.hpp>

#include "global_parameters.h"
#include "estimator.h"

using namespace std;

// struct LINEAR_ACCELERATION
// {
//     double linear_acceleration_x;
//     double linear_acceleration_y;
//     double linear_acceleration_z;
// };

// struct ANGULAR_VELOCITY
// {
//     double angular_velocity_x;
//     double angular_velocity_y;
//     double angular_velocity_z;
// };

// struct IMU_MSG
// {
//     double timestamp;
//     LINEAR_ACCELERATION linear_acceleration;
//     ANGULAR_VELOCITY angular_velocity;
// };

class Relocalizer;
class Visualizer;

class VIEstimator 
{
public:
    VIEstimator();
    ~VIEstimator();
    void init();
    void setRelocalizer(Relocalizer* reloc_ptr);
    void setVisualizer(Visualizer* vis_ptr);
    void prepareFeatures(const IMG_MSG &img_msg); // passed from tracker
    void prepareImu(const IMU_MSG &imu_msg); // passed from imu reader
    void prepareRolocalizationResult(const RELOC_MSG &reloc_msg); // passed from pose graph
    void fusion(); // visual-inertial odometry
    void quit();
private:

    bool feature_init = false;
    bool imu_init = true;
    std::condition_variable con;
    std::mutex m_buf;
    std::mutex m_state;
    std::mutex i_buf;
    std::mutex m_estimator; 
    queue<IMG_MSG> feature_buf;
    queue<RELOC_MSG> reloc_buf;
    queue<IMU_MSG> imu_buf;

    double last_imu_time = 0.0;
    double latest_time = 0.0;
    double current_time = -1.0;
    
    int sum_of_wait = 0;

    Eigen::Vector3d tmp_P;
    Eigen::Quaterniond tmp_Q;
    Eigen::Vector3d tmp_V;
    Eigen::Vector3d tmp_Ba;
    Eigen::Vector3d tmp_Bg;
    Eigen::Vector3d acc_0;
    Eigen::Vector3d gyr_0;
    
    // for stats
    double sum_of_path;
    Eigen::Vector3d last_path;
    double sum_of_time;
    int sum_of_calculation;

    Estimator* estimator = nullptr;

    Relocalizer* relocalizer_ptr = nullptr;

    Visualizer* visualizer_ptr = nullptr;

    std::thread measurement_process;

    bool is_running = false;

    void predict(const IMU_MSG &imu_msg);
    void update();
    void printStatistics(double t);
    void pubRelocalization();
    void pubOdometry(double timestamp);
    void pubCameraPose(double timestamp);
    void pubKeyframe();
    void pubPointCloud();
    void pubTF();
    void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, 
                            const Eigen::Vector3d &V, const double timestamp);

    std::vector<std::pair<std::vector<IMU_MSG>, IMG_MSG> > getMeasurements();
};