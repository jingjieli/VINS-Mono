#pragma once

// from feature_tracker

#include <vector>
#include <fstream>
#include <cstdio>
#include <eigen3/Eigen/Dense>
#include "camodocal/camera_models/Camera.h"

const double FOCAL_LENGTH = 460.0;
const int NUM_OF_CAM = 1;
const int WINDOW_SIZE = 10;
const int NUM_OF_F = 1000;
extern std::vector<std::string> CAM_NAMES;

// ****** tracker ******
extern int ROW;
extern int COL;

extern int MAX_CNT;
extern int MIN_DIST;
extern int FREQ;
extern double F_THRESHOLD;
extern int SHOW_TRACK;
extern bool STEREO_TRACK;
extern int EQUALIZE;
extern int FISHEYE;
extern std::string FISHEYE_MASK;
extern bool PUB_THIS_FRAME;

// -----------------------
// estimator
extern double SOLVER_TIME;
extern int NUM_ITERATIONS;
extern double MIN_PARALLAX;
extern double ACC_N, ACC_W;
extern double GYR_N, GYR_W;
extern Eigen::Vector3d G;
extern int ESTIMATE_EXTRINSIC;
extern std::string EX_CALIB_RESULT_PATH;
extern std::vector<Eigen::Matrix3d> RIC;
extern std::vector<Eigen::Vector3d> TIC;
extern double INIT_DEPTH;
extern double BIAS_ACC_THRESHOLD;
extern double BIAS_GYR_THRESHOLD;
extern double TD;
extern double TR;
extern int ESTIMATE_TD;
extern int ROLLING_SHUTTER;

// relocalizer
extern int SKIP_CNT;
extern double SKIP_DIS;
extern int VISUALIZATION_SHIFT_X;
extern int VISUALIZATION_SHIFT_Y;
extern int LOOP_CLOSURE;
extern std::string BRIEF_PATTERN_FILE;
extern camodocal::CameraPtr m_camera;
extern int VISUALIZE_IMU_FORWARD;
extern int LOAD_PREVIOUS_POSE_GRAPH;
extern int FAST_RELOCALIZATION;
extern int SKIP_FIRST_CNT;
extern Eigen::Vector3d reloc_tic;
extern Eigen::Matrix3d reloc_qic;

enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1
};

enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};

struct FEATURE_POINT
{
    float id_of_point;
    float u_of_point;
    float v_of_point;
    float velocity_x_of_point;
    float velocity_y_of_point;
    float undistorted_x;
    float undistorted_y;
    float undistorted_z;
};

struct IMG_MSG
{
    double timestamp;
    std::vector<FEATURE_POINT> features;
}; 

struct FRAME_MSG
{
    double timestamp;
    cv::Mat frame;
};

struct LINEAR_ACCELERATION
{
    double linear_acceleration_x;
    double linear_acceleration_y;
    double linear_acceleration_z;
};

struct ANGULAR_VELOCITY
{
    double angular_velocity_x;
    double angular_velocity_y;
    double angular_velocity_z;
};

struct IMU_MSG
{
    double timestamp;
    LINEAR_ACCELERATION linear_acceleration;
    ANGULAR_VELOCITY angular_velocity;
};

struct POSE_MSG
{
    double timestamp;
    double position_x;
    double position_y;
    double position_z;
    double orientation_w;
    double orientation_x;
    double orientation_y;
    double orientation_z;
    double linear_x;
    double linear_y;
    double linear_z;
};

struct EDGE
{
    Eigen::Vector3d p0;
    Eigen::Vector3d p1;
};

struct MATCH_POINT
{
    float matched_x;
    float matched_y;
    float matched_z; 
};

struct RELOC_MSG
{
    double timestamp;
    std::vector<MATCH_POINT> matched_points;
    int idx;
    std::vector<double> transform;

    void printMessage()
    {
        printf("[RELOC_MSG] time: %lf, idx: %d, matched points: %zu\n", 
            timestamp, idx, matched_points.size());
        for (unsigned int i = 0; i < matched_points.size(); ++i)
        {
            printf("[RELOC_MSG] matched points: %f, %f, %f\n", 
                matched_points[i].matched_x,
                matched_points[i].matched_y,
                matched_points[i].matched_z
            );
        }

        printf("[RELOC_MSG] transform points: %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", 
            transform[0],
            transform[1],
            transform[2],
            transform[3],
            transform[4],
            transform[5],
            transform[6]
        );
    }
};

struct Data
{
    Data(FILE *f)
    {
        if (fscanf(f, " %lf,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", &t,
               &px, &py, &pz,
               &qw, &qx, &qy, &qz,
               &vx, &vy, &vz,
               &wx, &wy, &wz,
               &ax, &ay, &az) != EOF)
        {
            t /= 1e9;
        }
    }
    double t;
    float px, py, pz;
    float qw, qx, qy, qz;
    float vx, vy, vz;
    float wx, wy, wz;
    float ax, ay, az;
};

extern std::vector<Data> benchmark;
extern int SAVE_POSE_GRAPH;
extern int SAVE_NO_LOOP_PATH;
extern int SAVE_LOOP_PATH;
extern int DEBUG_IMAGE;
extern std::string POSE_GRAPH_SAVE_PATH;
extern std::string VINS_RESULT_PATH;
extern std::string VINS_RESULT_NO_LOOP_PATH;
extern std::string VINS_RESULT_LOOP_PATH;