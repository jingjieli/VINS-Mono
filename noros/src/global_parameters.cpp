#include "global_parameters.h"

std::vector<std::string> CAM_NAMES;
std::vector<Data> benchmark;

// ****** tracker ******
int ROW;
int COL;

int MAX_CNT;
int MIN_DIST;
int FREQ;
double F_THRESHOLD;
int SHOW_TRACK;
bool STEREO_TRACK;
int EQUALIZE;
int FISHEYE;
bool PUB_THIS_FRAME;

// -----------------------
// estimator
double SOLVER_TIME;
int NUM_ITERATIONS;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;
Eigen::Vector3d G;
int ESTIMATE_EXTRINSIC;
std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;
double INIT_DEPTH;
double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double TD;
double TR;
int ESTIMATE_TD;
int ROLLING_SHUTTER;

// relocalizer
int SKIP_CNT;
double SKIP_DIS;
int VISUALIZATION_SHIFT_X;
int VISUALIZATION_SHIFT_Y;
int LOOP_CLOSURE;
std::string BRIEF_PATTERN_FILE;
camodocal::CameraPtr m_camera;
int VISUALIZE_IMU_FORWARD;
int LOAD_PREVIOUS_POSE_GRAPH;
int FAST_RELOCALIZATION;
int SKIP_FIRST_CNT;
Eigen::Vector3d reloc_tic;
Eigen::Matrix3d reloc_qic;
