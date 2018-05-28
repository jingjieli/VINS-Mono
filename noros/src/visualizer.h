#pragma once 

#include <thread>
#include <mutex>
#include "global_parameters.h"
#include "pangolin/pangolin.h"

class Visualizer 
{
public:
    Visualizer();
    ~Visualizer();
    void init();
    void updateCameraPose(const POSE_MSG &pose_msg);
    void updateCameraPose(const Eigen::Vector3d &P, const Eigen::Matrix3d &R);
    void addKeyframePose(const POSE_MSG &pose_msg);
    void updatePointCloud(const std::vector<Eigen::Vector3d> &points);
    void updateTrajectory(const POSE_MSG &pose_msg);
    void updateNoLoopPath(const POSE_MSG &pose_msg);
    void updateRelocPath(const POSE_MSG &pose_msg);
    void addLoopEdge(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1);
    void updatePoseGraph(const std::vector<POSE_MSG> &path);
    void updatePoseGraphPath(const std::vector<POSE_MSG> &path);
    void resetPoseGraph();
    void quit();
private:
    pangolin::OpenGlMatrix Twc;

    std::vector<pangolin::OpenGlMatrix> keyframe_poses;

    std::vector<pangolin::OpenGlMatrix> pose_graph_poses;

    std::vector<Eigen::Vector3d> pose_graph_traj;

    std::vector<Eigen::Vector3d> trajectory;

    std::vector<Eigen::Vector3d> no_loop_path;

    std::vector<Eigen::Vector3d> reloc_path;

    std::vector<Eigen::Vector3d> point_cloud;

    std::vector<EDGE> loop_edges;

    std::mutex render_mtx;
    std::thread rendering_thread;

    bool is_running = false;

    void start();
    void drawCurrentCamera(const pangolin::OpenGlMatrix &M);
    void drawKeyframes();
    void drawPointCloud();
    void drawTrajectory();
    void drawNoLoopPath();
    void drawRelocPath();
    void drawLoopEdge();
    void drawPoseGraph();
    void drawPoseGraphPath();
    void poseToGlMatrix(const Eigen::Vector3d &P, const Eigen::Matrix3d &R, pangolin::OpenGlMatrix &M);
};