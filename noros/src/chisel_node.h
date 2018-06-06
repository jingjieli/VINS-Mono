#pragma once
// #include <iostream>
// #include <memory>
//#include "chisel_server.h"
#include "global_parameters.h"

class ChiselServer;

class ChiselNode
{
public:
    ChiselNode();
    ~ChiselNode();
    void init();
    void setColorImage(const FRAME_MSG &frame_msg);
    void setDepthImage(const FRAME_MSG &frame_msg);
    void setColorCameraInfo(const CAMERA_INFO &cam_info);
    void setDepthCameraInfo(const CAMERA_INFO &cam_info);
private:
    int chunkSizeX = 32;
    int chunkSizeY = 32;
    int chunkSizeZ = 32;
    double voxelResolution;
    double truncationDistScale = 8.0;
    int weight = 1;
    bool useCarving = true;
    bool useColor = true;
    double carvingDist = 0.05;
    double nearPlaneDist = 0.05;
    double farPlaneDist = 5.0;
    std::string depthImageTopic = "/depth_image";
    std::string depthImageInfoTopic = "/depth_camera_info";
    std::string depthImageTransform = "/camera_depth_optical_frame";
    std::string colorImageTopic = "/color_image";
    std::string colorImageInfoTopic = "/color_camera_info";
    std::string colorImageTransform = "/camera_rgb_optical_frame";
    std::string baseTransform = "/camera_link";
    std::string meshTopic = "full_mesh";
    std::string chunkBoxTopic = "chunk_boxes";
    FusionMode mode = FusionMode::DepthImage;;
    std::string modeString = "DepthImage";
    std::string pointCloudTopic = "/camera/depth_registered/points";

    ChiselServer* chisel_server = nullptr;
};