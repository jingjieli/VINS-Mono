#pragma once 

#include <memory>
#include <open_chisel/Chisel.h>
#include <open_chisel/ProjectionIntegrator.h>
#include <open_chisel/camera/PinholeCamera.h>
#include <open_chisel/camera/DepthImage.h>
#include <open_chisel/camera/ColorImage.h>
#include <open_chisel/pointcloud/PointCloud.h>

#include "global_parameters.h"

typedef float DepthData;
typedef uint8_t ColorData;

class ChiselServer
{
public:

    struct CameraTopic
    {
        std::string imageTopic;
        std::string infoTopic;
        std::string transform;
        chisel::PinholeCamera cameraModel;
        // ros::Subscriber imageSubscriber;
        // ros::Subscriber infoSubscriber;
        // ros::Publisher lastPosePublisher;
        // ros::Publisher frustumPublisher;
        chisel::Transform lastPose;
        //ros::Time lastImageTimestamp;
        double lastImageTimestamp;
        bool gotPose;
        bool gotInfo;
        bool gotImage;

        // message_filters::Subscriber<sensor_msgs::Image> *sub_image;
        // message_filters::Subscriber<sensor_msgs::CameraInfo> *sub_info;
    };

    struct PointCloudTopic
    {
        std::string cloudTopic;
        std::string transform;
        //ros::Subscriber cloudSubscriber;
        chisel::Transform lastPose;
        //ros::Time lastTimestamp;
        double lastTimestamp;
        bool gotPose;
        bool gotCloud;
        //message_filters::Subscriber<sensor_msgs::PointCloud2> *sub_point_cloud;
    };

    ChiselServer(int chunkSizeX, int chunkSizeY, int chunkSizeZ, float resolution, bool color, FusionMode fusionMode);
    ~ChiselServer();
    void SetupProjectionIntegrator(chisel::TruncatorPtr truncator, uint16_t weight, bool useCarving, float carvingDist);
    
    inline chisel::ChiselPtr GetChiselMap()
    {
        return chiselMap;
    }
    inline void SetChiselMap(const chisel::ChiselPtr value)
    {
        chiselMap = value;
    }

    inline float GetNearPlaneDist() const
    {
        return nearPlaneDist;
    }
    inline float GetFarPlaneDist() const
    {
        return farPlaneDist;
    }
    
    inline void SetNearPlaneDist(float dist)
    {
        nearPlaneDist = dist;
    }

    inline void SetFarPlaneDist(float dist)
    {
        farPlaneDist = dist;
    }

    void SubscribeAll(const std::string &depth_imageTopic, const std::string &depth_infoTopic,
                      const std::string &color_imageTopic, const std::string &color_infoTopic,
                      const std::string &transform, const std::string &point_cloud_topic);
    void SubscribePointCloud(const std::string &topic);
    void SubscribeDepthImage(const std::string &depthImageTopic, const std::string &cameraInfoTopic, const std::string &transform);
    void SubscribeColorImage(const std::string &colorImageTopic, const std::string &cameraInfoTopic, const std::string &transform);
    void SetupDepthPosePublisher(const std::string &depthPoseTopic);
    void SetupColorPosePublisher(const std::string &colorPoseTopic);
    void SetupDepthFrustumPublisher(const std::string &frustumTopic);
    void SetupColorFrustumPublisher(const std::string &frustumTopic);
    void AdvertiseServices();

    inline void SetBaseTransform(const std::string &frameName)
    {
        baseTransform = frameName;
    }

    inline bool HasNewData()
    {
        return hasNewData;
    }

    inline bool IsPaused()
    {
        return isPaused;
    }
    inline void SetPaused(bool paused)
    {
        isPaused = paused;
    }

    inline FusionMode GetMode()
    {
        return mode;
    }
    inline void SetMode(const FusionMode &m)
    {
        mode = m;
    }

    void SetupMeshPublisher(const std::string &meshTopic);
    void SetupChunkBoxPublisher(const std::string &boxTopic);

    // replcae all callbacks here
    void setDepthImage(const FRAME_MSG &frame_msg);
    void setDepthCameraInfo(const CAMERA_INFO &cam_info);
    void setColorImage(const FRAME_MSG &frame_msg);
    void setColorCameraInfo(const CAMERA_INFO &cam_info);
    void setPointCloud(const POINT_CLOUD_MSG &pc_msg);

private:

    chisel::ChiselPtr chiselMap;
    std::shared_ptr<chisel::DepthImage<DepthData> > lastDepthImage;
    std::shared_ptr<chisel::ColorImage<ColorData> > lastColorImage;
    chisel::PointCloudPtr lastPointCloud;
    chisel::ProjectionIntegrator projectionIntegrator;
    std::string baseTransform;
    std::string meshTopic;
    std::string chunkBoxTopic;
    bool useColor;
    bool hasNewData;
    float nearPlaneDist;
    float farPlaneDist;
    bool isPaused;
    FusionMode mode;
    CameraTopic depthCamera;
    CameraTopic colorCamera;
    PointCloudTopic pointcloudTopic;

    void IntegrateLastDepthImage();
    void IntegrateLastPointCloud();
    void PublishLatestChunkBoxes();
    void PublishDepthFrustum();
    void PublishDepthPose();
    void PublishColorPose();
    void PublishColorFrustum();
    void PublishMeshes();
    void PublishChunkBoxes();
};