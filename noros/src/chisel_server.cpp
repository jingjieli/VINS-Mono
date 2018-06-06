#include <memory>
#include <cmath>
#include <open_chisel/truncation/Truncator.h>
#include <open_chisel/truncation/QuadraticTruncator.h>
#include <open_chisel/weighting/ConstantWeighter.h>
#include "chisel_server.h"
#include "conversion.h"

ChiselServer::ChiselServer(int chunkSizeX, int chunkSizeY, int chunkSizeZ, float resolution, bool color, FusionMode fusionMode)
{
    useColor = color;
    hasNewData = false;
    isPaused = false;
    mode = fusionMode;

    chiselMap.reset(new chisel::Chisel(Eigen::Vector3i(chunkSizeX, chunkSizeY, chunkSizeZ), resolution, color));
}
    
ChiselServer::~ChiselServer()
{

}
    
void ChiselServer::SetupProjectionIntegrator(chisel::TruncatorPtr truncator, uint16_t weight, bool useCarving, float carvingDist)
{
    projectionIntegrator.SetCentroids(GetChiselMap()->GetChunkManager().GetCentroids());
    projectionIntegrator.SetTruncator(truncator);
    projectionIntegrator.SetWeighter(chisel::WeighterPtr(new chisel::ConstantWeighter(weight)));
    projectionIntegrator.SetCarvingDist(carvingDist);
    projectionIntegrator.SetCarvingEnabled(useCarving);
}

void ChiselServer::SubscribeAll(const std::string &depth_imageTopic, const std::string &depth_infoTopic,
                      const std::string &color_imageTopic, const std::string &color_infoTopic,
                      const std::string &transform, const std::string &point_cloud_topic)
{
    // FIX ME
    // depthCamera.imageTopic = depth_imageTopic;
    // depthCamera.infoTopic = depth_infoTopic;
    // depthCamera.transform = transform;
    // depthCamera.sub_image = new message_filters::Subscriber<sensor_msgs::Image>(nh, depth_imageTopic, 100);
    // depthCamera.sub_info = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, depth_infoTopic, 100);

    // colorCamera.imageTopic = color_imageTopic;
    // colorCamera.infoTopic = color_infoTopic;
    // colorCamera.transform = transform;
    // colorCamera.sub_image = new message_filters::Subscriber<sensor_msgs::Image>(nh, color_imageTopic, 100);
    // colorCamera.sub_info = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, color_infoTopic, 100);

    // pointcloudTopic.cloudTopic = point_cloud_topic;
    // pointcloudTopic.gotCloud = false;
    // pointcloudTopic.gotPose = false;
    // pointcloudTopic.sub_point_cloud = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, point_cloud_topic, 100);

    // sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(1000),
    //                                                        *(depthCamera.sub_image), *(depthCamera.sub_info),
    //                                                        *(colorCamera.sub_image), *(colorCamera.sub_info),
    //                                                        *(pointcloudTopic.sub_point_cloud));
    // sync->registerCallback(boost::bind(&ChiselServer::CallbackAll, this, _1, _2, _3, _4, _5));
}
    
void ChiselServer::SubscribePointCloud(const std::string &topic)
{

}

void ChiselServer::SubscribeDepthImage(const std::string &depthImageTopic, const std::string &cameraInfoTopic, const std::string &transform)
{
    // depthCamera.imageTopic = imageTopic;
    // depthCamera.infoTopic = infoTopic;
    // depthCamera.transform = transform;
    // depthCamera.imageSubscriber = nh.subscribe(depthCamera.imageTopic, 20, &ChiselServer::DepthImageCallback, this);
    // depthCamera.infoSubscriber = nh.subscribe(depthCamera.infoTopic, 20, &ChiselServer::DepthCameraInfoCallback, this);
}

void ChiselServer::SubscribeColorImage(const std::string &colorImageTopic, const std::string &cameraInfoTopic, const std::string &transform)
{
    // colorCamera.imageTopic = imageTopic;
    // colorCamera.infoTopic = infoTopic;
    // colorCamera.transform = transform;
    // colorCamera.imageSubscriber = nh.subscribe(colorCamera.imageTopic, 20, &ChiselServer::ColorImageCallback, this);
    // colorCamera.infoSubscriber = nh.subscribe(colorCamera.infoTopic, 20, &ChiselServer::ColorCameraInfoCallback, this);
}

void ChiselServer::SetupDepthPosePublisher(const std::string &depthPoseTopic)
{
    //depthCamera.lastPosePublisher = nh.advertise<geometry_msgs::PoseStamped>(depthPoseTopic, 1);
}

void ChiselServer::SetupColorPosePublisher(const std::string &colorPoseTopic)
{
    //colorCamera.lastPosePublisher = nh.advertise<geometry_msgs::PoseStamped>(colorPoseTopic, 1);
}

void ChiselServer::SetupDepthFrustumPublisher(const std::string &frustumTopic)
{
    //depthCamera.frustumPublisher = nh.advertise<visualization_msgs::Marker>(frustumTopic, 1);
}

void ChiselServer::SetupColorFrustumPublisher(const std::string &frustumTopic)
{
    //colorCamera.frustumPublisher = nh.advertise<visualization_msgs::Marker>(frustumTopic, 1);
}

void ChiselServer::AdvertiseServices()
{
    // resetServer = nh.advertiseService("Reset", &ChiselServer::Reset, this);
    // pauseServer = nh.advertiseService("TogglePaused", &ChiselServer::TogglePaused, this);
    // saveMeshServer = nh.advertiseService("SaveMesh", &ChiselServer::SaveMesh, this);
    // getAllChunksServer = nh.advertiseService("GetAllChunks", &ChiselServer::GetAllChunks, this);
}

void ChiselServer::SetupMeshPublisher(const std::string &meshTopic)
{
    // meshTopic = topic;
    // meshPublisher = nh.advertise<visualization_msgs::Marker>(meshTopic, 1);
}

void ChiselServer::SetupChunkBoxPublisher(const std::string &boxTopic)
{
    // chunkBoxTopic = boxTopic;
    // chunkBoxPublisher = nh.advertise<visualization_msgs::Marker>(chunkBoxTopic, 1);
    // latestChunkPublisher = nh.advertise<visualization_msgs::Marker>(chunkBoxTopic + "/latest", 1);
}

void ChiselServer::setDepthImage(const FRAME_MSG &frame_msg)
{
    // FIX ME
    // if (IsPaused())
    //     return;
    if (!lastDepthImage.get())
    {
        lastDepthImage.reset(new chisel::DepthImage<DepthData>(frame_msg.frame.cols, frame_msg.frame.rows));
    }

    CvMatToDepthImg(frame_msg.frame, lastDepthImage.get());
    depthCamera.lastImageTimestamp = frame_msg.timestamp;
    depthCamera.gotImage = true;

    // bool gotTransform = false;
    // tf::StampedTransform tf;

    // int tries = 0;
    // int maxTries = 1;

    // while (!gotTransform && tries < maxTries)
    // {
    //     tries++;
    //     try
    //     {
    //         transformListener.waitForTransform(depthCamera.transform, baseTransform, depthImage->header.stamp, ros::Duration(0.1));
    //         transformListener.lookupTransform(depthCamera.transform, baseTransform, depthImage->header.stamp, tf);
    //         depthCamera.gotPose = true;
    //         gotTransform = true;
    //     }
    //     catch (std::exception &e)
    //     {
    //         ros::Rate lookupRate(0.5f);
    //         ROS_WARN("%s\n", e.what());
    //     }
    // }

    depthCamera.gotPose = true;
    POSE_MSG pose_msg;
    depthCamera.lastPose = VinsTfToChiselTf(pose_msg);

    hasNewData = true;
    if (!IsPaused() && HasNewData())
    {
        printf("[ChiselServer] Got data.\n");
        auto start = std::chrono::system_clock::now();
        switch (GetMode())
        {
        case FusionMode::DepthImage:
            IntegrateLastDepthImage();
            break;
        case FusionMode::PointCloud:
            IntegrateLastPointCloud();
            break;
        }
        std::chrono::duration<double> elapsed = std::chrono::system_clock::now() - start;
        printf("[ChiselServer] CHISEL: Done with scan, %lf ms\n", elapsed.count() * 1000);

        PublishChunkBoxes();
        if (chiselMap->GetMeshesToUpdate().size() == 0)
        {
            auto start = std::chrono::system_clock::now();
            PublishMeshes();
            std::chrono::duration<double> elapsed = std::chrono::system_clock::now() - start;
            printf("[ChiselServer] CHISEL: Done with publish, %lf ms\n", elapsed.count() * 1000);
        }

        if (mode == FusionMode::DepthImage)
        {
            PublishDepthPose();
            PublishDepthFrustum();

            if (useColor)
            {
                PublishColorPose();
                PublishColorFrustum();
            }
        }
        //puts("");
    }
}

void ChiselServer::setDepthCameraInfo(const CAMERA_INFO &cam_info)
{
    depthCamera.cameraModel = VinsCameraToChiselCamera(cam_info);
    depthCamera.cameraModel.SetNearPlane(GetNearPlaneDist());
    depthCamera.cameraModel.SetFarPlane(GetFarPlaneDist());
    depthCamera.gotInfo = true;
}

void ChiselServer::setColorImage(const FRAME_MSG &frame_msg)
{
    if (IsPaused())
        return;
    if (!lastColorImage.get())
    {
        lastColorImage.reset(CvMatToColorImg<ColorData>(frame_msg.frame));
    }

    CvMatToColorImg(frame_msg.frame, lastColorImage.get());

    colorCamera.lastImageTimestamp = frame_msg.timestamp;
    colorCamera.gotImage = true;

    // FIX ME
    // bool gotTransform = false;
    // tf::StampedTransform tf;

    // int tries = 0;
    // int maxTries = 1;

    // while (!gotTransform && tries < maxTries)
    // {
    //     tries++;
    //     try
    //     {
    //         transformListener.waitForTransform(colorCamera.transform, baseTransform, colorImage->header.stamp, ros::Duration(0.5));
    //         transformListener.lookupTransform(colorCamera.transform, baseTransform, colorImage->header.stamp, tf);
    //         colorCamera.gotPose = true;
    //         gotTransform = true;
    //     }
    //     catch (std::exception &e)
    //     {
    //         ros::Rate lookupRate(0.5f);
    //         ROS_WARN("%s\n", e.what());
    //     }
    // }

    colorCamera.gotPose = true;
    POSE_MSG pose_msg;
    colorCamera.lastPose = VinsTfToChiselTf(pose_msg);
}

void ChiselServer::setColorCameraInfo(const CAMERA_INFO &cam_info)
{
    colorCamera.cameraModel = VinsCameraToChiselCamera(cam_info);
    colorCamera.cameraModel.SetNearPlane(GetNearPlaneDist());
    colorCamera.cameraModel.SetFarPlane(GetFarPlaneDist());
    colorCamera.gotInfo = true;
}

void ChiselServer::setPointCloud(const POINT_CLOUD_MSG &pc_msg)
{
    printf("[ChiselServer] PointCloudCallback");
    if (IsPaused())
        return;
    if (!lastPointCloud.get())
    {
        lastPointCloud.reset(new chisel::PointCloud());
    }
    // FIX ME
    // ROSPointCloudToChisel(pointcloud, lastPointCloud.get());
    // pointcloudTopic.transform = pointcloud->header.frame_id;
    // bool gotTransform = false;
    // tf::StampedTransform tf;

    // int tries = 0;
    // int maxTries = 1;

    // while (!gotTransform && tries < maxTries)
    // {
    //     tries++;
    //     try
    //     {
    //         transformListener.waitForTransform(pointcloudTopic.transform, baseTransform, pointcloud->header.stamp, ros::Duration(0.5));
    //         transformListener.lookupTransform(pointcloudTopic.transform, baseTransform, pointcloud->header.stamp, tf);
    //         pointcloudTopic.gotPose = true;
    //         gotTransform = true;
    //     }
    //     catch (std::exception &e)
    //     {
    //         ros::Rate lookupRate(0.5f);
    //         ROS_WARN("%s\n", e.what());
    //     }
    // }

    // pointcloudTopic.lastPose = RosTfToChiselTf(tf);
    // pointcloudTopic.lastTimestamp = pointcloud->header.stamp;
    // hasNewData = true;
}

void ChiselServer::IntegrateLastDepthImage()
{
    if (!IsPaused() && depthCamera.gotInfo && depthCamera.gotPose && lastDepthImage.get())
    {
        printf("[ChiselServer] CHISEL: Integrating depth scan\n");
        auto start = std::chrono::system_clock::now();
        if (useColor)
        {
            chiselMap->IntegrateDepthScanColor<DepthData, ColorData>(projectionIntegrator, lastDepthImage, depthCamera.lastPose, depthCamera.cameraModel, lastColorImage, colorCamera.lastPose, colorCamera.cameraModel);
        }
        else
        {
            chiselMap->IntegrateDepthScan<DepthData>(projectionIntegrator, lastDepthImage, depthCamera.lastPose, depthCamera.cameraModel);
        }
        std::chrono::duration<double> elapsed = std::chrono::system_clock::now() - start;
        printf("[ChiselServer] CHISEL: Done with scan, %lf ms\n", elapsed.count() * 1000);
        PublishLatestChunkBoxes();
        PublishDepthFrustum();

        start = std::chrono::system_clock::now();
        printf("[ChiselServer] CHISEL: Updating meshes\n");
        chiselMap->UpdateMeshes();

        elapsed = std::chrono::system_clock::now() - start;
        printf("[ChiselServer] CHISEL: Done with mesh, %lf ms\n", elapsed.count() * 1000);
        hasNewData = false;
    }
}

void ChiselServer::IntegrateLastPointCloud()
{
    if (!IsPaused() && pointcloudTopic.gotPose && lastPointCloud.get())
    {
        printf("[ChiselServer] Integrating point cloud\n");
        chiselMap->IntegratePointCloud(projectionIntegrator, *lastPointCloud, pointcloudTopic.lastPose, 0.1f, farPlaneDist);
        PublishLatestChunkBoxes();
        chiselMap->UpdateMeshes();
        hasNewData = false;
    }
}

void ChiselServer::PublishLatestChunkBoxes()
{
    // FIX ME
    // if (!latestChunkPublisher)
    //     return;
    // const chisel::ChunkManager &chunkManager = chiselMap->GetChunkManager();
    // visualization_msgs::Marker marker;
    // marker.header.stamp = ros::Time::now();
    // marker.header.frame_id = baseTransform;
    // marker.ns = "chunk_box";
    // marker.type = visualization_msgs::Marker::CUBE_LIST;
    // marker.scale.x = chunkManager.GetChunkSize()(0) * chunkManager.GetResolution();
    // marker.scale.y = chunkManager.GetChunkSize()(1) * chunkManager.GetResolution();
    // marker.scale.z = chunkManager.GetChunkSize()(2) * chunkManager.GetResolution();
    // marker.pose.position.x = 0;
    // marker.pose.position.y = 0;
    // marker.pose.position.z = 0;
    // marker.pose.orientation.x = 0.0;
    // marker.pose.orientation.y = 0.0;
    // marker.pose.orientation.z = 0.0;
    // marker.pose.orientation.w = 1.0;
    // marker.color.r = 0.3f;
    // marker.color.g = 0.95f;
    // marker.color.b = 0.3f;
    // marker.color.a = 0.6f;
    // const chisel::ChunkSet &latest = chiselMap->GetMeshesToUpdate();
    // for (const std::pair<chisel::ChunkID, bool> &id : latest)
    // {
    //     if (chunkManager.HasChunk(id.first))
    //     {
    //         chisel::AABB aabb = chunkManager.GetChunk(id.first)->ComputeBoundingBox();
    //         chisel::Vec3 center = aabb.GetCenter();
    //         geometry_msgs::Point pt;
    //         pt.x = center.x();
    //         pt.y = center.y();
    //         pt.z = center.z();
    //         marker.points.push_back(pt);
    //     }
    // }

    // latestChunkPublisher.publish(marker);
}

void ChiselServer::PublishDepthFrustum()
{
    chisel::Frustum frustum;
    depthCamera.cameraModel.SetupFrustum(depthCamera.lastPose, &frustum);
    // FIX ME
    // visualization_msgs::Marker marker = CreateFrustumMarker(frustum);
    // depthCamera.frustumPublisher.publish(marker);
}

void ChiselServer::PublishDepthPose()
{
    chisel::Transform lastPose = depthCamera.lastPose;
    // FIX ME
    // geometry_msgs::PoseStamped pose;
    // pose.header.frame_id = baseTransform;
    // pose.header.stamp = depthCamera.lastImageTimestamp;
    // pose.pose.position.x = lastPose.translation()(0);
    // pose.pose.position.y = lastPose.translation()(1);
    // pose.pose.position.z = lastPose.translation()(2);

    // chisel::Quaternion quat(lastPose.rotation());
    // pose.pose.orientation.x = quat.x();
    // pose.pose.orientation.y = quat.y();
    // pose.pose.orientation.z = quat.z();
    // pose.pose.orientation.w = quat.w();

    // depthCamera.lastPosePublisher.publish(pose);
}
    
void ChiselServer::PublishMeshes()
{
    // FIX ME
    // visualization_msgs::Marker marker;
    // visualization_msgs::Marker marker2;
    // FillMarkerTopicWithMeshes(&marker, &marker2);

    // if (!marker2.points.empty())
    // {
    //     meshPublisher.publish(marker);
    //     meshPublisher.publish(marker2);
    // }
}

void ChiselServer::PublishChunkBoxes()
{
    // FIX ME
    const chisel::ChunkManager &chunkManager = chiselMap->GetChunkManager();
    // visualization_msgs::Marker marker;
    // marker.header.stamp = ros::Time::now();
    // marker.header.frame_id = baseTransform;
    // marker.ns = "chunk_box";
    // marker.type = visualization_msgs::Marker::CUBE_LIST;
    // marker.scale.x = chunkManager.GetChunkSize()(0) * chunkManager.GetResolution();
    // marker.scale.y = chunkManager.GetChunkSize()(1) * chunkManager.GetResolution();
    // marker.scale.z = chunkManager.GetChunkSize()(2) * chunkManager.GetResolution();
    // marker.pose.position.x = 0;
    // marker.pose.position.y = 0;
    // marker.pose.position.z = 0;
    // marker.pose.orientation.x = 0.0;
    // marker.pose.orientation.y = 0.0;
    // marker.pose.orientation.z = 0.0;
    // marker.pose.orientation.w = 1.0;
    // marker.color.r = 0.95f;
    // marker.color.g = 0.3f;
    // marker.color.b = 0.3f;
    // marker.color.a = 0.6f;
    // for (const std::pair<chisel::ChunkID, chisel::ChunkPtr> &pair : chunkManager.GetChunks())
    // {
    //     chisel::AABB aabb = pair.second->ComputeBoundingBox();
    //     chisel::Vec3 center = aabb.GetCenter();
    //     geometry_msgs::Point pt;
    //     pt.x = center.x();
    //     pt.y = center.y();
    //     pt.z = center.z();
    //     marker.points.push_back(pt);
    // }

    // chunkBoxPublisher.publish(marker);
}

void ChiselServer::PublishColorPose()
{
    // FIX ME
    chisel::Transform lastPose = colorCamera.lastPose;

    // geometry_msgs::PoseStamped pose;
    // pose.header.frame_id = baseTransform;
    // pose.header.stamp = colorCamera.lastImageTimestamp;
    // pose.pose.position.x = lastPose.translation()(0);
    // pose.pose.position.y = lastPose.translation()(1);
    // pose.pose.position.z = lastPose.translation()(2);

    // chisel::Quaternion quat(lastPose.rotation());
    // pose.pose.orientation.x = quat.x();
    // pose.pose.orientation.y = quat.y();
    // pose.pose.orientation.z = quat.z();
    // pose.pose.orientation.w = quat.w();

    // colorCamera.lastPosePublisher.publish(pose);
}

void ChiselServer::PublishColorFrustum()
{
    chisel::Frustum frustum;
    colorCamera.cameraModel.SetupFrustum(colorCamera.lastPose, &frustum);
    // FIX ME
    // visualization_msgs::Marker marker = CreateFrustumMarker(frustum);
    // colorCamera.frustumPublisher.publish(marker);
}