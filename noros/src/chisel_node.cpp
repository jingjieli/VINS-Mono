//#include <open_chisel/truncation/QuadraticTruncator.h>
#include <memory>
#include <open_chisel/truncation/Truncator.h>
#include <open_chisel/truncation/InverseTruncator.h>
#include "chisel_node.h"
#include "chisel_server.h"

ChiselNode::ChiselNode()
{
    if (!chisel_server)
    {
        chisel_server = new ChiselServer(chunkSizeX, chunkSizeY, chunkSizeZ, voxelResolution, useColor, mode);
    }
}

ChiselNode::~ChiselNode()
{
    if (chisel_server)
    {
        delete chisel_server;
    }
    chisel_server = nullptr;
}

void ChiselNode::init()
{
    chisel::TruncatorPtr truncator(new chisel::InverseTruncator(truncationDistScale));

    chisel_server->SetupProjectionIntegrator(truncator, static_cast<uint16_t>(weight), useCarving, carvingDist);

    assert(mode == FusionMode::DepthImage);
    assert(useColor && mode == FusionMode::DepthImage);

    chisel_server->SetNearPlaneDist(nearPlaneDist);
    chisel_server->SetFarPlaneDist(farPlaneDist);

    if (depthImageTransform == colorImageTransform)
    {
        chisel_server->SubscribeAll(depthImageTopic, depthImageInfoTopic,
                             colorImageTopic, colorImageInfoTopic,
                             depthImageTransform,
                             pointCloudTopic);
        //chisel_server->SubscribePointCloud(pointCloudTopic);
    }
    else
    {
        chisel_server->SubscribeDepthImage(depthImageTopic, depthImageInfoTopic, depthImageTransform);
        chisel_server->SubscribeColorImage(colorImageTopic, colorImageInfoTopic, colorImageTransform);
    }

    chisel_server->SetupDepthPosePublisher("last_depth_pose");
    chisel_server->SetupDepthFrustumPublisher("last_depth_frustum");

    chisel_server->SetupColorPosePublisher("last_color_pose");
    chisel_server->SetupColorFrustumPublisher("last_color_frustum");

    chisel_server->AdvertiseServices();

    chisel_server->SetBaseTransform(baseTransform);
    chisel_server->SetupMeshPublisher(meshTopic);
    chisel_server->SetupChunkBoxPublisher(chunkBoxTopic);
    printf("[ChiselNode] Beginning to loop.\n");
}

void ChiselNode::setColorImage(const FRAME_MSG &frame_msg)
{
    assert(chisel_server != nullptr);
    chisel_server->setColorImage(frame_msg);
}

void ChiselNode::setDepthImage(const FRAME_MSG &frame_msg)
{
    assert(chisel_server != nullptr);
    chisel_server->setDepthImage(frame_msg);
}

void ChiselNode::setColorCameraInfo(const CAMERA_INFO &cam_info)
{
    assert(chisel_server != nullptr);
    chisel_server->setColorCameraInfo(cam_info);
}

void ChiselNode::setDepthCameraInfo(const CAMERA_INFO &cam_info)
{
    assert(chisel_server != nullptr);
    chisel_server->setDepthCameraInfo(cam_info);
}