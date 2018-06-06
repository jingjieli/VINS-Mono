#pragma once

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <Eigen/Geometry>
#include <open_chisel/geometry/Geometry.h>
#include <open_chisel/camera/DepthImage.h>
#include <open_chisel/pointcloud/PointCloud.h>
#include "global_parameters.h"

template <class DataType>
void CvMatToDepthImg(const cv::Mat &image, chisel::DepthImage<DataType> *depthImage)
{
    printf("Got depth image of type %d\n", image.type());
    bool mmImage = false;

    //if (image->encoding == "16UC1")
    if (image.type() == 2)
    {
        mmImage = true;
    }
    //else if (image->encoding == "32FC1")
    if (image.type() == 5)
    {
        mmImage = false;
    }
    else
    {
        printf("Unrecognized depth image format.\n");
        assert(1 == 0);
        //return;
    }

    if (!mmImage)
    {
        size_t dataSize = image.step / image.cols;
        assert(depthImage->GetHeight() == static_cast<int>(image.rows) && depthImage->GetWidth() == static_cast<int>(image.cols));
        assert(dataSize == sizeof(DataType));
        const DataType *imageData = reinterpret_cast<const DataType *>(image.data);
        DataType *depthImageData = depthImage->GetMutableData();
        int totalPixels = image.cols * image.rows;
        for (int i = 0; i < totalPixels; i++)
        {
            depthImageData[i] = imageData[i];
        }
    }
    else
    {
        assert(depthImage->GetHeight() == static_cast<int>(image.rows) && depthImage->GetWidth() == static_cast<int>(image.cols));
        const uint16_t *imageData = reinterpret_cast<const uint16_t *>(image.data);
        DataType *depthImageData = depthImage->GetMutableData();
        int totalPixels = image.cols * image.rows;
        for (int i = 0; i < totalPixels; i++)
        {
            depthImageData[i] = (1.0f / 1000.0f) * imageData[i];
        }
    }
}

template <class DataType>
void CvMatToColorImg(const cv::Mat &image, chisel::ColorImage<DataType> *colorImage)
{
    size_t numChannels = colorImage->GetNumChannels();
    size_t dataSize = image.step / image.cols;
    assert(colorImage->GetHeight() == static_cast<int>(image.rows) && colorImage->GetWidth() == static_cast<int>(image.cols));

    if (dataSize != numChannels * sizeof(DataType))
    {
        printf("[Conversion] Inconsistent channel width: %lu. Expected %lu\n", dataSize, numChannels * sizeof(DataType));
        return;
    }

    const DataType *imageData = reinterpret_cast<const DataType *>(image.data);
    DataType *colorImageData = colorImage->GetMutableData();
    int totalPixels = image.cols * image.rows * numChannels;
    for (int i = 0; i < totalPixels; i++)
    {
        colorImageData[i] = imageData[i];
    }
}

template <class DataType>
chisel::ColorImage<DataType>* CvMatToColorImg(const cv::Mat &image)
{
    size_t numChannels = 0;

    if (image.channels() == 1)
    {
        numChannels = 1;
    }
    else if (image.channels() == 3)
    {
        numChannels = 3;
    }
    else if (image.channels() == 4)
    {
        numChannels = 4;
    }
    else
    {
        printf("[Conversion] Unsupported color image format %d. Supported formats are mono8, rgb8, bgr8, and bgra8\n", image.type());
    }

    chisel::ColorImage<DataType> *toReturn = new chisel::ColorImage<DataType>(image.cols, image.rows, numChannels);
    CvMatToColorImg(image, toReturn);
    return toReturn;
}

inline chisel::Transform VinsTfToChiselTf(const POSE_MSG &pose_msg)
{
    chisel::Transform transform;
    transform.translation()(0) = pose_msg.position_x;
    transform.translation()(1) = pose_msg.position_y;
    transform.translation()(2) = pose_msg.position_z;

    chisel::Quaternion quat;
    quat.x() = pose_msg.orientation_x;
    quat.y() = pose_msg.orientation_y;
    quat.z() = pose_msg.orientation_z;
    quat.w() = pose_msg.orientation_w;
    transform.linear() = quat.toRotationMatrix();

    return transform.inverse();
}

inline chisel::PinholeCamera VinsCameraToChiselCamera(const CAMERA_INFO &cam_info)
{
    chisel::PinholeCamera cameraToReturn;
    chisel::Intrinsics intrinsics;
    intrinsics.SetFx(cam_info.P[0]);
    intrinsics.SetFy(cam_info.P[5]);
    intrinsics.SetCx(cam_info.P[2]);
    intrinsics.SetCy(cam_info.P[6]);
    cameraToReturn.SetIntrinsics(intrinsics);
    cameraToReturn.SetWidth(cam_info.width);
    cameraToReturn.SetHeight(cam_info.height);
    return cameraToReturn;
}