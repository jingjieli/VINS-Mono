#include "mapper.h"
#include "stereo_mapper.h"
#include "chisel_node.h"

Mapper::Mapper()
{
    if (!stereo_mapper)
    {
        stereo_mapper = new StereoMapper();
    }
}

Mapper::~Mapper()
{
    if (stereo_mapper)
    {
        delete stereo_mapper;
    }
    stereo_mapper = nullptr;
}

void Mapper::init()
{
    cv::FileStorage param_reader_l("/home/gingerli/vins_ws/src/VINS-Mono/stereo_mapper/config/euroc_left.yaml", 
                                    cv::FileStorage::READ);
    param_reader_l["camera_matrix"] >> K1;
    param_reader_l["distortion_coefficients"] >> D1;

    cv::cv2eigen(K1, K_eigen);
    // FIX ME ?
    //cv::cv2eigen(K, K_eigen);
    std::cout << "[Mapper] K_eigen: " << K_eigen << std::endl;
    std::cout << "[Mapper] K1: " << K1 << std::endl;
    std::cout << "[Mapper] D1: " << D1 << std::endl;
    stereo_mapper->initIntrinsic(K1, D1, K1, D1);

    for (int i = 1; i <= DEP_CNT; i++)
        std::cout << 1.0f / (DEP_SAMPLE * i) << std::endl;
}

void Mapper::setChiselNode(ChiselNode* chisel_node)
{
    if (chisel_node)
    {
        chisel_ptr = chisel_node;
    }
}

void Mapper::prepareFrame(const FRAME_MSG &frame_msg)
{
    img_pool[std::to_string(frame_msg.timestamp)] = frame_msg.frame.clone();
    img_pool_r[std::to_string(frame_msg.timestamp)] = frame_msg.frame.clone();

    // FIX ME
    // if (OFFLINE)

    // if (BENCHMARK)
}

void Mapper::updateCurrentAndRefPose(const POSE_MSG &curr_pose, const POSE_MSG &ref_pose)
{
    std::string ref_time = std::to_string(ref_pose.timestamp);
    std::string cur_time = std::to_string(curr_pose.timestamp);
    std::map<std::string, cv::Mat>::iterator ref_it = img_pool.find(ref_time);
    std::map<std::string, cv::Mat>::iterator cur_it = img_pool.find(cur_time);
    //img_pool.erase(img_pool.begin(), ref_it);
    if (ref_it != img_pool.end() && cur_it != img_pool.end())
    {
        std::cout << "ref_time: " << ref_time << " cur_time: " << cur_time << std::endl;

        Eigen::Matrix3d R_l = Eigen::Quaterniond{ref_pose.orientation_w,
                                                 ref_pose.orientation_x,
                                                 ref_pose.orientation_y,
                                                 ref_pose.orientation_z}.toRotationMatrix();
        Eigen::Vector3d T_l = Eigen::Vector3d{ref_pose.position_x,
                                              ref_pose.position_y,
                                              ref_pose.position_z};

        cv::Mat cv_R_l, cv_T_l;
        cv::eigen2cv(R_l, cv_R_l);
        cv::eigen2cv(T_l, cv_T_l);

        Eigen::Matrix3d R_r = Eigen::Quaterniond{curr_pose.orientation_w,
                                                 curr_pose.orientation_x,
                                                 curr_pose.orientation_y,
                                                 curr_pose.orientation_z}.toRotationMatrix();
        Eigen::Vector3d T_r = Eigen::Vector3d{curr_pose.position_x,
                                              curr_pose.position_y,
                                              curr_pose.position_z};

        cv::Mat cv_R_r, cv_T_r;
        cv::eigen2cv(R_r, cv_R_r);
        cv::eigen2cv(T_r, cv_T_r);

        if (last_time != ref_time)
        {

            last_time = ref_time;

            stereo_mapper->initReference(ref_it->second);
            img1 = stereo_mapper->img_intensity;

            //cv::imshow("ep line", img1);
            cv::cv2eigen(cv_R_l, R1_eigen);
            cv::cv2eigen(cv_T_l, T1_eigen);
        }

        // FIX ME
        // {
        //     // camera frame
        //     static tf::TransformBroadcaster br;
        //     tf::Transform transform;
        //     tf::Quaternion q;
        //     transform.setOrigin(tf::Vector3(T_l.x(),
        //                                     T_l.y(),
        //                                     T_l.z()));
        //     q.setW(Eigen::Quaterniond(R_l).w());
        //     q.setX(Eigen::Quaterniond(R_l).x());
        //     q.setY(Eigen::Quaterniond(R_l).y());
        //     q.setZ(Eigen::Quaterniond(R_l).z());
        //     transform.setRotation(q);
        //     br.sendTransform(tf::StampedTransform(transform, ref_pose_ptr->header.stamp, "base", "ref_frame"));
        //     key_header = ref_pose_ptr->header;
        // }

        start = true;

        //cv::undistort(cur_it->second, img2, K1, D1);
        cv::cv2eigen(cv_R_r, R2_eigen);
        cv::cv2eigen(cv_T_r, T2_eigen);

        //cv::imshow("ep line", img1);

        TicToc t_update;
        stereo_mapper->update(cur_it->second, cv_R_l, cv_T_l, cv_R_r, cv_T_r);
        printf("[Mapper]update costs: %lf ms", t_update.toc());

        result = stereo_mapper->output();

        printf("[Mapper]publish to Fusion: %lf", ref_pose.timestamp);
        // FIX ME
        sendCloud(result, img1);

//         printf("[Mapper]publish point cloud: %lf", ref_pose.timestamp);
//         {
//             cv_bridge::CvImage out_msg;
//             out_msg.header = key_header;
//             out_msg.header.frame_id = "camera";
//             out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
//             out_msg.image = result.clone();
//             pub_disp_img.publish(out_msg.toImageMsg());
//         }

//         {
//             cv_bridge::CvImage out_msg;
//             out_msg.header = key_header;
//             out_msg.header.frame_id = "camera";
//             out_msg.encoding = sensor_msgs::image_encodings::MONO8;
//             out_msg.image = img1.clone();
//             pub_color_img.publish(out_msg.toImageMsg());
//         }

//         {
            CAMERA_INFO camera_info;
            camera_info.timestamp = ref_pose.timestamp;
#if DOWNSAMPLE
            camera_info.P[0] = K1.at<double>(0, 0) / 2;
            camera_info.P[5] = K1.at<double>(1, 1) / 2;
            camera_info.P[2] = K1.at<double>(0, 2) / 2;
            camera_info.P[6] = K1.at<double>(1, 2) / 2;
#else
            camera_info.P[0] = K1.at<double>(0, 0);
            camera_info.P[5] = K1.at<double>(1, 1);
            camera_info.P[2] = K1.at<double>(0, 2);
            camera_info.P[6] = K1.at<double>(1, 2);
#endif
            camera_info.width = WIDTH;
            camera_info.height = HEIGHT;
            // FIX ME
            // pub_color_img_info.publish(camera_info);
            // pub_disp_img_info.publish(camera_info);
//         }
    }
}


void Mapper::sendCloud(const cv::Mat &dense_points_, const cv::Mat &un_img_l0)
{
    // FIX ME
//     sensor_msgs::PointCloud2Ptr points(new sensor_msgs::PointCloud2);
//     points->header = key_header;
//     points->header.frame_id = "ref_frame";

//     points->height = dense_points_.rows;
//     points->width = dense_points_.cols;
//     points->fields.resize(4);
//     points->fields[0].name = "x";
//     points->fields[0].offset = 0;
//     points->fields[0].count = 1;
//     points->fields[0].datatype = sensor_msgs::PointField::FLOAT32;
//     points->fields[1].name = "y";
//     points->fields[1].offset = 4;
//     points->fields[1].count = 1;
//     points->fields[1].datatype = sensor_msgs::PointField::FLOAT32;
//     points->fields[2].name = "z";
//     points->fields[2].offset = 8;
//     points->fields[2].count = 1;
//     points->fields[2].datatype = sensor_msgs::PointField::FLOAT32;
//     points->fields[3].name = "rgb";
//     points->fields[3].offset = 12;
//     points->fields[3].count = 1;
//     points->fields[3].datatype = sensor_msgs::PointField::FLOAT32;
//     //points.is_bigendian = false; ???
//     points->point_step = 16;
//     points->row_step = points->point_step * points->width;
//     points->data.resize(points->row_step * points->height);
//     points->is_dense = false; // there may be invalid points

//     float bad_point = std::numeric_limits<float>::quiet_NaN();
//     int i = 0;
//     for (int32_t u = 0; u < dense_points_.rows; ++u)
//     {
//         for (int32_t v = 0; v < dense_points_.cols; ++v, ++i)
//         {
//             float dep = dense_points_.at<float>(u, v);
// #if DOWNSAMPLE
//             float x = dep * (v - K1.at<double>(0, 2) / 2) / (K1.at<double>(0, 0) / 2);
//             float y = dep * (u - K1.at<double>(1, 2) / 2) / (K1.at<double>(1, 1) / 2);
// #else
//             float x = dep * (v - K1.at<double>(0, 2)) / K1.at<double>(0, 0);
//             float y = dep * (u - K1.at<double>(1, 2)) / K1.at<double>(1, 1);
// #endif
//             if (dep < DEP_INF)
//             {
//                 uint8_t g = un_img_l0.at<uint8_t>(u, v);
//                 int32_t rgb = (g << 16) | (g << 8) | g;
//                 memcpy(&points->data[i * points->point_step + 0], &x, sizeof(float));
//                 memcpy(&points->data[i * points->point_step + 4], &y, sizeof(float));
//                 memcpy(&points->data[i * points->point_step + 8], &dep, sizeof(float));
//                 memcpy(&points->data[i * points->point_step + 12], &rgb, sizeof(int32_t));
//             }
//             else
//             {
//                 memcpy(&points->data[i * points->point_step + 0], &bad_point, sizeof(float));
//                 memcpy(&points->data[i * points->point_step + 4], &bad_point, sizeof(float));
//                 memcpy(&points->data[i * points->point_step + 8], &bad_point, sizeof(float));
//                 memcpy(&points->data[i * points->point_step + 12], &bad_point, sizeof(float));
//             }
//         }
//     }
//     pub_point_cloud2.publish(points);
}