#include "relocalizer.h"
#include "keyframe.h"
#include "pose_graph.h"
#include "vi_estimator.h"
#include "visualizer.h"

Relocalizer::Relocalizer()
{
    if (!pose_graph)
    {
        pose_graph = new PoseGraph();
    }

    printf("[Relocalizer] init done.\n");
}

Relocalizer::~Relocalizer()
{
    if (pose_graph)
    {
        delete pose_graph;
    }
    pose_graph = nullptr;
}

void Relocalizer::init()
{
    std::string file_path = "config.yaml"; // FIX ME: dummy
    readParameters(file_path);

    is_running = true;
}

void Relocalizer::setVIEstimator(VIEstimator* estimator_ptr)
{
    if (estimator_ptr)
    {
        vi_estimator_ptr = estimator_ptr;
    }
}

void Relocalizer::setVisualizer(Visualizer* vis_ptr)
{
    if (vis_ptr)
    {
        visualizer_ptr = vis_ptr;
        pose_graph->setVisualizer(visualizer_ptr);
    }
}

void Relocalizer::initLoopClosure()
{
    std::string vocabulary_file = "../../support_files/brief_k10L6.bin";
    std::cout << "vocabulary_file: " << vocabulary_file << std::endl;
    pose_graph->loadVocabulary(vocabulary_file);

    BRIEF_PATTERN_FILE = "../../support_files/brief_pattern.yml";
    std::cout << "BRIEF_PATTERN_FILE: " << BRIEF_PATTERN_FILE << std::endl;

    if (LOAD_PREVIOUS_POSE_GRAPH)
    {
        printf("load pose graph\n");
        assert(vi_estimator_ptr != nullptr);
        assert(visualizer_ptr != nullptr);
        m_process.lock();
        pose_graph->loadPoseGraph(vi_estimator_ptr);
        m_process.unlock();
        printf("load pose graph finish\n");
        load_flag = 1;
    }
    else
    {
        printf("no previous pose graph\n");
        load_flag = 1;
    }
}

void Relocalizer::readParameters(std::string file_path)
{

}

void Relocalizer::quit()
{
    is_running = false;
}

void Relocalizer::processVIO(const POSE_MSG &pose_msg)
{
    Vector3d vio_t(pose_msg.position_x, pose_msg.position_y, pose_msg.position_z);
    Quaterniond vio_q;
    vio_q.w() = pose_msg.orientation_w;
    vio_q.x() = pose_msg.orientation_x;
    vio_q.y() = pose_msg.orientation_y;
    vio_q.z() = pose_msg.orientation_z;

    vio_t = pose_graph->w_r_vio * vio_t + pose_graph->w_t_vio;
    vio_q = pose_graph->w_r_vio *  vio_q;

    vio_t = pose_graph->r_drift * vio_t + pose_graph->t_drift;
    vio_q = pose_graph->r_drift * vio_q;

    Vector3d vio_t_cam;
    Quaterniond vio_q_cam;
    vio_t_cam = vio_t + vio_q * reloc_tic;
    vio_q_cam = vio_q * reloc_qic;        

    // FIX ME
    // if (!VISUALIZE_IMU_FORWARD)
    // {
    //     cameraposevisual.reset();
    //     cameraposevisual.add_pose(vio_t_cam, vio_q_cam);
    //     cameraposevisual.publish_by(pub_camera_pose_visual, pose_msg->header);
    // }

    odometry_buf.push(vio_t_cam);
    if (odometry_buf.size() > 10)
    {
        odometry_buf.pop();
    }

    // FIX ME
    // for (unsigned int i = 0; i < odometry_buf.size(); i++)
    // {
    //     geometry_msgs::Point pose_marker;
    //     Vector3d vio_t;
    //     vio_t = odometry_buf.front();
    //     odometry_buf.pop();
    //     pose_marker.x = vio_t.x();
    //     pose_marker.y = vio_t.y();
    //     pose_marker.z = vio_t.z();
    //     key_odometrys.points.push_back(pose_marker);
    //     odometry_buf.push(vio_t);
    // }
    // pub_key_odometrys.publish(key_odometrys);

    if (!LOOP_CLOSURE)
    {
        POSE_MSG new_pose_msg;
        new_pose_msg.timestamp = pose_msg.timestamp;
        new_pose_msg.position_x = vio_t.x();
        new_pose_msg.position_y = vio_t.y();
        new_pose_msg.position_z = vio_t.z();
        
        assert(visualizer_ptr != nullptr);
        visualizer_ptr->updateNoLoopPath(new_pose_msg);
    }

}

void Relocalizer::prepareFeatures(const IMG_MSG &img_msg)
{
    if(!LOOP_CLOSURE)
        return;
    m_buf.lock();
    point_buf.push(img_msg);
    m_buf.unlock();

    // for (unsigned int i = 0; i < img_msg.features.size(); i++)
    // {
    //     printf("%d, 3D point: %lf, %lf, %lf 2D point %lf, %lf \n",
    //         i, 
    //         img_msg.features[i].undistorted_x,
    //         img_msg.features[i].undistorted_y,
    //         img_msg.features[i].undistorted_z,
    //         img_msg.features[i].velocity_x_of_point,
    //         img_msg.features[i].velocity_y_of_point
    //     );
    // }
}

void Relocalizer::prepareFrame(const FRAME_MSG &frame_msg)
{
    if (!LOOP_CLOSURE)
        return;

    m_buf.lock();
    image_buf.push(frame_msg);
    m_buf.unlock();
    printf("[Relocalizer] image time %lf w: %d h: %d\n", frame_msg.timestamp, 
                frame_msg.frame.cols, frame_msg.frame.rows);

    // detect unstable camera stream
    if (last_image_time < 0)
        last_image_time = frame_msg.timestamp;
    else if (frame_msg.timestamp - last_image_time > 1.0 || frame_msg.timestamp < last_image_time)
    {
        printf("[Relocalizer] image discontinue! detect a new sequence!\n");
        // FIX ME
        //new_sequence();
    }
    last_image_time = frame_msg.timestamp;
}

void Relocalizer::preparePose(const POSE_MSG &pose_msg)
{
    if (!LOOP_CLOSURE)
        return;

    m_buf.lock();
    pose_buf.push(pose_msg);
    m_buf.unlock();
}

void Relocalizer::updateExtrinsic(const POSE_MSG &pose_msg)
{
    m_process.lock();
    reloc_tic = Vector3d(pose_msg.position_x,
                   pose_msg.position_y,
                   pose_msg.position_z);
    reloc_qic = Quaterniond(pose_msg.orientation_w,
                      pose_msg.orientation_x,
                      pose_msg.orientation_y,
                      pose_msg.orientation_z).toRotationMatrix();
    m_process.unlock();
}

void Relocalizer::updateRelativePose(const POSE_MSG &pose_msg)
{
    Vector3d relative_t = Vector3d(pose_msg.position_x,
                                   pose_msg.position_y,
                                   pose_msg.position_z);
    Quaterniond relative_q;
    relative_q.w() = pose_msg.orientation_w;
    relative_q.x() = pose_msg.orientation_x;
    relative_q.y() = pose_msg.orientation_y;
    relative_q.z() = pose_msg.orientation_z;
    double relative_yaw = pose_msg.linear_x;
    int index = pose_msg.linear_y;
    printf("[Relocalizer] receive index %d \n", index );
    Eigen::Matrix<double, 8, 1 > loop_info;
    loop_info << relative_t.x(), relative_t.y(), relative_t.z(),
                 relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
                 relative_yaw;
    pose_graph->updateKeyFrameLoop(index, loop_info);
}

void Relocalizer::process()
{
    if (!LOOP_CLOSURE)
        return;

    printf("[Relocalizer] process() has started\n");
    while (is_running)
    {
        FRAME_MSG image_msg;
        IMG_MSG point_msg;
        POSE_MSG pose_msg;

        // find out the messages with same time stamp
        m_buf.lock();
        if(!image_buf.empty() && !point_buf.empty() && !pose_buf.empty())
        {
            if (image_buf.front().timestamp > pose_buf.front().timestamp)
            {
                pose_buf.pop();
                printf("throw pose at beginning\n");
            }
            else if (image_buf.front().timestamp > point_buf.front().timestamp)
            {
                point_buf.pop();
                printf("throw point at beginning\n");
            }
            else if (image_buf.back().timestamp >= pose_buf.front().timestamp 
                && point_buf.back().timestamp >= pose_buf.front().timestamp)
            {
                pose_msg = pose_buf.front();
                pose_buf.pop();
                while (!pose_buf.empty())
                    pose_buf.pop();
                while (image_buf.front().timestamp < pose_msg.timestamp)
                    image_buf.pop();
                image_msg = image_buf.front();
                image_buf.pop();

                while (point_buf.front().timestamp < pose_msg.timestamp)
                    point_buf.pop();
                point_msg = point_buf.front();
                point_buf.pop();
            }
        }
        m_buf.unlock();

        if (pose_msg.timestamp > 0 
            && point_msg.timestamp > 0
            && !image_msg.frame.empty())
        {
            // printf(" [Relocalizer] pose time %lf \n", pose_msg.timestamp);
            // printf(" [Relocalizer] point time %lf \n", point_msg.timestamp);
            // printf(" [Relocalizer] image time %lf \n", image_msg.timestamp);
            // skip fisrt few
            if (skip_first_cnt < SKIP_FIRST_CNT)
            {
                skip_first_cnt++;
                continue;
            }

            if (skip_cnt < SKIP_CNT)
            {
                skip_cnt++;
                continue;
            }
            else
            {
                skip_cnt = 0;
            }
            
            printf("[Relocalizer] image_msg time: %lf, w: %d, h: %d\n",
                    image_msg.timestamp, image_msg.frame.cols, image_msg.frame.rows);
            cv::Mat image = image_msg.frame;
            // build keyframe
            Vector3d T = Vector3d(pose_msg.position_x,
                                  pose_msg.position_y,
                                  pose_msg.position_z);
            Matrix3d R = Quaterniond(pose_msg.orientation_w,
                                     pose_msg.orientation_x,
                                     pose_msg.orientation_y,
                                     pose_msg.orientation_z).toRotationMatrix();
            if((T - last_t).norm() > SKIP_DIS)
            {
                std::vector<cv::Point3f> point_3d; 
                std::vector<cv::Point2f> point_2d_uv; 
                std::vector<cv::Point2f> point_2d_normal;
                std::vector<double> point_id;

                for (unsigned int i = 0; i < point_msg.features.size(); i++)
                {
                    cv::Point3f p_3d;
                    p_3d.x = point_msg.features[i].undistorted_x;
                    p_3d.y = point_msg.features[i].undistorted_y;
                    p_3d.z = point_msg.features[i].undistorted_z;
                    point_3d.push_back(p_3d);

                    cv::Point2f p_2d_uv, p_2d_normal;
                    double p_id;
                    // FIX ME
                    p_2d_normal.x = point_msg.features[i].velocity_x_of_point;
                    p_2d_normal.y = point_msg.features[i].velocity_y_of_point;
                    p_2d_uv.x = point_msg.features[i].u_of_point;
                    p_2d_uv.y = point_msg.features[i].v_of_point;
                    p_id = point_msg.features[i].id_of_point;
                    point_2d_normal.push_back(p_2d_normal);
                    point_2d_uv.push_back(p_2d_uv);
                    point_id.push_back(p_id);

                    //printf("u %lf, v %lf \n", p_2d_uv.x, p_2d_uv.y);
                }
                printf("[Relocalizer] build new keyframe\n");
                KeyFrame* keyframe = new KeyFrame(pose_msg.timestamp, frame_index, T, R, image,
                                   point_3d, point_2d_uv, point_2d_normal, point_id, sequence);   
                printf("[Relocalizer] new keyframe built\n");
                m_process.lock();
                start_flag = true;

                printf("[Relocalizer] add keyframe\n");             
                assert(vi_estimator_ptr != nullptr);
                assert(visualizer_ptr != nullptr);
                pose_graph->addKeyFrame(keyframe, 1, vi_estimator_ptr);
                printf("[Relocalizer] keyframe added\n");

                m_process.unlock();
                frame_index++;
                last_t = T;
            }
        }

        // FIX ME
        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}