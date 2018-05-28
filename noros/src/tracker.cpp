#include "tracker.h"
#include "vi_estimator.h"
#include "relocalizer.h"

Tracker::Tracker()
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        if (!trackerData[i])
        {
            trackerData[i] = new FeatureTracker();
        }
    }

    printf("[Tracker] init done.\n");
}
  
Tracker::~Tracker()
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        if (trackerData[i])
        {
            delete trackerData[i];
        }
        trackerData[i] = nullptr;
    }
}

void Tracker::init()
{
    // FIX ME: read parameters first

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        trackerData[i]->readIntrinsicParameter(CAM_NAMES[i]);
    }

    // FIX ME
    // if(FISHEYE)
    // {
    //     for (int i = 0; i < NUM_OF_CAM; i++)
    //     {
    //         trackerData[i]->fisheye_mask = cv::imread(FISHEYE_MASK, 0);
    //         if(!trackerData[i]->fisheye_mask.data)
    //         {
    //             printf("load mask fail\n");
    //         }
    //         else
    //         {
    //             printf("load mask success\n");
    //         }
    //     }
    // }
}

void Tracker::setVIEstimator(VIEstimator* estimator_ptr)
{
    if (estimator_ptr)
    {
        vi_estimator_ptr = estimator_ptr;
    }
}

void Tracker::setRelocalizer(Relocalizer* relocalizer)
{
    if (relocalizer)
    {
        relocalizer_ptr = relocalizer;
    }
}
  
void Tracker::processFrame(double img_timestamp, cv::Mat &input_frame)
{
    if (first_image_flag)
    {
        first_image_flag = false;
        first_image_time = img_timestamp;
        last_image_time = img_timestamp;
        return;
    }

    // detect unstable camera stream 
    if (img_timestamp - last_image_time > 1.0 || img_timestamp < last_image_time)
    {
        printf("unstable image stream. reset tracker\n");
        first_image_flag = true;
        first_image_time = 0.0;
        last_image_time = 0.0;
        pub_count = 1;
        // FIX ME: 
        // publish restart flag
        // 
        return;
    }
    last_image_time = img_timestamp;
    
    // frequency control
    if (round(1.0 * pub_count / (img_timestamp - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // reset frequency control
        if (abs(1.0 * pub_count / (img_timestamp - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = img_timestamp;
            pub_count = 0;
        }
    }
    else 
    {
        PUB_THIS_FRAME = false;
    }

    TicToc t_r;

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        trackerData[i]->readImage(input_frame, img_timestamp);
    }

#ifdef SHOW_UNDISTORTION
    trackerData[i]->showUndistortion("undistrotion_" + std::to_string(i));
#endif

    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        for (int j = 0; j < NUM_OF_CAM; j++)
            if (j != 1 || !STEREO_TRACK)
                completed |= trackerData[j]->updateID(i);
        if (!completed)
            break;
    }

    if (PUB_THIS_FRAME)
    {
        pub_count++;
        publishFrame(img_timestamp, input_frame);
        visualizeTracking(input_frame);
    }

    printf("whole feature tracker processing costs: %lf\n", t_r.toc());
}

void Tracker::publishFrame(double img_timestamp, cv::Mat &input_frame)
{
    IMG_MSG img_msg;
    img_msg.timestamp = img_timestamp;
    
    std::vector<set<int> > hash_ids(NUM_OF_CAM);

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        auto &un_pts = trackerData[i]->cur_un_pts;
        auto &cur_pts = trackerData[i]->cur_pts;
        auto &ids = trackerData[i]->ids;
        auto &pts_velocity = trackerData[i]->pts_velocity;
        for (unsigned int j = 0; j < ids.size(); j++)
        {
            if (trackerData[i]->track_cnt[j] > 1)
            {
                int p_id = ids[j];
                hash_ids[i].insert(p_id);

                FEATURE_POINT feature;
                feature.id_of_point = p_id * NUM_OF_CAM + i;
                feature.u_of_point = cur_pts[j].x;
                feature.v_of_point = cur_pts[j].y;
                feature.velocity_x_of_point = pts_velocity[j].x;
                feature.velocity_y_of_point = pts_velocity[j].y;
                feature.undistorted_x = un_pts[j].x;
                feature.undistorted_y = un_pts[j].y;
                feature.undistorted_z = 1;

                img_msg.features.push_back(feature);
            }
        }
    }

    // skip the first image; since no optical speed on frist image
    if (!pub_init)
    {
        pub_init = true;
    }
    else 
    {
        // FIX ME: publish img_msg
        assert(vi_estimator_ptr != nullptr);
        vi_estimator_ptr->prepareFeatures(img_msg);

        assert(relocalizer_ptr != nullptr);
        FRAME_MSG frame_msg;
        cv::Mat reloc_frame = input_frame.clone();
        frame_msg.timestamp = img_timestamp;
        frame_msg.frame = reloc_frame;
        relocalizer_ptr->prepareFrame(frame_msg);
    }

    // FIX ME: publish match
    // WINDOW_SIZE = 20
    // publish frame to relocalizaiton here

}

void Tracker::visualizeTracking(cv::Mat &input_frame)
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        if (input_frame.channels() == 1) 
        {
            cv::cvtColor(input_frame, input_frame, CV_GRAY2RGBA);
        }

        for (unsigned int j = 0; j < trackerData[i]->cur_pts.size(); j++)
        {
            double len = std::min(1.0, 1.0 * trackerData[i]->track_cnt[j] / 20);
            cv::circle(input_frame, trackerData[i]->cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
            
            //draw speed line
            Vector2d tmp_cur_un_pts (trackerData[i]->cur_un_pts[j].x, trackerData[i]->cur_un_pts[j].y);
            Vector2d tmp_pts_velocity (trackerData[i]->pts_velocity[j].x, trackerData[i]->pts_velocity[j].y);
            Vector3d tmp_prev_un_pts;
            tmp_prev_un_pts.head(2) = tmp_cur_un_pts - 0.10 * tmp_pts_velocity;
            tmp_prev_un_pts.z() = 1;
            Vector2d tmp_prev_uv;
            trackerData[i]->m_camera->spaceToPlane(tmp_prev_un_pts, tmp_prev_uv);
            cv::line(input_frame, trackerData[i]->cur_pts[j], cv::Point2f(tmp_prev_uv.x(), tmp_prev_uv.y()), cv::Scalar(255 , 0, 0), 1 , 8, 0);
            
            char name[10];
            sprintf(name, "%d", trackerData[i]->ids[j]);
            cv::putText(input_frame, name, trackerData[i]->cur_pts[j], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
        }
    }
}