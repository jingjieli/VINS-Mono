#include "vi_estimator.h"
#include "relocalizer.h"
#include "visualizer.h"

VIEstimator::VIEstimator()
{
    if (!estimator)
    {
        estimator = new Estimator();
    }

    //measurement_process = std::thread(fusion);
    printf("[VIEstimator] init done.\n");
}
    
VIEstimator::~VIEstimator()
{
    if (estimator)
    {
        delete estimator;
    }
    estimator = nullptr;
}    

void VIEstimator::init()
{
    // FIX ME: read parameters first
    if (estimator)
    {
        estimator->setParameter();

        is_running = true;
    }
}

void VIEstimator::setRelocalizer(Relocalizer* reloc_ptr)
{
    if (reloc_ptr)
    {
        relocalizer_ptr = reloc_ptr;
    }
}

void VIEstimator::setVisualizer(Visualizer* vis_ptr)
{   
    if (vis_ptr)
    {
        visualizer_ptr = vis_ptr;
    }
}

void VIEstimator::quit()
{
    is_running = false;
    con.notify_one();
}

void VIEstimator::prepareFeatures(const IMG_MSG &img_msg)
{
    if (!feature_init)
    {
        //skip the first detected feature, which doesn't contain optical flow speed
        feature_init = true;
        return;
    }
    m_buf.lock();
    feature_buf.push(img_msg);
    m_buf.unlock();
    con.notify_one();
}
    
void VIEstimator::prepareImu(const IMU_MSG &imu_msg)
{
    if (imu_msg.timestamp <= last_imu_time)
    {
        printf("imu in wrong order\n");
        return;
    }
    last_imu_time = imu_msg.timestamp;

    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
    con.notify_one();

    std::lock_guard<std::mutex> lg(m_state);
    predict(imu_msg);

    if (estimator->solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        // FIX ME: this is ROS related
        pubLatestOdometry(tmp_P, tmp_Q, tmp_V, imu_msg.timestamp);
    }
}

void VIEstimator::prepareRolocalizationResult(const RELOC_MSG &reloc_msg)
{
    m_buf.lock();
    reloc_buf.push(reloc_msg);
    m_buf.unlock();
}

void VIEstimator::fusion()
{
    printf("[VIEstimator] fusion started.\n");
    while (is_running)
    {
        std::vector<std::pair<std::vector<IMU_MSG>, IMG_MSG> > measurements;
        std::unique_lock<std::mutex> lk(m_buf);
        printf("checkpoint 0 ......\n");
        con.wait(lk, [&]
                 {
            return (measurements = getMeasurements()).size() != 0 || !is_running;
                 });
        lk.unlock();

        printf("checkpoint 1 ......\n");

        m_estimator.lock();
        for (auto &measurement : measurements)
        {
            auto img_msg = measurement.second;
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
            for (auto &imu_msg : measurement.first)
            {
                double t = imu_msg.timestamp;
                double img_t = img_msg.timestamp + estimator->td;
                if (t <= img_t)
                { 
                    if (current_time < 0)
                        current_time = t;
                    double dt = t - current_time;
                    assert(dt >= 0);
                    current_time = t;
                    dx = imu_msg.linear_acceleration.linear_acceleration_x;
                    dy = imu_msg.linear_acceleration.linear_acceleration_y;
                    dz = imu_msg.linear_acceleration.linear_acceleration_z;
                    rx = imu_msg.angular_velocity.angular_velocity_x;
                    ry = imu_msg.angular_velocity.angular_velocity_y;
                    rz = imu_msg.angular_velocity.angular_velocity_z;
                    estimator->processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("imu: dt:%lf a: %lf %lf %lf w: %lf %lf %lf\n", dt, dx, dy, dz, rx, ry, rz);
                }
                else
                {
                    double dt_1 = img_t - current_time;
                    double dt_2 = t - img_t;
                    current_time = img_t;
                    assert(dt_1 >= 0);
                    assert(dt_2 >= 0);
                    assert(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    dx = w1 * dx + w2 * imu_msg.linear_acceleration.linear_acceleration_x;
                    dy = w1 * dy + w2 * imu_msg.linear_acceleration.linear_acceleration_y;
                    dz = w1 * dz + w2 * imu_msg.linear_acceleration.linear_acceleration_z;
                    rx = w1 * rx + w2 * imu_msg.angular_velocity.angular_velocity_x;
                    ry = w1 * ry + w2 * imu_msg.angular_velocity.angular_velocity_y;
                    rz = w1 * rz + w2 * imu_msg.angular_velocity.angular_velocity_z;
                    estimator->processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("dimu: dt:%lf a: %lf %lf %lf w: %lf %lf %lf\n", dt_1, dx, dy, dz, rx, ry, rz);
                }
            }

            // FIX ME: RELO && forget mutex?
            // set relocalization frame
            RELOC_MSG reloc_msg;
            while (!reloc_buf.empty())
            {
                reloc_msg = reloc_buf.front();
                reloc_buf.pop();
            }

            //printf("[VIEstimator] reloc timestamp: %lf\n", reloc_msg.timestamp);
            
            if (reloc_msg.timestamp > 0
                && reloc_msg.matched_points.size() > 0) // msg not null
            {
                //reloc_msg.printMessage();
                vector<Vector3d> match_points;
                double frame_stamp = reloc_msg.timestamp;
                for (unsigned int i = 0; i < reloc_msg.matched_points.size(); i++)
                {
                    Vector3d u_v_id;
                    u_v_id.x() = reloc_msg.matched_points[i].matched_x;
                    u_v_id.y() = reloc_msg.matched_points[i].matched_y;
                    u_v_id.z() = reloc_msg.matched_points[i].matched_z;

                    match_points.push_back(u_v_id);
                }
                Vector3d relo_t(reloc_msg.transform[0], reloc_msg.transform[1], reloc_msg.transform[2]);
                Quaterniond relo_q(reloc_msg.transform[3], reloc_msg.transform[4], reloc_msg.transform[5], reloc_msg.transform[6]);
                Matrix3d relo_r = relo_q.toRotationMatrix();
                int frame_index;
                frame_index = reloc_msg.idx;
                printf("[VIEstimator] setReloFrame\n");
                estimator->setReloFrame(frame_stamp, frame_index, match_points, relo_t, relo_r);
                printf("[VIEstimator] setReloFrame with timestamp: %lf\n", frame_stamp);

                pubRelocalization();
                printf("[VIEstimator] pubRelocalization done\n");
            }

            printf("[VIEstimator] processing vision data with stamp %lf\n", img_msg.timestamp);

            TicToc t_s;
            std::map<int, std::vector<pair<int, Eigen::Matrix<double, 7, 1> > > > image;
            for (unsigned int i = 0; i < img_msg.features.size(); i++)
            {
                int v = img_msg.features[i].id_of_point + 0.5f;
                int feature_id = v / NUM_OF_CAM;
                int camera_id = v % NUM_OF_CAM;
                double x = img_msg.features[i].undistorted_x;
                double y = img_msg.features[i].undistorted_y;
                double z = img_msg.features[i].undistorted_z;
                double p_u = img_msg.features[i].u_of_point;
                double p_v = img_msg.features[i].v_of_point;
                double velocity_x = img_msg.features[i].velocity_x_of_point;
                double velocity_y = img_msg.features[i].velocity_y_of_point;
                assert(z == 1);
                Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                image[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
            }
            estimator->processImage(image, img_msg.timestamp);
            
            // FIX ME: this is ROS related
            double whole_t = t_s.toc();
            printStatistics(whole_t);
            // std_msgs::Header header = img_msg->header;
            // header.frame_id = "world";

            pubOdometry(img_msg.timestamp);
            printf("[VIEstimator] pubOdometry done\n");
            // pubKeyPoses(estimator, header);
            pubCameraPose(img_msg.timestamp);
            pubPointCloud();
            pubTF();
            pubKeyframe();
            printf("[VIEstimator] pubKeyframe done\n");
        }
        m_estimator.unlock();

        m_buf.lock();
        m_state.lock();
        if (estimator->solver_flag == Estimator::SolverFlag::NON_LINEAR)
            update();
        m_state.unlock();
        m_buf.unlock();
    }
}

void VIEstimator::predict(const IMU_MSG &imu_msg)
{
    double t = imu_msg.timestamp;
    if (imu_init)
    {
        latest_time = t;
        imu_init = false;
        return;
    }
    double dt = t - latest_time;
    latest_time = t;

    double dx = imu_msg.linear_acceleration.linear_acceleration_x;
    double dy = imu_msg.linear_acceleration.linear_acceleration_y;
    double dz = imu_msg.linear_acceleration.linear_acceleration_z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};

    double rx = imu_msg.angular_velocity.angular_velocity_x;
    double ry = imu_msg.angular_velocity.angular_velocity_y;
    double rz = imu_msg.angular_velocity.angular_velocity_z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};

    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - estimator->g;

    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - estimator->g;

    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void VIEstimator::update()
{
    latest_time = current_time;
    tmp_P = estimator->Ps[WINDOW_SIZE];
    tmp_Q = estimator->Rs[WINDOW_SIZE];
    tmp_V = estimator->Vs[WINDOW_SIZE];
    tmp_Ba = estimator->Bas[WINDOW_SIZE];
    tmp_Bg = estimator->Bgs[WINDOW_SIZE];
    acc_0 = estimator->acc_0;
    gyr_0 = estimator->gyr_0;

    queue<IMU_MSG> tmp_imu_buf = imu_buf;
    while (!tmp_imu_buf.empty())
    {
        predict(tmp_imu_buf.front());
        tmp_imu_buf.pop();
    }
}

std::vector<std::pair<std::vector<IMU_MSG>, IMG_MSG> > VIEstimator::getMeasurements()
{
    std::vector<std::pair<std::vector<IMU_MSG>, IMG_MSG> > measurements;

    while (true)
    {
        if (imu_buf.empty() || feature_buf.empty())
            return measurements;

        if (!(imu_buf.back().timestamp > feature_buf.front().timestamp + estimator->td))
        {
            printf("[VIEstimator] wait for imu, only should happen at the beginning\n");
            sum_of_wait++;
            return measurements;
        }

        if (!(imu_buf.front().timestamp < feature_buf.front().timestamp + estimator->td))
        {
            printf("[VIEstimator] throw img, only should happen at the beginning\n");
            feature_buf.pop();
            continue;
        }
        IMG_MSG img_msg = feature_buf.front();
        feature_buf.pop();

        std::vector<IMU_MSG> IMUs;
        while (imu_buf.front().timestamp < img_msg.timestamp + estimator->td)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        IMUs.emplace_back(imu_buf.front());
        if (IMUs.empty())
            printf("[VIEstimator] no imu between two image\n");
        measurements.emplace_back(IMUs, img_msg);
    }
    return measurements;
}

void VIEstimator::printStatistics(double t)
{
    if (estimator->solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;
    printf("position: %lf, %lf, %lf\n", estimator->Ps[WINDOW_SIZE].x(), estimator->Ps[WINDOW_SIZE].y(), estimator->Ps[WINDOW_SIZE].z());
    std::cout << "position: " << estimator->Ps[WINDOW_SIZE].transpose() << std::endl;
    std::cout << "orientation: " << estimator->Vs[WINDOW_SIZE].transpose() << std::endl;
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        //printf("calibration result for camera %d", i);
        std::cout << "extirnsic tic: " << estimator->tic[i].transpose() << std::endl;
        std::cout << "extrinsic ric: " << Utility::R2ypr(estimator->ric[i]).transpose() << std::endl;
        // FIX ME
        // if (ESTIMATE_EXTRINSIC)
        // {
        //     cv::FileStorage fs(EX_CALIB_RESULT_PATH, cv::FileStorage::WRITE);
        //     Eigen::Matrix3d eigen_R;
        //     Eigen::Vector3d eigen_T;
        //     eigen_R = estimator->ric[i];
        //     eigen_T = estimator->tic[i];
        //     cv::Mat cv_R, cv_T;
        //     cv::eigen2cv(eigen_R, cv_R);
        //     cv::eigen2cv(eigen_T, cv_T);
        //     fs << "extrinsicRotation" << cv_R << "extrinsicTranslation" << cv_T;
        //     fs.release();
        // }
    }

    sum_of_time += t;
    sum_of_calculation++;
    printf("vo solver costs: %lf ms\n", t);
    printf("average of time %lf ms\n", sum_of_time / sum_of_calculation);

    sum_of_path += (estimator->Ps[WINDOW_SIZE] - last_path).norm();
    last_path = estimator->Ps[WINDOW_SIZE];
    printf("sum of path %lf\n", sum_of_path);
    if (ESTIMATE_TD)
        printf("td %lf\n", estimator->td);
}

void VIEstimator::pubRelocalization()
{
    assert(relocalizer_ptr != nullptr);
    POSE_MSG pose_msg;

    pose_msg.timestamp = estimator->relo_frame_stamp;
    pose_msg.position_x = estimator->relo_relative_t.x();
    pose_msg.position_y = estimator->relo_relative_t.y();
    pose_msg.position_z = estimator->relo_relative_t.z();
    pose_msg.orientation_x = estimator->relo_relative_q.x();
    pose_msg.orientation_y = estimator->relo_relative_q.y();
    pose_msg.orientation_z = estimator->relo_relative_q.z();
    pose_msg.orientation_w = estimator->relo_relative_q.w();
    pose_msg.linear_x = estimator->relo_relative_yaw;
    pose_msg.linear_y = estimator->relo_frame_index;

    relocalizer_ptr->updateRelativePose(pose_msg);
}

void VIEstimator::pubOdometry(double timestamp)
{
    if (estimator->solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        POSE_MSG pose_msg;
        Quaterniond tmp_Q;
        tmp_Q = Quaterniond(estimator->Rs[WINDOW_SIZE]);
        pose_msg.position_x = estimator->Ps[WINDOW_SIZE].x();
        pose_msg.position_y = estimator->Ps[WINDOW_SIZE].y();
        pose_msg.position_z = estimator->Ps[WINDOW_SIZE].z();
        pose_msg.orientation_x = tmp_Q.x();
        pose_msg.orientation_y = tmp_Q.y();
        pose_msg.orientation_z = tmp_Q.z();
        pose_msg.orientation_w = tmp_Q.w();
        pose_msg.linear_x = estimator->Vs[WINDOW_SIZE].x();
        pose_msg.linear_y = estimator->Vs[WINDOW_SIZE].y();
        pose_msg.linear_z = estimator->Vs[WINDOW_SIZE].z();

        assert(relocalizer_ptr != nullptr);
        relocalizer_ptr->processVIO(pose_msg);

        assert(visualizer_ptr != nullptr);
        visualizer_ptr->updateTrajectory(pose_msg);

        // geometry_msgs::PoseStamped pose_stamped;
        // pose_stamped.header = header;
        // pose_stamped.header.frame_id = "world";
        // pose_stamped.pose = odometry.pose.pose;
        // path.header = header;
        // path.header.frame_id = "world";
        // path.poses.push_back(pose_stamped);
        // pub_path.publish(path);

        POSE_MSG new_pose_msg;
        Vector3d correct_t;
        Vector3d correct_v;
        Quaterniond correct_q;
        correct_t = estimator->drift_correct_r * estimator->Ps[WINDOW_SIZE] + estimator->drift_correct_t;
        correct_q = estimator->drift_correct_r * estimator->Rs[WINDOW_SIZE];
        new_pose_msg.timestamp = timestamp;
        new_pose_msg.position_x = correct_t.x();
        new_pose_msg.position_y = correct_t.y();
        new_pose_msg.position_z = correct_t.z();
        new_pose_msg.orientation_x = correct_q.x();
        new_pose_msg.orientation_y = correct_q.y();
        new_pose_msg.orientation_z = correct_q.z();
        new_pose_msg.orientation_w = correct_q.w();

        assert(visualizer_ptr != nullptr);
        visualizer_ptr->updateRelocPath(new_pose_msg);

        // write result to file
        ofstream foutC(VINS_RESULT_NO_LOOP_PATH, ios::app);
        foutC.setf(ios::fixed, ios::floatfield);
        foutC.precision(0);
        foutC << timestamp * 1e9 << ",";
        foutC.precision(5);
        foutC << estimator->Ps[WINDOW_SIZE].x() << ","
              << estimator->Ps[WINDOW_SIZE].y() << ","
              << estimator->Ps[WINDOW_SIZE].z() << ","
              << tmp_Q.w() << ","
              << tmp_Q.x() << ","
              << tmp_Q.y() << ","
              << tmp_Q.z() << ","
              << estimator->Vs[WINDOW_SIZE].x() << ","
              << estimator->Vs[WINDOW_SIZE].y() << ","
              << estimator->Vs[WINDOW_SIZE].z() << "," << endl;
        foutC.close();
    }
}

void VIEstimator::pubKeyframe()
{
    // pub camera pose, 2D-3D points of keyframe
    if (estimator->solver_flag == Estimator::SolverFlag::NON_LINEAR && estimator->marginalization_flag == 0)
    {
        int i = WINDOW_SIZE - 2;
        //Vector3d P = estimator->Ps[i] + estimator->Rs[i] * estimator->tic[0];
        Vector3d P = estimator->Ps[i];
        Quaterniond R = Quaterniond(estimator->Rs[i]);

        POSE_MSG pose_msg;
        pose_msg.timestamp = estimator->Headers[WINDOW_SIZE - 2];
        pose_msg.position_x = P.x();
        pose_msg.position_y = P.y();
        pose_msg.position_z = P.z();
        pose_msg.orientation_x = R.x();
        pose_msg.orientation_y = R.y();
        pose_msg.orientation_z = R.z();
        pose_msg.orientation_w = R.w();

        printf("[VIEstimator] time: %lf t: %lf %lf %lf r: %lf %lf %lf %lf\n", 
                pose_msg.timestamp, P.x(), P.y(), P.z(), R.w(), R.x(), R.y(), R.z());

        assert(relocalizer_ptr != nullptr);
        relocalizer_ptr->preparePose(pose_msg);
        printf("[VIEstimator] preparePose done\n");

        assert(visualizer_ptr != nullptr);
        visualizer_ptr->addKeyframePose(pose_msg);

        IMG_MSG img_msg;
        img_msg.timestamp = estimator->Headers[WINDOW_SIZE - 2];
        for (auto &it_per_id : estimator->f_manager.feature)
        {
            int frame_size = it_per_id.feature_per_frame.size();
            if(it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.start_frame + frame_size - 1 >= WINDOW_SIZE - 2 && it_per_id.solve_flag == 1)
            {

                int imu_i = it_per_id.start_frame;
                Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
                Vector3d w_pts_i = estimator->Rs[imu_i] * (estimator->ric[0] * pts_i + estimator->tic[0])
                                        + estimator->Ps[imu_i];

                FEATURE_POINT f_point;
                int imu_j = WINDOW_SIZE - 2 - it_per_id.start_frame;
                f_point.undistorted_x = w_pts_i(0);
                f_point.undistorted_y = w_pts_i(1);
                f_point.undistorted_z = w_pts_i(2);
                
                f_point.velocity_x_of_point = it_per_id.feature_per_frame[imu_j].point.x();
                f_point.velocity_y_of_point = it_per_id.feature_per_frame[imu_j].point.y();
                f_point.u_of_point = it_per_id.feature_per_frame[imu_j].uv.x();
                f_point.v_of_point = it_per_id.feature_per_frame[imu_j].uv.y();
                f_point.id_of_point = it_per_id.feature_id;

                img_msg.features.push_back(f_point);
            }

        }
        assert(relocalizer_ptr != nullptr);
        relocalizer_ptr->prepareFeatures(img_msg);
        printf("[VIEstimator] prepareFeatures done\n");
    }
}

void VIEstimator::pubPointCloud()
{
    assert(visualizer_ptr != nullptr);

    std::vector<Eigen::Vector3d> points;
    for (auto &it_per_id : estimator->f_manager.feature)
    {
        int used_num;
        used_num = it_per_id.feature_per_frame.size();
        if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        if (it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id.solve_flag != 1)
            continue;
        int imu_i = it_per_id.start_frame;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
        Vector3d w_pts_i = estimator->Rs[imu_i] * (estimator->ric[0] * pts_i + estimator->tic[0]) + estimator->Ps[imu_i];

        Eigen::Vector3d point(w_pts_i(0), w_pts_i(1), w_pts_i(2));

        points.push_back(point);
    }
    
    visualizer_ptr->updatePointCloud(points);
}

void VIEstimator::pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, 
                                const Eigen::Vector3d &V, const double timestamp)
{
    Eigen::Quaterniond quadrotor_Q = Q ;

    POSE_MSG pose_msg;
    pose_msg.timestamp = timestamp;
    pose_msg.position_x = P.x();
    pose_msg.position_y = P.y();
    pose_msg.position_z = P.z();
    pose_msg.orientation_x = quadrotor_Q.x();
    pose_msg.orientation_y = quadrotor_Q.y();
    pose_msg.orientation_z = quadrotor_Q.z();
    pose_msg.orientation_w = quadrotor_Q.w();
    pose_msg.linear_x = V.x();
    pose_msg.linear_y = V.y();
    pose_msg.linear_z = V.z();
    // FIX ME
    //pub_latest_odometry.publish(odometry);
}

void VIEstimator::pubCameraPose(double timestamp)
{
    assert(visualizer_ptr != nullptr);

    if (estimator->solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        int i = WINDOW_SIZE - 1;
        Vector3d P = estimator->Ps[i] + estimator->Rs[i] * estimator->tic[0];
        Quaterniond Q = Quaterniond(estimator->Rs[i] * estimator->ric[0]);
        Matrix3d R = Q.toRotationMatrix();

        // POSE_MSG pose_msg;

        // pose_msg.timestamp = timestamp;
        // pose_msg.position_x = P.x();
        // pose_msg.position_y = P.y();
        // pose_msg.position_z = P.z();
        // pose_msg.orientation_x = R.x();
        // pose_msg.orientation_y = R.y();
        // pose_msg.orientation_z = R.z();
        // pose_msg.orientation_w = R.w();

        // visualizer_ptr->updateCameraPose(pose_msg);
        visualizer_ptr->updateCameraPose(P, R);
    }
}

void VIEstimator::pubTF()
{
    if (estimator->solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;

    POSE_MSG pose_msg;
    pose_msg.position_x = estimator->tic[0].x();
    pose_msg.position_y = estimator->tic[0].y();
    pose_msg.position_z = estimator->tic[0].z();
    Quaterniond tmp_q{estimator->ric[0]};
    pose_msg.orientation_x = tmp_q.x();
    pose_msg.orientation_y = tmp_q.y();
    pose_msg.orientation_z = tmp_q.z();
    pose_msg.orientation_w = tmp_q.w();

    assert(relocalizer_ptr != nullptr);
    relocalizer_ptr->updateExtrinsic(pose_msg);
}
