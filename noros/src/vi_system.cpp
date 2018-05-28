#include "vi_system.h"
#include "tracker.h"
#include "vi_estimator.h"
#include "relocalizer.h"
#include "visualizer.h"

VISystem::VISystem()
{
    if (!tracker) 
    {
        tracker = new Tracker();
    }

    if (!vi_estimator)
    {
        vi_estimator = new VIEstimator();
    }

    if (!relocalizer)
    {
        relocalizer = new Relocalizer();
    }

    if (!visualizer)
    {
        visualizer = new Visualizer();
    }
}

VISystem::~VISystem()
{
    if (tracker) 
    {
        delete tracker;
    }

    if (vi_estimator)
    {
        delete vi_estimator;
    }

    if (relocalizer)
    {
        delete relocalizer;
    }

    if (visualizer)
    {
        delete visualizer;
    }

    tracker = nullptr;
    vi_estimator = nullptr;
    relocalizer = nullptr;
    visualizer = nullptr;
}

void VISystem::init()
{
    if (tracker && vi_estimator && relocalizer && readParameters("../config/euroc_config.yaml"))
    {
        tracker->init();
        vi_estimator->init();
        relocalizer->init();
        visualizer->init();

        tracker->setVIEstimator(vi_estimator);
        tracker->setRelocalizer(relocalizer);

        vi_estimator->setRelocalizer(relocalizer);
        vi_estimator->setVisualizer(visualizer);

        relocalizer->setVIEstimator(vi_estimator);
        relocalizer->setVisualizer(visualizer);

        if (LOOP_CLOSURE)
        {
            relocalizer->initLoopClosure();
        }

        estimator_thread = std::thread(&VIEstimator::fusion, vi_estimator);
        reloc_thread = std::thread(&Relocalizer::process, relocalizer);
    }
}
    
void VISystem::quit()
{
    visualizer->quit();
    relocalizer->quit();
    vi_estimator->quit();

    reloc_thread.join();
    printf("[VISystem] reloc thread stops\n");

    estimator_thread.join();
    printf("[VISystem] estimator thread stops\n");

    if (SAVE_POSE_GRAPH)
    {
        relocalizer->savePoseGraph();
    }
}

bool VISystem::readParameters(std::string config_file)
{
    cv::FileStorage vi_config(config_file, cv::FileStorage::READ);

    if (!vi_config.isOpened())
    {
        printf("[VISystem] failed to opened file at %s\n", config_file.c_str());
        return false;
    }

    // tracker params
    MAX_CNT = vi_config["max_cnt"];
    MIN_DIST = vi_config["min_dist"];
    ROW = vi_config["image_height"];
    COL = vi_config["image_width"];
    FREQ = vi_config["freq"];
    F_THRESHOLD = vi_config["F_threshold"];
    SHOW_TRACK = vi_config["show_track"];
    EQUALIZE = vi_config["equalize"];
    FISHEYE = vi_config["fisheye"];
    // FIX ME
    // if (FISHEYE == 1)
    //     FISHEYE_MASK = VINS_FOLDER_PATH + "config/fisheye_mask.jpg";

    CAM_NAMES.push_back(config_file);

    STEREO_TRACK = false;
    PUB_THIS_FRAME = false;

    if (FREQ == 0)
        FREQ = 100;

    // estimator params
    SOLVER_TIME = vi_config["max_solver_time"];
    NUM_ITERATIONS = vi_config["max_num_iterations"];
    MIN_PARALLAX = vi_config["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;
    ACC_N = vi_config["acc_n"];
    ACC_W = vi_config["acc_w"];
    GYR_N = vi_config["gyr_n"];
    GYR_W = vi_config["gyr_w"];
    G.z() = vi_config["g_norm"];

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    TD = vi_config["td"];
    ESTIMATE_TD = vi_config["estimate_td"];
    if (ESTIMATE_TD)
        printf("Unsynchronized sensors, online estimate time offset, initial td: %lf\n", TD);
    else
        printf("Synchronized sensors, fixed time offset: %lf\n", TD);

    ROLLING_SHUTTER = vi_config["rolling_shutter"];
    if (ROLLING_SHUTTER)
    {
        TR = vi_config["rolling_shutter_tr"];
        printf("rolling shutter camera, read out time per line: %lf\n", TR);
    }
    else
    {
        TR = 0.0;
    }

    // relocalizer params
    VISUALIZATION_SHIFT_X = vi_config["visualization_shift_x"];
    VISUALIZATION_SHIFT_Y = vi_config["visualization_shift_y"];
    SKIP_CNT = vi_config["skip_cnt"];
    SKIP_DIS = vi_config["skip_dis"]; 
    SKIP_FIRST_CNT = vi_config["skip_first_cnt"];
    LOOP_CLOSURE = vi_config["loop_closure"];

    if (LOOP_CLOSURE)
    {
        VISUALIZE_IMU_FORWARD = vi_config["visualize_imu_forward"];
        LOAD_PREVIOUS_POSE_GRAPH = vi_config["load_previous_pose_graph"];
        FAST_RELOCALIZATION = vi_config["fast_relocalization"];
        m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(config_file.c_str());
    }

    SAVE_POSE_GRAPH = vi_config["save_pose_graph"];
    SAVE_NO_LOOP_PATH = vi_config["save_no_loop_path"];
    SAVE_LOOP_PATH = vi_config["save_loop_path"];
    DEBUG_IMAGE = vi_config["save_image"];
    vi_config["pose_graph_save_path"] >> POSE_GRAPH_SAVE_PATH;
    vi_config["output_path"] >> VINS_RESULT_PATH;
    VINS_RESULT_NO_LOOP_PATH = VINS_RESULT_PATH + "/vins_result_no_loop.csv";
    VINS_RESULT_LOOP_PATH = VINS_RESULT_PATH + "/vins_result_loop.csv";

    std::ofstream fout(VINS_RESULT_NO_LOOP_PATH, std::ios::out);
    fout.close();

    fout = std::ofstream(VINS_RESULT_LOOP_PATH, std::ios::out);
    fout.close();

    ESTIMATE_EXTRINSIC = vi_config["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)
    {
        printf("have no prior about extrinsic param, calibrate extrinsic param\n");
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        EX_CALIB_RESULT_PATH = VINS_RESULT_PATH + "/extrinsic_parameter.csv"; // FIX ME
    }
    else 
    {
        if (ESTIMATE_EXTRINSIC == 1)
        {
            printf("Optimize extrinsic param around initial guess!\n");
            EX_CALIB_RESULT_PATH = VINS_RESULT_PATH + "/extrinsic_parameter.csv";
        }
        if (ESTIMATE_EXTRINSIC == 0)
            printf("fixed extrinsic param \n");

        cv::Mat cv_R, cv_T;
        vi_config["extrinsicRotation"] >> cv_R;
        vi_config["extrinsicTranslation"] >> cv_T;
        Eigen::Matrix3d eigen_R;
        Eigen::Vector3d eigen_T;
        cv::cv2eigen(cv_R, eigen_R);
        cv::cv2eigen(cv_T, eigen_T);
        Eigen::Quaterniond Q(eigen_R);
        eigen_R = Q.normalized();
        RIC.push_back(eigen_R);
        TIC.push_back(eigen_T);
        std::cout << "Extrinsic_R : " << std::endl << RIC[0] << std::endl;
        std::cout << "Extrinsic_T : " << std::endl << TIC[0].transpose() << std::endl;
        
    } 

    vi_config.release();

    return true;
}

void VISystem::processFrame(double img_timestamp, cv::Mat &input_frame)
{
    if (tracker)
    {
        tracker->processFrame(img_timestamp, input_frame);
    }
}

void VISystem::processImu(const IMU_MSG &imu_msg)
{
    if (vi_estimator)
    {
        vi_estimator->prepareImu(imu_msg);
    }
}
