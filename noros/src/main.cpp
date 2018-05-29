#include <iostream>
#include <string>
#include <thread>
#include <fstream>
#include <sstream>
#include <math.h>
#include <vector>

#include <opencv2/opencv.hpp>
#include "vi_system.h"

using namespace std;
using namespace cv;

void readEurocImuData(const std::string &imu_path);

void readEurocTimestamp(const std::string &data_path);

void readGroundtruth(const std::string &groundtruth_path);

void imuLoop();

VISystem* vi_system = nullptr;
std::thread read_img_thread;
std::thread read_imu_thread;

std::vector< std::vector<double> > imu_storage;
std::vector<long> euroc_timestamp_storage;

int main(int argc, char** argv)
{
    if (argc != 2) 
    {
        std::cerr << "Usage: ./vi_executor config_file" << std::endl;
        return 0;
    }

    int dataset_num;
    std::cout << "How many sequences to be processed? ";
    std::cin >> dataset_num;

    std::vector<std::string> datasets(dataset_num);

    int idx = 0;
    while (idx < dataset_num)
    {
        printf("Enter dataset %d path\n", idx + 1);
        std::cin >> datasets[idx];
        idx++;
    }

    idx = 0;

    while (idx < dataset_num)
    {
        const std::string dataset_path = datasets[idx];

        const std::string config_path = "";

        const std::string euroc_dataset_path = dataset_path + "/cam0/data.csv";
        const std::string euroc_imu_path = dataset_path + "/imu0/data.csv";

        const std::string euroc_groundtruth_path = dataset_path + "/state_groundtruth_estimate0/data.csv";

        imu_storage.clear();

        euroc_timestamp_storage.clear();

        readEurocImuData(euroc_imu_path);

        readEurocTimestamp(euroc_dataset_path);

        readGroundtruth(euroc_groundtruth_path);

        if (!vi_system)
        {
            vi_system = new VISystem();

            vi_system->init(argv[1]);

        }

        read_imu_thread = std::thread(imuLoop);

        for (size_t i = 0; i < euroc_timestamp_storage.size(); ++i)
        {
            const std::string img_path(dataset_path + "/cam0/data/" + std::to_string(euroc_timestamp_storage[i]) + ".png");
            
            //std::cout << "Image path: " << img_path << "\n";
            
            cv::Mat input_frame = cv::imread(img_path, CV_LOAD_IMAGE_GRAYSCALE);

            if (input_frame.cols == 0 || input_frame.rows == 0) continue;
            
            //std::cout << "Image w: " << input_frame.cols << " h: " << input_frame.rows << "\n";

            vi_system->processFrame(euroc_timestamp_storage[i] / pow(10, 9), input_frame);

            cv::imshow("frame", input_frame);

            int key = cv::waitKey(1);

            if (key == 'q')
                break;
        }

        read_imu_thread.join();

        printf("[main] read_imu_thread stops\n");

        idx++;

    }

    int key = cv::waitKey(0);
    if (key == 'q') 
    {
        cv::destroyWindow("frame");
    }

    vi_system->quit();

    return 0;
}

void readEurocTimestamp(const std::string &file_path) 
{

    ifstream in_stream(file_path, ios::in);

    std::string curr_line;

    if (in_stream.is_open()) {

        getline(in_stream, curr_line);
        
        while (getline(in_stream, curr_line)) {

            //std::cout << curr_line << "\n";

            istringstream iss(curr_line);

            long timestamp_long;

            while (iss >> timestamp_long) {

                //std::cout << "Timestamp is: " << timestamp_long << "\n";

                euroc_timestamp_storage.push_back(timestamp_long);

                if (iss.peek() == ',') {
                    break;
                }

            }
        }

        in_stream.close();
    }

    printf("readEurocTimestamp found %lu images\n", euroc_timestamp_storage.size());

}

void readEurocImuData(const std::string &file_path) 
{

    ifstream in_stream(file_path, ios::in);

    std::string curr_line;

    if (in_stream.is_open()) {

        getline(in_stream, curr_line);

        std::vector<double> imu_data(7);

        while (getline(in_stream, curr_line)) {

            imu_data.clear();
            
            istringstream iss(curr_line);

            double data;

            while (iss >> data) {

                imu_data.push_back(data);

                if (iss.peek() == ',') {
                    iss.ignore();
                }

            }

            // printf("%f %f %f %f %f %f %f\n", imu_data[0], imu_data[1], imu_data[2],
            //                     imu_data[3], imu_data[4], imu_data[5], imu_data[6]);

            imu_storage.push_back(imu_data);

        }

        in_stream.close();
    }

    printf("readEurocImuData found %lu imu data\n", imu_storage.size());

}

void readGroundtruth(const std::string &groundtruth_path) 
{
    std::cout << "load ground truth " << groundtruth_path << std::endl;

    FILE *f = fopen(groundtruth_path.c_str(), "r");
    if (f == NULL)
    {
      printf("can't load ground truth; wrong path\n");
      //std::cerr << "can't load ground truth; wrong path " << csv_file << std::endl;
      return;
    }
    char tmp[10000];
    if (fgets(tmp, 10000, f) == NULL)
    {
        printf("can't load ground truth; no data available");
    }
    while (!feof(f))
        benchmark.emplace_back(f);
    fclose(f);
    benchmark.pop_back();
    printf("Data loaded: %d", (int)benchmark.size());
}

void imuLoop()
{
    for (size_t i = 0; i < imu_storage.size(); ++i)
    {
        IMU_MSG imu_msg = {
            imu_storage[i][0] / pow(10, 9),
            {imu_storage[i][4], imu_storage[i][5], imu_storage[i][6]},
            {imu_storage[i][1], imu_storage[i][2], imu_storage[i][3]},
        };

        assert(vi_system != nullptr);

        vi_system->processImu(imu_msg);

        // printf("%lf %lf %lf %lf %lf %lf %lf\n", 
        //     imu_msg.timestamp, 
        //     imu_msg.linear_acceleration.linear_acceleration_x,
        //     imu_msg.linear_acceleration.linear_acceleration_y,
        //     imu_msg.linear_acceleration.linear_acceleration_z,
        //     imu_msg.angular_velocity.angular_velocity_x,
        //     imu_msg.angular_velocity.angular_velocity_y,
        //     imu_msg.angular_velocity.angular_velocity_z
        // );

        std::this_thread::sleep_for(std::chrono::milliseconds(2));

    }
}
