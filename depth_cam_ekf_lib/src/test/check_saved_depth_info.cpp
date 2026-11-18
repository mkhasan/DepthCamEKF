//
// Created by Hasan on 11/15/26.
//

#include <fstream>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <boost/filesystem.hpp>
#include <unistd.h>




using namespace std;
using namespace cv;
namespace fs = boost::filesystem;

int read_values(const string& pathname, vector<float>& values);

const int HEIGHT = 480;
const int WIDTH = 640;
const float MAX_DEPTH = 5.0f;
unsigned char pixels[WIDTH * HEIGHT] = {0, };
int read_values(const string& pathname, vector<float>& values);
int get_image(const fs::path& pathname, Mat& img);
int get_depth_image(const string& pathname, Mat& img);
std::vector<std::string> get_sorted_filenames(const std::string& folder_path);

int main(int argc, char *argv[])
{

    //string path = argc > 1 ? argv[1] : "/home/mecasys/extension/MyProjects/SLAM/temp/realtime/depth_info/depth_000024.dp";

    string depth_info_folder_name = "/home/mecasys/extension/MyProjects/SLAM/temp/realtime/depth_info";
    if (argc > 1 && string(argv[1]) == "grayscale") {
        depth_info_folder_name = "/home/mecasys/extension/MyProjects/SLAM/temp/realtime/frames";
    }
    vector<string> filenames = get_sorted_filenames(depth_info_folder_name);

    if (filenames.size() == 0) {
        cerr << "No filenames provided" << endl;
        return -1;
    }





    const auto window_name = "Display Image";
    namedWindow(window_name, WINDOW_AUTOSIZE);
    int index = 0;
    while (cv::waitKey(1) < 0 && cv::getWindowProperty(window_name, cv::WND_PROP_AUTOSIZE) >= 0)
    {
        cv::Mat img;

        const auto& filename = filenames[index];
        fs::path fullPath = fs::path (depth_info_folder_name) / filename;

        if (get_image(fullPath, img)  == 0) {

            imshow(window_name, img);

        }
        // Update the window with new data

        index = (index + 1) % filenames.size();
    }







    //cv::Mat image = cv::imread("/home/mecasys/extension/MyProjects/SLAM/temp/realsense/frame_000124.pgm", cv::IMREAD_COLOR);
    // cv::Mat image = cv::imread(path, cv::IMREAD_COLOR);
    //
    // if (image.empty())
    // {
    //     std::cerr << "Failed to load PGM file" << std::endl;
    //     return -1;
    // }
    //
    // std::cout << "Width: " << image.cols << std::endl;
    // std::cout << "Height: " << image.rows << std::endl;
    // std::cout << "Type: " << image.type() << std::endl;
    //
    // cv::imshow("PGM Image", image);
    // cv::waitKey(0);
    //
    // if (read_values(path) != 0) {
    //     cerr << "Error reading from file " << path << endl;
    //     return 1;
    // }
    return 0;
}

int read_values(const string& pathname, vector<float>& values) {
    std::ifstream file(pathname);

    if (!file.is_open())
    {
        std::cerr << "Failed to open file!" << std::endl;
        return -1;
    }


    float value;

    while (file >> value)
    {
        values.push_back(value);
    }

    file.close();


    return 0;

}

int get_image(const fs::path& pathname, Mat& img) {
    string ext = pathname.extension().string();
    if (ext == ".dp") {
        img = Mat(HEIGHT, WIDTH, CV_8UC3);
        get_depth_image(pathname.string(), img);
        usleep(20000);
        return 0;
    }
    else {

        img = imread(pathname.string(), cv::IMREAD_COLOR);
        usleep(100000);
        return 0;
    }

    return -1;

}


int get_depth_image(const string& pathname, Mat& img) {


    vector<float> depth_values;

    if (read_values(pathname, depth_values) != 0) {
        cerr << "Error reading value" << endl;
        return 1;
    }

    if (depth_values.size() == WIDTH * HEIGHT) {
        float min = 1000.f;
        float max = -1.f;

        for (int i=0; i<img.rows; i++) {
            for (int j=0; j<img.cols; j++) {
                if (depth_values[i * WIDTH + j] > MAX_DEPTH) {
                    depth_values[i * WIDTH + j] = MAX_DEPTH;
                }
                int value = 255*depth_values[i * WIDTH + j] / MAX_DEPTH;
                if (depth_values[i * WIDTH + j] < 0.1) {
                    value = 0;
                }
                if (value > 255) {
                    value = 255;
                }
                if (value < 0) {
                    value = 0;
                }
                //pixels[i * WIDTH + j] = (unsigned char) (value & 0xFF);
                cv::Vec3b& pixel = img.at<cv::Vec3b>(i, j);

                pixel[2] = value;  // Red
                pixel[1] = pixel[0] = 0;
            }

        }

    }

    // Create grayscale image


    // Copy array data into Mat
    //memcpy(img.data, pixels, WIDTH * HEIGHT );
    return 0;
}

std::vector<std::string> get_sorted_filenames(const std::string& folder_path) {
    std::vector<std::string> filenames;
    fs::path dir(folder_path);

    if (fs::exists(dir) && fs::is_directory(dir)) {
        for (const auto& entry : fs::directory_iterator(dir)) {
            // Filter out subdirectories and special files
            if (fs::is_regular_file(entry.status())) {
                filenames.push_back(entry.path().filename().string());
            }
        }
        std::sort(filenames.begin(), filenames.end());
    }

    return filenames;
}