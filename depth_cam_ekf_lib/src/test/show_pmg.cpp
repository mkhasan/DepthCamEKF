//
// Created by Hasan on 11/10/26.
//

#include <opencv2/opencv.hpp>
#include <iostream>

int main()
{
    cv::Mat image = cv::imread("/home/mecasys/extension/MyProjects/SLAM/temp/realsense/frame_000124.pgm", cv::IMREAD_COLOR);
    //cv::Mat image = cv::imread("/home/mecasys/extension/MyProjects/SLAM/temp/frames/frame_001485.pgm", cv::IMREAD_COLOR);

    if (image.empty())
    {
        std::cerr << "Failed to load PGM file" << std::endl;
        return -1;
    }

    std::cout << "Width: " << image.cols << std::endl;
    std::cout << "Height: " << image.rows << std::endl;
    std::cout << "Type: " << image.type() << std::endl;

    cv::imshow("PGM Image", image);
    cv::waitKey(0);

    return 0;
}