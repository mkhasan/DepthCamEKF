//
// Created by Hasan on 11/6/26.
//

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

int main()
{
    rs2::pipeline pipe;
    rs2::config cfg;

    // Enable only color stream
    cfg.enable_stream(
        RS2_STREAM_COLOR,
        640, 480,
        RS2_FORMAT_BGR8,
        30
    );

    pipe.start(cfg);

    while (true)
    {
        rs2::frameset frames = pipe.wait_for_frames();

        rs2::video_frame color = frames.get_color_frame();

        cv::Mat image(
            color.get_height(),
            color.get_width(),
            CV_8UC3,
            (void*)color.get_data(),
            cv::Mat::AUTO_STEP
        );

        std::cout << "Width: " << image.cols << std::endl;
        std::cout << "Height: " << image.rows << std::endl;
        std::cout << "Type: " << image.type() << std::endl;
        cv::imshow("RGB", image);

        if (cv::waitKey(1) == 27)
            break;
    }

    pipe.stop();
    return 0;
}