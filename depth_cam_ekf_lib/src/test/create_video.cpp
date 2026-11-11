//
// Created by Hasan on 11/10/26.
//



#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <cstdlib>

int main(int argc, char* argv[])
{
    // RTSP URL
    // Example: rtsp://username:password@192.168.1.100:554/stream
    std::string rtsp_url =
        "rtsp://root:root@192.168.2.207:554/cam0_0";

    if (argc > 1)
        rtsp_url = argv[1];

    // Configure FFmpeg to use TCP transport
    setenv("OPENCV_FFMPEG_CAPTURE_OPTIONS",
           "rtsp_transport;tcp", 1);

    // Open RTSP stream
    cv::VideoCapture cap(rtsp_url);

    if (!cap.isOpened())
    {
        std::cerr << "Failed to open RTSP stream!"
                  << std::endl;
        return -1;
    }

    // Get stream properties
    int width = (int)cap.get(CV_CAP_PROP_FRAME_WIDTH);
    int height = (int)cap.get(CV_CAP_PROP_FRAME_HEIGHT);

    double fps = cap.get(CV_CAP_PROP_FPS);

    if (fps <= 0 || fps > 120)
        fps = 25.0;

    std::cout << "Resolution: "
              << width << "x" << height << std::endl;

    std::cout << "FPS: " << fps << std::endl;

    // Create AVI writer
    std::string output_file = "output.avi";

    int fourcc = CV_FOURCC('M', 'J', 'P', 'G');

    cv::VideoWriter writer(
        output_file,
        fourcc,
        fps,
        cv::Size(width, height),
        true
    );

    if (!writer.isOpened())
    {
        std::cerr << "Failed to create AVI file!"
                  << std::endl;

        cap.release();
        return -1;
    }

    std::cout << "Recording started. Press ESC to stop."
              << std::endl;

    cv::Mat frame;

    while (true)
    {
        // Capture RTSP frame
        if (!cap.read(frame))
        {
            std::cerr << "Failed to receive frame!"
                      << std::endl;
            break;
        }

        if (frame.empty())
            continue;

        // Write frame to AVI
        writer.write(frame);

        // Display frame (optional)
        cv::imshow("RTSP Recording", frame);

        if (cv::waitKey(1) == 27)
            break;
    }

    // Release resources
    writer.release();
    cap.release();

    cv::destroyAllWindows();

    std::cout << "Recording saved: "
              << output_file << std::endl;

    return 0;
}
