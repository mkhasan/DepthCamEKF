//
// Created by Hasan on 11/10/26.
//

//
// Created by Hasan on 11/6/26.
//

#include <iomanip>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <sys/stat.h>

using namespace std;

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        cout << "Usage: " << argv[0]
             << " <output_directory>" << endl;
        return -1;
    }


    string outputDir = argv[1];

    // Create output directory if it does not exist
    mkdir(outputDir.c_str(), 0755);

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

    int frameIndex = 0;
    cv::Mat grayFrame;
    while (true)
    {
        rs2::frameset frames = pipe.wait_for_frames();

        rs2::video_frame color = frames.get_color_frame();

        cv::Mat frame(
            color.get_height(),
            color.get_width(),
            CV_8UC3,
            (void*)color.get_data(),
            cv::Mat::AUTO_STEP
        );

        // cv::imshow("RGB", image);
        //
        // if (cv::waitKey(1) == 27)
        //     break;

        cvtColor(frame, grayFrame, CV_BGR2GRAY);

        // Generate output filename
        stringstream filename;
        filename << outputDir << "/frame_"
                 << setfill('0') << setw(6)
                 << frameIndex << ".pgm";

        // Save grayscale frame as PGM
        if (!imwrite(filename.str(), grayFrame))
        {
            cerr << "Error saving: "
                 << filename.str() << endl;
            break;
        }

        cout << "Saved: " << filename.str() << endl;

        frameIndex++;
    }

    pipe.stop();
    return 0;
}