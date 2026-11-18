
#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        cout << "Usage: " << argv[0]
             << " <video_file> <output_directory>" << endl;
        return -1;
    }

    string videoFile = argv[1];

    string outputDir = argv[2];

    // Create output directory if it does not exist
    mkdir(outputDir.c_str(), 0755);

    // Open video file
    VideoCapture cap(videoFile);

    if (!cap.isOpened())
    {
        cerr << "Error: Cannot open video: "
             << videoFile << endl;
        return -1;
    }

    Mat frame;
    Mat grayFrame;

    int frameIndex = 0;

    while (true)
    {
        // Read next frame
        if (!cap.read(frame))
            break;

        // Convert to grayscale
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

    cap.release();

    cout << "Finished. Total frames saved: "
         << frameIndex << endl;

    return 0;
}
