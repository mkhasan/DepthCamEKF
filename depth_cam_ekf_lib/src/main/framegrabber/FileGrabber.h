//
// Created by mecasys on 11/10/26.
//

#ifndef DEPTHCAMEKF_FILEGRABBER_H
#define DEPTHCAMEKF_FILEGRABBER_H

#include "FrameGrabber.h"

#include <boost/filesystem.hpp>
#include <boost/thread.hpp>

#include <opencv2/opencv.hpp>
namespace depth_cam_ekf {
    class FileGrabber {
    public:
        FileGrabber();
        ~FileGrabber();

        void Init(const std::string &path, FrameGrabber *frame_grabber);
        void operator()();

    private:
        void ProcessFiles(const boost::filesystem::path &directory);
        cv::Mat GetImageFile(const std::string &file_full_path);

        std::vector<std::pair<std::string, std::string>>  files_vec_;
        boost::thread   fg_thread_;

        bool            initialised_;
        int             frame_id_;
        FrameGrabber    *frame_grabber_;
    };

}


#endif //DEPTHCAMEKF_FILEGRABBER_H