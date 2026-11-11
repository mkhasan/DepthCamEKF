//
// Created by Hasan on 11/10/26.
//

#include "FileGrabber.h"

using namespace std;
depth_cam_ekf::FileGrabber::FileGrabber()
{
    initialised_ = false;
    frame_id_ = 0;
}

depth_cam_ekf::FileGrabber::~FileGrabber()
{
    if(!files_vec_.empty()) {
        files_vec_.clear();
    }
}

void depth_cam_ekf::FileGrabber::Init(const string &path, FrameGrabber *frame_grabber)
{
    ProcessFiles(path);

    frame_grabber_  = frame_grabber;
    initialised_    = true;

    fg_thread_ = boost::thread(boost::ref(*this));
}

void depth_cam_ekf::FileGrabber::ProcessFiles(const boost::filesystem::path &directory)
{
    if (exists(directory)) {
        boost::filesystem::directory_iterator end;

        for (boost::filesystem::directory_iterator iter(directory); iter!=end; ++iter) {
            if (is_directory(*iter)) {
                ProcessFiles(*iter);
            } else {
                string  path_name = iter->path().string();

                files_vec_.push_back(path_name);
            }
        }

        sort(files_vec_.begin(), files_vec_.end());
    }
    else {
        char buf[256];
        sprintf(buf, "%s directory doesn't exist!", directory.string().c_str());
        throw std::runtime_error(buf);
    }
}

void depth_cam_ekf::FileGrabber::operator ()()
{
    while (initialised_) {
        if (frame_grabber_->IsFrameBufferFull()==false) {
            if (files_vec_.size() > (unsigned int)frame_id_) {
                Frame frame;
                string  file_full_path = files_vec_.at(frame_id_);

                frame.frame_id = frame_id_;
                frame.data = GetImageFile(file_full_path);
                ++frame_id_;

                frame_grabber_->SetFrame(frame);
            }
        }
        else {
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));
        }
    }
}

cv::Mat depth_cam_ekf::FileGrabber::GetImageFile(const string &file_full_path)
{
    return  cv::imread(file_full_path, 0);
}