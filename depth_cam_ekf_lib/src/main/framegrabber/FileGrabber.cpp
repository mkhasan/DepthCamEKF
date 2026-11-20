//
// Created by Hasan on 11/10/26.
//

#include "FileGrabber.h"
#include <Log.h>


using namespace std;
using namespace Log;

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
    std::map<string, const boost::filesystem::path> file_paths;
    vector<string> file_names[2];

    file_paths.insert({"image", directory / "frames"});
    file_paths.insert({"depth", directory / "depth_info"});

    string last_directories[] = {"frames", "depth_info"};

    bool error = false;
    for (int i = 0; i < 2; i++) {
        if (boost::filesystem::exists(directory)) {
            auto last_dir = directory / last_directories[i];
            if (!exists(last_dir)) {
                error = true;
                break;
            }
            boost::filesystem::directory_iterator end;
            for (boost::filesystem::directory_iterator iter(last_dir); iter!=end; ++iter) {
                if (is_directory(*iter)) {
                    error = true;
                    break;
                } else {
                    string  path_name = iter->path().string();
                    file_names[i].push_back(path_name);
                }
            }
            if (error) {
                break;
            }

            sort(file_names[i].begin(), file_names[i].end());

        }
    }

    if (error) {
        char buf[256];
        sprintf(buf, "%s directory format missmatch", directory.string().c_str());
        throw std::runtime_error(buf);
    }

    if (file_names[0].size() != file_names[1].size()) {
        char buf[256];
        sprintf(buf, "%s directory contins different number of frames and depth_infos", directory.string().c_str());
        throw std::runtime_error(buf);

    }
    for (int k=0; k<file_names[0].size(); k++) {
        files_vec_.push_back(std::make_pair(file_names[0][k], file_names[1][k]));
    }
}

void depth_cam_ekf::FileGrabber::operator ()()
{
    while (initialised_) {
        if (frame_grabber_->IsFrameBufferFull()==false) {
            if (files_vec_.size() > (unsigned int)frame_id_) {
                Frame frame;
                frame.frame_id = frame_id_;
                frame.data = GetImageFile(files_vec_.at(frame_id_).first);
                frame.depths = GetDepthInfo(files_vec_.at(frame_id_).second);
                if (frame.depths.size() == frame.data.rows*frame.data.cols) {
                    frame_grabber_->SetFrame(frame);
                }
                else {
                    log(Level::ERROR, "frame data dim mismatched ... row: %d col: %d depth-info-len: %d", frame.data.rows, frame.data.cols, frame.depths.size());
                }
                ++frame_id_;




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

vector<float> depth_cam_ekf::FileGrabber::GetDepthInfo(const string& pathname) {
    vector<float> depth_info;
    std::ifstream file(pathname);

    if (!file.is_open())
    {
        std::cerr << "Failed to open file!" << std::endl;
        return {};
    }


    float value;

    while (file >> value)
    {
        depth_info.push_back(value);
    }

    file.close();

    return depth_info;


}