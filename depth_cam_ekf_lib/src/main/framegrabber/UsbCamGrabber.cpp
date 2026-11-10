//
// Created by mecasys on 11/10/26.
//

#include "UsbCamGrabber.h"

#include <librealsense2/rs.hpp>

depth_cam_ekf::UsbCamGrabber::UsbCamGrabber()
{
    initialised_  = false;
    frame_id_     = 0;

    // video_        = NULL;
    // video_format_ = NULL;
}

depth_cam_ekf::UsbCamGrabber::~UsbCamGrabber()
{
    // if (video_ != NULL) {
    //     delete  video_;
    // }
    //
    // if (video_format_ != NULL) {
    //     delete  video_format_;
    // }

}

void depth_cam_ekf::UsbCamGrabber::Init(const std::string &uri, depth_cam_ekf::FrameGrabber *frame_grabber)
{
    // video_              = new pangolin::VideoInput(uri);
    // video_format_       = new pangolin::VideoPixelFormat;
    // *video_format_      = pangolin::VideoFormatFromString(video_->PixFormat());
    // video_width_        = video_->Width();
    // video_height_       = video_->Height();

    frame_grabber_      = frame_grabber;
    initialised_        = true;

    ucg_thread_ = boost::thread(boost::ref(*this));
}

void depth_cam_ekf::UsbCamGrabber::operator ()()
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
    while (initialised_) {
        if (frame_grabber_->IsFrameBufferFull()==false) {

            Frame   frame;
            cv::Mat temp1, temp2;

            // if (!video_->PixFormat().format.compare("YUV422P"))
            //     temp1.create(video_->Height(), video_->Width(), CV_8UC2);
            // else
            //     temp1.create(video_->Height(), video_->Width(), CV_8UC3);
            //
            // temp2.create(video_->Height(), video_->Width(), CV_8UC1);
            //
            // video_->GrabNext(temp1.data, true);
            //
            // if (!video_->PixFormat().format.compare("YUV422P"))
            //     cv::cvtColor(temp1, temp2, CV_YUV2GRAY_Y422);
            // else
            //     cv::cvtColor(temp1, temp2, CV_RGB2GRAY);

            rs2::frameset frames = pipe.wait_for_frames();

            rs2::video_frame color = frames.get_color_frame();

            cv::Mat image(
                color.get_height(),
                color.get_width(),
                CV_8UC3,
                (void*)color.get_data(),
                cv::Mat::AUTO_STEP
            );

            frame.frame_id = frame_id_;
            frame.data = image;
            // frame.data.create(240, 320, CV_8UC1);
            //
            // if ((temp2.cols!=320) || (temp2.rows!=240))
            //     cv::resize(temp2, frame.data, cv::Size(320,240), 1, 1, CV_INTER_LINEAR);
            // else
            //     frame.data = temp2.clone();

            ++frame_id_;

            frame_grabber_->SetFrame(frame);
        }
        else {
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));
        }
    }
}

