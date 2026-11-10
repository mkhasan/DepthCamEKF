//
// Created by Hasan on 11/10/26.
//

#include "FrameGrabber.h"

#include "UsbCamGrabber.h"

using namespace std;

depth_cam_ekf::FrameGrabber::FrameGrabber()
{
    //file_grabber_ = NULL;
    usb_cam_grabber_ = NULL;
}

depth_cam_ekf::FrameGrabber::~FrameGrabber()
{
    // if (file_grabber_ != NULL)
    //     delete  file_grabber_;

    if (usb_cam_grabber_ != NULL)
        delete  usb_cam_grabber_;

    while (!frame_buffer_.empty()) {
        frame_buffer_.pop();
    }
}


void depth_cam_ekf::FrameGrabber::Init(const string &dev, const bool mode) {
    assert(mode);
    usb_cam_grabber_ = new UsbCamGrabber;
    usb_cam_grabber_->Init(dev, this);
}

bool depth_cam_ekf::FrameGrabber::GetFrame(int frame_id, Frame *frame)
{
    boost::mutex::scoped_lock lock(fg_mutex_);

    if (frame_buffer_.size() < 1) {
        return  false;
    }

    *frame = frame_buffer_.front();
    //  assert(frame->frame_id == frame_id);
    frame_buffer_.pop();

    return  true;
}

void depth_cam_ekf::FrameGrabber::SetFrame(const Frame &frame)
{
    boost::mutex::scoped_lock lock(fg_mutex_);

    frame_buffer_.push(frame);
}

bool depth_cam_ekf::FrameGrabber::IsFrameBufferFull()
{
    boost::mutex::scoped_lock lock(fg_mutex_);

    if (frame_buffer_.size() < 50) {
        return  false;
    }

    return  true;
}