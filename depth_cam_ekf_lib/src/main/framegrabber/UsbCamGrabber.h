//
// Created by Hasan on 11/10/26.
//

#ifndef DEPTHCAMEKF_USBCAMGRABBER_H
#define DEPTHCAMEKF_USBCAMGRABBER_H

#include "FrameGrabber.h"


#include <string>

namespace depth_cam_ekf {

    class UsbCamGrabber {
    public:
        UsbCamGrabber();
        ~UsbCamGrabber();

        void Init(const std::string &uri, FrameGrabber *frame_grabber);
        void operator()();

    private:
        boost::thread ucg_thread_;

        bool  initialised_;
        int frame_id_;
        FrameGrabber  *frame_grabber_;

        // pangolin::VideoInput  *video_;
        // pangolin::VideoPixelFormat  *video_format_;
        int video_width_;
        int video_height_;
    };

}


#endif //DEPTHCAMEKF_USBCAMGRABBER_H