//
// Created by mecasys on 11/10/26.
//

#ifndef DEPTHCAMEKF_FRAMEGRABBER_H
#define DEPTHCAMEKF_FRAMEGRABBER_H

#include <string>
#include <queue>

#include <boost/thread.hpp>

#include <opencv2/opencv.hpp>




namespace depth_cam_ekf {
    struct Frame {
        int     frame_id;
        cv::Mat data;
        std::vector<float> depths;
    };

    class FileGrabber;
    class UsbCamGrabber;

    class FrameGrabber {
    public:
        FrameGrabber();
        ~FrameGrabber();

        void Init(const std::string &dev, const bool mode);

        bool GetFrame(int frame_id, Frame *frame);
        void SetFrame(const Frame &frame);
        bool IsFrameBufferFull();

    private:
        std::queue<Frame> frame_buffer_;
        boost::mutex  fg_mutex_;

        FileGrabber   *file_grabber_;
        UsbCamGrabber *usb_cam_grabber_;
    };

}


#endif //DEPTHCAMEKF_FRAMEGRABBER_H