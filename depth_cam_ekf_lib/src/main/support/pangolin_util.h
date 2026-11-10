//
// Created by mecasys on 11/10/26.
//

#ifndef DEPTHCAMEKF_PANGOLIN_UTIL_H
#define DEPTHCAMEKF_PANGOLIN_UTIL_H

#include "DepthCamSLAM.h"

#include <pangolin/pangolin.h>


namespace depth_cam_ekf {
    struct Handler3D : pangolin::Handler3D {
        Handler3D(pangolin::OpenGlRenderState& view_state, DepthCamSLAM *depth_cam_slam,
                    pangolin::AxisDirection enforce_up=pangolin::AxisNone,
                    float trans_scale=0.01f) :
            pangolin::Handler3D(view_state, enforce_up, trans_scale) {  depth_cam_slam_ptr_ = depth_cam_slam; view_state_ = &view_state;  }

        void Mouse(pangolin::View&, pangolin::MouseButton button, int x, int y, bool pressed, int button_state);

        DepthCamSLAM  *depth_cam_slam_ptr_;
        pangolin::OpenGlRenderState *view_state_;
    };

    struct Handler2D : pangolin::Handler {
        Handler2D(DepthCamSLAM *depth_cam_slam) { depth_cam_slam_ptr_ = depth_cam_slam; }

        void Mouse(pangolin::View&, pangolin::MouseButton button, int x, int y, bool pressed, int button_state);

        DepthCamSLAM  *depth_cam_slam_ptr_;
    };
}
#endif //DEPTHCAMEKF_PANGOLIN_UTIL_H