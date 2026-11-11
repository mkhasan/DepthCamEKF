//
// Created by Hasan on 11/10/26.
//

#include "pangolin_util.h"

void depth_cam_ekf::Handler3D::Mouse(pangolin::View& display, pangolin::MouseButton button, int x, int y, bool pressed, int button_state)
{
    pangolin::Handler3D::Mouse(display, button, x, y, pressed, button_state);

    if(pressed && (button_state == 1)) {
        int picked;

        display.ActivateScissorAndClear(*view_state_);
        picked = depth_cam_slam_ptr_->graphic_tool_->Picker(x, y, true);

        if (picked != 0) {
            depth_cam_slam_ptr_->toggle_feature_lab(picked-1);
            depth_cam_slam_ptr_->mark_feature_by_lab(picked-1);
        }
    }
}

void depth_cam_ekf::Handler2D::Mouse(pangolin::View& display, pangolin::MouseButton button, int x, int y, bool pressed, int button_state)
{
    if(pressed && (button_state == 1)) {
        int picked;

        double  scale_w = (double)depth_cam_slam_ptr_->camera_->width_ / (double)display.v.w;
        double  scale_h = (double)depth_cam_slam_ptr_->camera_->height_ / (double)display.v.h;

        int local_x = (int)((double)(x - display.v.l) * scale_w);
        int local_y = (int)((double)((display.v.b + display.v.h) - y) * scale_h);

        display.ActivateScissorAndClear();
        picked = depth_cam_slam_ptr_->graphic_tool_->Picker(x, y, false);

        if (picked != 0) {
            depth_cam_slam_ptr_->toggle_feature_lab(picked-1);
            depth_cam_slam_ptr_->mark_feature_by_lab(picked-1);
        }
        else {
            depth_cam_slam_ptr_->uu_ = local_x;
            depth_cam_slam_ptr_->vv_ = local_y;
            depth_cam_slam_ptr_->location_selected_flag_ = true;
        }
    }
}
