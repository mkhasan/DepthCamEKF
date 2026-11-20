//
// Created by Hasan on 11/9/26.
//





#include "DepthCamSLAM.h"
#include "support/pangolin_util.h"
#include "Log.h"

#include <pangolin/pangolin.h>

#include <iostream>

using namespace std;
using namespace Log;

depth_cam_ekf::DepthCamSLAM *g_depth_cam_slam;
int main(int argc, char * argv[]) {

    bool  g_next = false;
    bool  g_play = false;
    int   g_frame_id  = 0;

    depth_cam_ekf::Frame  frame;


    set_level(Level::DEBUG);
    g_depth_cam_slam = new depth_cam_ekf::DepthCamSLAM();

    g_depth_cam_slam->Init("/home/mecasys/extension/MyProjects/SLAM/DepthCamEKF/depth_cam_ekf_lib/data/DepthCamLib.cfg");


    pangolin::CreateWindowAndBind("DepthCamTest", 800, 480);
#ifndef __APPLE__
    glutInit(&argc, argv);
#endif


    pangolin::OpenGlRenderState view_state_3d(pangolin::ProjectionMatrix(g_depth_cam_slam->camera_->width_,
                                                                           g_depth_cam_slam->camera_->height_,
                                                                           g_depth_cam_slam->camera_->fku_,
                                                                           g_depth_cam_slam->camera_->fkv_,
                                                                           g_depth_cam_slam->camera_->centre_(0),
                                                                           g_depth_cam_slam->camera_->centre_(1),
                                                                           0.00001, 1000),
                                               pangolin::ModelViewLookAt(-0.0, 0.18, -1.4, 0.0, 0.13, 0.0, pangolin::AxisY));


    // Add 2 panels, 3D and camera viewers
    pangolin::View  &left_panel1  = pangolin::CreatePanel("left_panel1").SetBounds(0.5, 1.0, 0.0, 0.3);
    pangolin::View  &left_panel2  = pangolin::CreatePanel("left_panel2").SetBounds(0.0, 0.5, 0.0, 0.3);
    pangolin::View  &right_panel1 = pangolin::CreatePanel("right_panel1").SetBounds(0.5, 1.0, 0.3, 0.6);
    pangolin::View  &right_panel2 = pangolin::CreatePanel("right_panel2").SetBounds(0.0, 0.5, 0.3, 0.6);
    pangolin::View  &view_3d      = pangolin::CreateDisplay().SetBounds(0.5, 1.0, 0.6, 1.0, -4./3.)
        .SetHandler(new depth_cam_ekf::Handler3D(view_state_3d, g_depth_cam_slam));
    pangolin::View  &view_cam     = pangolin::Display("view_cam").SetBounds(0.0, 0.5, 0.6, 1.0, 4./3.)
        .SetHandler(new depth_cam_ekf::Handler2D(g_depth_cam_slam));


    while (!pangolin::ShouldQuit()) {
        if (pangolin::DisplayBase().IsShown()) {
            pangolin::DisplayBase().ActivateScissorAndClear();
        }

        // Create GUIs
        static pangolin::Var<string>  cap_graphics_toggles("left_panel1.Graphics Toggles", "");
        static pangolin::Var<bool>    chk_rectify_image_display("left_panel1.Rectify Image Display", false);
        static pangolin::Var<bool>    chk_display_trajectory("left_panel1.Display Trajectory", false);
        static pangolin::Var<bool>    chk_display_3d_features("left_panel1.Display 3D Features", true);
        static pangolin::Var<bool>    chk_display_3d_uncertainties("left_panel1.Display 3D Uncertainties", true);
        static pangolin::Var<bool>    chk_display_2d_descriptors("left_panel1.Display 2D Descriptors", true);
        static pangolin::Var<bool>    chk_display_2d_search_regions("left_panel1.Display 2D Search Regions", true);
        static pangolin::Var<bool>    chk_display_initialisation("left_panel1.Display Initialisation", true);

        static pangolin::Var<string>  cap_main_controls("left_panel2.Main Controls", "");
        static pangolin::Var<bool>    btn_continuous("left_panel2.Continuous", false, false);
        static pangolin::Var<bool>    btn_next("left_panel2.Next", false, false);
        static pangolin::Var<bool>    btn_stop("left_panel2.Stop", false, false);

        static pangolin::Var<string>  cap_controls_toggles("right_panel1.Controls Toggles", "");
        static pangolin::Var<bool>    chk_toggle_tracking("right_panel1.Toggle Tracking", false);
        static pangolin::Var<bool>    chk_enable_mapping("right_panel1.Enable Mapping", true);
        static pangolin::Var<bool>    chk_output_tracked_images("right_panel1.Output Tracked Images", false);
        static pangolin::Var<bool>    chk_output_raw_images("right_panel1.Output Raw Images", false);

        static pangolin::Var<string>  cap_action_controls("right_panel2.Action Controls", "");
        static pangolin::Var<bool>    btn_initialise_manual_feature("right_panel2.Initialise Manual Feature", false, false);
        static pangolin::Var<bool>    btn_initialise_auto_feature("right_panel2.Initialise Auto Feature", false, false);
        static pangolin::Var<bool>    btn_print_robot_state("right_panel2.Print Robot State", false, false);
        static pangolin::Var<bool>    btn_delete_feature("right_panel2.Delete Feature", false, false);
        static pangolin::Var<bool>    btn_save_patch("right_panel2.Save Patch", false, false);
        static pangolin::Var<bool>    btn_quit("right_panel2.Quit", false, false);

        //=========================================================================
        // Activate efficiently by object
        // (3D Handler requires depth testing to be enabled)
        view_3d.ActivateScissorAndClear(view_state_3d);


        g_depth_cam_slam->graphic_tool_->Draw3dScene(chk_display_trajectory,
                                                chk_display_3d_features,
                                                chk_display_3d_uncertainties);

        return 0;
        //=========================================================================
        // GET & PROCESS A NEW FRAME & CAMERA VIEW DISPLAY
        view_cam.ActivateScissorAndClear();

        if (g_next) {
            g_depth_cam_slam->frame_grabber_->GetFrame(g_frame_id, &frame);

            if (chk_toggle_tracking) {
                g_depth_cam_slam->GoOneStep(frame.data,
                                    chk_display_trajectory,
                                    chk_enable_mapping);
            }

            ++g_frame_id;
        }



        g_depth_cam_slam->graphic_tool_->DrawAR(frame.data,
                                        chk_rectify_image_display,
                                        chk_display_trajectory,
                                        chk_display_3d_features,
                                        chk_display_3d_uncertainties,
                                        chk_display_2d_descriptors,
                                        chk_display_2d_search_regions,
                                        chk_display_initialisation);




        //=========================================================================
        // Render our UI panel when we receive input
        //if (pangolin::Pushed(pangolin::KeySpace)) {
        //  left_panel1.Render();
        //  left_panel2.Render();
        //  right_panel1.Render();
        //  right_panel2.Render();
        //}

        // pangolin::RegisterKeyPressCallback('r', []() {
        //   left_panel1.Render();
        //   left_panel2.Render();
        //   right_panel1.Render();
        //   right_panel2.Render();
        // });
        //if (pangolin::HadInput()) {

        g_next = false;

        // Buttons handling
        if (pangolin::Pushed(btn_continuous))
            g_play = true;

        if (pangolin::Pushed(btn_next))
            g_next = true;

        if (pangolin::Pushed(btn_stop))
            g_play = false;


        if (pangolin::Pushed(btn_quit))
            exit(0);

        if (g_play)
            g_next = true;

        // Swap frames and Process Events
        pangolin::FinishFrame();
    }

    return  0;


}