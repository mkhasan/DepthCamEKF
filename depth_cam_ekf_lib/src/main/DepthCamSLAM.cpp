//
// Created by Hasan on 11/9/26.
//


#include  "Params.h"
#include "DepthCamSLAM.h"

#include <pangolin/pangolin.h>
#include <string>




using namespace std;

depth_cam_ekf::DepthCamSLAM::DepthCamSLAM() :
        kBoxSize_(11), kNoSigma_(3.0), kCorrThresh2_(0.40),
        kCorrelationSigmaThreshold_(10.0)
    {
        camera_ = nullptr;
        motion_model_ = nullptr;
        feature_model_ = nullptr;
        frame_grabber_ = nullptr;
        graphic_tool_ = nullptr;
        //
        // kalman_ = NULL;
    }

depth_cam_ekf::DepthCamSLAM::~DepthCamSLAM() {
    if (camera_ != nullptr)
        delete  camera_;

    if (motion_model_ != nullptr)
        delete motion_model_;

    if (feature_model_ != nullptr)

    if (frame_grabber_ != nullptr)
        delete  frame_grabber_;

    if (graphic_tool_ != nullptr)
        delete  graphic_tool_;

    // if (kalman_ != NULL)
    //     delete  kalman_;

    while (!feature_list_.empty())
        feature_list_.pop_back();

    while (!selected_feature_list_.empty())
        selected_feature_list_.pop_back();

    while (!trajectory_store_.empty())
        trajectory_store_.pop_back();
}

void depth_cam_ekf::DepthCamSLAM::Init(const string &config_path) {

    pangolin::ParseVarsFile(config_path);

    pangolin::Var<bool>   input_mode("input.mode", false);
    pangolin::Var<string> input_name("input.name", "empty");

    pangolin::Var<double> delta_t("params.delta_t", 0.0);
    pangolin::Var<int>    number_of_features_to_select("params.number_of_features_to_select", 0);
    pangolin::Var<int>    number_of_features_to_keep_visible("params.number_of_features_to_keep_visible", 0);
    pangolin::Var<int>    max_features_to_init_at_once("params.max_features_to_init_at_once", 0);
    pangolin::Var<double> min_lambda("params.min_lambda", 0.0);
    pangolin::Var<double> max_lambda("params.max_lambda", 0.0);
    pangolin::Var<int>    number_of_particles("params.number_of_particles", 0);
    pangolin::Var<double> standard_deviation_depth_ratio("params.standard_deviation_depth_ratio", 0.0);
    pangolin::Var<int>    min_number_of_particles("params.min_number_of_particles", 0);
    pangolin::Var<double> prune_probability_threshold("params.prune_probability_threshold", 0.0);
    pangolin::Var<int>    erase_partially_init_feature_after_this_many_attempts("params.erase_partially_init_feature_after_this_many_attempts", 0);

    pangolin::Var<int>    cam_width("cam.width", 0);
    pangolin::Var<int>    cam_height("cam.height", 0);
    pangolin::Var<int>    cam_fku("cam.fku", 0);
    pangolin::Var<int>    cam_fkv("cam.fkv", 0);
    pangolin::Var<int>    cam_u0("cam.u0", 0);
    pangolin::Var<int>    cam_v0("cam.v0", 0);
    pangolin::Var<double> cam_kd1("cam.kd1", 0.0);
    pangolin::Var<int>    image_sd("image.sd", 0);
    pangolin::Var<int>    depth_sd("depth.sd", 0);

    pangolin::Var<double> state_rw_x("state.rw_x", 0.0);
    pangolin::Var<double> state_rw_y("state.rw_y", 0.0);
    pangolin::Var<double> state_rw_z("state.rw_z", 0.0);
    pangolin::Var<double> state_qwr_x("state.qwr_x", 0.0);
    pangolin::Var<double> state_qwr_y("state.qwr_y", 0.0);
    pangolin::Var<double> state_qwr_z("state.qwr_z", 0.0);
    pangolin::Var<double> state_qwr_w("state.qwr_w", 0.0);
    pangolin::Var<double> state_vw_x("state.vw_x", 0.0);
    pangolin::Var<double> state_vw_y("state.vw_y", 0.0);
    pangolin::Var<double> state_vw_z("state.vw_z", 0.0);
    pangolin::Var<double> state_ww_x("state.ww_x", 0.0);
    pangolin::Var<double> state_ww_y("state.ww_y", 0.0);
    pangolin::Var<double> state_ww_z("state.ww_z", 0.0);

    pangolin::Var<double> state_pxx0_0("state.pxx0_0", 0.0);
    pangolin::Var<double> state_pxx0_1("state.pxx0_1", 0.0);
    pangolin::Var<double> state_pxx0_2("state.pxx0_2", 0.0);
    pangolin::Var<double> state_pxx0_3("state.pxx0_3", 0.0);
    pangolin::Var<double> state_pxx0_4("state.pxx0_4", 0.0);
    pangolin::Var<double> state_pxx0_5("state.pxx0_5", 0.0);
    pangolin::Var<double> state_pxx0_6("state.pxx0_6", 0.0);
    pangolin::Var<double> state_pxx0_7("state.pxx0_7", 0.0);
    pangolin::Var<double> state_pxx0_8("state.pxx0_8", 0.0);
    pangolin::Var<double> state_pxx0_9("state.pxx0_9", 0.0);
    pangolin::Var<double> state_pxx0_10("state.pxx0_10", 0.0);
    pangolin::Var<double> state_pxx0_11("state.pxx0_11", 0.0);
    pangolin::Var<double> state_pxx0_12("state.pxx0_12", 0.0);

    pangolin::Var<double> state_pxx1_0("state.pxx1_0", 0.0);
    pangolin::Var<double> state_pxx1_1("state.pxx1_1", 0.0);
    pangolin::Var<double> state_pxx1_2("state.pxx1_2", 0.0);
    pangolin::Var<double> state_pxx1_3("state.pxx1_3", 0.0);
    pangolin::Var<double> state_pxx1_4("state.pxx1_4", 0.0);
    pangolin::Var<double> state_pxx1_5("state.pxx1_5", 0.0);
    pangolin::Var<double> state_pxx1_6("state.pxx1_6", 0.0);
    pangolin::Var<double> state_pxx1_7("state.pxx1_7", 0.0);
    pangolin::Var<double> state_pxx1_8("state.pxx1_8", 0.0);
    pangolin::Var<double> state_pxx1_9("state.pxx1_9", 0.0);
    pangolin::Var<double> state_pxx1_10("state.pxx1_10", 0.0);
    pangolin::Var<double> state_pxx1_11("state.pxx1_11", 0.0);
    pangolin::Var<double> state_pxx1_12("state.pxx1_12", 0.0);

    pangolin::Var<double> state_pxx2_0("state.pxx2_0", 0.0);
    pangolin::Var<double> state_pxx2_1("state.pxx2_1", 0.0);
    pangolin::Var<double> state_pxx2_2("state.pxx2_2", 0.0);
    pangolin::Var<double> state_pxx2_3("state.pxx2_3", 0.0);
    pangolin::Var<double> state_pxx2_4("state.pxx2_4", 0.0);
    pangolin::Var<double> state_pxx2_5("state.pxx2_5", 0.0);
    pangolin::Var<double> state_pxx2_6("state.pxx2_6", 0.0);
    pangolin::Var<double> state_pxx2_7("state.pxx2_7", 0.0);
    pangolin::Var<double> state_pxx2_8("state.pxx2_8", 0.0);
    pangolin::Var<double> state_pxx2_9("state.pxx2_9", 0.0);
    pangolin::Var<double> state_pxx2_10("state.pxx2_10", 0.0);
    pangolin::Var<double> state_pxx2_11("state.pxx2_11", 0.0);
    pangolin::Var<double> state_pxx2_12("state.pxx2_12", 0.0);

    pangolin::Var<double> state_pxx3_0("state.pxx3_0", 0.0);
    pangolin::Var<double> state_pxx3_1("state.pxx3_1", 0.0);
    pangolin::Var<double> state_pxx3_2("state.pxx3_2", 0.0);
    pangolin::Var<double> state_pxx3_3("state.pxx3_3", 0.0);
    pangolin::Var<double> state_pxx3_4("state.pxx3_4", 0.0);
    pangolin::Var<double> state_pxx3_5("state.pxx3_5", 0.0);
    pangolin::Var<double> state_pxx3_6("state.pxx3_6", 0.0);
    pangolin::Var<double> state_pxx3_7("state.pxx3_7", 0.0);
    pangolin::Var<double> state_pxx3_8("state.pxx3_8", 0.0);
    pangolin::Var<double> state_pxx3_9("state.pxx3_9", 0.0);
    pangolin::Var<double> state_pxx3_10("state.pxx3_10", 0.0);
    pangolin::Var<double> state_pxx3_11("state.pxx3_11", 0.0);
    pangolin::Var<double> state_pxx3_12("state.pxx3_12", 0.0);

    pangolin::Var<double> state_pxx4_0("state.pxx4_0", 0.0);
    pangolin::Var<double> state_pxx4_1("state.pxx4_1", 0.0);
    pangolin::Var<double> state_pxx4_2("state.pxx4_2", 0.0);
    pangolin::Var<double> state_pxx4_3("state.pxx4_3", 0.0);
    pangolin::Var<double> state_pxx4_4("state.pxx4_4", 0.0);
    pangolin::Var<double> state_pxx4_5("state.pxx4_5", 0.0);
    pangolin::Var<double> state_pxx4_6("state.pxx4_6", 0.0);
    pangolin::Var<double> state_pxx4_7("state.pxx4_7", 0.0);
    pangolin::Var<double> state_pxx4_8("state.pxx4_8", 0.0);
    pangolin::Var<double> state_pxx4_9("state.pxx4_9", 0.0);
    pangolin::Var<double> state_pxx4_10("state.pxx4_10", 0.0);
    pangolin::Var<double> state_pxx4_11("state.pxx4_11", 0.0);
    pangolin::Var<double> state_pxx4_12("state.pxx4_12", 0.0);

    pangolin::Var<double> state_pxx5_0("state.pxx5_0", 0.0);
    pangolin::Var<double> state_pxx5_1("state.pxx5_1", 0.0);
    pangolin::Var<double> state_pxx5_2("state.pxx5_2", 0.0);
    pangolin::Var<double> state_pxx5_3("state.pxx5_3", 0.0);
    pangolin::Var<double> state_pxx5_4("state.pxx5_4", 0.0);
    pangolin::Var<double> state_pxx5_5("state.pxx5_5", 0.0);
    pangolin::Var<double> state_pxx5_6("state.pxx5_6", 0.0);
    pangolin::Var<double> state_pxx5_7("state.pxx5_7", 0.0);
    pangolin::Var<double> state_pxx5_8("state.pxx5_8", 0.0);
    pangolin::Var<double> state_pxx5_9("state.pxx5_9", 0.0);
    pangolin::Var<double> state_pxx5_10("state.pxx5_10", 0.0);
    pangolin::Var<double> state_pxx5_11("state.pxx5_11", 0.0);
    pangolin::Var<double> state_pxx5_12("state.pxx5_12", 0.0);

    pangolin::Var<double> state_pxx6_0("state.pxx6_0", 0.0);
    pangolin::Var<double> state_pxx6_1("state.pxx6_1", 0.0);
    pangolin::Var<double> state_pxx6_2("state.pxx6_2", 0.0);
    pangolin::Var<double> state_pxx6_3("state.pxx6_3", 0.0);
    pangolin::Var<double> state_pxx6_4("state.pxx6_4", 0.0);
    pangolin::Var<double> state_pxx6_5("state.pxx6_5", 0.0);
    pangolin::Var<double> state_pxx6_6("state.pxx6_6", 0.0);
    pangolin::Var<double> state_pxx6_7("state.pxx6_7", 0.0);
    pangolin::Var<double> state_pxx6_8("state.pxx6_8", 0.0);
    pangolin::Var<double> state_pxx6_9("state.pxx6_9", 0.0);
    pangolin::Var<double> state_pxx6_10("state.pxx6_10", 0.0);
    pangolin::Var<double> state_pxx6_11("state.pxx6_11", 0.0);
    pangolin::Var<double> state_pxx6_12("state.pxx6_12", 0.0);

    pangolin::Var<double> state_pxx7_0("state.pxx7_0", 0.0);
    pangolin::Var<double> state_pxx7_1("state.pxx7_1", 0.0);
    pangolin::Var<double> state_pxx7_2("state.pxx7_2", 0.0);
    pangolin::Var<double> state_pxx7_3("state.pxx7_3", 0.0);
    pangolin::Var<double> state_pxx7_4("state.pxx7_4", 0.0);
    pangolin::Var<double> state_pxx7_5("state.pxx7_5", 0.0);
    pangolin::Var<double> state_pxx7_6("state.pxx7_6", 0.0);
    pangolin::Var<double> state_pxx7_7("state.pxx7_7", 0.0);
    pangolin::Var<double> state_pxx7_8("state.pxx7_8", 0.0);
    pangolin::Var<double> state_pxx7_9("state.pxx7_9", 0.0);
    pangolin::Var<double> state_pxx7_10("state.pxx7_10", 0.0);
    pangolin::Var<double> state_pxx7_11("state.pxx7_11", 0.0);
    pangolin::Var<double> state_pxx7_12("state.pxx7_12", 0.0);

    pangolin::Var<double> state_pxx8_0("state.pxx8_0", 0.0);
    pangolin::Var<double> state_pxx8_1("state.pxx8_1", 0.0);
    pangolin::Var<double> state_pxx8_2("state.pxx8_2", 0.0);
    pangolin::Var<double> state_pxx8_3("state.pxx8_3", 0.0);
    pangolin::Var<double> state_pxx8_4("state.pxx8_4", 0.0);
    pangolin::Var<double> state_pxx8_5("state.pxx8_5", 0.0);
    pangolin::Var<double> state_pxx8_6("state.pxx8_6", 0.0);
    pangolin::Var<double> state_pxx8_7("state.pxx8_7", 0.0);
    pangolin::Var<double> state_pxx8_8("state.pxx8_8", 0.0);
    pangolin::Var<double> state_pxx8_9("state.pxx8_9", 0.0);
    pangolin::Var<double> state_pxx8_10("state.pxx8_10", 0.0);
    pangolin::Var<double> state_pxx8_11("state.pxx8_11", 0.0);
    pangolin::Var<double> state_pxx8_12("state.pxx8_12", 0.0);

    pangolin::Var<double> state_pxx9_0("state.pxx9_0", 0.0);
    pangolin::Var<double> state_pxx9_1("state.pxx9_1", 0.0);
    pangolin::Var<double> state_pxx9_2("state.pxx9_2", 0.0);
    pangolin::Var<double> state_pxx9_3("state.pxx9_3", 0.0);
    pangolin::Var<double> state_pxx9_4("state.pxx9_4", 0.0);
    pangolin::Var<double> state_pxx9_5("state.pxx9_5", 0.0);
    pangolin::Var<double> state_pxx9_6("state.pxx9_6", 0.0);
    pangolin::Var<double> state_pxx9_7("state.pxx9_7", 0.0);
    pangolin::Var<double> state_pxx9_8("state.pxx9_8", 0.0);
    pangolin::Var<double> state_pxx9_9("state.pxx9_9", 0.0);
    pangolin::Var<double> state_pxx9_10("state.pxx9_10", 0.0);
    pangolin::Var<double> state_pxx9_11("state.pxx9_11", 0.0);
    pangolin::Var<double> state_pxx9_12("state.pxx9_12", 0.0);

    pangolin::Var<double> state_pxx10_0("state.pxx10_0", 0.0);
    pangolin::Var<double> state_pxx10_1("state.pxx10_1", 0.0);
    pangolin::Var<double> state_pxx10_2("state.pxx10_2", 0.0);
    pangolin::Var<double> state_pxx10_3("state.pxx10_3", 0.0);
    pangolin::Var<double> state_pxx10_4("state.pxx10_4", 0.0);
    pangolin::Var<double> state_pxx10_5("state.pxx10_5", 0.0);
    pangolin::Var<double> state_pxx10_6("state.pxx10_6", 0.0);
    pangolin::Var<double> state_pxx10_7("state.pxx10_7", 0.0);
    pangolin::Var<double> state_pxx10_8("state.pxx10_8", 0.0);
    pangolin::Var<double> state_pxx10_9("state.pxx10_9", 0.0);
    pangolin::Var<double> state_pxx10_10("state.pxx10_10", 0.0);
    pangolin::Var<double> state_pxx10_11("state.pxx10_11", 0.0);
    pangolin::Var<double> state_pxx10_12("state.pxx10_12", 0.0);

    pangolin::Var<double> state_pxx11_0("state.pxx11_0", 0.0);
    pangolin::Var<double> state_pxx11_1("state.pxx11_1", 0.0);
    pangolin::Var<double> state_pxx11_2("state.pxx11_2", 0.0);
    pangolin::Var<double> state_pxx11_3("state.pxx11_3", 0.0);
    pangolin::Var<double> state_pxx11_4("state.pxx11_4", 0.0);
    pangolin::Var<double> state_pxx11_5("state.pxx11_5", 0.0);
    pangolin::Var<double> state_pxx11_6("state.pxx11_6", 0.0);
    pangolin::Var<double> state_pxx11_7("state.pxx11_7", 0.0);
    pangolin::Var<double> state_pxx11_8("state.pxx11_8", 0.0);
    pangolin::Var<double> state_pxx11_9("state.pxx11_9", 0.0);
    pangolin::Var<double> state_pxx11_10("state.pxx11_10", 0.0);
    pangolin::Var<double> state_pxx11_11("state.pxx11_11", 0.0);
    pangolin::Var<double> state_pxx11_12("state.pxx11_12", 0.0);

    pangolin::Var<double> state_pxx12_0("state.pxx12_0", 0.0);
    pangolin::Var<double> state_pxx12_1("state.pxx12_1", 0.0);
    pangolin::Var<double> state_pxx12_2("state.pxx12_2", 0.0);
    pangolin::Var<double> state_pxx12_3("state.pxx12_3", 0.0);
    pangolin::Var<double> state_pxx12_4("state.pxx12_4", 0.0);
    pangolin::Var<double> state_pxx12_5("state.pxx12_5", 0.0);
    pangolin::Var<double> state_pxx12_6("state.pxx12_6", 0.0);
    pangolin::Var<double> state_pxx12_7("state.pxx12_7", 0.0);
    pangolin::Var<double> state_pxx12_8("state.pxx12_8", 0.0);
    pangolin::Var<double> state_pxx12_9("state.pxx12_9", 0.0);
    pangolin::Var<double> state_pxx12_10("state.pxx12_10", 0.0);
    pangolin::Var<double> state_pxx12_11("state.pxx12_11", 0.0);
    pangolin::Var<double> state_pxx12_12("state.pxx12_12", 0.0);

    pangolin::Var<string> f1_identifier("f1.identifier", "empty");
    pangolin::Var<double> f1_yi_x("f1.yi_x", 0.0);
    pangolin::Var<double> f1_yi_y("f1.yi_y", 0.0);
    pangolin::Var<double> f1_yi_z("f1.yi_z", 0.0);
    pangolin::Var<double> f1_xp_org_0("f1.xp_org_0", 0.0);
    pangolin::Var<double> f1_xp_org_1("f1.xp_org_1", 0.0);
    pangolin::Var<double> f1_xp_org_2("f1.xp_org_2", 0.0);
    pangolin::Var<double> f1_xp_org_3("f1.xp_org_3", 0.0);
    pangolin::Var<double> f1_xp_org_4("f1.xp_org_4", 0.0);
    pangolin::Var<double> f1_xp_org_5("f1.xp_org_5", 0.0);
    pangolin::Var<double> f1_xp_org_6("f1.xp_org_6", 0.0);

    pangolin::Var<string> f2_identifier("f2.identifier", "empty");
    pangolin::Var<double> f2_yi_x("f2.yi_x", 0.0);
    pangolin::Var<double> f2_yi_y("f2.yi_y", 0.0);
    pangolin::Var<double> f2_yi_z("f2.yi_z", 0.0);
    pangolin::Var<double> f2_xp_org_0("f2.xp_org_0", 0.0);
    pangolin::Var<double> f2_xp_org_1("f2.xp_org_1", 0.0);
    pangolin::Var<double> f2_xp_org_2("f2.xp_org_2", 0.0);
    pangolin::Var<double> f2_xp_org_3("f2.xp_org_3", 0.0);
    pangolin::Var<double> f2_xp_org_4("f2.xp_org_4", 0.0);
    pangolin::Var<double> f2_xp_org_5("f2.xp_org_5", 0.0);
    pangolin::Var<double> f2_xp_org_6("f2.xp_org_6", 0.0);

    pangolin::Var<string> f3_identifier("f3.identifier", "empty");
    pangolin::Var<double> f3_yi_x("f3.yi_x", 0.0);
    pangolin::Var<double> f3_yi_y("f3.yi_y", 0.0);
    pangolin::Var<double> f3_yi_z("f3.yi_z", 0.0);
    pangolin::Var<double> f3_xp_org_0("f3.xp_org_0", 0.0);
    pangolin::Var<double> f3_xp_org_1("f3.xp_org_1", 0.0);
    pangolin::Var<double> f3_xp_org_2("f3.xp_org_2", 0.0);
    pangolin::Var<double> f3_xp_org_3("f3.xp_org_3", 0.0);
    pangolin::Var<double> f3_xp_org_4("f3.xp_org_4", 0.0);
    pangolin::Var<double> f3_xp_org_5("f3.xp_org_5", 0.0);
    pangolin::Var<double> f3_xp_org_6("f3.xp_org_6", 0.0);

    pangolin::Var<string> f4_identifier("f4.identifier", "empty");
    pangolin::Var<double> f4_yi_x("f4.yi_x", 0.0);
    pangolin::Var<double> f4_yi_y("f4.yi_y", 0.0);
    pangolin::Var<double> f4_yi_z("f4.yi_z", 0.0);
    pangolin::Var<double> f4_xp_org_0("f4.xp_org_0", 0.0);
    pangolin::Var<double> f4_xp_org_1("f4.xp_org_1", 0.0);
    pangolin::Var<double> f4_xp_org_2("f4.xp_org_2", 0.0);
    pangolin::Var<double> f4_xp_org_3("f4.xp_org_3", 0.0);
    pangolin::Var<double> f4_xp_org_4("f4.xp_org_4", 0.0);
    pangolin::Var<double> f4_xp_org_5("f4.xp_org_5", 0.0);
    pangolin::Var<double> f4_xp_org_6("f4.xp_org_6", 0.0);

    // Keep this order!
    //  1. Camera
    //  2. MotionModel
    //  3. FullFeatureModel and PartFeatureModel
    camera_ = new Camera();
    camera_->SetCameraParameters(cam_width, cam_height, cam_fku, cam_fkv,
                                cam_u0, cam_v0, cam_kd1, image_sd, depth_sd);

    motion_model_ = new MotionModel();

    feature_model_ = new DepthCamFeatureModel(3, 3, 3, camera_, motion_model_);


    // Initialise constant-like variables
    kDeltaT_ = delta_t;
    kNumberOfFeaturesToSelect_ = number_of_features_to_select;
    kNumberOfFeaturesToKeepVisible_ = number_of_features_to_keep_visible;
    kMaxFeaturesToInitAtOnce_ = max_features_to_init_at_once;
    kMinLambda_ = min_lambda;
    kMaxLambda_ = max_lambda;
    kNumberOfParticles_ = number_of_particles;
    kStandardDeviationDepthRatio_ = standard_deviation_depth_ratio;
    kMinNumberOfParticles_ = min_number_of_particles;
    kPruneProbabilityThreshold_ = prune_probability_threshold;
    kErasePartiallyInitFeatureAfterThisManyAttempts_ = erase_partially_init_feature_after_this_many_attempts;

    number_of_visible_features_ = 0;
    minimum_attempted_measurements_of_feature_ = 10;
    successful_match_fraction_ = 0.5;
    next_free_label_ = 0;
    marked_feature_label_ = -1;
    total_state_size_ = motion_model_->kStateSize_;

    xv_.resize(motion_model_->kStateSize_);
    xv_ << state_rw_x, state_rw_y, state_rw_z,
            state_qwr_w, state_qwr_x, state_qwr_y, state_qwr_z,
            state_vw_x, state_vw_y, state_vw_z,
            state_ww_x, state_ww_y, state_ww_z;

    Pxx_.resize(motion_model_->kStateSize_, motion_model_->kStateSize_);
    Pxx_ << state_pxx0_0,state_pxx0_1,state_pxx0_2,state_pxx0_3,
            state_pxx0_4,state_pxx0_5,state_pxx0_6,state_pxx0_7,
            state_pxx0_8,state_pxx0_9,state_pxx0_10,state_pxx0_11,state_pxx0_12,

            state_pxx1_0,state_pxx1_1,state_pxx1_2,state_pxx1_3,
            state_pxx1_4,state_pxx1_5,state_pxx1_6,state_pxx1_7,
            state_pxx1_8,state_pxx1_9,state_pxx1_10,state_pxx1_11,state_pxx1_12,

            state_pxx2_0,state_pxx2_1,state_pxx2_2,state_pxx2_3,
            state_pxx2_4,state_pxx2_5,state_pxx2_6,state_pxx2_7,
            state_pxx2_8,state_pxx2_9,state_pxx2_10,state_pxx2_11,state_pxx2_12,

            state_pxx3_0,state_pxx3_1,state_pxx3_2,state_pxx3_3,
            state_pxx3_4,state_pxx3_5,state_pxx3_6,state_pxx3_7,
            state_pxx3_8,state_pxx3_9,state_pxx3_10,state_pxx3_11,state_pxx3_12,

            state_pxx4_0,state_pxx4_1,state_pxx4_2,state_pxx4_3,
            state_pxx4_4,state_pxx4_5,state_pxx4_6,state_pxx4_7,
            state_pxx4_8,state_pxx4_9,state_pxx4_10,state_pxx4_11,state_pxx4_12,

            state_pxx5_0,state_pxx5_1,state_pxx5_2,state_pxx5_3,
            state_pxx5_4,state_pxx5_5,state_pxx5_6,state_pxx5_7,
            state_pxx5_8,state_pxx5_9,state_pxx5_10,state_pxx5_11,state_pxx5_12,

            state_pxx6_0,state_pxx6_1,state_pxx6_2,state_pxx6_3,
            state_pxx6_4,state_pxx6_5,state_pxx6_6,state_pxx6_7,
            state_pxx6_8,state_pxx6_9,state_pxx6_10,state_pxx6_11,state_pxx6_12,

            state_pxx7_0,state_pxx7_1,state_pxx7_2,state_pxx7_3,
            state_pxx7_4,state_pxx7_5,state_pxx7_6,state_pxx7_7,
            state_pxx7_8,state_pxx7_9,state_pxx7_10,state_pxx7_11,state_pxx7_12,

            state_pxx8_0,state_pxx8_1,state_pxx8_2,state_pxx8_3,
            state_pxx8_4,state_pxx8_5,state_pxx8_6,state_pxx8_7,
            state_pxx8_8,state_pxx8_9,state_pxx8_10,state_pxx8_11,state_pxx8_12,

            state_pxx9_0,state_pxx9_1,state_pxx9_2,state_pxx9_3,
            state_pxx9_4,state_pxx9_5,state_pxx9_6,state_pxx9_7,
            state_pxx9_8,state_pxx9_9,state_pxx9_10,state_pxx9_11,state_pxx9_12,

            state_pxx10_0,state_pxx10_1,state_pxx10_2,state_pxx10_3,
            state_pxx10_4,state_pxx10_5,state_pxx10_6,state_pxx10_7,
            state_pxx10_8,state_pxx10_9,state_pxx10_10,state_pxx10_11,state_pxx10_12,

            state_pxx11_0,state_pxx11_1,state_pxx11_2,state_pxx11_3,
            state_pxx11_4,state_pxx11_5,state_pxx11_6,state_pxx11_7,
            state_pxx11_8,state_pxx11_9,state_pxx11_10,state_pxx11_11,state_pxx11_12,

            state_pxx12_0,state_pxx12_1,state_pxx12_2,state_pxx12_3,
            state_pxx12_4,state_pxx12_5,state_pxx12_6,state_pxx12_7,
            state_pxx12_8,state_pxx12_9,state_pxx12_10,state_pxx12_11,state_pxx12_12;

    // Add initial features
    // Eigen::VectorXd y_temp(3), xp_temp(7);
    //
    // y_temp << f1_yi_x,f1_yi_y,f1_yi_z;
    // xp_temp << f1_xp_org_0,f1_xp_org_1,f1_xp_org_2,f1_xp_org_3,f1_xp_org_4,f1_xp_org_5,f1_xp_org_6;
    // AddNewKnownFeature(y_temp, xp_temp, f1_identifier);
    //
    // y_temp << f2_yi_x,f2_yi_y,f2_yi_z;
    // xp_temp << f2_xp_org_0,f2_xp_org_1,f2_xp_org_2,f2_xp_org_3,f2_xp_org_4,f2_xp_org_5,f2_xp_org_6;
    // AddNewKnownFeature(y_temp, xp_temp, f2_identifier);
    //
    // y_temp << f3_yi_x,f3_yi_y,f3_yi_z;
    // xp_temp << f3_xp_org_0,f3_xp_org_1,f3_xp_org_2,f3_xp_org_3,f3_xp_org_4,f3_xp_org_5,f3_xp_org_6;
    // AddNewKnownFeature(y_temp, xp_temp, f3_identifier);
    //
    // y_temp << f4_yi_x,f4_yi_y,f4_yi_z;
    // xp_temp << f4_xp_org_0,f4_xp_org_1,f4_xp_org_2,f4_xp_org_3,f4_xp_org_4,f4_xp_org_5,f4_xp_org_6;
    // AddNewKnownFeature(y_temp, xp_temp, f4_identifier);
    //
    // // Good for the last step, ready for capturing, drawing and processing
    // kalman_ = new Kalman();
    graphic_tool_ = new GraphicTool(this);
    frame_grabber_ = new FrameGrabber();
    frame_grabber_->Init(input_name, input_mode);

    init_feature_search_region_defined_flag_ = false;
    location_selected_flag_ = false;

    srand48(0); // Always the same seed (pick a number), so deterministic


}

bool depth_cam_ekf::DepthCamSLAM::GoOneStep(cv::Mat frame, bool save_trajectory, bool enable_mapping) {
    return true;
}

bool depth_cam_ekf::DepthCamSLAM::AutoInitialiseFeature(cv::Mat frame, const vector<float>& depths, const Eigen::Vector3d &u)
{


    // Project a point a "reasonable" distance forward from there along
    // the optic axis
    const double FEATURE_INIT_DEPTH_HYPOTHESIS = 2.5;

    // First find a suitable patch
    const double SUITABLE_PATCH_SCORE_THRESHOLD = 20000;

    Eigen::VectorXd local_u(motion_model_->kControlSize_);

    if (FindNonOverlappingRegion(u,
                                init_feature_search_ustart_,
                                init_feature_search_vstart_,
                                init_feature_search_ufinish_,
                                init_feature_search_vfinish_)) {

        init_feature_search_region_defined_flag_ = true;

        if (set_image_selection_automatically(frame, depths,
                                            init_feature_search_ustart_,
                                            init_feature_search_vstart_,
                                            init_feature_search_ufinish_,
                                            init_feature_search_vfinish_) > SUITABLE_PATCH_SCORE_THRESHOLD) {
        // Then initialise it
            InitialiseFeature(frame);
        }
        else {
            return  false;
        }
    }
    else {
        return  false;
    }

    return  true;
}

bool depth_cam_ekf::DepthCamSLAM::FindNonOverlappingRegion(Eigen::VectorXd local_u,
                                        int &init_feature_search_ustart,
                                        int &init_feature_search_vstart,
                                        int &init_feature_search_ufinish,
                                        int &init_feature_search_vfinish)
{



    // So, the limits of a "safe" region within which we can initialise
    // features so that they end up staying within the screen
    // (Making the approximation that the whole screen moves like the
    // centre point)
    motion_model_->func_xp(xv_);
    int safe_feature_search_ustart = (kBoxSize_-1)/2 + 1;
    int safe_feature_search_ufinish = (int) camera_->width_ - (kBoxSize_-1)/2 - 1;
    int safe_feature_search_vstart = (kBoxSize_-1)/2 + 1;
    int safe_feature_search_vfinish = camera_->height_ - (kBoxSize_-1)/2 - 1;

    return  FindNonOverlappingRegionNoPredict(safe_feature_search_ustart,
                                                safe_feature_search_vstart,
                                                safe_feature_search_ufinish,
                                                safe_feature_search_vfinish,
                                                init_feature_search_ustart,
                                                init_feature_search_vstart,
                                                init_feature_search_ufinish,
                                                init_feature_search_vfinish);
}

bool depth_cam_ekf::DepthCamSLAM::FindNonOverlappingRegionNoPredict(int safe_feature_search_ustart,
                                                 int safe_feature_search_vstart,
                                                 int safe_feature_search_ufinish,
                                                 int safe_feature_search_vfinish,
                                                 int &init_feature_search_ustart,
                                                 int &init_feature_search_vstart,
                                                 int &init_feature_search_ufinish,
                                                 int &init_feature_search_vfinish)
{
    const int INIT_FEATURE_SEARCH_WIDTH = 80;
    const int INIT_FEATURE_SEARCH_HEIGHT = 60;

    // Within this, choose a random region
    // Check that we've got some room for manouevre
    if (safe_feature_search_ufinish - safe_feature_search_ustart > INIT_FEATURE_SEARCH_WIDTH &&
        safe_feature_search_vfinish - safe_feature_search_vstart > INIT_FEATURE_SEARCH_HEIGHT) {

        // Try a few times to get one that's not overlapping with any features
        // we know about
        const int NUMBER_OF_RANDOM_INIT_FEATURE_SEARCH_REGION_TRIES = 5;
        const int FEATURE_SEPARATION_MINIMUM = 10;

        // Build vectors of feature positions so we only have to work them out once
        vector<double>  u_array;
        vector<double>  v_array;

        for (vector<Feature *>::const_iterator it = feature_list_.begin();
            it != feature_list_.end(); ++it) {

            if ((*it)->fully_initialised_flag_) {
                feature_model_->func_hi_and_dhi_by_dxp_and_dhi_by_dyi((*it)->y_, motion_model_->xpRES_);

                feature_model_->func_zeroedyigraphics_and_Pzeroedyigraphics(
                    (*it)->y_,
                    xv_,
                    Pxx_,
                    (*it)->Pxy_,
                    (*it)->Pyy_);

                if (feature_model_->zeroedyigraphicsRES_(2) > 0) {
                    u_array.push_back(feature_model_->hiRES_(0));
                    v_array.push_back(feature_model_->hiRES_(1));
                }
            }
        }

        int i = 0;

        while (i < NUMBER_OF_RANDOM_INIT_FEATURE_SEARCH_REGION_TRIES) {

            int u_offset = int((safe_feature_search_ufinish - safe_feature_search_ustart - INIT_FEATURE_SEARCH_WIDTH) * drand48());
            int v_offset = int((safe_feature_search_vfinish - safe_feature_search_vstart - INIT_FEATURE_SEARCH_HEIGHT) * drand48());

            init_feature_search_ustart = safe_feature_search_ustart + u_offset;
            init_feature_search_ufinish = init_feature_search_ustart + INIT_FEATURE_SEARCH_WIDTH;
            init_feature_search_vstart = safe_feature_search_vstart + v_offset;
            init_feature_search_vfinish = init_feature_search_vstart + INIT_FEATURE_SEARCH_HEIGHT;

            bool found_a_feature_in_region_flag = false;

            // These arrays will be the same size
            vector<double>::const_iterator uit = u_array.begin();
            vector<double>::const_iterator vit = v_array.begin();

            while(uit != u_array.end()) {
                if (*uit >= init_feature_search_ustart - FEATURE_SEPARATION_MINIMUM &&
                    *uit < init_feature_search_ufinish + FEATURE_SEPARATION_MINIMUM &&
                    *vit >= init_feature_search_vstart - FEATURE_SEPARATION_MINIMUM &&
                    *vit < init_feature_search_vfinish + FEATURE_SEPARATION_MINIMUM) {
                found_a_feature_in_region_flag = true;
                break;
                }
                ++uit;
                ++vit;
            }

            // If no clashes, fine, leave while loop
            // Else go round again
            if (!found_a_feature_in_region_flag) {
                break;
            }

            ++i;
        }

        if (i == NUMBER_OF_RANDOM_INIT_FEATURE_SEARCH_REGION_TRIES) {
        return false;
        }
    }
    else {
        return  false;
    }

    return  true;
}

// Search a region for the best image patch, and set the current selection to
// this. This just calls find_best_patch_inside_region() to find a patch using the
// Shi and Tomasi criterion.
// @param ustart The x-cordinate of the start of the region
// @param vstart The y-cordinate of the start of the region
// @param ufinish The x-cordinate of the end of the region
// @param vfinish The y-cordinate of the end of the region
// @returns The smallest eigenvalue of the best patch (high means good for
// correlation)
double depth_cam_ekf::DepthCamSLAM::set_image_selection_automatically(cv::Mat frame, const vector<float>& depths,
                                                   int ustart,
                                                   int vstart,
                                                   int ufinish,
                                                   int vfinish)
{
    double  evbest = 0;

    find_best_patch_inside_region(frame, &uu_, &vv_, &evbest, kBoxSize_, ustart, vstart, ufinish, vfinish);

    get_median_depth_around_a_pixel(depths, frame.rows, frame.cols, uu_, vv_, dd_);
    if (dd_ < 0.01) {
        evbest = 0;
    }
    location_selected_flag_ = true;

    return evbest;
}

// Function to scan over (a window in an) image and find the best patch by the Shi
// and Tomasi criterion.
// Method: as described in notes from 1/7/97. Form sums in an incremental
// way to speed things up.
// @param image The image to scan
// @param ubest The x-co-ordinate of the best patch
// @param vbest The y-co-ordinate of the best patch
// @param evbest The smallest eigenvalue of the best patch (larger is better)
// @param BOXSIZE The size of the patch to use
// @param ustart The x-co-ordinate of the start of the search window
// @param vstart The y-co-ordinate of the start of the search window
// @param ufinish The x-co-ordinate of the end of the search window
// @param vfinish The v-co-ordinate of the end of the search window
void depth_cam_ekf::DepthCamSLAM::find_best_patch_inside_region(const cv::Mat &image,
                                             int *ubest,
                                             int *vbest,
                                             double *evbest,
                                             const int BOXSIZE,
                                             int ustart,
                                             int vstart,
                                             int ufinish,
                                             int vfinish)
{
    // Check that these limits aren't too close to the image edges.
    // Note that we can't use the edge pixels because we can't form
    // gradients here.
    if (ustart < (BOXSIZE-1)/2 + 1)
        ustart = (BOXSIZE-1)/2 + 1;
    if (ufinish > image.size().width - (BOXSIZE-1)/2 - 1)
        ufinish = image.size().width - (BOXSIZE-1)/2 - 1;
    if (vstart < (BOXSIZE-1)/2 + 1)
        vstart = (BOXSIZE-1)/2 + 1;
    if (vfinish > image.size().height - (BOXSIZE-1)/2 - 1)
        vfinish = image.size().height - (BOXSIZE-1)/2 - 1;

    // Is there anything left to search? If not, set the score to zero and return.
    if(vstart >= vfinish || ustart >= ufinish) {
        *ubest = ustart;
        *vbest = vstart;
        *evbest = 0;
        return;
    }

    // Calculate the width we need to find gradients in.
    int calc_width = ufinish - ustart + BOXSIZE - 1;

    // Arrays which store column sums of height BOXSIZE
    double CSgxsq[calc_width], CSgysq[calc_width], CSgxgy[calc_width];

    // For the final sums at each position (u, v)
    double TSgxsq = 0.0, TSgysq = 0.0, TSgxgy = 0.0;

    double gx, gy;
    int u = ustart, v = vstart;
    double eval1, eval2;

    // Initial stage: fill these sums for the first horizontal position
    int cstart = ustart - (BOXSIZE-1)/2;
    int cfinish = ufinish + (BOXSIZE-1)/2;
    int rstart = vstart - (BOXSIZE-1)/2;
    int i;
    int c, r;

    for (c = cstart, i = 0; c < cfinish; ++c, ++i) {
        CSgxsq[i] = 0; CSgysq[i] = 0; CSgxgy[i] = 0;

        for (r = rstart; r < rstart + BOXSIZE; ++r) {

            gx = (image.at<unsigned char>(r, c+1) - image.at<unsigned char>(r, c-1)) / 2.0;
            gy = (image.at<unsigned char>(r+1, c) - image.at<unsigned char>(r-1, c)) / 2.0;

            CSgxsq[i] += gx * gx;
            CSgysq[i] += gy * gy;
            CSgxgy[i] += gx * gy;
        }
    }

    // Now loop through u and v to form sums
    *evbest = 0;

    for (v = vstart; v < vfinish; ++v) {
        u = ustart;

        // Form first sums for u = ustart
        TSgxsq = 0.0, TSgysq = 0.0, TSgxgy = 0.0;

        for (i = 0; i < BOXSIZE; ++i) {
            TSgxsq += CSgxsq[i];
            TSgysq += CSgysq[i];
            TSgxgy += CSgxgy[i];
        }

        for (u = ustart; u < ufinish; ++u) {
            if (u != ustart) {
                // Subtract old column, add new one
                TSgxsq += CSgxsq[u - ustart + BOXSIZE - 1] - CSgxsq[u - ustart - 1];
                TSgysq += CSgysq[u - ustart + BOXSIZE - 1] - CSgysq[u - ustart - 1];
                TSgxgy += CSgxgy[u - ustart + BOXSIZE - 1] - CSgxgy[u - ustart - 1];
            }

            find_eigenvalues(TSgxsq, TSgxgy, TSgysq, &eval1, &eval2);

            // eval2 will be the smaller eigenvalue. Compare it with the one
            // we've already got
            if (eval2 > *evbest) {
                *ubest = u;
                *vbest = v;
                *evbest = eval2;
            }
        }

        if (v != vfinish - 1) {

            // Update the column sums for the next v
            for (c = cstart, i = 0; c < cfinish; ++c, ++i) {
                // Subtract the old top pixel
                gx = (image.at<unsigned char>(v - (BOXSIZE-1)/2, c+1) -
                    image.at<unsigned char>(v - (BOXSIZE-1)/2, c-1)) / 2.0;
                gy = (image.at<unsigned char>(v - (BOXSIZE-1)/2 + 1, c) -
                    image.at<unsigned char>(v - (BOXSIZE-1)/2 - 1, c)) / 2.0;

                CSgxsq[i] -= gx * gx;
                CSgysq[i] -= gy * gy;
                CSgxgy[i] -= gx * gy;

                // Add the new bottom pixel
                gx = (image.at<unsigned char>(v + (BOXSIZE-1)/2 + 1, c+1) -
                    image.at<unsigned char>(v + (BOXSIZE-1)/2 + 1, c-1)) / 2.0;
                gy = (image.at<unsigned char>(v + (BOXSIZE-1)/2 + 1 + 1, c) -
                    image.at<unsigned char>(v + (BOXSIZE-1)/2 + 1 - 1, c)) / 2.0;

                CSgxsq[i] += gx * gx;
                CSgysq[i] += gy * gy;
                CSgxgy[i] += gx * gy;
            }
        }
    }
}

// Simple function to find the eigenvalues of the 2*2 symmetric matrix
// \f$ \begin{pmatrix}A & B \\ B & C \end{pmatrix} \nonumber \f$
// https://chatgpt.com/share/6aec7549-b158-83ec-b86b-e62a7202087f
void depth_cam_ekf::DepthCamSLAM::find_eigenvalues(double A, double B, double C,
                                double *eval1ptr, double *eval2ptr)
{
    double BB = sqrt((A + C) * (A + C) - 4 * (A * C - B * B));

    *eval1ptr = (A + C + BB) / 2.0;
    *eval2ptr = (A + C - BB) / 2.0;
}

// Initialise a feature at the currently-selected image location. The image
// selection is usually either chosen automatically with
// Robot::set_image_selection_automatically() or manually by the user by calling
// Robot::set_image_selection().
void depth_cam_ekf::DepthCamSLAM::InitialiseFeature(cv::Mat frame)
{
    // Fill in z and id for the currently-selected feature
    Eigen::VectorXd z(feature_model_->kMeasurementSize_);
    z << uu_, vv_, dd_;

    cv::Mat patch(kBoxSize_, kBoxSize_, frame.type());
    copy_into_patch(frame, patch);

    add_new_feature(patch, z);


}

// Little service function to copy image region centred at uu, vv into patch
void depth_cam_ekf::DepthCamSLAM::copy_into_patch(const cv::Mat frame, cv::Mat patch)
{
    int r_mul, v_mul;

    for (int r = 0; r < kBoxSize_; ++r) {

        r_mul = r * kBoxSize_;
        v_mul = (r + vv_ - (kBoxSize_-1) / 2) * frame.size().width;

        for (int c = 0; c < kBoxSize_; ++c) {
            *(patch.data +r_mul +c) = *(frame.data +v_mul +c + uu_ - (kBoxSize_-1) / 2);
        }
    }
}

void depth_cam_ekf::DepthCamSLAM::add_new_feature(
    cv::Mat patch, const Eigen::VectorXd &y) {
    Feature *nf = new Feature(patch, y.segment(0,2), y(2), this);

    feature_list_.push_back(nf);
    total_state_size_ += feature_model_->kFeatureStateSize_;
    ++next_free_label_; // Potential millenium-style bug when this overloads
}

void depth_cam_ekf::DepthCamSLAM::get_median_depth_around_a_pixel(const vector<float>& depths, int width, int height, int uu, int vv, double& med_depth) {
    const int BOX_LEN = 10;
    const int MAX_DEPTH = 5.0;
    const int MIN_DEPTH = 0.0;
    assert(depths.size() == width*height);
    assert (uu > BOX_LEN && uu < width-BOX_LEN && vv > BOX_LEN && vv < height-BOX_LEN);

    vector<float> sample;
    for (int i=-BOX_LEN; i<BOX_LEN; ++i) {
        for (int j=-BOX_LEN; j<BOX_LEN; ++j) {
            int x = uu + i;
            int y = vv + j;
            sample.push_back(depths[y*width + x]);
        }
    }

    sort(sample.begin(), sample.end());
    float temp = sample[sample.size()/2];
    if (temp < MAX_DEPTH && temp > MIN_DEPTH) {
        med_depth = temp;
    }
    else {
        med_depth = 0;
    }
}