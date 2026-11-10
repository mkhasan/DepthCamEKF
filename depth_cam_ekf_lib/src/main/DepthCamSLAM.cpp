//
// Created by Hasan on 11/9/26.
//


#include  "Params.h"
#include "DepthCamSLAM.h"
#include <pangolin/pangolin.h>
#include <string>


namespace depth_cam_ekf {

using namespace std;

DepthCamSLAM::DepthCamSLAM() {

}

DepthCamSLAM::~DepthCamSLAM() {

}

void DepthCamSLAM::Init(const string &config_path) {

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
    pangolin::Var<int>    cam_sd("cam.sd", 0);

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

}

bool DepthCamSLAM::GoOneStep(cv::Mat frame, bool save_trajectory, bool enable_mapping) {
    return true;
}

}