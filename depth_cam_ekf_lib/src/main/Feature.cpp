//
// Created by Hasan on 11/10/26.
//

#include "Feature.h"
#include "DepthCamFeatureModel.h"
#include "DepthCamSLAM.h"

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

using namespace std;

depth_cam_ekf::Feature::Feature(cv::Mat patch,
                                const Eigen::VectorXd &h, double depth,
                                DepthCamSLAM *depth_cam_slam)
{
    feature_model_ = (DepthCamFeatureModel *)depth_cam_slam->feature_model_;

    Initialise();

    patch_ = patch;

    label_ = depth_cam_slam->next_free_label_;
    position_in_list_ = depth_cam_slam->feature_list_.size();
    position_in_total_state_vector_ = depth_cam_slam->total_state_size_;

    // Save the vehicle position where this feature was acquired
    depth_cam_slam->motion_model_->func_xp(depth_cam_slam->xv_);

    xp_org_.resize(depth_cam_slam->motion_model_->xpRES_.size());
    xp_org_ = depth_cam_slam->motion_model_->xpRES_;

    // Call model functions to calculate feature state, measurement noise
    // and associated Jacobians. Results are stored in RES matrices

    // First calculate "position state" and Jacobian
    depth_cam_slam->motion_model_->func_xp(depth_cam_slam->xv_);
    depth_cam_slam->motion_model_->func_dxp_by_dxv(depth_cam_slam->xv_);

    // Now ask the model to initialise the state vector and calculate Jacobians
    // so that I can go and calculate the covariance matrices
    depth_cam_slam->feature_model_->func_yi_and_dyi_by_dxp_and_dyi_by_dhdi_and_Ri(h, depth, depth_cam_slam->motion_model_->xpRES_);

    // State y
    y_.resize(depth_cam_slam->feature_model_->yiRES_.size());
    y_ = depth_cam_slam->feature_model_->yiRES_;

    // Temp_FS1 will store dypi_by_dxv
    Eigen::MatrixXd Temp_FS1 = depth_cam_slam->feature_model_->dyi_by_dxpRES_ * depth_cam_slam->motion_model_->dxp_by_dxvRES_;

    // Pxy
    Pxy_ = depth_cam_slam->Pxx_ * Temp_FS1.transpose();

    // Pyy
    Pyy_ = Temp_FS1 * depth_cam_slam->Pxx_ * Temp_FS1.transpose() +
            depth_cam_slam->feature_model_->dyi_by_dhdiRES_ *
            depth_cam_slam->feature_model_->RiRES_ *
            depth_cam_slam->feature_model_->dyi_by_dhdiRES_.transpose();


    // Covariances of this feature with others
    auto it = depth_cam_slam->feature_list_.begin();

    for (int j = 0; j < position_in_list_; ++j) {
        Eigen::MatrixXd newPyjyi_to_store = (Temp_FS1 * (*it)->Pxy_).transpose();

        matrix_block_list_.push_back(newPyjyi_to_store);

        ++it;
    }

}

depth_cam_ekf::Feature::~Feature()
{
}

void depth_cam_ekf::Feature::Initialise()
{
    selected_flag_ = false;        // Feature is unselected when first detected
    scheduled_for_termination_flag_ = false;
    attempted_measurements_of_feature_ = 0;
    successful_measurements_of_feature_ = 0;

    // Allocate matrices for storing predicted and actual measurements
    h_.resize(feature_model_->kMeasurementSize_);
    z_.resize(feature_model_->kMeasurementSize_);
    nu_.resize(feature_model_->kMeasurementSize_);

    dh_by_dxv_.resize(feature_model_->kMeasurementSize_, feature_model_->motion_model_->kStateSize_);
    dh_by_dy_.resize(feature_model_->kMeasurementSize_, feature_model_->kFeatureStateSize_);
    R_.resize(feature_model_->kMeasurementSize_, feature_model_->kMeasurementSize_);
    S_.resize(feature_model_->kMeasurementSize_, feature_model_->kMeasurementSize_);
}

