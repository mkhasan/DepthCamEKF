//
// Created by Hasan on 11/17/26.
//

#include "DepthCamFeatureModel.h"

depth_cam_ekf::DepthCamFeatureModel::DepthCamFeatureModel(int measurement_size, int feature_state_size,
                                                  int graphics_state_size, Camera *camera,
                                                  MotionModel *motion_model) :
    FeatureModel(measurement_size, feature_state_size, graphics_state_size, camera, motion_model),
    kMaximumLengthRatio_(2.0),
    kMaximumAngleDifference_(M_PI * 45.0 / 180.0),
    kImageSearchBoundary_(20.0)
{
    hiRES_.resize(measurement_size);
    hiRES_.setZero();
    nuiRES_.resize(measurement_size);
    nuiRES_.setZero();
    dhi_by_dxpRES_.resize(measurement_size,motion_model->kPositionStateSize_);
    dhi_by_dxpRES_.setZero();
    dhi_by_dyiRES_.resize(measurement_size,feature_state_size);
    dhi_by_dyiRES_.setZero();
}

depth_cam_ekf::DepthCamFeatureModel::~DepthCamFeatureModel()
{
}

void depth_cam_ekf::DepthCamFeatureModel::func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi(
    const Eigen::VectorXd &yi, const Eigen::VectorXd &xp)
{
    // Extract cartesian and quaternion components of xp
    motion_model_->func_r(xp);
    motion_model_->func_q(xp);

    Eigen::Vector3d yWiminusrW = yi - motion_model_->rRES_;

    Eigen::Quaterniond qRW = motion_model_->qRES_.inverse();
    Eigen::Matrix4d dqRW_by_dq = dqbar_by_dq();

    // Rotation RRW
    Eigen::Matrix3d RRW = qRW.toRotationMatrix();

    // Position of feature relative to robot in robot frame
    Eigen::Vector3d zeroedyi = RRW * yWiminusrW;
    zeroedyiRES_ = zeroedyi;

    // Now calculate Jacobians
    // dzeroedyi_by_dyi is RRW
    dzeroedyi_by_dyiRES_ = RRW;

    // dzeroedyi_by_dxp has 2 partitions:
    // dzeroedyi_by_dr (3 * 3)
    // dzeroedyi_by_dq (3 * 4)
    Eigen::Matrix3d dzeroedyi_by_dr = RRW;
    dzeroedyi_by_dr *= -1.0;    // jacobian of RRW*(yi - motion_model_->rRES_) wrt motion_model_->rRES_

    Eigen::Matrix<double, 3, 4> dzeroedyi_by_dqRW = dRq_times_a_by_dq(qRW, yWiminusrW);
    Eigen::Matrix<double, 3, 4> dzeroedyi_by_dq = dzeroedyi_by_dqRW * dqRW_by_dq;

    dzeroedyi_by_dxpRES_.block(0, 0, 3, 3) = dzeroedyi_by_dr;
    dzeroedyi_by_dxpRES_.block(0, 3, 3, 4) = dzeroedyi_by_dq;
}

int depth_cam_ekf::DepthCamFeatureModel::visibility_test(const Eigen::VectorXd &xp,
                                                     const Eigen::VectorXd &yi,
                                                     const Eigen::VectorXd &xp_orig,
                                                     const Eigen::VectorXd &hi)
{
    int cant_see_flag = 0;

    // Test image boundaries
    if (hi(0) < 0.0 + kImageSearchBoundary_ ||
        hi(0) > (double)(camera_->width_ - 1 - kImageSearchBoundary_)) {
        cant_see_flag |= kLeftRightFail_;
    }
    if (hi(1) < 0.0 + kImageSearchBoundary_ ||
        hi(1) > (double)(camera_->height_ - 1 - kImageSearchBoundary_)) {
        cant_see_flag |= kUpDownFail_;
    }

    // Do tests on length and angle of predicted view

    // hLWi is current predicted vector from head to feature in
    // world frame

    // This function gives relative position of feature
    func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi(yi, xp);

    // Test the feature's not behind the camera (because projection
    // may do strange things)
    if (zeroedyiRES_(2) <= 0) {
        cant_see_flag |= kBehindCameraFail_;
    }

    motion_model_->func_q(xp);
    Eigen::Matrix3d RWR = motion_model_->qRES_.toRotationMatrix();

    Eigen::Vector3d hLWi = RWR * Eigen::Vector3d(zeroedyiRES_);

    // hLWi_orig is vector from head to feature in world frame
    // WHEN THAT FEATURE WAS FIRST MEASURED: i.e. when the image
    // patch was saved
    func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi(yi, xp_orig);

    motion_model_->func_q(xp_orig);
    Eigen::Matrix3d RWR_orig = motion_model_->qRES_.toRotationMatrix();

    Eigen::Vector3d hLWi_orig = RWR_orig * Eigen::Vector3d(zeroedyiRES_);

    // Compare hLWi and hLWi_orig for length and orientation
    double mod_hLWi = hLWi.norm();
    double mod_hLWi_orig = hLWi_orig.norm();

    double length_ratio = mod_hLWi / mod_hLWi_orig;

    if (length_ratio > kMaximumLengthRatio_ ||
        length_ratio < (1.0 / kMaximumLengthRatio_)) {
        cant_see_flag |= kDistanceFail_;
    }

    double dot_prod = hLWi.dot(hLWi_orig);

    double angle = acos(dot_prod / (mod_hLWi * mod_hLWi_orig));
    angle = (angle >= 0.0 ? angle : -angle);  // Make angle positive

    if (angle > kMaximumAngleDifference_) {
        cant_see_flag |= kAngleFail_;
    }

    return cant_see_flag;   // 0 if OK, otherwise error code
}

double depth_cam_ekf::DepthCamFeatureModel::selection_score(const Eigen::MatrixXd &Si)
{
    // Return the trace of the innovation covariance
    return Si.trace();
}

void depth_cam_ekf::DepthCamFeatureModel::func_hi_and_dhi_by_dxp_and_dhi_by_dyi(const Eigen::VectorXd &yi,
                                                                            const Eigen::VectorXd &xp)
{
    // This function gives relative position of feature: also call this hR
    // (vector from camera to feature in robot frame)
    func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi(yi, xp);

    // Project this into the image using our camera
    hiRES_.segment(0, 2) = camera_->Project(zeroedyiRES_);
    hiRES_(2) = zeroedyiRES_(2);

    // And ask the camera what the Jacobian of this projection was
    Eigen::MatrixXd dhid_by_dzeroedyi(3,3);
    Eigen::Matrix<double, 1, 3> temp;
    temp << 0.0, 0.0, 1.0;

    dhid_by_dzeroedyi.block(0, 0, 2, 3) = camera_->ProjectionJacobian();
    dhid_by_dzeroedyi.block(2, 0, 1, 3) = temp;

    // Form the required Jacobians
    dhi_by_dxpRES_ = dhid_by_dzeroedyi * dzeroedyi_by_dxpRES_;
    dhi_by_dyiRES_ = dhid_by_dzeroedyi * dzeroedyi_by_dyiRES_;
}

void depth_cam_ekf::DepthCamFeatureModel::func_nui(const Eigen::VectorXd &hi, const Eigen::VectorXd &zi)
{
    nuiRES_ = zi - hi;
}
