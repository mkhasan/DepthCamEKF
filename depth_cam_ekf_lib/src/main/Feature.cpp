//
// Created by Hasan on 11/10/26.
//

#include "Feature.h"
#include "PartFeatureModel.h"
#include "DepthCamSLAM.h"

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

using namespace std;

depth_cam_ekf::Feature::Feature(cv::Mat patch,
                                const Eigen::VectorXd &h,
                                DepthCamSLAM *depth_cam_slam)
{
    // Need to get FeatureModel before calling Initialise()
    fully_initialised_flag_ = false;
    feature_model_ = (PartFeatureModel *)depth_cam_slam->part_feature_model_;

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
    depth_cam_slam->part_feature_model_->func_ypi_and_dypi_by_dxp_and_dypi_by_dhi_and_Ri(h, depth_cam_slam->motion_model_->xpRES_);

    // State y
    y_.resize(depth_cam_slam->part_feature_model_->ypiRES_.size());
    y_ = depth_cam_slam->part_feature_model_->ypiRES_;

    // Temp_FS1 will store dypi_by_dxv
    Eigen::MatrixXd Temp_FS1 = depth_cam_slam->part_feature_model_->dypi_by_dxpRES_ * depth_cam_slam->motion_model_->dxp_by_dxvRES_;

    // Pxy
    Pxy_ = depth_cam_slam->Pxx_ * Temp_FS1.transpose();

    // Pyy
    Pyy_ = Temp_FS1 * depth_cam_slam->Pxx_ * Temp_FS1.transpose() +
            depth_cam_slam->part_feature_model_->dypi_by_dhiRES_ *
            depth_cam_slam->part_feature_model_->RiRES_ *
            depth_cam_slam->part_feature_model_->dypi_by_dhiRES_.transpose();

    // Covariances of this feature with others
    vector<Feature *>::iterator it = depth_cam_slam->feature_list_.begin();

    for (int j = 0; j < position_in_list_; ++j) {
        Eigen::MatrixXd newPyjypi_to_store = (Temp_FS1 * (*it)->Pxy_).transpose();

        matrix_block_list_.push_back(newPyjypi_to_store);

        ++it;
    }
}

// Constructor for known features. The different number of
// arguments differentiates it from the constructor for partially-initialised features
depth_cam_ekf::Feature::Feature(FeatureModel *feature_model,
                                const Eigen::VectorXd &y, const Eigen::VectorXd &xp,
                                const int label, const int total_state_size, const string &identifier,
                                vector<Feature *> feature_list)
{
    // Need to get FeatureModel before calling Initialise()
    fully_initialised_flag_ = true;
    feature_model_ = (FullFeatureModel *)feature_model;

    Initialise();

    patch_ = cv::imread(identifier, 0);

    label_ = label;
    position_in_list_ = feature_list.size();  // Position of new feature in list
    position_in_total_state_vector_ = total_state_size;

    // Save the vehicle position where this feature was acquired
    xp_org_.resize(xp.size());
    xp_org_ = xp;

    // Straighforward initialisation of state and covariances
    y_.resize(y.size());
    y_ = y;
    Pxy_.resize(feature_model_->motion_model_->kStateSize_, feature_model_->kFeatureStateSize_);
    Pxy_.setZero();
    Pyy_.resize(feature_model_->kFeatureStateSize_, feature_model_->kFeatureStateSize_);
    Pyy_.setZero();

    vector<Feature *>::iterator it = feature_list.begin();

    // Fill covariance cross-terms with zero
    for (int i = 0; i < position_in_list_; ++i) {
        Eigen::MatrixXd newPyjyi_to_store((*it)->feature_model_->kFeatureStateSize_,
                                        feature_model_->kFeatureStateSize_);
        newPyjyi_to_store.setZero();

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

// Convert a partially-initialised feature to a fully-initialised feature,
// given information about the free parameters \f$ \vct{\lambda} \f$.
// @param lambda The mean value for \f$ \vct{\lambda} \f$
// @param Plambda The covariance for \f$ \vct{\lambda} \f$
// @param scene The SLAM map
//
// The new state \f$ \vct{y}_{fi} \f$ is given by calling
// Partially_Initialised_Feature_Measurement_Model::func_yfi_and_dyfi_by_dypi_and_dyfi_by_dlambda().
// The new feature covariances are
// \f[
// \begin{aligned}
// \mat{P}_{\vct{y}_{fi}\vct{y}_{fi}} &=
// \partfrac{\vct{y}_{fi}}{\vct{y}_{pi}}
// \mat{P}_{\vct{y}_{pi}\vct{y}_{pi}} \partfrac{\vct{y}_{fi}}{\vct{y}_{pi}}^T +
// \partfrac{\vct{y}_{fi}}{\vct{\lambda}} \mat{P}_{\vct{\lambda}}
// \partfrac{\vct{y}_{fi}}{\vct{\lambda}}^T
//
// \mat{P}_{\vct{x}\vct{y}_{fi}} &=
// \mat{P}_{\vct{x}\vct{y}_{pi}} \partfrac{\vct{y}_{fi}}{\vct{y}_{pi}}^T
//
// \mat{P}_{\vct{y}_j\vct{y}_{fi}} &=
// \mat{P}_{\vct{y}_j\vct{y}_{pi}} \partfrac{\vct{y}_{fi}}{\vct{y}_{pi}}^T
//
// \mat{P}_{\vct{y}_{fi}\vct{y}_{j}} &=
// \partfrac{\vct{y}_{pi}}{\vct{y}_{j}} \mat{P}_{\vct{y}_fi\vct{y}_{pi}}
// \end{aligned}
// \f]
// where the various Jacobians are returned by calls to
// Partially_Initialised_Feature_Measurement_Model, and the covariance matrices
// \f$ \mat{P}_{kl} \f$ are already known and stored in the class, except for \f$
// \mat{P}_{\vct{\lambda}} \f$, which is passed to the function.
void depth_cam_ekf::Feature::convert_from_partially_to_fully_initialised(
    const Eigen::VectorXd &lambda, const Eigen::MatrixXd &Plambda,
    DepthCamSLAM *depth_cam_slam)
{
    // We'll do all the work here in feature.cc though probably this only
    // works with scene_single...

    // We calculate new state yfi(ypi, lambda)
    // New feature covariance
    // Pyfiyfi = dyfi_by_dypi Pypiypi dyfi_by_dypiT +
    //           dyfi_by_dlambda Plambda dyfi_by_dlambdaT
    // And we change cross covariances as follows:
    // Pxyfi = Pxypi dyfi_by_dypiT
    // Pyjyfi = Pyjypi dyfi_by_dypiT   for j < i (since we only store top-right
    // Pyfiyj = dyfi_by_dypi Pypiyj    for j > i  part of covariance matrix)
    depth_cam_slam->part_feature_model_->func_yfi_and_dyfi_by_dypi_and_dyfi_by_dlambda(y_, lambda);

    Eigen::MatrixXd dyfi_by_dypiT = depth_cam_slam->part_feature_model_->dyfi_by_dypiRES_.transpose();
    Eigen::MatrixXd dyfi_by_dlambdaT = depth_cam_slam->part_feature_model_->dyfi_by_dlambdaRES_.transpose();

    // Replace y
    y_ = depth_cam_slam->part_feature_model_->yfiRES_;

    // Replace Pxy
    Pxy_ = Pxy_ * dyfi_by_dypiT;

    // Replace Pyy
    Eigen::MatrixXd Pypiypi_1 = depth_cam_slam->part_feature_model_->dyfi_by_dypiRES_ * Pyy_ * dyfi_by_dypiT;
    Eigen::MatrixXd Pypiypi_2 = depth_cam_slam->part_feature_model_->dyfi_by_dlambdaRES_ * Plambda * dyfi_by_dlambdaT;
    Pyy_ = Pypiypi_1 + Pypiypi_2;

    // Pyjyi elements for j < i (covariance with features before i in list)
    const int i = position_in_list_;

    for (vector<Eigen::MatrixXd>::iterator it = matrix_block_list_.begin();
        it != matrix_block_list_.begin() + position_in_list_; ++it) {
        *it = *it * dyfi_by_dypiT;
    }

    // Pyjyi elements for j > i (covariance with features after i in list)
    vector<Feature *>::iterator it = depth_cam_slam->feature_list_.begin();

    // Start at the element after the current feature
    while (*it != this)
        ++it; // now pointing at current feature

    ++it; // now pointing at the next one

    for (; it != depth_cam_slam->feature_list_.end(); ++it) {
        (*it)->matrix_block_list_[i] = depth_cam_slam->part_feature_model_->dyfi_by_dypiRES_ * (*it)->matrix_block_list_[i];
        (*it)->position_in_total_state_vector_ -= feature_model_->kFeatureStateSize_;
    }

    // Change the total state size in scene, here with a negative increment
    depth_cam_slam->total_state_size_ += (depth_cam_slam->full_feature_model_->kFeatureStateSize_ -
                                    depth_cam_slam->part_feature_model_->kFeatureStateSize_);

    // Change fmm for this model to fully-initialised one
    feature_model_ = depth_cam_slam->full_feature_model_;

    // Need to reallocate any other matrices
    // Assume that measurement size doesn't change
    dh_by_dy_.resize(feature_model_->kMeasurementSize_, feature_model_->kFeatureStateSize_);

    fully_initialised_flag_ = true;
}
