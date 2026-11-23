//
// Created by Hasan on 11/20/26.
//

#include "Kalman.h"

using namespace std;

depth_cam_ekf::Kalman::Kalman()
{

}

depth_cam_ekf::Kalman::~Kalman() {
}

void depth_cam_ekf::Kalman::KalmanFilterPredict(DepthCamSLAM *depth_cam_slam, Eigen::Vector3d &u)
{
    // Make model calculations: results are stored in RES matrices
    depth_cam_slam->motion_model_->func_fv_and_dfv_by_dxv(depth_cam_slam->xv_, u,
                                                    depth_cam_slam->kDeltaT_);
    depth_cam_slam->motion_model_->func_Q(depth_cam_slam->xv_, u, depth_cam_slam->kDeltaT_);

    depth_cam_slam->xv_ = depth_cam_slam->motion_model_->fvRES_;

    depth_cam_slam->Pxx_ = depth_cam_slam->motion_model_->dfv_by_dxvRES_ *
                    depth_cam_slam->Pxx_ *
                    depth_cam_slam->motion_model_->dfv_by_dxvRES_.transpose() +
                    depth_cam_slam->motion_model_->QxRES_;

    // Change the covariances between vehicle state and feature states
    for (vector<Feature *>::iterator it = depth_cam_slam->feature_list_.begin();
        it != depth_cam_slam->feature_list_.end(); ++it) {
        (*it)->Pxy_ = depth_cam_slam->motion_model_->dfv_by_dxvRES_ * (*it)->Pxy_;
    }
}

// Update the filter in a simple overall way
void depth_cam_ekf::Kalman::KalmanFilterUpdate(DepthCamSLAM *depth_cam_slam)
{
    // Steps to update the total filter:
    //   1. Form h and dh_by_dx and R(k+1) and z
    //   2. Calculate S(k+1)
    //   3. Calculate W(k+1)
    //   4. Calculate x(k+1|k+1)
    //   5. Calculate P(k+1|k+1)
    int size = depth_cam_slam->successful_measurement_vector_size_;
    int size2 = depth_cam_slam->total_state_size_;

    Eigen::VectorXd x(size2);
    Eigen::MatrixXd P(size2, size2);

    x.setZero();
    depth_cam_slam->construct_total_state(x);

    P.setZero();
    depth_cam_slam->construct_total_covariance(P);

    // 1. Form nu and dh_by_dx
    Eigen::VectorXd nu_tot(size);
    Eigen::MatrixXd dh_by_dx_tot(size, size2);
    Eigen::MatrixXd R_tot(size, size);

    depth_cam_slam->construct_total_measurement_stuff(nu_tot, dh_by_dx_tot, R_tot);

    // 2. Calculate S(k+1)
    Eigen::MatrixXd S = dh_by_dx_tot * P * dh_by_dx_tot.transpose();
    S += R_tot;

    // 3. Calculate W(k+1)
    Eigen::LLT<Eigen::MatrixXd>  S_cholesky(S);
    Eigen::MatrixXd S_L = S_cholesky.matrixL();
    Eigen::MatrixXd S_Linv = S_L.inverse();
    Eigen::MatrixXd Sinv = S_Linv.transpose() * S_Linv;

    Eigen::MatrixXd W = P * dh_by_dx_tot.transpose() * Sinv;

    // 4. Calculate x(k+1|k+1)
    x += W * nu_tot;

    // 5. Calculate P(k+1|k+1)
    P -= W * S * W.transpose();

    depth_cam_slam->fill_states(x);
    depth_cam_slam->fill_covariances(P);
}



