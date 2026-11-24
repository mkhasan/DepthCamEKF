//
// Created by Hasan on 11/10/26.
//

#include "MotionModel.h"

#include "support/math_util.h"

depth_cam_ekf::MotionModel::MotionModel()
    : phiRES_(0), kPositionStateSize_(3), kStateSize_(6), kControlSize_(2), // z, x and phi
      kSdAComponentFilter_(4.0), kSdAlphaComponentFilter_(6.0) {
    fvRES_.resize(kStateSize_);
    dfv_by_dxvRES_.resize(kStateSize_, kStateSize_);
    QxRES_.resize(kStateSize_, kStateSize_);
    xpRES_.resize(kPositionStateSize_);
    dxp_by_dxvRES_.resize(kPositionStateSize_, kStateSize_);
    xvnormRES_.resize(kStateSize_);
    dxvnorm_by_dxvRES_.resize(kStateSize_, kStateSize_);

    dr_by_dxpRES_.resize(2, 3); // equivalent to VNL::MatrixFixed<3,7,double>
}

depth_cam_ekf::MotionModel::~MotionModel()
{
}

// Extract cartesian part of position state vector
void depth_cam_ekf::MotionModel::func_r(const Eigen::VectorXd &xp)
{
    rRES_ << xp(0), xp(1);
}

// And Jacobian
void depth_cam_ekf::MotionModel::func_dr_by_dxp(const Eigen::VectorXd &xp)
{
    dr_by_dxpRES_.setZero();

    dr_by_dxpRES_(0, 0) = 1.0;
    dr_by_dxpRES_(1, 1) = 1.0;
}

// Extract quaternion part of position state vector
void depth_cam_ekf::MotionModel::func_phi(const Eigen::VectorXd &xp)
{
    phiRES_ = xp[2];
}

void depth_cam_ekf::MotionModel::func_fv_and_dfv_by_dxv(const Eigen::VectorXd &xv,
                                                        const Eigen::VectorXd &u,
                                                        const double delta_t)
{
    double phiold, phinew, omegaold, omeganew;
    Eigen::Vector2d rold, vold, rnew, vnew;


    // Separate things out to make it clearer
    extract_r_phi_v_omega(xv, rold, phiold, vold, omegaold);

    Eigen::Vector2d acceleration(u);

    // rnew = r + v * delta_t
    rnew = rold + vold * delta_t;

    phinew = phiold + omegaold * delta_t;

    // vnew = v
    vnew = vold + acceleration * delta_t;

    // omeganew = omega
    omeganew = omegaold;

    // Put it all together
    compose_xv(rnew, phinew, vnew, omeganew, fvRES_);

    // Now on to the Jacobian...
    // Identity is a good place to start since overall structure is like this
    // I       0             I * delta_t   0
    // 0       1             0             delta_t
    // 0       0             I             0
    // 0       0             0             1

    //https://chatgpt.com/share/6b04d92b-8aec-83ec-980d-1b84f7158855
    dfv_by_dxvRES_.setIdentity();

    // Fill in dxnew_by_dv = I * delta_t
    Eigen::Matrix3d Temp33A;

    Temp33A.setIdentity();
    Temp33A *= delta_t;
    dfv_by_dxvRES_.block(0, 3, 3, 3) = Temp33A;

}

void depth_cam_ekf::MotionModel::func_Q(const Eigen::VectorXd &xv, const Eigen::VectorXd &u, const double delta_t)
{
    // Fill noise covariance matrix Pnn: this is the covariance of
    // the noise vector (V)
    //                  (Omega)
    // that gets added to the state.
    // Form of this could change later, but for now assume that
    // V and Omega are independent, and that each of their components is
    // independent...
    double linear_velocity_noise_variance = kSdAComponentFilter_ * kSdAComponentFilter_ *
                                            delta_t * delta_t;
    double angular_velocity_noise_variance = kSdAlphaComponentFilter_ * kSdAlphaComponentFilter_ *
                                            delta_t * delta_t;

    // Independence means that the matrix is diagonal
    Eigen::MatrixXd Pnn(3,3);
    Pnn.setZero();
    Pnn(0,0) = linear_velocity_noise_variance;
    Pnn(1,1) = linear_velocity_noise_variance;
    Pnn(2,2) = angular_velocity_noise_variance;

    // Form Jacobian dxnew_by_dn
    // Is like this:
    // I * delta_t     0
    // 0               delta_t
    // I               0
    // 0               1

    // Start by zeroing
    Eigen::MatrixXd dxnew_by_dn(6,3);
    dxnew_by_dn.setZero();

    // Fill in easy bits first
    Eigen::Matrix3d Temp33A;

    Temp33A.setIdentity();

    dxnew_by_dn.block(3, 0, 3, 3) = Temp33A;
    Temp33A *= delta_t;
    dxnew_by_dn.block(0, 0, 3, 3) = Temp33A;

    // Finally do Q = dxnew_by_dn . Pnn . dxnew_by_dnT
    QxRES_ = dxnew_by_dn * Pnn * dxnew_by_dn.transpose();
}

void depth_cam_ekf::MotionModel::func_xp(const Eigen::VectorXd &xv)
{
    xpRES_ << xv(0), xv(1), xv(2);
}

void depth_cam_ekf::MotionModel::func_dxp_by_dxv(const Eigen::VectorXd &xv)
{
    dxp_by_dxvRES_.setZero();

    dxp_by_dxvRES_(0,0) = 1.0;
    dxp_by_dxvRES_(1,1) = 1.0;
    dxp_by_dxvRES_(2,2) = 1.0;

}


// Easy access to state blocks: fill matrices r, q, v, omega with
// values based on state xv
void depth_cam_ekf::MotionModel::extract_r_phi_v_omega(const Eigen::VectorXd &xv, Eigen::Vector2d &r,
                                double &phi, Eigen::Vector2d &v,
                                double &omega)
{
    r << xv(0),xv(1);

    phi = xv(2);

    v << xv(3),xv(4);
    omega = xv(5);
}

// The opposite: put r, q, v, omega back into vector xv
void depth_cam_ekf::MotionModel::compose_xv(const Eigen::Vector2d &r, const double &phi,
                        const Eigen::Vector2d &v, const double &omega,
                        Eigen::VectorXd &xv)
{
    xv << r, phi, v, omega;
}


