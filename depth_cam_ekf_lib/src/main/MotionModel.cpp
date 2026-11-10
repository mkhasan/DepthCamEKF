//
// Created by Hasan on 11/10/26.
//

#include "MotionModel.h"

#include "support/math_util.h"

depth_cam_ekf::MotionModel::MotionModel()
    : kPositionStateSize_(7), kStateSize_(13), kControlSize_(3),
        kSdAComponentFilter_(4.0), kSdAlphaComponentFilter_(6.0)
{
    fvRES_.resize(kStateSize_);
    dfv_by_dxvRES_.resize(kStateSize_, kStateSize_);
    QxRES_.resize(kStateSize_, kStateSize_);
    xpRES_.resize(kPositionStateSize_);
    dxp_by_dxvRES_.resize(kPositionStateSize_, kStateSize_);
    xvnormRES_.resize(kStateSize_);
    dxvnorm_by_dxvRES_.resize(kStateSize_, kStateSize_);

    dr_by_dxpRES_.resize(3,7);  // equivalent to VNL::MatrixFixed<3,7,double>
}

depth_cam_ekf::MotionModel::~MotionModel()
{
}

// Extract cartesian part of position state vector
void depth_cam_ekf::MotionModel::func_r(const Eigen::VectorXd &xp)
{
    rRES_ << xp(0), xp(1), xp(2);
}

// And Jacobian
void depth_cam_ekf::MotionModel::func_dr_by_dxp(const Eigen::VectorXd &xp)
{
    dr_by_dxpRES_.setZero();

    dr_by_dxpRES_(0, 0) = 1.0;
    dr_by_dxpRES_(1, 1) = 1.0;
    dr_by_dxpRES_(2, 2) = 1.0;
}

// Extract quaternion part of position state vector
void depth_cam_ekf::MotionModel::func_q(const Eigen::VectorXd &xp)
{
    qRES_ = Eigen::Quaterniond(xp(3), xp(4), xp(5), xp(6));
}

void depth_cam_ekf::MotionModel::func_fv_and_dfv_by_dxv(const Eigen::VectorXd &xv,
                                                        const Eigen::VectorXd &u,
                                                        const double delta_t)
{
    Eigen::Vector3d rold, vold, omegaold, rnew, vnew, omeganew;
    Eigen::Quaterniond qold, qnew;

    // Separate things out to make it clearer
    extract_r_q_v_omega(xv, rold, qold, vold, omegaold);

    Eigen::Vector3d acceleration(u);

    // rnew = r + v * delta_t
    rnew = rold + vold * delta_t;

    // qnew = q x q(omega * delta_t)
    // Keep qwt ( = q(omega * delta_t)) for later use
    Eigen::Quaterniond qwt = depth_cam_ekf::QuaternionFromAngularVelocity(omegaold * delta_t);
    qnew = qold * qwt;

    // vnew = v
    vnew = vold + acceleration * delta_t;

    // omeganew = omega
    omeganew = omegaold;

    // Put it all together
    compose_xv(rnew, qnew, vnew, omeganew, fvRES_);

    // Now on to the Jacobian...
    // Identity is a good place to start since overall structure is like this
    // I       0             I * delta_t   0
    // 0       dqnew_by_dq   0             dqnew_by_domega
    // 0       0             I             0
    // 0       0             0             I
    dfv_by_dxvRES_.setIdentity();

    // Fill in dxnew_by_dv = I * delta_t
    Eigen::Matrix3d Temp33A;

    // https://chatgpt.com/share/6ad18d71-4c14-83ee-8994-1147ae5676c9

    Temp33A.setIdentity();
    Temp33A *= delta_t;
    dfv_by_dxvRES_.block(0, 7, 3, 3) = Temp33A;

    // Fill in dqnew_by_dq
    // qnew = qold x qwt  ( = q3 = q2 x q1 in Scene/newbits.cc language)
    Eigen::Matrix4d Temp44A = dq3_by_dq2(qwt);
    dfv_by_dxvRES_.block(3, 3, 4, 4) = Temp44A;

    // Fill in dqnew_by_domega = d(q x qwt)_by_dqwt . dqwt_by_domega
    Temp44A = dq3_by_dq1(qold); // Temp44A is d(q x qwt) by dqwt

    // Use function below for dqwt_by_domega
    Eigen::MatrixXd Temp43A(4,3);
    dqomegadt_by_domega(omegaold, delta_t, Temp43A);

    // Multiply them together
    Eigen::MatrixXd Temp43B(4,3);
    Temp43B = Temp44A * Temp43A;

    // And plug it in
    dfv_by_dxvRES_.block(3, 10, 4, 3) = Temp43B;
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
    Eigen::MatrixXd Pnn(6,6);
    Pnn.setZero();
    Pnn(0,0) = linear_velocity_noise_variance;
    Pnn(1,1) = linear_velocity_noise_variance;
    Pnn(2,2) = linear_velocity_noise_variance;
    Pnn(3,3) = angular_velocity_noise_variance;
    Pnn(4,4) = angular_velocity_noise_variance;
    Pnn(5,5) = angular_velocity_noise_variance;

    // Form Jacobian dxnew_by_dn
    // Is like this:
    // I * delta_t     0
    // 0               dqnew_by_dOmega
    // I               0
    // 0               I

    // Start by zeroing
    Eigen::MatrixXd dxnew_by_dn(13,6);
    dxnew_by_dn.setZero();

    // Fill in easy bits first
    Eigen::Matrix3d Temp33A;

    Temp33A.setIdentity();

    dxnew_by_dn.block(7, 0, 3, 3) = Temp33A;
    dxnew_by_dn.block(10, 3, 3, 3) = Temp33A;
    Temp33A *= delta_t;
    dxnew_by_dn.block(0, 0, 3, 3) = Temp33A;

    // Tricky bit is dqnew_by_dOmega
    // Is actually the same calculation as in func_fv...
    // Since omega and Omega are additive...?
    Eigen::Vector3d rold, vold, omegaold;
    Eigen::Quaterniond qold;
    extract_r_q_v_omega(xv, rold, qold, vold, omegaold); // overkill but easy

    // Fill in dqnew_by_domega = d(q x qwt)_by_dqwt . dqwt_by_domega
    // Temp44A is d(q x qwt) by dqwt
    Eigen::Matrix4d Temp44A = dq3_by_dq1(qold);

    // Use function below for dqwt_by_domega
    Eigen::MatrixXd Temp43A(4,3);
    dqomegadt_by_domega(omegaold, delta_t, Temp43A);

    // Multiply them together
    Eigen::MatrixXd Temp43B(4,3);
    Temp43B = Temp44A * Temp43A;

    // And then plug into Jacobian
    dxnew_by_dn.block(3, 3, 4, 3) = Temp43B;

    // Finally do Q = dxnew_by_dn . Pnn . dxnew_by_dnT
    QxRES_ = dxnew_by_dn * Pnn * dxnew_by_dn.transpose();
}

void depth_cam_ekf::MotionModel::func_xp(const Eigen::VectorXd &xv)
{
    xpRES_ << xv(0), xv(1), xv(2), xv(3), xv(4), xv(5), xv(6);
}

void depth_cam_ekf::MotionModel::func_dxp_by_dxv(const Eigen::VectorXd &xv)
{
    dxp_by_dxvRES_.setZero();

    dxp_by_dxvRES_(0,0) = 1.0;
    dxp_by_dxvRES_(1,1) = 1.0;
    dxp_by_dxvRES_(2,2) = 1.0;
    dxp_by_dxvRES_(3,3) = 1.0;
    dxp_by_dxvRES_(4,4) = 1.0;
    dxp_by_dxvRES_(5,5) = 1.0;
    dxp_by_dxvRES_(6,6) = 1.0;
}

void depth_cam_ekf::MotionModel::func_xvnorm_and_dxvnorm_by_dxv(const Eigen::VectorXd &xv)
{
    // Normalise the state vector: since quaternion is redundant we sometimes
    // need to enforce that it stays with size 1

    // Most parts of the state vector don't change so copy as starting point
    xvnormRES_ = xv;

    // Most parts of Jacobian are identity
    dxvnorm_by_dxvRES_.setIdentity();

    // Extract quaternion
    func_xp(xv);
    func_q(xpRES_);

    Eigen::Quaterniond Tempqa = qRES_;

    Eigen::Quaterniond Tempqb = Tempqa;
    Eigen::Matrix4d Temp44a = dqnorm_by_dq(Tempqa); // check it later

    xvnormRES_(3) = Tempqb.w();
    xvnormRES_(4) = Tempqb.x();
    xvnormRES_(5) = Tempqb.y();
    xvnormRES_(6) = Tempqb.z();

    dxvnorm_by_dxvRES_.block(3,3,4,4) = Temp44a;
}

// Easy access to state blocks: fill matrices r, q, v, omega with
// values based on state xv
void depth_cam_ekf::MotionModel::extract_r_q_v_omega(const Eigen::VectorXd &xv, Eigen::Vector3d &r,
                                                     Eigen::Quaterniond &q, Eigen::Vector3d &v,
                                                     Eigen::Vector3d &omega)
{
    r << xv(0),xv(1),xv(2);

    q.w() = xv(3);
    q.x() = xv(4);
    q.y() = xv(5);
    q.z() = xv(6);

    v << xv(7),xv(8),xv(9);
    omega << xv(10),xv(11),xv(12);
}

// The opposite: put r, q, v, omega back into vector xv
void depth_cam_ekf::MotionModel::compose_xv(const Eigen::Vector3d &r, const Eigen::Quaterniond &q,
                                            const Eigen::Vector3d &v, const Eigen::Vector3d &omega,
                                            Eigen::VectorXd &xv)
{
    xv << r, q.w(), q.x(), q.y(), q.z(), v, omega;
}

void depth_cam_ekf::MotionModel::dqomegadt_by_domega(const Eigen::Vector3d &omega,
                                                     const double delta_t,
                                                     Eigen::MatrixXd &dqomegadt_by_domega)
{
    // Modulus
    double omegamod = sqrt(omega(0) * omega(0) + omega(1) * omega(1) +
                            omega(2) * omega(2));

    // Use generic ancillary functions to calculate components of Jacobian
    dqomegadt_by_domega(0, 0) = dq0_by_domegaA(omega(0), omegamod, delta_t);
    dqomegadt_by_domega(0, 1) = dq0_by_domegaA(omega(1), omegamod, delta_t);
    dqomegadt_by_domega(0, 2) = dq0_by_domegaA(omega(2), omegamod, delta_t);
    dqomegadt_by_domega(1, 0) = dqA_by_domegaA(omega(0), omegamod, delta_t);
    dqomegadt_by_domega(1, 1) = dqA_by_domegaB(omega(0), omega(1), omegamod, delta_t);
    dqomegadt_by_domega(1, 2) = dqA_by_domegaB(omega(0), omega(2), omegamod, delta_t);
    dqomegadt_by_domega(2, 0) = dqA_by_domegaB(omega(1), omega(0), omegamod, delta_t);
    dqomegadt_by_domega(2, 1) = dqA_by_domegaA(omega(1), omegamod, delta_t);
    dqomegadt_by_domega(2, 2) = dqA_by_domegaB(omega(1), omega(2), omegamod, delta_t);
    dqomegadt_by_domega(3, 0) = dqA_by_domegaB(omega(2), omega(0), omegamod, delta_t);
    dqomegadt_by_domega(3, 1) = dqA_by_domegaB(omega(2), omega(1), omegamod, delta_t);
    dqomegadt_by_domega(3, 2) = dqA_by_domegaA(omega(2), omegamod, delta_t);
}

//
// DQ0 BY DOMEGAA
// Ancillary function to calculate part of Jacobian \f$ \partial q / \partial
// \omega \f$ which is repeatable due to symmetry. Here omegaA is one of omegax,
// omegay, omegaz.
double depth_cam_ekf::MotionModel::dq0_by_domegaA(const double omegaA, const double omega,
                                                  const double delta_t)
{
    return (-delta_t / 2.0) * (omegaA / omega) * sin(omega * delta_t / 2.0);
}

//
// DQA BY DOMEGAA
// Ancillary function to calculate part of Jacobian \f$ \partial q / \partial
// \omega \f$ which is repeatable due to symmetry. Here omegaA is one of omegax,
// omegay, omegaz and similarly with qA.
double depth_cam_ekf::MotionModel::dqA_by_domegaA(const double omegaA, const double omega,
                                                  const double delta_t)
{
    return (delta_t / 2.0) * omegaA * omegaA / (omega * omega)
        * cos(omega * delta_t / 2.0)
        + (1.0 / omega) * (1.0 - omegaA * omegaA / (omega * omega))
        * sin(omega * delta_t / 2.0);
}

//
// DQA BY DOMEGAB
// Ancillary function to calculate part of Jacobian \f$ \partial q / \partial
// \omega \f$ which is repeatable due to symmetry. Here omegaB is one of omegax,
// omegay, omegaz and similarly with qA.
double depth_cam_ekf::MotionModel::dqA_by_domegaB(const double omegaA, const double omegaB,
                                                  const double omega, double delta_t)
{
    return (omegaA * omegaB / (omega * omega)) *
        ( (delta_t / 2.0) * cos(omega * delta_t / 2.0)
        - (1.0 / omega) * sin(omega * delta_t / 2.0) );
}

Eigen::Matrix4d depth_cam_ekf::MotionModel::dqnorm_by_dq(const Eigen::Quaterniond &q)
{
    Eigen::Matrix4d M;

    double qq = q.w()*q.w() + q.x()*q.x() + q.y()*q.y() + q.z()*q.z();

    M << dqi_by_dqi(q.w(), qq), dqi_by_dqj(q.w(), q.x(), qq),
        dqi_by_dqj(q.w(), q.y(), qq), dqi_by_dqj(q.w(), q.z(), qq),
        dqi_by_dqj(q.x(), q.w(), qq), dqi_by_dqi(q.x(), qq),
        dqi_by_dqj(q.x(), q.y(), qq), dqi_by_dqj(q.x(), q.z(), qq),
        dqi_by_dqj(q.y(), q.w(), qq), dqi_by_dqj(q.y(), q.x(), qq),
        dqi_by_dqi(q.y(), qq), dqi_by_dqj(q.y(), q.z(), qq),
        dqi_by_dqj(q.z(), q.w(), qq), dqi_by_dqj(q.z(), q.x(), qq),
        dqi_by_dqj(q.z(), q.y(), qq), dqi_by_dqi(q.z(), qq);

    return  M;
}

// Auxiliary functions used by dqnorm_by_dq()
// Value of diagonal element of Jacobian
double depth_cam_ekf::MotionModel::dqi_by_dqi(double qi, double qq)
{
    return (1 - qi*qi / (qq*qq)) / qq;
}

// Value of off-diagonal element of Jacobian
double depth_cam_ekf::MotionModel::dqi_by_dqj(double qi, double qj, double qq)
{
    return -qi * qj / (qq*qq*qq);
}


