//
// Created by mecasys on 11/10/26.
//

#ifndef DEPTHCAMEKF_MOTIONMODEL_H
#define DEPTHCAMEKF_MOTIONMODEL_H

#include <Eigen/Eigen>

namespace depth_cam_ekf {

    class MotionModel {
        public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        MotionModel();
        ~MotionModel();


        // Extract cartesian part of position state vector
        void func_r(const Eigen::VectorXd &xp);
        // And Jacobian
        void func_dr_by_dxp(const Eigen::VectorXd &xp);
        // Extract quaternion part of position state vector
        void func_q(const Eigen::VectorXd &xp);

        // Redefined virtual functions
        void func_fv_and_dfv_by_dxv(const Eigen::VectorXd &xv, const Eigen::VectorXd &u,
                                    const double delta_t);
        void func_Q(const Eigen::VectorXd &xv, const Eigen::VectorXd &u, const double delta_t);
        void func_xp(const Eigen::VectorXd &xv);
        void func_dxp_by_dxv(const Eigen::VectorXd &xv);
        void func_xvnorm_and_dxvnorm_by_dxv(const Eigen::VectorXd &xv);

        // Easy access to state blocks: fill matrices r, q, v, omega with
        // values based on state xv
        void extract_r_q_v_omega(const Eigen::VectorXd &xv, Eigen::Vector3d &r,
                                Eigen::Quaterniond &q, Eigen::Vector3d &v,
                                Eigen::Vector3d &omega);
        // The opposite: put r, q, v, omega back into vector xv
        void compose_xv(const Eigen::Vector3d &r, const Eigen::Quaterniond &q,
                        const Eigen::Vector3d &v, const Eigen::Vector3d &omega,
                        Eigen::VectorXd &xv);
        // Calculate commonly used Jacobian part dq(omega * delta_t) by domega
        void dqomegadt_by_domega(const Eigen::Vector3d &omega, const double delta_t,
                                Eigen::MatrixXd &dqomegadt_by_domega);

        // Ancillary functions: calculate parts of Jacobian dq_by_domega
        // which are repeatable due to symmetry.
        double dq0_by_domegaA(const double omegaA, const double omega, const double delta_t);
        double dqA_by_domegaA(const double omegaA, const double omega, const double delta_t);
        double dqA_by_domegaB(const double omegaA, const double omegaB, const double omega,
                                const double delta_t);

        Eigen::Matrix4d dqnorm_by_dq(const Eigen::Quaterniond &q);
        double dqi_by_dqi(double qi, double qq);
        double dqi_by_dqj(double qi, double qj, double qq);

        Eigen::VectorXd fvRES_;
        Eigen::VectorXd xpRES_;
        Eigen::VectorXd xvnormRES_;
        Eigen::MatrixXd dfv_by_dxvRES_;
        Eigen::MatrixXd QxRES_;
        Eigen::MatrixXd dxp_by_dxvRES_;
        Eigen::MatrixXd dxvnorm_by_dxvRES_;

        Eigen::Vector3d rRES_;
        Eigen::MatrixXd dr_by_dxpRES_;
        Eigen::Quaterniond  qRES_;

        const int  kPositionStateSize_;
        const int  kStateSize_;
        const int  kControlSize_;
        const double  kSdAComponentFilter_;
        const double  kSdAlphaComponentFilter_;
    };

}



#endif //DEPTHCAMEKF_MOTIONMODEL_H