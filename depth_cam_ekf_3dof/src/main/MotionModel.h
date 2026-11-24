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
        void func_phi(const Eigen::VectorXd &xp);

        // Redefined virtual functions
        void func_fv_and_dfv_by_dxv(const Eigen::VectorXd &xv, const Eigen::VectorXd &u,
                                    const double delta_t);
        void func_Q(const Eigen::VectorXd &xv, const Eigen::VectorXd &u, const double delta_t);
        void func_xp(const Eigen::VectorXd &xv);
        void func_dxp_by_dxv(const Eigen::VectorXd &xv);
        void func_xvnorm_and_dxvnorm_by_dxv(const Eigen::VectorXd &xv);

        // Easy access to state blocks: fill matrices r, q, v, omega with
        // values based on state xv
        void extract_r_phi_v_omega(const Eigen::VectorXd &xv, Eigen::Vector2d &r,
                                double &phi, Eigen::Vector2d &v,
                                double &omega);
        // The opposite: put r, q, v, omega back into vector xv
        void compose_xv(const Eigen::Vector2d &r, const double &q,
                        const Eigen::Vector2d &v, const double &omega,
                        Eigen::VectorXd &xv);

        Eigen::VectorXd fvRES_;
        Eigen::VectorXd xpRES_;
        Eigen::VectorXd xvnormRES_;
        Eigen::MatrixXd dfv_by_dxvRES_;
        Eigen::MatrixXd QxRES_;
        Eigen::MatrixXd dxp_by_dxvRES_;
        Eigen::MatrixXd dxvnorm_by_dxvRES_;

        Eigen::Vector3d rRES_;
        Eigen::MatrixXd dr_by_dxpRES_;
        double  phiRES_;

        const int  kPositionStateSize_;
        const int  kStateSize_;
        const int  kControlSize_;
        const double  kSdAComponentFilter_;
        const double  kSdAlphaComponentFilter_;
    };

}



#endif //DEPTHCAMEKF_MOTIONMODEL_H