//
// Created by mecasys on 11/10/26.
//

#ifndef DEPTHCAMEKF_FEATUREMODEL_H
#define DEPTHCAMEKF_FEATUREMODEL_H

#include "MotionModel.h"
#include "Camera.h"

#include <Eigen/Eigen>

namespace depth_cam_ekf {
    class FeatureModel {
        public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        FeatureModel(int measurement_size, int feature_state_size, int graphics_state_size,
                    Camera *camera, MotionModel *motion_model);

        virtual ~FeatureModel();

        // Innovation Covariance: this is not a virtual function as it is generic
        // (though it can be over-ridden if necessary)
        void func_Si(const Eigen::MatrixXd &Pxx, const Eigen::MatrixXd &Pxyi,
                    const Eigen::MatrixXd &Pyiyi, const Eigen::MatrixXd &dhi_by_dxv,
                    const Eigen::MatrixXd &dhi_by_dyi, const Eigen::MatrixXd &Ri);

        void func_yigraphics_and_Pyiyigraphics(const Eigen::VectorXd &yi,
                                                const Eigen::MatrixXd &Pyiyi);

        void func_zeroedyigraphics_and_Pzeroedyigraphics(const Eigen::VectorXd &yi,
                                                        const Eigen::VectorXd &xv,
                                                        const Eigen::MatrixXd &Pxx,
                                                        const Eigen::MatrixXd &Pxyi,
                                                        const Eigen::MatrixXd &Pyiyi);
        void func_Ri(const Eigen::VectorXd &hi);

        // Jacobian of quaternion inverse: this is a constant matrix that doesn't
        // depend on the quaternion
        Eigen::Matrix4d dqbar_by_dq();

        // Jacobian for rotation derived from quaternion multiplied by a vector
        // This is tricky because really we need a tensor to do it neatly I think!
        // But do it here without
        // Return 3x4 matrix
        Eigen::MatrixXd dRq_times_a_by_dq (const Eigen::Quaterniond &q, const Eigen::Vector3d &a);

        // Component Jacobians: each returns a 3x3 matrix which is the derivative
        // of the rotation matrix derived from a quaternion w.r.t. each quaternion
        // element
        Eigen::Matrix3d dR_by_dq0(const Eigen::Quaterniond &q);
        Eigen::Matrix3d dR_by_dqx(const Eigen::Quaterniond &q);
        Eigen::Matrix3d dR_by_dqy(const Eigen::Quaterniond &q);
        Eigen::Matrix3d dR_by_dqz(const Eigen::Quaterniond &q);

        // Virtual Functions
        virtual void func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi(
            const Eigen::VectorXd &yi, const Eigen::VectorXd &xp) = 0;

        virtual int visibility_test(const Eigen::VectorXd &xp,
                                    const Eigen::VectorXd &yi,
                                    const Eigen::VectorXd &xp_orig,
                                    const Eigen::VectorXd &hi) = 0;

        virtual double selection_score(const Eigen::MatrixXd &Si) = 0;

        Eigen::VectorXd yigraphicsRES_;
        Eigen::VectorXd zeroedyigraphicsRES_;
        Eigen::VectorXd zeroedyiRES_;
        Eigen::MatrixXd PyiyigraphicsRES_;
        Eigen::MatrixXd PzeroedyigraphicsRES_;
        Eigen::MatrixXd RiRES_;
        Eigen::MatrixXd SiRES_;
        Eigen::MatrixXd dzeroedyi_by_dxpRES_;
        Eigen::MatrixXd dzeroedyi_by_dyiRES_;

        const int  kMeasurementSize_;
        const int  kFeatureStateSize_;
        const int  kGraphicsStateSize_;

        Camera *camera_;
        MotionModel *motion_model_;
    };

}


#endif //DEPTHCAMEKF_FEATUREMODEL_H