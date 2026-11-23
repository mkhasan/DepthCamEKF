//
// Created by mecasys on 11/17/26.
//

#ifndef DEPTHCAMEKF_DEPTHCAMFEATUREMODEL_H
#define DEPTHCAMEKF_DEPTHCAMFEATUREMODEL_H

#include "FeatureModel.h"

namespace depth_cam_ekf {
    class DepthCamFeatureModel : public FeatureModel {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        DepthCamFeatureModel(int measurement_size, int feature_state_size, int graphics_state_size,
                        Camera *camera, MotionModel *motion_model);
        ~DepthCamFeatureModel();

        void func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi(const Eigen::VectorXd &yi, const Eigen::VectorXd &xp);
        void func_yi_and_dyi_by_dxp_and_dyi_by_dhi_and_Ri(const Eigen::VectorXd &hi, const Eigen::VectorXd &xp);

        int visibility_test(const Eigen::VectorXd &xp, const Eigen::VectorXd &yi, const Eigen::VectorXd &xp_orig, const Eigen::VectorXd &hi);
        double selection_score(const Eigen::MatrixXd &Si);

        // Redefined virtual functions from
        // Fully_Initialised_Feature_Measurement_Model
        void func_hi_and_dhi_by_dxp_and_dhi_by_dyi(const Eigen::VectorXd &yi, const Eigen::VectorXd &xp);
        void func_nui(const Eigen::VectorXd &hi, const Eigen::VectorXd &zi);
        Eigen::Matrix3d dvnorm_by_dv(Eigen::Vector3d v);
        double dqi_by_dqi(double qi, double qq);
        double dqi_by_dqj(double qi, double qj, double qq);

        Eigen::VectorXd yiRES_;
        Eigen::MatrixXd dyi_by_dxpRES_;
        Eigen::MatrixXd dyi_by_dhiRES_;

        Eigen::VectorXd hiRES_;
        Eigen::VectorXd nuiRES_;
        Eigen::MatrixXd dhi_by_dxpRES_;
        Eigen::MatrixXd dhi_by_dyiRES_;

        const double kMaximumLengthRatio_;
        const double kMaximumAngleDifference_;
        const double kImageSearchBoundary_;

        // The return flag from visibility_test() ORs these together
        // to indicate the reasons for failure.
        enum visibility{kLeftRightFail_ = 1,
                        kUpDownFail_ = 2,
                        kDistanceFail_ = 4,
                        kAngleFail_ = 8,
                        kBehindCameraFail_ = 16};
    };

}


#endif //DEPTHCAMEKF_DEPTHCAMFEATUREMODEL_H