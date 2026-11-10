//
// Created by mecasys on 11/10/26.
//

#ifndef DEPTHCAMEKF_PARTFEATUREMODEL_H
#define DEPTHCAMEKF_PARTFEATUREMODEL_H

#include "FeatureModel.h"

namespace depth_cam_ekf {
    class PartFeatureModel : public FeatureModel {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        PartFeatureModel(int measurement_size, int feature_state_size, int graphics_state_size,
                        Camera *camera, MotionModel *motion_model,
                        int full_feature_state_size);
        ~PartFeatureModel() override;

        void func_zeroedyi_and_dzeroedyi_by_dxp_and_dzeroedyi_by_dyi(const Eigen::VectorXd &yi, const Eigen::VectorXd &xp) override;
        int visibility_test(const Eigen::VectorXd &xp, const Eigen::VectorXd &yi, const Eigen::VectorXd &xp_orig, const Eigen::VectorXd &hi) override;

        double selection_score(const Eigen::MatrixXd &Si) override;

        // Redefined virtual functions from
        // Partially_Initialised_Fully_Initialised_Feature_Measurement_Model
        void func_ypi_and_dypi_by_dxp_and_dypi_by_dhi_and_Ri(const Eigen::VectorXd &hi, const Eigen::VectorXd &xp);
        void func_hpi_and_dhpi_by_dxp_and_dhpi_by_dyi(const Eigen::VectorXd &yi, const Eigen::VectorXd &xp, const Eigen::VectorXd &lambda);
        void func_yfi_and_dyfi_by_dypi_and_dyfi_by_dlambda(const Eigen::VectorXd &ypi, const Eigen::VectorXd &lambda);

        // Special functions: extract r and hhat parts of line
        void func_ri(const Eigen::VectorXd &ypi);
        void func_hhati(const Eigen::VectorXd &ypi);

        Eigen::Matrix3d dvnorm_by_dv(Eigen::Vector3d v);
        double dqi_by_dqi(double qi, double qq);
        double dqi_by_dqj(double qi, double qj, double qq);

        Eigen::Vector3d riRES_;
        Eigen::Vector3d hhatiRES_;

        Eigen::VectorXd ypiRES_;
        Eigen::VectorXd hpiRES_;
        Eigen::VectorXd yfiRES_;
        Eigen::MatrixXd dypi_by_dxpRES_;
        Eigen::MatrixXd dypi_by_dhiRES_;
        Eigen::MatrixXd dhpi_by_dxpRES_;
        Eigen::MatrixXd dhpi_by_dyiRES_;
        Eigen::MatrixXd dyfi_by_dypiRES_;
        Eigen::MatrixXd dyfi_by_dlambdaRES_;

        const int  kFreeParameterSize_;
        const int  kFullFeatureStateSize_;
    };

}



#endif //DEPTHCAMEKF_PARTFEATUREMODEL_H