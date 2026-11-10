//
// Created by mecasys on 11/9/26.
//

#ifndef DEPTHCAMEKF_DEPTHCAMSLAM_H
#define DEPTHCAMEKF_DEPTHCAMSLAM_H

#include <vector>
#include <Eigen/Eigen>
#include <opencv2/core/core.hpp>


namespace depth_cam_ekf {
    class DepthCamSLAM {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        DepthCamSLAM();
        ~DepthCamSLAM();
        void Init(const std::string &config_path);
        bool GoOneStep(cv::Mat frame, bool save_trajectory, bool enable_mapping);


    };
}
#endif //DEPTHCAMEKF_DEPTHCAMSLAM_H