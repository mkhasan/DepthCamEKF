//
// Created by mecasys on 11/20/26.
//

#ifndef DEPTHCAMEKF_KALMAN_H
#define DEPTHCAMEKF_KALMAN_H

#include "DepthCamSLAM.h"

namespace depth_cam_ekf {

    class Kalman {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Kalman();
        ~Kalman();

        void KalmanFilterPredict(DepthCamSLAM *depth_cam_slam, Eigen::Vector3d &u);
        void KalmanFilterUpdate(DepthCamSLAM *depth_cam_slam);
    };
}


#endif //DEPTHCAMEKF_KALMAN_H