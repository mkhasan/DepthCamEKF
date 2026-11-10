//
// Created by mecasys on 11/10/26.
//

#ifndef DEPTHCAMEKF_MATH_UTIL_H
#define DEPTHCAMEKF_MATH_UTIL_H


#include <Eigen/Eigen>

namespace depth_cam_ekf {
    const double  GetAngleFromQuaternion(const Eigen::Quaterniond &q);
    const Eigen::Vector3d GetAxisFromQuaternion(const Eigen::Quaterniond &q);
    Eigen::Quaterniond QuaternionFromAngularVelocity(const Eigen::Vector3d av);
    Eigen::Matrix4d dq3_by_dq1(const Eigen::Quaterniond &q1);
    Eigen::Matrix4d dq3_by_dq2(const Eigen::Quaterniond &q2);

    void VectorUpdate(Eigen::VectorXd &dst, const Eigen::VectorXd &src, const int pos);

}

#endif //DEPTHCAMEKF_MATH_UTIL_H