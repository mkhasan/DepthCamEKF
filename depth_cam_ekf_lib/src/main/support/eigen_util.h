//
// Created by mecasys on 11/10/26.
//

#ifndef DEPTHCAMEKF_EIGEN_UTIL_H
#define DEPTHCAMEKF_EIGEN_UTIL_H

#include <Eigen/Eigen>
#include <sstream>
#include <string>

namespace depth_cam_ekf {
    void VectorUpdate(Eigen::VectorXd &dst, const Eigen::VectorXd &src, const int pos);
    Eigen::VectorXd VectorExtract(const Eigen::VectorXd &src, const int pos, const int size);


    std::string MatStr(const Eigen::MatrixXd& item);
}

#endif //DEPTHCAMEKF_EIGEN_UTIL_H