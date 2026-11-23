//
// Created by Hasan on 11/10/26.
//

#include "eigen_util.h"



using namespace std;
void depth_cam_ekf::VectorUpdate(Eigen::VectorXd &dst, const Eigen::VectorXd &src, const int pos)
{
    for (int i=0; i<src.size(); ++i) {
        dst(pos+i) = src(i);
    }
}

Eigen::VectorXd depth_cam_ekf::VectorExtract(const Eigen::VectorXd &src, const int pos, const int size)
{
    Eigen::VectorXd dst(size);

    for (int i=0; i<size; ++i) {
        dst(i) = src(pos+i);
    }

    return  dst;
}

string depth_cam_ekf::MatStr(const Eigen::MatrixXd &item) {
    std::stringstream ss;
    ss << item;
    return ss.str();
}



