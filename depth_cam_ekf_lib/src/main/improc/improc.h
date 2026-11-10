//
// Created by mecasys on 11/10/26.
//

#ifndef DEPTHCAMEKF_IMPROC_H
#define DEPTHCAMEKF_IMPROC_H
#include <opencv2/opencv.hpp>

namespace depth_cam_ekf {
    double correlate2_warning(const int x0, const int y0,
                          const int x0lim, const int y0lim,
                          const int x1, const int y1,
                          const cv::Mat &p0,
                          const cv::Mat &p1,
                          double *sd0ptr, double *sd1ptr);

}

#endif //DEPTHCAMEKF_IMPROC_H