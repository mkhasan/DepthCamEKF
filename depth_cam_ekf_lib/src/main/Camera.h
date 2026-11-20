//
// Created by mecasys on 11/9/26.
//

#ifndef DEPTHCAMEKF_CAMERA_H
#define DEPTHCAMEKF_CAMERA_H

#include <Eigen/Eigen>

namespace depth_cam_ekf {

    class Camera {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Camera();
        ~Camera();

        void SetCameraParameters(const int camera_width, const int camera_height,
                                const double fku, const double fkv,
                                const double u0, const double v0,
                                const double kd1, const int sd, const int depth_sd);

        Eigen::Vector2d Project(const Eigen::Vector3d &c);
        Eigen::Vector3d Unproject(const Eigen::Vector2d& image);
        Eigen::MatrixXd ProjectionJacobian();
        Eigen::MatrixXd UnprojectionJacobian();
        Eigen::Matrix3d MeasurementNoise(const Eigen::Vector2d& h);

        int               width_;
        int               height_;
        double            kd1_;
        double            fku_;
        double            fkv_;
        Eigen::Vector2d   centre_;
        double            measurement_sd_;
        double            depth_sd_;
        Eigen::Matrix3d   c_;
        Eigen::Matrix3d   c_inv_;
        Eigen::Vector3d   last_camera_;
        Eigen::Vector2d   last_image_centred_;

        double            xL_, xR_, yB_, yT_;
        double            xW_, xM_, yH_, yM_;

        const double      kNear_;
        const double      kFar_;
    };

}

#endif //DEPTHCAMEKF_CAMERA_H