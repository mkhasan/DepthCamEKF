//
// Created by Hasan on 11/10/26.
//

#include "math_util.h"

const double  depth_cam_ekf::GetAngleFromQuaternion(const Eigen::Quaterniond &q)
{
    return  (2.0 * atan2(sqrt(q.x()*q.x() + q.y()*q.y() + q.z()*q.z()), q.w()));
}

const Eigen::Vector3d depth_cam_ekf::GetAxisFromQuaternion(const Eigen::Quaterniond &q)
{
    Eigen::Vector3d axis(0, 0, 0);
    const double  ang = GetAngleFromQuaternion(q);

    if (ang > 0.0) {
        const double  scale = ang / sin(ang / 2.0);

        axis(0) = q.x() * scale;
        axis(1) = q.y() * scale;
        axis(2) = q.z() * scale;
    }

    return  axis;
}

Eigen::Quaterniond depth_cam_ekf::QuaternionFromAngularVelocity(const Eigen::Vector3d av)
{
    Eigen::Quaterniond  q;
    const double        angle = sqrt(av(0)*av(0) + av(1)*av(1) + av(2)*av(2));

    if (angle > 0.0) {
        const double s = sin(angle/2.0) / angle;
        const double c = cos(angle/2.0);

        q.x() = s * av(0);
        q.y() = s * av(1);
        q.z() = s * av(2);
        q.w() = c;
    } else {
        q.x() = q.y() = q.z() = 0.0;
        q.w() = 1.0;
    }

    return  q;
}

Eigen::Matrix4d depth_cam_ekf::dq3_by_dq1(const Eigen::Quaterniond &q1)
{
    Eigen::Matrix4d m;

    double  x = q1.x();
    double  y = q1.y();
    double  z = q1.z();
    double  w = q1.w();

    m << w, -x, -y, -z,
        x,  w, -z,  y,
        y,  z,  w, -x,
        z, -y,  x,  w;

    return m;
}

Eigen::Matrix4d depth_cam_ekf::dq3_by_dq2(const Eigen::Quaterniond &q2)
{
    Eigen::Matrix4d m;

    double  x = q2.x();
    double  y = q2.y();
    double  z = q2.z();
    double  w = q2.w();

    m << w, -x, -y, -z,
        x,  w,  z, -y,
        y, -z,  w,  x,
        z,  y, -x,  w;

    return m;
}
