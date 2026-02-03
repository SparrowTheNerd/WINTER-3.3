#pragma once
#include "stm32h7xx_hal.h"
#include "Eigen/Dense"
using namespace Eigen;


Quaterniond QuaternionAverage(Quaterniond q1, Quaterniond q2) {
    Quaterniond r = q1.conjugate()*q2;
    double uMag = fabs(2*acos(r.w()));
    Vector3d u = r.vec() * (uMag / sin(uMag/2.0));
    r = Quaterniond(cos(uMag/4.0), sin(uMag/4.0)*u/uMag);

    return (q1*r).normalized();
}

Matrix3d SkewSymmetric(Vector3d v) {
    Matrix3d S;
    S <<     0, -v(2),  v(1),
          v(2),     0, -v(0),
         -v(1),  v(0),     0;
    return S;
}

Matrix3d Quat2DCM(Quaterniond q) {
    Matrix3d R;
    R << q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z(),     2*(q.x()*q.y() - q.w()*q.z()),             2*(q.x()*q.z() + q.w()*q.y()),
             2*(q.x()*q.y() + q.w()*q.z()),         q.w()*q.w() - q.x()*q.x() + q.y()*q.y() - q.z()*q.z(),   2*(q.y()*q.z() - q.w()*q.x()),
             2*(q.x()*q.z() - q.w()*q.y()),             2*(q.y()*q.z() + q.w()*q.x()),         q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z();
    return R;
}

Vector3d MapIMU(double input[3]) {
    return Vector3d(-input[2], input[1], input[0]); // map board axes (X up) to body axes (Z up)
}