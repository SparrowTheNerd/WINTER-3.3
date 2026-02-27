#pragma once
#include "stm32h7xx_hal.h"
#include "Eigen/Dense"
#include "abstract.h"
using namespace Eigen;

uint8_t usbTxBuf2[256];

void print_matrix2(Eigen::MatrixXd X)  
{
    uint16_t bufLen = 0;

    uint8_t nrow = X.rows();
    uint8_t ncol = X.cols();
    for (uint8_t i=0; i<nrow; i++) {   
        bufLen+=sprintf((char*)usbTxBuf2+bufLen, "[");

        for (uint8_t j=0; j<ncol; j++) {
            bufLen+=sprintf((char*)usbTxBuf2+bufLen, "%.5f", X(i,j));

            if(j<ncol-1) bufLen+=sprintf((char*)usbTxBuf2+bufLen, ",");
        }
        bufLen+=sprintf((char*)usbTxBuf2+bufLen, "]\n");
    }
    SerialPrintln(usbTxBuf2);
}

Quaterniond QuaternionAverage(Quaterniond q1, Quaterniond q2) {
    if (q1 == q2) return q1;
    Quaterniond r = q1.conjugate()*q2;
    // print_matrix2(r.coeffsScalarFirst());
    double uMag = fabs(2.0*acos(r.w()));
    Vector3d u = r.vec() * (uMag / sin(uMag/2.0));
    r = Quaterniond(cos(uMag/4.0), u/uMag * sin(uMag/4.0));

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