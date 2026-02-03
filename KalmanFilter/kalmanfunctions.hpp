#pragma once
#include "stm32h7xx_hal.h"
#include "helperfunctions.hpp"
#include "Eigen/Dense"
using namespace Eigen;

DiagonalMatrix<double,3> I3 = DiagonalMatrix<double,3>::Identity();



Matrix<double,18,18> StateTransition(Vector3d w, Vector3d a, Vector3d wP, Vector3d aP, Quaterniond q, Quaterniond qP) {
    Quaterniond qAvg = QuaternionAverage(qP, q);
    Matrix3d wx = -SkewSymmetric((wP+w)/2.0);
    Matrix3d Cbif = ((-Quat2DCM(q)*SkewSymmetric(aP)) + (-Quat2DCM(qP)*SkewSymmetric(a)))/2.0;
    Matrix3d Cbi = -Quat2DCM(qAvg);
    Matrix<double,18,18> F = Matrix<double,18,18>::Zero();
    F.block<3,3>(0,0) = wx;
    F.block<3,3>(3,0) = Cbif;
    F.block<3,3>(6,3) = I3;
    F.block<3,3>(0,9) = I3*-1.0;
    F.block<3,3>(3,12) = Cbi;
    return F;
}

Vector<double,10> InertialIntegration(Vector<double,10> x, Vector3d wRaw, Vector3d wBias, Vector3d aRaw, Vector3d aBias, double dt) {
    double dt2 = dt/2.0;
    Vector3d w = wRaw - wBias;
    Vector3d a = aRaw - aBias;
    Quaterniond q {x(0), x(1), x(2), x(3)};

    Quaterniond q1 {
        q.w() - dt2*w.x()*q.x() - dt2*w.y()*q.y() - dt2*w.z()*q.z(),
        q.x() + dt2*w.x()*q.w() - dt2*w.y()*q.z() + dt2*w.z()*q.y(),
        q.y() + dt2*w.x()*q.z() + dt2*w.y()*q.w() - dt2*w.z()*q.x(),
        q.z() - dt2*w.x()*q.y() + dt2*w.y()*q.x() + dt2*w.z()*q.w()
    };
    q1.normalize();
    Vector3d aG = Quat2DCM(q1)*a;
    aG.z() -= 9.80665; // subtract gravity
    Vector3d V = aG*dt;
    return Vector<double,10> {
        q1.w(),
        q1.x(),
        q1.y(),
        q1.z(),
        x(4) + V.x(),
        x(5) + V.y(),
        x(6) + V.z(),
        x(7) + V.x()*dt,
        x(8) + V.y()*dt,
        x(9) + V.z()*dt
    };
}