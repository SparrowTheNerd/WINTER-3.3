#include "kalman.h"
#include "kalmanfunctions.hpp"

void KalmanFilter::init(MMC5983* mag, ICM42688* imu, MS5607* baro, ADXL375* highG, SFE_UBLOX_GNSS* gnss) {
    this->mag = mag;
    this->imu = imu;
    this->baro = baro;
    this->highG = highG;
    this->gnss = gnss;
}
void KalmanFilter::predict(double dt) {
    imu->ReadIMU();
    highG->ReadAccel();
    Map<Vector3d> wRaw(imu->gyro_dps,3);
    Map<Vector3d> aRaw(imu->accel_ms2,3);
    Vector3d wBias = errState.segment<3>(9);
    Vector3d aBias = errState.segment<3>(12);
    intState = InterialIntegration(intState, wRaw, wBias, aRaw, aBias, dt);

    Quaterniond q {intState(0), intState(1), intState(2), intState(3)}; // can't use segment bc Eigen treats Vector4 as XYZW order, stupid
    F = StateTransition(wRaw, aRaw, wP, aP, q, qP);
    Phi = I18 + F*dt + 0.5*F*F*dt*dt;
    Pn1n = Phi*P*Phi.transpose() + Q;
}
void KalmanFilter::update() {
    // Implement the update step of the Kalman Filter here
}