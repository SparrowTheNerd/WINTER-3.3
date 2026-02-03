#include "kalman.h"
#include "kalmanfunctions.hpp"

void KalmanFilter::init(MMC5983* mag, ICM42688* imu, MS5607* baro, ADXL375* highG, SFE_UBLOX_GNSS* gnss) {
    this->mag = mag;
    this->imu = imu;
    this->baro = baro;
    this->highG = highG;
    this->gnss = gnss;
    double gOfst[3] = {0,0,0};

    for(int i=0; i<250; i++) {
        imu->ReadIMU();
        for(int j=0; j<3; j++) {
            gOfst[j] += imu->gyro_dps[j];
        }
        HAL_Delay(10);
    }
    for(int j=0; j<3; j++) {
        imu->gyro_ofst[j] = gOfst[j]/250.0;
    }
    // qP = Quaterniond {0.7071, 0.0, 0.7071, 0.0};
    intState << 0.7071, 0.0, 0.7071, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    errState.setZero();
}
void KalmanFilter::predict(double dt) {
    imu->ReadIMU();
    // highG->ReadAccel();

    Vector3d aRaw = MapIMU(imu->accel_ms2);
    Vector3d wRaw = MapIMU(imu->gyro_dps)*M_PI/180.0;

    // Vector3d wBias = errState.segment<3>(9);
    // Vector3d aBias = errState.segment<3>(12);
    Vector3d wBias = Vector3d::Zero();
    Vector3d aBias = Vector3d::Zero();

    intStatePriori = InertialIntegration(intState, wRaw, wBias, aRaw, aBias, dt);

    // Quaterniond q {intStatePriori(0), intStatePriori(1), intStatePriori(2), intStatePriori(3)}; // can't use segment bc Eigen treats Vector4 as XYZW order, stupid
    // F = StateTransition(wRaw, aRaw, wP, aP, q, qP);
    // Phi = I18 + F*dt + 0.5*F*F*dt*dt;
    // Pn1n = Phi*P*Phi.transpose() + Q;

    // wP = wRaw; aP = aRaw; 

    // code for doing raw integration tests
    // qP = q;
    intState = intStatePriori;
    // P = Pn1n;

}
void KalmanFilter::update() {
    // Implement the update step of the Kalman Filter here
}