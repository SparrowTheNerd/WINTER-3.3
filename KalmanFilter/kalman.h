#pragma once
#include "stm32h7xx_hal.h"
#include "Eigen/Dense"
#include "MMC5983.h"
#include "ICM42688.h"
#include "MS5607.h"
#include "ADXL375.h"
#include "SparkFun_u-blox_GNSS_v3.h"

using namespace Eigen;

class KalmanFilter {
    public:

        void init(MMC5983* mag, ICM42688* imu, MS5607* baro, ADXL375* highG, SFE_UBLOX_GNSS* gnss);
        void predict(double dt);
        void update();

        Vector<double,10> intState; // inertial state vector (quaternion, velocity, position)
        Vector<double,18> errState;  // error state vector (angle error, velocity error, position error, gyro bias, accel bias, mag bias)

    private:
        Vector<double,10> intStatePriori; // prior inertial state vector
        Vector<double,18> errStatePriori; // prior error state vector
        Matrix<double,18,18> P;      // error covariance matrix
        Matrix<double,18,18> Pn1n; // prior error covariance matrix
        Matrix<double,18,18> F;      // state transition matrix
        Matrix<double,18,18> Phi;    // state transition matrix second-order integration
        Matrix<double,18,18> Q;      // process noise covariance

        Matrix<double,18,18> I18 = Matrix<double,18,18>::Identity(); // 18x18 identity matrix

        Vector3d wP; // prior gyro reading
        Vector3d aP; // prior accel reading
        Quaterniond qP; // prior quaternion

        MMC5983* mag;
        ICM42688* imu;
        MS5607* baro;
        ADXL375* highG;
        SFE_UBLOX_GNSS* gnss;

};