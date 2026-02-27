#pragma once
#include "stm32h7xx_hal.h"
#include "helperfunctions.hpp"
#include "Eigen/Dense"
using namespace Eigen;

Vector3d MapIMU(double input[3]) {
    return Vector3d(-input[2], input[1], input[0]); // map board axes (X up) to body axes (Z up)
}

void BaroMeas(double posZ, double baroAlt, Matrix<double,1,18>* hB, double* dB) {
    // measurement matrix
    hB->setZero();
    (*hB)(8) = 1.0; // position Z

    // innovation
    (*dB) = baroAlt - posZ;
}