# WINTER-3.3
Development of the WINTER (Wireless Inertial iNformation and Telemetry for Energetics Regulation) flight computer using STM32 LL and HAL

This board features the following list of sensors:
* **ICM-42688-P** –– 3-axis accelerometer and gyroscope
* **ADXL375** –– 200G 3-axis accelerometer
* **MMC5983MA** –– 3-axis magnetometer
* **MS5607** –– High-altitude barometer
* **UBlox Max-M10-S** –– Precision GPS

These sensors contribute to a **nonlinear Kalman Filter** to obtain continuous position, velocity, and orientation data. This data will be used for control of high-power amateur rockets using advanced systems such as multi-stage and thrust vectoring.