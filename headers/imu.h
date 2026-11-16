#ifndef IMU_H
#define IMU_H

#include <Arduino.h>

void writeMpuRegister(uint8_t registerAddress, uint8_t value);
int16_t readTwoBytesAsNumber();
void mpuSetup();
void readSensor(float &accelAngleDegrees, float &gyroDegPerSec);
void calibrateGyro(float& gyroOffset);

#endif