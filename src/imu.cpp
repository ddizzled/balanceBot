#include <Arduino.h>
#include "../headers/config.h"
#include <Wire.h>


void writeMpuRegister(uint8_t registerAddress, uint8_t value) {
  Wire.beginTransmission(mpuAddress);  // Start talking to the sensor
  Wire.write(registerAddress);              // Say which setting we want to change
  Wire.write(value);                        // Say what to change it to
  Wire.endTransmission(true);               // Finish and release the I2C bus
}

int16_t readTwoBytesAsNumber() {
  int16_t highByte = Wire.read();           // First byte received (big part of the number)
  int16_t lowByte  = Wire.read();           // Second byte received (small part of the number)
  return (highByte << 8) | lowByte;         // Slide the high byte left 8 bits, then add the low byte
}

// Wakes the sensor up and sets its measurement ranges.
void mpuSetup() {
  Wire.begin();                             // Start the I2C bus
  Wire.setClock(400000);                    // Run I2C at 400 kHz so reading is fast

  writeMpuRegister(0x6B, 0x00);             // Register 0x6B = power control. 0 = wake up (it starts asleep)
  writeMpuRegister(0x1B, 0x00);             // Register 0x1B = gyro range. 0 = +/-250 degrees per second
  writeMpuRegister(0x1C, 0x00);             // Register 0x1C = accel range. 0 = +/-2 g
}

// Reads the sensor and gives back two things:
//   accelAngleDegrees : tilt calculated from gravity (accurate over time, noisy when shaking)
//   gyroDegPerSec     : how fast the sensor is rotating (smooth, but drifts over time)
void readSensor(float &accelAngleDegrees, float &gyroDegPerSec) {
    // Ask the sensor to start sending data from register 0x3B (the first accelerometer value)
    Wire.beginTransmission(mpuAddress);
    Wire.write(0x3B);
    Wire.endTransmission(false);

    // Request 14 bytes: accel X/Y/Z, temperature, gyro X/Y/Z (2 bytes each)
    Wire.requestFrom(mpuAddress, (uint8_t)14, (uint8_t)true);

    // The values arrive in a fixed order, so we must read them in that order
    int16_t accelX = readTwoBytesAsNumber();
    int16_t accelY = readTwoBytesAsNumber();
    int16_t accelZ = readTwoBytesAsNumber();
    readTwoBytesAsNumber();                   // Temperature: read it and ignore it
    readTwoBytesAsNumber();                   // Gyro X: ignore
    int16_t gyroY = readTwoBytesAsNumber();   // Gyro Y: the one we use for pitch
    readTwoBytesAsNumber();                   // Gyro Z: ignore

    // Work out tilt from the direction gravity points.
    // sqrt(Y^2 + Z^2) is the "sideways and downward" part of gravity.
    float sidewaysAndDown = sqrt((float)accelY * accelY + (float)accelZ * accelZ);
    float tiltInRadians = atan2((float)accelX, sidewaysAndDown);
    accelAngleDegrees = tiltInRadians * 180.0 / PI;   // Convert radians to degrees

    // Convert the raw gyro number to degrees per second (131 raw units = 1 deg/s at this range)
    gyroDegPerSec = gyroY / 131.0;
}

    
// Measures the gyro's reading while the device sits still.
    // Any reading while still is "offset error", which we subtract later.
void calibrateGyro(float& gyroOffset) {
    const int NUMBER_OF_SAMPLES = 500;
    float total = 0;
    float unusedAngle;
    float gyroSample;

    for (int i = 0; i < NUMBER_OF_SAMPLES; i++) {
        readSensor(unusedAngle, gyroSample);       // We only want the gyro value here
        total = total + gyroSample;
        delay(3);                               // Short pause between samples
    }

    gyroOffset = total / NUMBER_OF_SAMPLES;  // Average of all samples
}
