#include <Arduino.h>
#include "../headers/config.h"
#include "../headers/imu.h"
#include "../headers/motors.h"

const float targetAngleDegrees = 0.0;
float gyroOffset = 0;
float currentAngle = 0;
unsigned long previousLoopStartTime = 0;
float rotationSpeedDegPerSec = 0; 
float integralOutputSum = 0;
float gyroOffsetDegPerSec = 0;
float currentAngleDegrees = 0;

float calculatePid(float angle, float rotationSpeed, float secondsSinceLastLoop) {
    // How far off target are we? Positive = tilted one way, negative = the other
    float error = targetAngleDegrees - angle;

    // P (Proportional): bigger error -> bigger push
    float proportionalPart = gainProportional * error;

    // I (Integral): add up error over time, so a small lasting error slowly builds a bigger push
    float integralOutputSum = integralOutputSum + gainIntegral * error * secondsSinceLastLoop;
    integralOutputSum = constrain(integralOutputSum, -maxIntegralOutput, maxIntegralOutput);
    float integralPart = integralOutputSum;

    // D (Derivative): push against fast movement to prevent overshoot.
    // The gyro already tells us how fast the angle is changing, so we use it directly.
    // The minus sign is because we want to oppose the motion.
    float derivativePart = -gainDerivative * rotationSpeed;

    return proportionalPart + integralPart + derivativePart;
}
void setup(){
    Serial.begin(115200);

    motorSetup();
    mpuSetup();

    Serial.println("Keep the device still: calibrating gyro...");
    calibrateGyro(gyroOffset);                          // Measure the gyro's resting offset

    // Start the angle estimate from the accelerometer so we begin near the true tilt
    float startingAngle;
    float unusedGyro;
    readSensor(startingAngle, unusedGyro);
    currentAngle = startingAngle;

    previousLoopStartTime = micros();         // Record the starting time for loop timing
}

void loop(){
      // Timing: wait until 5 ms have passed since the last cycle ----
    unsigned long currentTime = micros();
    unsigned long timeSinceLastLoop = currentTime - previousLoopStartTime;

    if (timeSinceLastLoop < loopTimeMicroSec) {
        return;                                 // Not time yet, so exit and check again
    }

    float secondsSinceLastLoop = timeSinceLastLoop / 1000000.0;  // Microseconds -> seconds
    previousLoopStartTime = currentTime;

    // ---- Step 1: read the sensor ----
    float accelAngleDegrees;
    float gyroDegPerSec;
    readSensor(accelAngleDegrees, gyroDegPerSec);

    rotationSpeedDegPerSec = gyroDegPerSec - gyroOffsetDegPerSec;  // Remove the resting offset

    // ---- Step 2: estimate the angle (complementary filter) ----
    // Gyro method: take last angle and add how much we rotated since (fast, but drifts)
    float angleFromGyro = currentAngleDegrees + rotationSpeedDegPerSec * secondsSinceLastLoop;
    // Accelerometer method: angle from gravity (no drift, but noisy)
    // Blend them: mostly gyro, with a small pull toward the accelerometer to cancel drift
    currentAngleDegrees = gyroThrust * angleFromGyro + (1.0 - gyroThrust) * accelAngleDegrees;

    // ---- Safety: stop everything if it has fallen over ----
    if (abs(currentAngleDegrees) > fallAngleDeg){
        driveBothMotors(0);                     // Motors off
        integralOutputSum = 0;                  // Clear the integral so it doesn't jerk when picked back up
        return;
    }

    // ---- Step 3: calculate the PID output ----
    float pidOutput = calculatePid(currentAngleDegrees, rotationSpeedDegPerSec, secondsSinceLastLoop);

    // ---- Step 4: drive the motors ----
    driveBothMotors((int)pidOutput);

    // ---- Debug: view in Tools > Serial Plotter while tuning ----
    Serial.print(currentAngleDegrees);
    Serial.print(",");
    Serial.println(pidOutput);
}

/*
    TUNING STEPS
    1. Set gainIntegral and gainDerivative to 0. Raise gainProportional until
       it oscillates quickly around upright.
    2. Raise gainDerivative until the oscillation calms down
       (too high makes it jitter or buzz).
    3. Lower gainProportional a little if it's still twitchy.
    4. Add a small gainIntegral only if it slowly drifts to one side.
    5. If it falls away from upright instead of correcting, flip the sign of
       the angle or swap the motor wires. The correction must oppose the tilt.
*/
