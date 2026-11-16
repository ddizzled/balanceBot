#include <Arduino.h>
#include "../headers/config.h"

// Runs one motor. Positive speed = forward, negative = reverse, 0 = stop.
void runMotor(int speedPin, int dirPinA, int dirPinB, int speed) {
    if (speed > 0) {
        digitalWrite(dirPinA, HIGH);            // Direction pins set to "forward"
        digitalWrite(dirPinB, LOW);
    } else if (speed < 0) {
        digitalWrite(dirPinA, LOW);             // Direction pins set to "reverse"
        digitalWrite(dirPinB, HIGH);
    } else {
        digitalWrite(dirPinA, LOW);             // Both LOW = motor stopped
        digitalWrite(dirPinB, LOW);
    }

    int pwmValue = abs(speed);                // PWM only takes positive numbers (direction is handled above)
    pwmValue = constrain(pwmValue, 0, 255);   // Keep within the allowed 0-255 range
    analogWrite(speedPin, pwmValue);          // Set the motor speed
}

// Sends the PID result to both motors.
void driveBothMotors(int pidOutput) {
    int motorSpeed = pidOutput;

    // Small outputs can't overcome motor friction, so we add a minimum "kick"
    if (motorSpeed > 0) {
        motorSpeed = motorSpeed + minPwm;
    } else if (motorSpeed < 0) {
        motorSpeed = motorSpeed - minPwm;
    }

    motorSpeed = constrain(motorSpeed, -255, 255);  // Never exceed full speed

    runMotor(pwmaAPin, aIn1Pin,  aIn2Pin,  motorSpeed);
    runMotor(pwmaBPin, bIn1Pin, bIn2Pin, motorSpeed);
    // If one motor spins the wrong way, put a minus sign in front of motorSpeed on that line
}

void motorSetup(){
    pinMode(pwmaAPin,  OUTPUT);
    pinMode(aIn1Pin,  OUTPUT);
    pinMode(aIn2Pin,  OUTPUT);
    pinMode(pwmaBPin, OUTPUT);
    pinMode(bIn1Pin, OUTPUT);
    pinMode(bIn2Pin, OUTPUT);
}