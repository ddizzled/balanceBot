#ifndef MOTORS_H
#define MOTORS_H

void runMotor(int speedPin, int dirPinA, int dirPinB, int speed);
void driveBothMotors(int pidOutput);
void motorSetup();

#endif