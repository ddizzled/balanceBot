#ifndef CONFIG_H
#define CONFIG_H

#define sdaPin 21
#define sclPin 22
#define mpuAddress 0x68

#define pwmaAPin 33
#define pwmaBPin 12
#define aIn2Pin 32
#define aIn1Pin 25
#define stbyPin 26
#define bIn1Pin 27
#define bIn2Pin 14
#define minPwm 25

#define maxIntegralOutput 100.0
#define loopTimeMicroSec 5000
#define fallAngleDeg 45.0
#define gyroThrust 0.98
#define gainProportional 20.0   // Kp: how hard to push per degree of tilt error
#define gainIntegral 0.5    // Ki: slowly corrects a steady lean that Kp alone can't fix
#define gainDerivative 0.8    // Kd: calms the motion so it doesn't overshoot or wobble

#endif