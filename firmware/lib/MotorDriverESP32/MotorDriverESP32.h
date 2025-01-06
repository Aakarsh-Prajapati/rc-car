#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>

class MotorDriver {
private:
  int pwmPinForward;          // PWM pin for forward direction
  int pwmPinBackward;         // PWM pin for backward direction
  int pwmChannelForward = 0;  // PWM channel for forward direction
  int pwmChannelBackward = 1; // PWM channel for backward direction
  int pwmFrequency = 5000;    // PWM frequency
  int pwmResolution = 8;      // PWM resolution (8-bit, 0-255)

public:
  // Constructor to initialize motor pins
  MotorDriver(int forwardPin, int backwardPin) {
    pwmPinForward = forwardPin;
    pwmPinBackward = backwardPin;

    // Initialize PWM pins
    ledcSetup(pwmChannelForward, pwmFrequency, pwmResolution);
    ledcSetup(pwmChannelBackward, pwmFrequency, pwmResolution);
    ledcAttachPin(pwmPinForward, pwmChannelForward);
    ledcAttachPin(pwmPinBackward, pwmChannelBackward);

    // Set the initial state to off
    ledcWrite(pwmChannelForward, 0);
    ledcWrite(pwmChannelBackward, 0);
  }

  // Function to set motor speed
  void setSpeed(int speed) {
    if (speed > 0) {
      // Forward direction
      ledcWrite(pwmChannelForward, speed); // Set speed for forward
      ledcWrite(pwmChannelBackward, 0);    // Stop backward
    } else if (speed < 0) {
      // Backward direction
      ledcWrite(pwmChannelForward, 0);       // Stop forward
      ledcWrite(pwmChannelBackward, -speed); // Set speed for backward
    } else {
      // Stop motor
      ledcWrite(pwmChannelForward, 0);
      ledcWrite(pwmChannelBackward, 0);
    }
  }
};

#endif
