#include <Arduino.h>
#include "CytronMotorDriver.h"

CytronMD motor2(PWM_PWM,D1,D2);   
CytronMD motor1(PWM_PWM, D3, D4 ); 

int max_pwm = 255;  // Max PWM value for motor control

void calculateMotorPWM(float v_x, float w, int max_pwm) {
    float wheel_base = 0.2;
    float v_left = v_x - (w * wheel_base / 2.0);
    float v_right = v_x + (w * wheel_base / 2.0);

    int left_pwm = constrain(v_left, -max_pwm, max_pwm);
    int right_pwm = constrain(v_right, -max_pwm, max_pwm);

    motor1.setSpeed(left_pwm);  
    motor2.setSpeed(right_pwm);  

    Serial.print("Left PWM: ");
    Serial.print(left_pwm);
    Serial.print("\tRight PWM: ");
    Serial.println(right_pwm);
}

void setup() {
  Serial.begin(9600);
}

void loop() {
  calculateMotorPWM(0,0,max_pwm);
}