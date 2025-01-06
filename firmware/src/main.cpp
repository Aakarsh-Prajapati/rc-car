#include <WiFi.h>
#include <esp_now.h>
#include <MotorDriverESP32.h>

// Structure to receive data
// This structure must match the sender's structure
typedef struct struct_message
{
  int vel_x;  // Velocity along the x-axis
  int vel_w;  // Angular velocity (w)
} struct_message;

// Create a struct_message object to store incoming data
struct_message myData;

// Initialize motor driver objects for two motors
MotorDriver motor1(18, 19);  // Motor 1 connected to pins 18 and 19
MotorDriver motor2(16, 17);  // Motor 2 connected to pins 16 and 17

// Callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  // Copy the received data into the myData structure
  memcpy(&myData, incomingData, sizeof(myData));

  // Print received data to the Serial Monitor
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Velocity X: ");
  Serial.println(myData.vel_x);
  Serial.print("Velocity W: ");
  Serial.println(myData.vel_w);
  Serial.println();
}

float wheelRadius = 0.10;  // Radius of the wheel in meters (e.g., 5 cm)
float trackWidth = 0.20;   // Distance between the left and right wheels in meters (e.g., 15 cm)
float maxVelocity = 255;   // Maximum velocity for the motors (default 255)
                           // 
// Function to calculate the left and right wheel velocities based on linear velocity (y) and angular velocity (w)
void calculateWheelVelocities(float y, float w, float &leftVel, float &rightVel, float maxVel = 255) {
  // Normalize the input velocities to match the PWM range of -255 to 255
  float maxLinearSpeed = maxVel;  // Maximum speed in linear direction
  float maxAngularSpeed = maxVel; // Maximum speed for angular velocity

  // Calculate the wheel velocities based on the kinematic model
  leftVel = (y - w * trackWidth / 2) / wheelRadius;
  rightVel = (y + w * trackWidth / 2) / wheelRadius;

  // Scale the wheel velocities based on the max velocity
  leftVel = constrain(leftVel, -maxLinearSpeed, maxLinearSpeed)/10;
  rightVel = constrain(rightVel, -maxLinearSpeed, maxLinearSpeed)/10;
}

float leftVel = 0;
float rightVel = 0;

void setup() {
  // Initialize Serial Monitor for debugging
  Serial.begin(115200);  // ESP32 default baud rate is 115200
  
  // Set the ESP32 to Wi-Fi Station mode
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW communication protocol
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Register the callback function to handle received data
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  // Set motor speeds based on the received data
  motor1.setSpeed(-70);  // Set speed of motor1 to -70 (reverse direction)
  motor2.setSpeed(-150); // Set speed of motor2 to -150 (reverse direction)

  // Call the function to calculate wheel velocities
  calculateWheelVelocities(255, 0, leftVel, rightVel, maxVelocity);

  // Print the calculated wheel velocities to the Serial Monitor
  Serial.print("Left Wheel Velocity: ");
  Serial.println(leftVel); // Print left wheel velocity in rad/s
  Serial.print("Right Wheel Velocity: ");
  Serial.println(rightVel); // Print right wheel velocity in rad/s
}
