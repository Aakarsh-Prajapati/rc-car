#include "Kinematics.h" // Include the Kinematics library
#include <MotorDriverESP32.h>
#include <WiFi.h>
#include <esp_now.h>

// Structure to receive data
typedef struct struct_message {
  float vel_x; // Velocity along the x-axis
  float vel_w; // Angular velocity (w)
} struct_message;

// Create a struct_message object to store incoming data
struct_message myData;

// Initialize motor driver objects for two motors
MotorDriver motor1(18, 19); // Motor 1 connected to pins 18 and 19
MotorDriver motor2(16, 17); // Motor 2 connected to pins 16 and 17

// Kinematics parameters
float wheelRadius = 0.10; // Radius of the wheel in meters (e.g., 10 cm)
float trackWidth =
    0.20; // Distance between the left and right wheels in meters (e.g., 20 cm)
int motor_max_rpm = 255; // Maximum RPM for the motors (default 255)

// Create the Kinematics object for differential drive
Kinematics kinematics(motor_max_rpm, wheelRadius * 2, trackWidth, trackWidth,
                      8); // 8-bit PWM resolution

// Callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  // Copy the received data into the myData structure
  memcpy(&myData, incomingData, sizeof(myData));
}

float leftVel = 0;
float rightVel = 0;

void setup() {
  // Initialize Serial Monitor for debugging
  Serial.begin(115200); // ESP32 default baud rate is 115200

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
  // vel x should be in range -3 to 3 and omega -10 to 10
  Kinematics::output motorRPM =
      kinematics.getRPM(myData.vel_x, 0.0, myData.vel_w);
  Kinematics::output motorPWM =
      kinematics.getPWM(myData.vel_x, 0.0, myData.vel_w);

  // Print the calculated wheel velocities to the Serial Monitor
  Serial.print("Left Wheel Velocity (RPM): ");
  Serial.println(motorRPM.motor1);
  Serial.print("Right Wheel Velocity (RPM): ");
  Serial.println(motorRPM.motor2);
  Serial.println("---");

  // Set motor speeds based on the received data
  motor1.setSpeed(50);
  motor2.setSpeed(50);

  // Add a small delay for stability
  delay(100);
}
