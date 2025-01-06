#include <ESP8266WiFi.h>
#include <espnow.h>
 #include "CytronMotorDriver.h"


// Structure example to receive data
// Must match the sender structure
typedef struct struct_message
{
  int vel_x;
  int vel_w;
} struct_message;

// Create a struct_message called myData
struct_message myData;

CytronMD motor1(PWM_PWM, D5, D6);   // PWM 1A = Pin 3, PWM 1B = Pin 9.
CytronMD motor2(PWM_PWM, D7, D8); // PWM 2A = Pin 10, PWM 2B = Pin 11.


// Callback function that will be executed when data is received
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Char: ");
  Serial.println(myData.vel_w);
  Serial.print("Int: ");
  Serial.println(myData.vel_x);
  Serial.println();
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  motor1.setSpeed(255);   // Motor 1 runs forward at full speed.
  motor2.setSpeed(-255);
}