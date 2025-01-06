#include <ESP8266WiFi.h>
#include <espnow.h>

// RC DRIVE'S MAC Address - D8:BF:C0:0E:63:05 esp32 24:d7:eb:0f:8c:74
uint8_t broadcastAddress[] = {0x24, 0xd7, 0xeb, 0x0f, 0x8c, 0x74};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  float vel_x;
  float vel_w;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// Timing variables
unsigned long previousMillis = 0;
unsigned long interval = 50; // 50 ms = 20 times per second

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0) {
    Serial.println("Delivery success");
  } else {
    Serial.println("Delivery fail");
  }
}

void setup() {
  // Init Serial Monitor
  Serial.begin(9600);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Transmitted packet
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
}

void loop() {
  unsigned long currentMillis = millis();

  // Check if 50 ms have passed (20 times per second)
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis; // Save the last time you sent data

    myData.vel_x = 0;
    myData.vel_w = 2;
    esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
  }
}
