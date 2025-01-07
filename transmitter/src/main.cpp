#include <Wire.h>
#include <espnow.h>
#include <ESP8266WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// RC DRIVE'S MAC Address - D8:BF:C0:0E:63:05 esp32 24:d7:eb:0f:8c:74
uint8_t broadcastAddress[] = {0x24, 0xd7, 0xeb, 0x0f, 0x8c, 0x74};

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1
#define SCREEN_ADDRESS 0x3C ///< Address of the display (0x3C for 128x64)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Must match the receiver structure
typedef struct struct_message
{
  float vel_x;
  float vel_w;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// Timing variables
unsigned long previousMillis = 0;
unsigned long interval = 50; // 50 ms = 20 times per second

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus)
{
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0)
  {
    Serial.println("Delivery success");
  }
  else
  {
    Serial.println("Delivery fail");
  }
}

void setup()
{
  // Init Serial Monitor
  Serial.begin(9600);
  
  // Initialize the OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Infinite loop if display doesn't initialize
  }
  
  // Clear the display and set up the text
  display.setRotation(2);
  display.clearDisplay();
  display.setTextSize(4);      // Set text size (2x scale)
  display.setTextColor(SSD1306_WHITE); // Set text color to white
  display.setCursor(0, 0);     // Start at the top-left corner

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != 0)
  {
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

void loop()
{
  unsigned long currentMillis = millis();

  // Check if 50 ms have passed (20 times per second)
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis; // Save the last time you sent data
    int potValue = analogRead(A0); // Read the potentiometer value (0 to 1023)

    // Calculate new_value for vel_w (velocity)
    float new_value = (float(potValue-532)/530.0)*10 ;

    // Assign values to the structure
    myData.vel_x = 0; // Set to 0 for now (no change, you can modify as needed)
    myData.vel_w = new_value; // -10 to 10

    // Send the data
    esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

    // Print the potentiometer value to the serial monitor
    Serial.print("Potentiometer Value: ");
    Serial.println(new_value);

    // Display vel_x and vel_w on OLED screen
    display.clearDisplay();  // Clear the display before updating
    display.setCursor(0, 0); // Reset cursor position
    display.setTextSize(1.5);  // Set smaller text size for the display
    display.print("Vel X: ");
    display.println(myData.vel_x, 2);  // Display vel_x value with 2 decimal places

    display.setCursor(0, 20);  // Move to a new line for vel_w
    display.print("Vel W: ");
    display.println(myData.vel_w, 2);  // Display vel_w value with 2 decimal places

    // Update the display to show the text
    display.display();
  }
}
