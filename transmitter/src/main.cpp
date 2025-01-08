#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <espnow.h>

// Pin definitions for buttons
#define BUTTON_LEFT D5
#define BUTTON_RIGHT D6

// Dead zone threshold for joystick sensitivity
#define DEAD_ZONE 10

// Variable to store the connection status
String drive_status;

// RC DRIVE'S MAC Address (Target device for ESP-NOW communication)
uint8_t broadcastAddress[] = {0x24, 0xd7, 0xeb, 0x0f, 0x8c, 0x74};

// OLED Display Configuration
#define SCREEN_WIDTH 128    // OLED display width in pixels
#define SCREEN_HEIGHT 64    // OLED display height in pixels
#define OLED_RESET -1       // Reset pin (-1 if not used)
#define SCREEN_ADDRESS 0x3C // I2C address of the OLED display

// Initialize the display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Define a structure to hold velocity data
typedef struct struct_message {
  float vel_x; // Linear velocity
  float vel_w; // Angular velocity
} struct_message;

// Create an instance of the velocity structure
struct_message myData;

// Timing variables for non-blocking updates
unsigned long previousMillis = 0;
unsigned long interval =
    50; // Interval for sending data (50ms = 20 updates per second)

/**
 * Callback function for ESP-NOW data transmission status.
 * Updates the drive_status variable based on the send status.
 */
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  if (sendStatus == 0) {
    drive_status = "Connected"; // Data successfully sent
  } else {
    drive_status = "Disconnected"; // Data transmission failed
  }
}

/**
 * Setup function - runs once when the microcontroller starts.
 * Initializes serial communication, buttons, OLED display, and ESP-NOW
 * communication.
 */
void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);

  // Configure buttons with pull-up resistors
  pinMode(BUTTON_LEFT, INPUT_PULLUP);
  pinMode(BUTTON_RIGHT, INPUT_PULLUP);

  // Initialize OLED Display
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Halt execution if display initialization fails
  }

  // Set initial display properties
  display.setRotation(2);
  display.clearDisplay();
  display.setTextSize(4);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  // Set the ESP8266 Wi-Fi mode to Station
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW communication
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Set the ESP-NOW role and register the send callback
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_register_send_cb(OnDataSent);

  // Add the peer device (receiver) for ESP-NOW communication
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
}

/**
 * Main loop - runs continuously after setup.
 * Reads joystick input, button states, and sends data via ESP-NOW.
 * Updates the OLED display with velocity data and connection status.
 */
void loop() {
  unsigned long currentMillis = millis();

  // Check if the update interval has passed
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis; // Update the last execution time

    // Read analog joystick value
    int potValue = analogRead(A0); // Range: 0 - 1024

    /*
     * Offset adjustment for joystick calibration.
     * Ensures the joystick's neutral position corresponds to zero velocity.
     */
    float offset = 30.0;
    float x_axis_pot =
        potValue - 512 - offset; // Normalize to range -512 to +512

    // Apply dead zone to ignore small joystick movements
    if (x_axis_pot > -DEAD_ZONE && x_axis_pot < DEAD_ZONE) {
      x_axis_pot = 0;
    }

    // Scale joystick value to a range of -10 to +10
    x_axis_pot = (x_axis_pot / 512.0) * 10;

    // Read button states for angular velocity control
    int button_left_state = digitalRead(BUTTON_LEFT);
    int button_right_state = digitalRead(BUTTON_RIGHT);

    /*
     * Determine angular velocity based on button presses:
     * - Left button pressed: Positive angular velocity
     * - Right button pressed: Negative angular velocity
     * - No button or both buttons pressed: No angular velocity
     */
    if (button_left_state == LOW && button_right_state == HIGH) {
      myData.vel_w = 3; // Only left button pressed
      Serial.println("Right");
    } else if (button_right_state == LOW && button_left_state == HIGH) {
      myData.vel_w = -3; // Only right button pressed
      Serial.println("Left");
    } else {
      myData.vel_w = 0; // No angular velocity
    }

    // Assign the joystick (linear velocity) value to the structure
    myData.vel_x = x_axis_pot;

    // Send velocity data via ESP-NOW
    esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

    // Update the OLED display with current status and velocities
    display.clearDisplay(); // Clear the previous display contents
    display.setCursor(0, 0);
    display.print("Status: " + drive_status);

    display.setCursor(0, 20); // Display linear velocity
    display.setTextSize(1.5);
    display.print("Vel X: ");
    display.println(myData.vel_x, 2);

    display.setCursor(0, 40); // Display angular velocity
    display.print("Vel W: ");
    display.println(myData.vel_w, 2);

    // Refresh the OLED display to show the updated content
    display.display();
  }
}
