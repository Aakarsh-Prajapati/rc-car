#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <espnow.h>

// Pin definitions for buttons
#define BUTTON_LEFT D6
#define BUTTON_RIGHT D5

// Dead zone threshold for joystick sensitivity
#define DEAD_ZONE 10

// OLED Display Configuration
#define SCREEN_WIDTH 128    // OLED display width in pixels
#define SCREEN_HEIGHT 64    // OLED display height in pixels
#define OLED_RESET -1       // Reset pin (-1 if not used)
#define SCREEN_ADDRESS 0x3C // I2C address of the OLED display

// RC DRIVE'S MAC Address (Target device for ESP-NOW communication)
uint8_t broadcastAddress[] = {0x24, 0xd7, 0xeb, 0x0f, 0x8c, 0x74};

// UART timeout configuration
#define UART_TIMEOUT 100 // Timeout in milliseconds for UART data reception

// Timing variables for non-blocking updates
#define INTERVAL 50 // Interval for sending data (50ms = 20 updates per second)

// Declare global variables
String drive_status;
float UART_ON = false;
String recived_data_from_UART = "";
unsigned long previous_UART_recived = 0;
unsigned long previousMillis = 0;
unsigned long interval = INTERVAL;

// Initialize the display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Define a structure to hold velocity data
typedef struct struct_message {
  float vel_x; // Linear velocity
  float vel_w; // Angular velocity
} struct_message;

// Create an instance of the velocity structure
struct_message myData;

/**
 * Callback function for ESP-NOW data transmission status.
 * Updates the drive_status variable based on the send status.
 */
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  if (sendStatus == 0) {
    drive_status = "UP"; // Data successfully sent
  } else {
    drive_status = "DOWN"; // Data transmission failed
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
 * Function to parse the data when the delimiter '|' is encountered.
 */
void parseData(String data) {
  if (data.startsWith("x")) {
    myData.vel_x = data.substring(1).toFloat();
  } else if (data.startsWith("w")) {
    myData.vel_w = data.substring(1).toFloat();
  } else if (data.startsWith("m")) {
    // Handle 'm' data if necessary
  } else {
    // Handle other cases if necessary
  }
}

/**
 * Main loop - runs continuously after setup.
 * Reads joystick input, button states, and sends data via ESP-NOW.
 * Updates the OLED display with velocity data and connection status.
 */
void loop() {
  unsigned long currentMillis = millis();

  /*
   * Read data coming from UART and set UART_ON to true if
   * data is incoming, else false.
   */
  if (Serial.available() > 0) {
    char incomingByte = Serial.read();
    recived_data_from_UART += incomingByte;
    previous_UART_recived = currentMillis;

    // If we encounter a '|' delimiter, process the data
    if (incomingByte == '|') {
      parseData(recived_data_from_UART);
      recived_data_from_UART = "";
    }
  }

  // Check if UART data timeout has occurred
  if (currentMillis - previous_UART_recived <= UART_TIMEOUT) {
    UART_ON = true;
  } else {
    UART_ON = false;
  }

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
     * Determine angular velocity based on button presses
     * and when no data is coming from UART:
     * - Left button pressed: Positive angular velocity
     * - Right button pressed: Negative angular velocity
     * - No button or both buttons pressed: No angular velocity
     */
    if (UART_ON == false) {
      if (button_left_state == LOW && button_right_state == HIGH) {
        myData.vel_w = 7; // Left turn
      } else if (button_right_state == LOW && button_left_state == HIGH) {
        myData.vel_w = -7; // Right turn
      } else {
        myData.vel_w = 0; // No angular velocity
      }
    }

    // Assign the joystick (linear velocity) value to the structure
    if (UART_ON == false) {
      myData.vel_x = x_axis_pot;
    }

    // Send velocity data via ESP-NOW
    esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

    // Update the OLED display with current status and velocities
    display.clearDisplay(); // Clear the previous display contents
    display.setCursor(0, 0);

    String uart_msg;
    if (UART_ON) {
      uart_msg = "UP";
    } else {
      uart_msg = "DOWN";
    }
    display.print("Uart:" + uart_msg + " " + "Drive:" + drive_status);
    display.setCursor(0, 20); // Display linear velocity
    display.setTextSize(1);
    display.print("Vel X: ");
    display.println(myData.vel_x, 2);

    display.setCursor(0, 40); // Display angular velocity
    display.print("Vel W: ");
    display.println(myData.vel_w, 2);

    // Refresh the OLED display to show the updated content
    display.display();
  }
}
