#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Define NRF24L01 pins
#define CE_PIN 7
#define CSN_PIN 8

// Define channel pins
#define SWITCH_1 2
#define SWITCH_2 3
#define POT_1 A4
#define POT_2 A5
#define JOY_L_X A2
#define JOY_L_Y A3
#define JOY_R_X A0
#define JOY_R_Y A1
#define BUZZER_PIN 9

// Create radio object
RF24 radio(CE_PIN, CSN_PIN);

// Define the data structure to be transmitted
struct DataPacket {
  uint16_t ch1; // Left Joystick X
  uint16_t ch2; // Left Joystick Y  
  uint16_t ch3; // Right Joystick X
  uint16_t ch4; // Right Joystick Y
  uint16_t ch5; // Switch 1
  uint16_t ch6; // Switch 2
  uint16_t ch7; // Potentiometer 1
  uint16_t ch8; // Potentiometer 2
  uint8_t connectionStatus; // Connection status flag
};

DataPacket txData;

// Address for communication
const byte address[6] = "00001";

// Connection monitoring variables
unsigned long lastConnectionTime = 0;
bool connectionLost = false;

void setup() {
  Serial.begin(9600);
  
  // Initialize pins
  pinMode(SWITCH_1, INPUT_PULLUP);
  pinMode(SWITCH_2, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  
  // Initialize radio
  if (!radio.begin()) {
    Serial.println("Radio initialization failed!");
    while (1);
  }
  
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(100);
  radio.openWritingPipe(address);
  radio.stopListening();
  
  // Initialize data structure
  memset(&txData, 0, sizeof(txData));
  
  Serial.println("Transmitter Initialized");
}

void readControls() {
  // Read joysticks (map from 0-1023 to 1000-2000 like RC standard)
  txData.ch1 = map(analogRead(JOY_L_X), 0, 1023, 1000, 2000);
  txData.ch2 = map(analogRead(JOY_L_Y), 0, 1023, 1000, 2000);
  txData.ch3 = map(analogRead(JOY_R_X), 0, 1023, 1000, 2000);
  txData.ch4 = map(analogRead(JOY_R_Y), 0, 1023, 1000, 2000);
  
  // Read switches (invert because INPUT_PULLUP gives HIGH when open)
  txData.ch5 = digitalRead(SWITCH_1) ? 1000 : 2000;
  txData.ch6 = digitalRead(SWITCH_2) ? 1000 : 2000;
  
  // Read potentiometers
  txData.ch7 = map(analogRead(POT_1), 0, 1023, 1000, 2000);
  txData.ch8 = map(analogRead(POT_2), 0, 1023, 1000, 2000);
  
  // Add connection status
  txData.connectionStatus = 1;
}

void connectionBeep() {
  // Beep pattern: two short beeps
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
  delay(50);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
}

void loop() {
  static unsigned long lastBeepTime = 0;
  static unsigned long lastTxTime = 0;
  
  readControls();
  
  // Try to send data
  bool txSuccess = radio.write(&txData, sizeof(txData));
  
  if (txSuccess) {
    lastConnectionTime = millis();
    connectionLost = false;
    digitalWrite(BUZZER_PIN, LOW); // Ensure buzzer is off when connected
  }
  
  // Check connection status
  if (millis() - lastConnectionTime > 1000) { // 1 second timeout
    connectionLost = true;
  }
  
  // Handle beeping when connection is lost
  if (connectionLost) {
    if (millis() - lastBeepTime > 2000) { // Beep every 2 seconds
      connectionBeep();
      lastBeepTime = millis();
    }
  }
  
  // Transmission rate control (approx 50Hz)
  unsigned long currentMillis = millis();
  if (currentMillis - lastTxTime < 20) {
    delay(20 - (currentMillis - lastTxTime));
  }
  lastTxTime = millis();
  
  // Debug output (optional)
  if (millis() % 1000 < 20) {
    Serial.print("CH1:");
    Serial.print(txData.ch1);
    Serial.print(" CH2:");
    Serial.print(txData.ch2);
    Serial.print(" Status:");
    Serial.println(connectionLost ? "Lost" : "Connected");
  }
}