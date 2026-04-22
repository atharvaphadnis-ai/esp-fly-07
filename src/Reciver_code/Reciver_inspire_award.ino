#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

// Define NRF24L01 pins
#define CE_PIN A3
#define CSN_PIN A2

// Define output pins for channels
#define CH1_PIN 2  // Left Joystick X -> BLDC (ESC)
#define CH2_PIN 3  // Left Joystick Y -> Servo
#define CH3_PIN 4  // Right Joystick X -> Servo
#define CH4_PIN 5  // Right Joystick Y -> Servo
#define CH5_PIN 6  // Switch 1 -> Servo
#define CH6_PIN 9  // Switch 2 -> Servo
#define CH7_PIN A0 // Potentiometer 1 -> Servo
#define CH8_PIN A1 // Potentiometer 2 -> Servo

// Create radio object
RF24 radio(CE_PIN, CSN_PIN);

// Create servo objects
Servo ch1_servo;  // Will be used for BLDC ESC
Servo ch2_servo;
Servo ch3_servo;
Servo ch4_servo;
Servo ch5_servo;
Servo ch6_servo;
Servo ch7_servo;
Servo ch8_servo;

// Define the data structure to be received
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

DataPacket rxData;

// Address for communication
const byte address[6] = "00001";

// Connection monitoring variables
unsigned long lastPacketTime = 0;
bool connectionLost = true;

// Safety variables for BLDC
uint16_t lastCh1Value = 1500; // Neutral position for BLDC
bool bldcArmed = false;
unsigned long bldcArmTime = 0;

void setup() {
  Serial.begin(9600);
  
  // Attach servos to pins
  ch1_servo.attach(CH1_PIN);  // BLDC ESC on CH1
  ch2_servo.attach(CH2_PIN);
  ch3_servo.attach(CH3_PIN);
  ch4_servo.attach(CH4_PIN);
  ch5_servo.attach(CH5_PIN);
  ch6_servo.attach(CH6_PIN);
  ch7_servo.attach(CH7_PIN);
  ch8_servo.attach(CH8_PIN);
  
  // Initialize all outputs to neutral position (1500µs)
  ch1_servo.writeMicroseconds(1500); // Critical for BLDC safety
  ch2_servo.writeMicroseconds(1500);
  ch3_servo.writeMicroseconds(1500);
  ch4_servo.writeMicroseconds(1500);
  ch5_servo.writeMicroseconds(1500);
  ch6_servo.writeMicroseconds(1500);
  ch7_servo.writeMicroseconds(1500);
  ch8_servo.writeMicroseconds(1500);
  
  // Initialize radio
  if (!radio.begin()) {
    Serial.println("Radio initialization failed!");
    while (1);
  }
  
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(100);
  radio.openReadingPipe(0, address);
  radio.startListening();
  
  // BLDC safety initialization
  delay(1000); // Wait for ESC to initialize
  ch1_servo.writeMicroseconds(1000); // Send minimum throttle
  delay(2000); // Wait for ESC to recognize
  ch1_servo.writeMicroseconds(1500); // Back to neutral
  
  Serial.println("Receiver Initialized - Waiting for connection...");
}

void processBLDCSafety(uint16_t ch1Value) {
  // BLDC/ESC Safety logic
  static uint16_t safeThrottle = 1000;
  
  if (connectionLost) {
    // If connection lost, set to minimum throttle
    safeThrottle = 1000;
    bldcArmed = false;
  } else {
    // Arm sequence: throttle must go to minimum first
    if (!bldcArmed) {
      if (ch1Value <= 1100) { // Throttle at minimum
        bldcArmed = true;
        bldcArmTime = millis();
        safeThrottle = 1000;
      } else {
        safeThrottle = 1000; // Keep at minimum until armed
      }
    } else {
      // After arming, apply throttle with limits
      if (millis() - bldcArmTime < 2000) {
        // Slow start for 2 seconds after arming
        safeThrottle = constrain(ch1Value, 1000, 1200);
      } else {
        // Normal operation with full range
        safeThrottle = constrain(ch1Value, 1000, 2000);
      }
    }
  }
  
  // Apply the safe throttle value
  ch1_servo.writeMicroseconds(safeThrottle);
}

void updateOutputs() {
  // Update all channels independently
  
  // Channel 1: BLDC with safety processing
  processBLDCSafety(rxData.ch1);
  
  // Channel 2-8: Servos with constraints
  ch2_servo.writeMicroseconds(constrain(rxData.ch2, 1000, 2000));
  ch3_servo.writeMicroseconds(constrain(rxData.ch3, 1000, 2000));
  ch4_servo.writeMicroseconds(constrain(rxData.ch4, 1000, 2000));
  ch5_servo.writeMicroseconds(constrain(rxData.ch5, 1000, 2000));
  ch6_servo.writeMicroseconds(constrain(rxData.ch6, 1000, 2000));
  ch7_servo.writeMicroseconds(constrain(rxData.ch7, 1000, 2000));
  ch8_servo.writeMicroseconds(constrain(rxData.ch8, 1000, 2000));
}

void loop() {
  // Check for incoming data
  if (radio.available()) {
    radio.read(&rxData, sizeof(rxData));
    lastPacketTime = millis();
    
    if (connectionLost) {
      connectionLost = false;
      Serial.println("Connection established!");
    }
    
    // Update outputs
    updateOutputs();
    
    // Optional debug output
    static unsigned long lastPrintTime = 0;
    if (millis() - lastPrintTime > 1000) {
      Serial.print("CH1:");
      Serial.print(rxData.ch1);
      Serial.print(" CH2:");
      Serial.print(rxData.ch2);
      Serial.print(" CH7:");
      Serial.print(rxData.ch7);
      Serial.print(" Connected:");
      Serial.println(rxData.connectionStatus);
      lastPrintTime = millis();
    }
  }
  
  // Check for connection timeout (2 seconds)
  if (millis() - lastPacketTime > 2000 && !connectionLost) {
    connectionLost = true;
    Serial.println("Connection lost! Entering failsafe mode.");
    
    // Failsafe: set all channels to neutral/safe positions
    ch1_servo.writeMicroseconds(1000); // BLDC to minimum throttle
    ch2_servo.writeMicroseconds(1500);
    ch3_servo.writeMicroseconds(1500);
    ch4_servo.writeMicroseconds(1500);
    ch5_servo.writeMicroseconds(1500);
    ch6_servo.writeMicroseconds(1500);
    ch7_servo.writeMicroseconds(1500);
    ch8_servo.writeMicroseconds(1500);
    
    bldcArmed = false; // Disarm BLDC
  }
}