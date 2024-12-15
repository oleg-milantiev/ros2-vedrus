/*
 * encoder_test.ino - Hardware Encoder Test for STM32 BluePill
 * 
 * This test firmware demonstrates the use of STM32's hardware quadrature encoder interface
 * using Timer4. It prints encoder position via Serial at 115200 baud rate.
 * 
 * Hardware Setup:
 * Encoder: Timer3 (PA6/PA7)
 * Uses hardware quadrature mode with 4x resolution
 */

#include <HardwareTimer.h>

// Define encoder pins
const int ENC_A = PA6;  // Timer3 Channel 1
const int ENC_B = PA7;  // Timer3 Channel 2

// Timer handle for encoder
HardwareTimer *EncoderTimer = new HardwareTimer(3);

// Last update time
unsigned long lastPrintTime = 0;
const int printInterval = 100;  // Print every 100ms

void setup() {
  // Initialize Serial communication
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect
  }
  
  // Configure encoder pins
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  
  // Configure Timer4 for Encoder
  EncoderTimer->pause();
  EncoderTimer->setMode(1, TIMER_ENCODER); // Channel 1
  EncoderTimer->setMode(2, TIMER_ENCODER); // Channel 2
  
  // Set encoder to count on both edges of both channels (4x resolution)
  EncoderTimer->setPrescaleFactor(1);
  EncoderTimer->setOverflow(65535);
  EncoderTimer->setCount(32767); // Start at midpoint
  
  EncoderTimer->resume();
  
  Serial.println("Hardware Encoder Test Started");
  Serial.println("Format: Encoder_Position");
}

void loop() {
  // Print encoder value periodically
  if (millis() - lastPrintTime >= printInterval) {
    // Get encoder position and adjust for midpoint offset
    long pos = EncoderTimer->getCount() - 32767;
    
    Serial.println(pos);
    
    lastPrintTime = millis();
  }
}
