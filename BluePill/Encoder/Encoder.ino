/*
 * encoder_test.ino - Hardware Encoder Test for STM32 BluePill
 * 
 * This test firmware demonstrates the use of STM32's hardware quadrature encoder interface
 * using Timer3. It prints encoder position via Serial at 115200 baud rate.
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
HardwareTimer *EncoderTimer = new HardwareTimer(TIM3);

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

  // Configure Timer3 for hardware encoder mode
  TIM_HandleTypeDef *htim3 = EncoderTimer->getHandle();
  __HAL_RCC_TIM3_CLK_ENABLE();

  htim3->Instance = TIM3;
  htim3->Init.Prescaler = 0; // No prescaler
  htim3->Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3->Init.Period = 65535; // Maximum count
  htim3->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

  // Initialize the encoder configuration structure
  TIM_Encoder_InitTypeDef encoderConfig = {
      .EncoderMode = TIM_ENCODERMODE_TI12,          // 4x resolution
      .IC1Polarity = TIM_ICPOLARITY_RISING,
      .IC1Selection = TIM_ICSELECTION_DIRECTTI,
      .IC1Prescaler = TIM_ICPSC_DIV1,
      .IC1Filter = 0,
      .IC2Polarity = TIM_ICPOLARITY_RISING,
      .IC2Selection = TIM_ICSELECTION_DIRECTTI,
      .IC2Prescaler = TIM_ICPSC_DIV1,
      .IC2Filter = 0};

  // Pass the address of the properly initialized structure
  if (HAL_TIM_Encoder_Init(htim3, &encoderConfig) != HAL_OK) {
    Serial.println("Error initializing encoder!");
    while (1); // Halt execution if encoder initialization fails
  }

  HAL_TIM_Encoder_Start(htim3, TIM_CHANNEL_ALL);

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
