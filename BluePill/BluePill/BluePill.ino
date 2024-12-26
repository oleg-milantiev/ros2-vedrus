/*
 * BluePill.ino - Motor Controller for Vedrus Robot
 * 
 * This firmware implements a motor controller for the Vedrus robot using STM32 BluePill board.
 * It utilizes hardware quadrature encoder support via Timer for precise
 * position and speed measurement. The controller implements PID control for motor speed
 * regulation and communicates with the main controller via Serial interface.
 * 
 * Hardware Setup:
 * - Encoder: Timer3 (PA6/PA7) in hardware quadrature mode (4x resolution)
 * - Motor Control: PWM output (PA2), direction control (PA1), brake control (PA0)
 * - Communication: USB Serial at 115200 baud
 * 
 * Protocol:
 * Commands:
 * - 'S': Stop motor (set target speed to 0)
 * - 'M': Set target speed (followed by signed integer value)
 * - 'P': Set PID Kp value (followed by float value)
 * - 'I': Set PID Ki value (followed by float value)
 * - 'D': Set PID Kd value (followed by float value)
 * 
 * Status Format: SIDE,POS:position,SPD:speed,TGT:target,PWR:power,PID:Kp/Ki/Kd
 * Where:
 * - SIDE is either VEDL (left) or VEDR (right)
 * - position: Current encoder position (ticks)
 * - speed: Current speed (ticks/second)
 * - target: Target speed (ticks/second)
 * - power: Current motor power (-PID_MAX to PID_MAX)
 * - Kp/Ki/Kd: Current PID parameters
 */

// https://github.com/br3ttb/Arduino-PID-Library
#include <PID_v1.h>
#include <Wire.h>
#include <HardwareTimer.h>

#define SLAVE_ADDRESS 0x08  // STM32 I2C address

// Configuration
const bool IS_LEFT_SIDE = true;  // Set to false for right side
const char* SIDE_NAME = IS_LEFT_SIDE ? "VEDL" : "VEDR";
const int TICKS_WHEEL = 16384;    // Encoder ticks per wheel revolution
const int MAX_PWM = 20;          // Maximum PWM value for motor control

// Pin definitions
const int LED_PIN = PC13;   // Internal LED pin
const int ENCODER_A = PA6;  // Timer 3 Channel 1
const int ENCODER_B = PA7;  // Timer 3 Channel 2
const int MOTOR_BRK = PA0;  // Break control pin TBD
const int MOTOR_DIR = PA1;  // Direction control pin
const int MOTOR_PWM = PA2;  // PWM output pin

// Motor control variables
double currentSpeed = 0;    // Current speed in ticks per second
double targetSpeed = 0;     // Target speed in ticks per second
double motorPower = 0;      // PWM output (-MAX_PWM to MAX_PWM)

// Timer handle for encoder
HardwareTimer *EncoderTimer = new HardwareTimer(TIM3);

// PID parameters
double Kp = 0.01;
double Ki = 0.03;
double Kd = 0.0;

// Create PID instance
PID motorPID(&currentSpeed, &motorPower, &targetSpeed, Kp, Ki, Kd, DIRECT);

// Timing variables
unsigned long lastSpeedCalc = 0;
const unsigned long SPEED_CALC_INTERVAL = 5; // 50ms interval for speed calculation
int32_t lastPosition = 0;

// Encoder overflow tracking
volatile int32_t encoderOverflows = 0;

void handleEncoderOverflow() {
  // Detect overflow direction
  if (EncoderTimer->getCount() < 32767) {
    encoderOverflows++;
  } else {
    encoderOverflows--;
  }
}

void receiveEvent(int howMany) {
    while (Wire.available()) {
        char received = Wire.read();  // Read data from Orange Pi
        Serial.print("Received: ");
        Serial.println(received);
    }
}

void requestEvent() {
    Wire.write("STM32 OK");  // Send data to Orange Pi
}

void setup() {
  // Initialize STM32 as I2C slave
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial) {}

  // Configure LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // Turn off LED initially (active LOW)
  
  // Configure motor control pins
  pinMode(MOTOR_DIR, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);
  
  // Configure encoder pins for Timer4
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  
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

  EncoderTimer->setCount(0);     // Start at zero
  
  // Enable overflow interrupts
  EncoderTimer->setOverflow(65535);
  EncoderTimer->attachInterrupt(handleEncoderOverflow);

  EncoderTimer->resume();

  // Configure PID
  motorPID.SetMode(AUTOMATIC);
  motorPID.SetOutputLimits(-MAX_PWM, MAX_PWM);
  motorPID.SetSampleTime(SPEED_CALC_INTERVAL);
}

uint8_t sendStatusCount = 0;

void loop() {
  // Process any incoming serial commands
  processSerialCommands();
  
  // Calculate current speed
  if (millis() - lastSpeedCalc >= SPEED_CALC_INTERVAL) {
    calculateSpeed();
    updateMotor();
    brightLED();
    if (sendStatusCount++ == 10) {
      sendStatus();
      sendStatusCount = 0;
    }
    lastSpeedCalc = millis();
  }
}

void calculateSpeed() {
  // Read encoder position using hardware timer
  uint16_t timerCount = EncoderTimer->getCount();
  int32_t newPosition = (encoderOverflows << 16) + timerCount;

  currentSpeed = ((newPosition - lastPosition) * 1000.0) / SPEED_CALC_INTERVAL;
  
  lastPosition = newPosition;
}

uint8_t hangCount = 0;

void updateMotor() {
  // Update PID and apply to motor
  motorPID.Compute();
  
  // Set direction and power based on calculated motor power
  digitalWrite(MOTOR_DIR, motorPower >= 0 ? HIGH : LOW);
  
  // Try to prevent controller hangup
  if ((abs(motorPower) > 0) && currentSpeed == 0 && ++hangCount > 10) {
    analogWrite(MOTOR_PWM, 0);
    hangCount = 0;
  }
  else {
    analogWrite(MOTOR_PWM, abs(motorPower));
  }
}

void processSerialCommands() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    delay(5);
    
    switch (cmd) {
      case 'S': // Stop
        targetSpeed = 0;
        break;
        
      case 'M': // Move with signed speed
        if (Serial.available()) {
          targetSpeed = Serial.parseInt();
          while (Serial.available()) Serial.read();
        }
        break;

      case 'P': // Set Kp
        if (Serial.available()) {
          Kp = Serial.parseFloat();
          motorPID.SetTunings(Kp, Ki, Kd);
          while (Serial.available()) Serial.read();
        }
        break;

      case 'I': // Set Ki
        if (Serial.available()) {
          Ki = Serial.parseFloat();
          motorPID.SetTunings(Kp, Ki, Kd);
          while (Serial.available()) Serial.read();
        }
        break;

      case 'D': // Set Kd
        if (Serial.available()) {
          Kd = Serial.parseFloat();
          motorPID.SetTunings(Kp, Ki, Kd);
          while (Serial.available()) Serial.read();
        }
        break;
    }
  }
}

void sendStatus() {
  uint16_t timerCount = EncoderTimer->getCount();
  int32_t position = (encoderOverflows << 16) + timerCount;
  
  Serial.print(SIDE_NAME);
  Serial.print(",POS:");
  Serial.print(position);
  Serial.print(",SPD:");
  Serial.print(currentSpeed);
  Serial.print(",TGT:");
  Serial.print(targetSpeed);
  Serial.print(",PWR:");
  Serial.print(motorPower);
  Serial.print(",PID:");
  Serial.print(Kp, 3);
  Serial.print("/");
  Serial.print(Ki, 3);
  Serial.print("/");
  Serial.println(Kd, 3);
}

void brightLED() {
  // Turn on LED if speed is greater than half of TICKS_WHEEL
  int absSpeed = abs(currentSpeed);
  bool isHighSpeed = absSpeed > (TICKS_WHEEL / 2);
  
  // PC13 is active LOW on BluePill, so use LOW for ON
  digitalWrite(LED_PIN, isHighSpeed ? LOW : HIGH);
}
