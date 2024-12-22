/*
 * BluePill.ino - Motor Controller for Vedrus Robot
 * 
 * This firmware implements a motor controller for the Vedrus robot using STM32 BluePill board.
 * It utilizes hardware quadrature encoder support via Timer4 (pins PB6/PB7) for precise
 * position and speed measurement. The controller implements PID control for motor speed
 * regulation and communicates with the main controller via Serial interface.
 * 
 * Hardware Setup:
 * - Encoder: Timer4 (PB6/PB7) in hardware quadrature mode (4x resolution)
 * - Motor Control: PWM output (PA0) and direction control (PA1)
 * - Communication: USB Serial at 115200 baud
 * 
 * Protocol:
 * Commands:
 * - 'S': Stop motor (set target speed to 0)
 * - 'M': Set target speed (followed by signed integer value)
 * 
 * Status Format: SIDE,POS:position,SPD:speed,TGT:target,PWR:power
 * Where SIDE is either VEDL (left) or VEDR (right)
 */

// https://github.com/br3ttb/Arduino-PID-Library
#include <PID_v1.h>
#include <HardwareTimer.h>

// Configuration
const bool IS_LEFT_SIDE = true;  // Set to false for right side
const char* SIDE_NAME = IS_LEFT_SIDE ? "VEDL" : "VEDR";
const int TICKS_WHEEL = 8192;    // Encoder ticks per wheel revolution
const int MAX_PWM = 10;          // Maximum PWM value for motor control

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
double motorPower = 0;      // PWM output (-255 to 255)

// Timer handle for encoder
HardwareTimer *EncoderTimer = new HardwareTimer(3);

// PID parameters
double Kp = 0.5;
double Ki = 0.3;
double Kd = 0.1;

// Create PID instance
PID motorPID(&currentSpeed, &motorPower, &targetSpeed, Kp, Ki, Kd, DIRECT);

// Timing variables
unsigned long lastSpeedCalc = 0;
const unsigned long SPEED_CALC_INTERVAL = 50; // 50ms interval for speed calculation
long lastPosition = 0;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  
  // Configure LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // Turn off LED initially (active LOW)
  
  // Configure motor control pins
  pinMode(MOTOR_DIR, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);
  
  // Configure encoder pins for Timer4
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  
  // Configure Timer4 for hardware encoder mode
  EncoderTimer->pause();
  EncoderTimer->setMode(1, TIMER_ENCODER); // Channel 1
  EncoderTimer->setMode(2, TIMER_ENCODER); // Channel 2
  
  // Set encoder to count both edges for better resolution
  EncoderTimer->setPrescaleFactor(1);
  EncoderTimer->setOverflow(65535);
  EncoderTimer->setCount(32767); // Start at midpoint to handle negative values
  
  EncoderTimer->resume();
  
  // Configure PID
  motorPID.SetMode(AUTOMATIC);
  motorPID.SetOutputLimits(-MAX_PWM, MAX_PWM);
  motorPID.SetSampleTime(SPEED_CALC_INTERVAL);
}

void loop() {
  // Process any incoming serial commands
  processSerialCommands();
  
  // Calculate current speed
  if (millis() - lastSpeedCalc >= SPEED_CALC_INTERVAL) {
    calculateSpeed();
    updateMotor();
    brightLED();
    sendStatus();
    lastSpeedCalc = millis();
  }
}

void calculateSpeed() {
  // Read encoder position using hardware timer
  long newPosition = EncoderTimer->getCount() - 32767;
  
  // Calculate speed in ticks per second
  currentSpeed = ((newPosition - lastPosition) * 1000.0) / SPEED_CALC_INTERVAL;
  lastPosition = newPosition;
}

void updateMotor() {
  // Update PID and apply to motor
  motorPID.Compute();
  
  // Set direction and power based on calculated motor power
  digitalWrite(MOTOR_DIR, motorPower >= 0 ? HIGH : LOW);
  analogWrite(MOTOR_PWM, abs(motorPower));
}

void processSerialCommands() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    switch (cmd) {
      case 'S': // Stop
        targetSpeed = 0;
        break;
        
      case 'M': // Move with signed speed
        // Wait a bit for the full number to arrive
        delay(5);
        if (Serial.available()) {
          targetSpeed = Serial.parseInt();
          // Clear any remaining characters in the buffer
          while (Serial.available()) {
            Serial.read();
          }
        }
        break;
    }
  }
}

void sendStatus() {
  Serial.print(SIDE_NAME);
  Serial.print(",POS:");
  Serial.print(EncoderTimer->getCount() - 32767); // Adjust for midpoint offset
  Serial.print(",SPD:");
  Serial.print(currentSpeed);
  Serial.print(",TGT:");
  Serial.print(targetSpeed);
  Serial.print(",PWR:");
  Serial.println(motorPower);
}

void brightLED() {
  // Turn on LED if speed is greater than half of TICKS_WHEEL
  int absSpeed = abs(currentSpeed);
  bool isHighSpeed = absSpeed > (TICKS_WHEEL / 2);
  
  // PC13 is active LOW on BluePill, so use LOW for ON
  digitalWrite(LED_PIN, isHighSpeed ? LOW : HIGH);
}
