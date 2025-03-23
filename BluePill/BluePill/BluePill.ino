/*
 * BluePill.ino - Motor Controller for Vedrus Robot
 * 
 * This firmware implements a motor controller for the Vedrus robot using STM32 BluePill board (STM32F103C6T6).
 * https://embeddeddesignblog.blogspot.com/2022/08/stm32f103c6t6-vs-stm32f103c8t6.html
 * It utilizes hardware quadrature encoder support via Timer for precise
 * position and speed measurement. The controller implements PID control for motor speed
 * regulation and communicates with the main controller via Serial interface.
 * 
 * Hardware Setup:
 * - EncoderLeft: Timer3 (PA6/PA7) in hardware quadrature mode (4x resolution)
 * - EncoderRight: Timer2 (PA0/PA1) in hardware quadrature mode (4x resolution)
 * - Left Motor Control: PWM output (PA2), direction control (PA1), brake control (PA0)
 * - Communication: USB Serial at 115200 baud
 * 
 * Protocol:
 * Commands (right motor):
 * - 'S': Stop motor (set target speed to 0)
 * - 'M': Set target speed (followed by signed integer value)
 * - 'P': Set PID Kp value (followed by float value)
 * - 'I': Set PID Ki value (followed by float value)
 * - 'D': Set PID Kd value (followed by float value)
 * Commands (left motor):
 * - same with lowercase: s, m, i, i, d
 * Command (common): 
 * - X: Set Maximum PWM for both motors
 *
 * Status Format: SIDE,POS:position,SPD:speed,TGT:target,PWR:power,PID:Kp/Ki/Kd
 * Where:
 * - SIDE is either LEFT or RIGHT
 * - position: Current encoder position (ticks)
 * - speed: Current speed (ticks/second)
 * - target: Target speed (ticks/second)
 * - power: Current motor power (-PID_MAX to PID_MAX)
 * - Kp/Ki/Kd: Current PID parameters
 */

// https://github.com/br3ttb/Arduino-PID-Library
#include <PID_v1.h>
#include <HardwareTimer.h>

// Configuration
const int TICKS_WHEEL = 16384;  // Encoder ticks per wheel revolution
int MAX_PWM = 20;               // Initial maximum PWM value for both motors

// Pin definitions
const int LED_PIN = PC13;         // Internal LED pin
const int ENCODER_LEFT_A = PA6;   // Timer 3 Channel 1
const int ENCODER_LEFT_B = PA7;   // Timer 3 Channel 2
const int ENCODER_RIGHT_A = PA0;  // Timer 2 Channel 1
const int ENCODER_RIGHT_B = PA1;  // Timer 2 Channel 2

const int MOTOR_LEFT_DIR = PA5;    // Direction control pin
const int MOTOR_LEFT_PWM = PB13;   // PWM output pin
const int MOTOR_RIGHT_DIR = PB9;   // Direction control pin
const int MOTOR_RIGHT_PWM = PB14;  // PWM output pin

// Motor control variables
double currentSpeedLeft = 0;   // Current speed in ticks per second
double targetSpeedLeft = 0;    // Target speed in ticks per second
double motorPowerLeft = 0;     // PWM output (-MAX_PWM to MAX_PWM)
double currentSpeedRight = 0;  // Current speed in ticks per second
double targetSpeedRight = 0;   // Target speed in ticks per second
double motorPowerRight = 0;    // PWM output (-MAX_PWM to MAX_PWM)

// Timer handle for encoders
HardwareTimer *EncoderTimerLeft = new HardwareTimer(TIM3);
HardwareTimer *EncoderTimerRight = new HardwareTimer(TIM2);
HardwareTimer timer1(TIM1);  // For PWM via Timer1

// PID parameters
double KpLeft = 0.0005;
double KiLeft = 0.008;
double KdLeft = 0.0;
double KpRight = 0.0005;
double KiRight = 0.008;
double KdRight = 0.0;

// Create PID instance
PID motorPIDLeft(&currentSpeedLeft, &motorPowerLeft, &targetSpeedLeft, KpLeft, KiLeft, KdLeft, DIRECT);
PID motorPIDRight(&currentSpeedRight, &motorPowerRight, &targetSpeedRight, KpRight, KiRight, KdRight, DIRECT);

// Timing variables
unsigned long lastSpeedCalc = 0;
const unsigned long SPEED_CALC_INTERVAL = 5;  // 5ms interval for speed calculation
int32_t lastPositionLeft = 0;
int32_t lastPositionRight = 0;

// Encoder overflow tracking
volatile int32_t encoderOverflowsLeft = 0;
volatile int32_t encoderOverflowsRight = 0;

bool isMovingLeft = false;
bool isMovingRight = false;

void handleEncoderOverflowLeft() {
  // Detect overflow direction
  if (EncoderTimerLeft->getCount() < 32767) {
    encoderOverflowsLeft++;
  } else {
    encoderOverflowsLeft--;
  }
}

void handleEncoderOverflowRight() {
  // Detect overflow direction
  if (EncoderTimerRight->getCount() < 32767) {
    encoderOverflowsRight++;
  } else {
    encoderOverflowsRight--;
  }
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial) {}

  // Configure LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // Turn off LED initially (active LOW)

  // Configure motor control pins
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);

  // Use Timer1 for PWM
  // Disable Break Input (PB4)
  pinMode(PB4, OUTPUT);
  digitalWrite(PB4, LOW);

  // Configure encoder pins for Timers (motor's encoders)
  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);

  EncoderTimerLeft->pause();
  EncoderTimerRight->pause();

  // Configure Timer3 for hardware encoder mode
  TIM_HandleTypeDef *htim3 = EncoderTimerLeft->getHandle();
  __HAL_RCC_TIM3_CLK_ENABLE();

  htim3->Instance = TIM3;
  htim3->Init.Prescaler = 0;  // No prescaler
  htim3->Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3->Init.Period = 65535;  // Maximum count
  htim3->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

  // Configure Timer2 for hardware encoder mode
  TIM_HandleTypeDef *htim2 = EncoderTimerRight->getHandle();
  __HAL_RCC_TIM2_CLK_ENABLE();

  htim2->Instance = TIM2;
  htim2->Init.Prescaler = 0;  // No prescaler
  htim2->Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2->Init.Period = 65535;  // Maximum count
  htim2->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

  // Initialize the encoder configuration structure
  TIM_Encoder_InitTypeDef encoderConfigLeft = {
    .EncoderMode = TIM_ENCODERMODE_TI12,  // 4x resolution
    .IC1Polarity = TIM_ICPOLARITY_RISING,
    .IC1Selection = TIM_ICSELECTION_DIRECTTI,
    .IC1Prescaler = TIM_ICPSC_DIV1,
    .IC1Filter = 0,
    .IC2Polarity = TIM_ICPOLARITY_RISING,
    .IC2Selection = TIM_ICSELECTION_DIRECTTI,
    .IC2Prescaler = TIM_ICPSC_DIV1,
    .IC2Filter = 0
  };
  TIM_Encoder_InitTypeDef encoderConfigRight = {
    .EncoderMode = TIM_ENCODERMODE_TI12,  // 4x resolution
    .IC1Polarity = TIM_ICPOLARITY_RISING,
    .IC1Selection = TIM_ICSELECTION_DIRECTTI,
    .IC1Prescaler = TIM_ICPSC_DIV1,
    .IC1Filter = 0,
    .IC2Polarity = TIM_ICPOLARITY_RISING,
    .IC2Selection = TIM_ICSELECTION_DIRECTTI,
    .IC2Prescaler = TIM_ICPSC_DIV1,
    .IC2Filter = 0
  };

  // Pass the address of the properly initialized structure
  if (
    (HAL_TIM_Encoder_Init(htim3, &encoderConfigLeft) != HAL_OK) || (HAL_TIM_Encoder_Init(htim2, &encoderConfigRight) != HAL_OK)) {
    Serial.println("Error initializing encoders!");
    while (1)
      ;  // Halt execution if encoder initialization fails
  }

  HAL_TIM_Encoder_Start(htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(htim2, TIM_CHANNEL_ALL);

  EncoderTimerLeft->setCount(0);   // Start at zero
  EncoderTimerRight->setCount(0);  // Start at zero

  // Enable overflow interrupts
  EncoderTimerLeft->setOverflow(65535);
  EncoderTimerLeft->attachInterrupt(handleEncoderOverflowLeft);
  EncoderTimerRight->setOverflow(65535);
  EncoderTimerRight->attachInterrupt(handleEncoderOverflowRight);

  EncoderTimerLeft->resume();
  EncoderTimerRight->resume();

  // Configure PID
  motorPIDLeft.SetMode(AUTOMATIC);
  motorPIDLeft.SetOutputLimits(-MAX_PWM, MAX_PWM);
  motorPIDLeft.SetSampleTime(SPEED_CALC_INTERVAL);
  motorPIDRight.SetMode(AUTOMATIC);
  motorPIDRight.SetOutputLimits(-MAX_PWM, MAX_PWM);
  motorPIDRight.SetSampleTime(SPEED_CALC_INTERVAL);
}

uint8_t sendStatusCount = 0;

void loop() {
  // Process any incoming serial commands
  processSerialCommands();

  // Calculate current speed at 200 Hz (every 5ms)
  if (millis() - lastSpeedCalc >= SPEED_CALC_INTERVAL) {
    calculateSpeed();
    updateMotors();
    brightLED();
    if (sendStatusCount++ == 20) {  // Send status at 10 Hz
      sendStatus();
      sendStatusCount = 0;
    }
    lastSpeedCalc = millis();
  }
}

void calculateSpeed() {
  // Read encoder position using hardware timer
  uint16_t timerCountLeft = EncoderTimerLeft->getCount();
  int32_t newPositionLeft = (encoderOverflowsLeft << 16) + timerCountLeft;

  currentSpeedLeft = ((newPositionLeft - lastPositionLeft) * 1000.0) / SPEED_CALC_INTERVAL;
  lastPositionLeft = newPositionLeft;

  uint16_t timerCountRight = EncoderTimerRight->getCount();
  int32_t newPositionRight = (encoderOverflowsRight << 16) + timerCountRight;

  currentSpeedRight = ((newPositionRight - lastPositionRight) * 1000.0) / SPEED_CALC_INTERVAL;
  lastPositionRight = newPositionRight;
}

uint8_t hangCountLeft = 0;
uint8_t zeroCountLeft = 0;
uint8_t hangCountRight = 0;
uint8_t zeroCountRight = 0;

void updateMotors() {
  // Update PID and apply to motors
  motorPIDLeft.Compute();
  motorPIDRight.Compute();

  if (!isMovingLeft) {
    motorPowerLeft = 0;
  }
  if (!isMovingRight) {
    motorPowerRight = 0;
  }

  // Set direction and power based on calculated motor power
  digitalWrite(MOTOR_LEFT_DIR, motorPowerLeft >= 0 ? HIGH : LOW);
  digitalWrite(MOTOR_RIGHT_DIR, motorPowerRight >= 0 ? HIGH : LOW);

  /**
   ** Try to prevent controller hangup
   [hang,zero]
   [0,0]..[0,0]..[0,0].. have power and speed, all correct
   no speed, but have power
   count cycles in hangCount: [1,0]..[2,0]..3..4..5..6..7..8..9..[10,0]
   [11,0]. Condition hangCount > 10 is true. Cure power reset to zero
   [12,1].. still no speed, but have power. Continue cure set power to zero. Prevent hangCount overflow by set it to 11
   [12,2].. same
   ..
   [12,6].. same (few cycles)
   zeroCount > 5, so must go back to normal check
   [0,0]
  **/
  if ((abs(motorPowerLeft) > 0.5) && currentSpeedLeft == 0) {
    if (++hangCountLeft > 10) {
      //analogWrite(MOTOR_LEFT_PWM, 0);
      timer1.setPWM(1, PB13, 1000, 0);
      hangCountLeft = 11;  // overflow protection
      if (++zeroCountLeft > 5) {
        // we can hold zero pwm only 5 cycles
        zeroCountLeft = 0;
        hangCountLeft = 0;
        //analogWrite(MOTOR_LEFT_PWM, abs(motorPowerLeft));
        timer1.setPWM(1, PB13, 1000, abs(motorPowerLeft));
      }
    }
  } else {
    //analogWrite(MOTOR_LEFT_PWM, abs(motorPowerLeft));
    timer1.setPWM(1, PB13, 1000, abs(motorPowerLeft));
    hangCountLeft = 0;
  }

  if ((abs(motorPowerRight) > 0.5) && currentSpeedRight == 0) {
    if (++hangCountRight > 10) {
      //analogWrite(MOTOR_RIGHT_PWM, 0);
      timer1.setPWM(2, PB14, 1000, 0);
      hangCountRight = 11;  // overflow protection
      if (++zeroCountRight > 5) {
        // we can hold zero pwm only 5 cycles
        zeroCountRight = 0;
        hangCountRight = 0;
        //analogWrite(MOTOR_RIGHT_PWM, abs(motorPowerRight));
        timer1.setPWM(2, PB14, 1000, abs(motorPowerRight));
      }
    }
  } else {
    //analogWrite(MOTOR_RIGHT_PWM, abs(motorPowerRight));
    timer1.setPWM(2, PB14, 1000, abs(motorPowerRight));
    hangCountRight = 0;
  }
}

void processSerialCommands() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    delay(5);

    switch (cmd) {
      case 's':  // Stop Left
        targetSpeedLeft = 0;
        isMovingLeft = false;
        break;
      case 'S':  // Stop Right
        targetSpeedRight = 0;
        isMovingRight = false;
        break;

      case 'm':  // Move left with signed speed
        if (Serial.available()) {
          targetSpeedLeft = Serial.parseInt();
          isMovingLeft = true;
          while (Serial.available()) Serial.read();
        }
        break;
      case 'M':  // Move right with signed speed
        if (Serial.available()) {
          isMovingRight = true;
          targetSpeedRight = Serial.parseInt();
          while (Serial.available()) Serial.read();
        }
        break;

      case 'p':  // Set Left Kp
        if (Serial.available()) {
          KpLeft = Serial.parseFloat();
          motorPIDLeft.SetTunings(KpLeft, KiLeft, KdLeft);
          while (Serial.available()) Serial.read();
        }
        break;
      case 'P':  // Set Right Kp
        if (Serial.available()) {
          KpRight = Serial.parseFloat();
          motorPIDRight.SetTunings(KpRight, KiRight, KdRight);
          while (Serial.available()) Serial.read();
        }
        break;

      case 'i':  // Set Left Ki
        if (Serial.available()) {
          KiLeft = Serial.parseFloat();
          motorPIDLeft.SetTunings(KpLeft, KiLeft, KdLeft);
          while (Serial.available()) Serial.read();
        }
        break;
      case 'I':  // Set Right Ki
        if (Serial.available()) {
          KiRight = Serial.parseFloat();
          motorPIDRight.SetTunings(KpRight, KiRight, KdRight);
          while (Serial.available()) Serial.read();
        }
        break;

      case 'd':  // Set Left Kd
        if (Serial.available()) {
          KdLeft = Serial.parseFloat();
          motorPIDLeft.SetTunings(KpLeft, KiLeft, KdLeft);
          while (Serial.available()) Serial.read();
        }
        break;
      case 'D':  // Set Right Kd
        if (Serial.available()) {
          KdRight = Serial.parseFloat();
          motorPIDRight.SetTunings(KpRight, KiRight, KdRight);
          while (Serial.available()) Serial.read();
        }
        break;

      case 'X':  // Set MAX_PWM
        if (Serial.available()) {
          MAX_PWM = Serial.parseInt();
          motorPIDLeft.SetOutputLimits(-MAX_PWM, MAX_PWM);
          motorPIDRight.SetOutputLimits(-MAX_PWM, MAX_PWM);
          while (Serial.available()) Serial.read();
        }
        break;
    }
  }
}

void sendStatus() {
  uint16_t timerCountLeft = EncoderTimerLeft->getCount();
  int32_t positionLeft = (encoderOverflowsLeft << 16) + timerCountLeft;
  uint16_t timerCountRight = EncoderTimerRight->getCount();
  int32_t positionRight = (encoderOverflowsRight << 16) + timerCountRight;

  Serial.print("LEFT,POS:");
  Serial.print(positionLeft);
  Serial.print(",SPD:");
  Serial.print(currentSpeedLeft);
  Serial.print(",TGT:");
  Serial.print(targetSpeedLeft);
  Serial.print(",PWR:");
  Serial.print(motorPowerLeft);
  Serial.print(",PID:");
  Serial.print(KpLeft, 4);
  Serial.print("/");
  Serial.print(KiLeft, 4);
  Serial.print("/");
  Serial.print(KdLeft, 4);
  Serial.print(",MAX:");
  Serial.println(MAX_PWM);

  Serial.print("RIGHT,POS:");
  Serial.print(positionRight);
  Serial.print(",SPD:");
  Serial.print(currentSpeedRight);
  Serial.print(",TGT:");
  Serial.print(targetSpeedRight);
  Serial.print(",PWR:");
  Serial.print(motorPowerRight);
  Serial.print(",PID:");
  Serial.print(KpRight, 4);
  Serial.print("/");
  Serial.print(KiRight, 4);
  Serial.print("/");
  Serial.print(KdRight, 4);
  Serial.print(",MAX:");
  Serial.println(MAX_PWM);
}

void brightLED() {
  // Turn on LED if speed is greater than half of TICKS_WHEEL
  int absSpeedLeft = abs(currentSpeedLeft);
  int absSpeedRight = abs(currentSpeedRight);
  bool isHighSpeed = (absSpeedLeft > (TICKS_WHEEL / 2)) || (absSpeedRight > (TICKS_WHEEL / 2));

  // PC13 is active LOW on BluePill, so use LOW for ON
  digitalWrite(LED_PIN, isHighSpeed ? LOW : HIGH);
}
