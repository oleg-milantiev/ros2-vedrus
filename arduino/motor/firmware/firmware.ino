// cmfX - command to move forward at X/256 power
// cmrX - command to move reverse at X/256 power
// cbXX - command to break

// TODO:
// - 

//#define DEBUG
//#define LEFT

// bme280
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
Adafruit_BME280 bme; // I2C

// сонар
#include <NewPing.h>

NewPing sonar1(A1, A0, 200);
NewPing sonar2(A2, 5, 200);

// BLDC
#define BLDC1_PWM 10
#define BLDC1_INT 2
#define BLDC1_DIR 8
#define BLDC1_BRK 6

#define BLDC2_PWM 11
#define BLDC2_INT 3
#define BLDC2_DIR 9
#define BLDC2_BRK 7

#define CURRENT1 A7
#define CURRENT2 A6
#define CURRENT_SYS A3

unsigned status;

void setup() {
  Serial.begin(115200);
  while(!Serial);    // time to get serial running

  // BLDC
  pinMode(BLDC1_PWM, OUTPUT);
  pinMode(BLDC1_DIR, OUTPUT);
  pinMode(BLDC1_BRK, OUTPUT);
  pinMode(BLDC1_INT, INPUT);
  digitalWrite(BLDC1_DIR, 1);

  pinMode(BLDC2_PWM, OUTPUT);
  pinMode(BLDC2_DIR, OUTPUT);
  pinMode(BLDC2_BRK, OUTPUT);
  pinMode(BLDC2_INT, INPUT);
  digitalWrite(BLDC2_DIR, 1);

  attachInterrupt(digitalPinToInterrupt(BLDC1_INT), speed1, RISING);
  attachInterrupt(digitalPinToInterrupt(BLDC2_INT), speed2, RISING);
  
  // bme280
  
  status = bme.begin(0x76);
}

int speed_1 = 0;
int speed_2 = 0;

void speed1() {
  speed_1++;
}
void speed2() {
  speed_2++;
}

byte commandStage = 0;
char command = ' ';
char mode = 'm';         // move mode ['m' = move; 'b' = break]
byte power = 0; // move|break power
byte dir = 0;   // move|break direction [0 = cw; 1 = ccw]

void loop() { 
  while (Serial.available()) {
    byte ch = Serial.read();
  
    switch (commandStage) {
      case 0:
        if (ch == 'c') {
          commandStage = 1;
        }
        break;
  
      case 1:
        command = ch;

        switch (command) {
          case 'm': // move wheels at X power
          case 'b': // break wheels at X power
            commandStage = 2;
            break;

          default:
            commandStage = 0;
        }
        break;

      case 2:
        switch (command) {
          case 'm': // move wheels at X power
          case 'b': // break wheels at X power
            switch (ch) {
              case 'f':
              case 'r':
                mode = command;
                commandStage = 3;
                dir = (ch == 'f');
                break;
    
              default:
                commandStage = 0;
            }
            break;

          default:
            commandStage = 0;
        }
        break;

      case 3:
        power = ch;
        commandStage = 0;
        break;
    }
  }

  // set BLDC state
  #ifdef DEBUG
    Serial.print("BLDC dir = ");
    Serial.print(dir);
    Serial.print(", mode = ");
    Serial.print(mode == 'b');
    Serial.print(", power = ");
    Serial.println(power);

    digitalWrite(BLDC1_DIR, dir);
    digitalWrite(BLDC2_DIR, dir);
  
    digitalWrite(BLDC1_BRK, mode == 'b');
    digitalWrite(BLDC2_BRK, mode == 'b');
  
    analogWrite(BLDC1_PWM, power);
    analogWrite(BLDC2_PWM, power);
  
  #else
    digitalWrite(BLDC1_DIR, dir);
    digitalWrite(BLDC2_DIR, dir);
  
    digitalWrite(BLDC1_BRK, mode == 'b');
    digitalWrite(BLDC2_BRK, mode == 'b');
  
    analogWrite(BLDC1_PWM, power);
    analogWrite(BLDC2_PWM, power);
  #endif
  
  // status
  int current, i, j;

  #ifdef LEFT
    Serial.print("VEDL,");
  #else
    Serial.print("VEDR,");
  #endif

  current = 0;
  for (i = 0; i < 16; i++) {
    current += analogRead(CURRENT1);
  }
  Serial.print(current / 16.);
  Serial.print(",");

  current = 0;
  for (i = 0; i < 16; i++) {
    current += analogRead(CURRENT2);
  }
  Serial.print(current / 16.);
  Serial.print(",");
  
  Serial.print(speed_1);
  Serial.print(",");
  Serial.print(speed_2);
  Serial.print(",");

  current = j = 0;
  for (i = 0; i < 4; i++) {
    int cm = sonar1.ping();

    if (cm > 0) {
      current += cm;
      j++;
    }
  }
//  Serial.print(current / j);
  Serial.print(power);
  Serial.print(",");

  current = j = 0;
  for (i = 0; i < 4; i++) {
    int cm = sonar1.ping();

    if (cm > 0) {
      current += cm;
      j++;
    }
  }
  //Serial.print(current / j);
  Serial.print(dir);
  Serial.print(",");
  
  Serial.print(bme.readTemperature());
  Serial.print(",");

  Serial.print(bme.readHumidity());
  Serial.print(",");

  Serial.print(bme.readPressure() * 0.0075);
  Serial.print(",");

  // отправка как есть. Наверху разбираться вольты это (0..1024) или амперы
  current = 0;
  for (i = 0; i < 16; i++) {
    current += analogRead(CURRENT_SYS);
  }
  Serial.print(current / 16.);
  Serial.println(",END");
}