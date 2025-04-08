/*
BlackPill.ino - Sensor Controller for Vedrus Robot
В этой программе используется FreeRTOS для организации многозадачности. Вот как взаимодействуют основные компоненты:

### 1. **Структура задач (Tasks)**
Программа создает три основные задачи:

- **vSensorTask** - Основная задача обработки датчиков и управления
- **vSonarTask** - Задача для работы с ультразвуковыми датчиками расстояния
- **vUartTask** - Задача для обмена данными через UART

### 2. **Механизмы синхронизации**
Для координации работы задач используются:

- **Мьютекс (xSensorMutex)**:
  - Защищает общую структуру `sensorData` от одновременного доступа
  - Используется во всех задачах при работе с общими данными

- **Очередь команд (xCommandQueue)**:
  - Передает команды от UART задачи к задаче датчиков
  - Формат команды: 2 байта [канал, значение]

### 3. **Поток данных**

1. **Прием команд (UART → SensorTask)**:
   - `vUartTask` получает команды через Serial в формате `#<channel><value><crc>`
   - После проверки CRC команда помещается в `xCommandQueue`
   - `vSensorTask` извлекает команды из очереди и выполняет:
     - Управление реле (каналы 0-3)
     - Управление PWM (каналы 16-31, если USE_PWM определено)

2. **Опрос датчиков**:
   - `vSensorTask`:
     - Читает BME280 (температура, влажность, давление)
     - Опционально: QMC5883L (магнитометр) и ICM-20948 (IMU)
     - Обновляет состояние реле и PWM в `sensorData`
   - `vSonarTask`:
     - Поочередно опрашивает сонары через `readSonar()`
     - Обновляет расстояния в `sensorData.sonar[]`

3. **Отправка данных (SensorTask → UART)**:
   - `vUartTask` периодически отправляет `sensorData` через Serial:
     - Формат: `@<данные><crc>`
     - Перед отправкой берет мьютекс для безопасного доступа к данным

### 4. **Приоритеты задач**
Задачи создаются с разными приоритетами:
1. `vSensorTask` (приоритет 3) - высший
2. `vSonarTask` (приоритет 2)
3. `vUartTask` (приоритет 1) - низший

Это обеспечивает своевременную обработку датчиков и команд.

### 5. **Временные параметры**
- `vSensorTask`: выполняется каждые 100 мс
- `vSonarTask`: опрашивает сонары каждые 25 мс (циклически для каждого датчика)
- `vUartTask`: проверяет UART и отправляет данные каждые 100 мс

### 6. **Разделяемые данные**
Структура `SensorData_t` - центральное хранилище всех данных, защищенное мьютексом. Содержит:
- Показания всех датчиков
- Состояние реле
- Значения PWM (если включено)

### Пример взаимодействия
1. Пользователь отправляет команду "#\x10\xFF" (установить PWM канал 0 в 255)
2. `vUartTask` получает команду, проверяет CRC и помещает в очередь
3. `vSensorTask` извлекает команду и устанавливает PWM через драйвер
4. При следующем цикле `vSensorTask` обновляет `sensorData.pwmValues[0]`
5. `vUartTask` при следующей отправке включает эти данные в пакет

Такая архитектура обеспечивает:
- Потокобезопасность при работе с общими данными
- Своевременную обработку датчиков
- Минимальные задержки при выполнении команд
- Предсказуемое поведение системы

 * Depends on
 * - stm32_freertos - https://github.com/stm32duino/STM32FreeRTOS
 * - Adafruit_BME280 (optional)
 * - Adafruit MPU6050 (optional)
 * - adafruit pwm servo driver (optional) - https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library
 * - QMC5883LCompass (optional)
 *
 * Connections:
 * 
 * PA0  - SONAR 1 ECHO
 * PA1  - SONAR 2 ECHO
 * PA2  - SONAR 3 ECHO
 * PA3  - SONAR 4 ECHO
 * PA4  - SONAR 1 TRIG
 * PA5  - SONAR 2 TRIG
 * PA6  - SONAR 3 TRIG
 * PA7  - SONAR 4 TRIG
 * PA8  - I2C2 SDA
 * PA9  - UART TX?
 * PA10 - UART RX?
 * PB4  - I2C2 SCL
 * PB6  - RELAY CH1
 * PB7  - RELAY CH2
 * PB8  - RELAY CH3
 * PB9  - RELAY CH4
 */

#include <STM32FreeRTOS.h>
#include "Adafruit_Sensor.h"
#include <Wire.h>
#include <HardwareTimer.h>

// Определения для опциональных датчиков
//#define USE_BME280
//#define USE_QMC5883L
#define USE_MPU6050
//#define USE_PWM
#define DEBUG

#ifdef USE_BME280
#include "Adafruit_BME280.h"
#endif

#ifdef USE_QMC5883L
#include "QMC5883LCompass.h"
#endif

#ifdef USE_MPU6050
#include "Adafruit_MPU6050.h"
#endif

#ifdef USE_PWM
#include "Adafruit_PWMServoDriver.h"
#endif

TwoWire Wire2(PB_4, PA_8);

// SONAR
const uint8_t SONAR_COUNT = 4;
const uint8_t sonarTrigPins[SONAR_COUNT] = {PA4, PA5, PA6, PA7};
const uint8_t sonarEchoPins[SONAR_COUNT] = {PA0, PA1, PA2, PA3};
const unsigned long SONAR_TIMEOUT = 25; 
uint16_t sonarDistances[SONAR_COUNT] = {0};
volatile uint8_t activeSonar = 0;

#pragma pack(push, 1)
typedef struct {
  #ifdef USE_BME280
  float temperature;
  float humidity;
  float pressure;
  #endif
  #ifdef USE_QMC5883L
  int16_t magX, magY, magZ;
  #endif
  #ifdef USE_MPU6050
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  #endif
  uint16_t sonar[SONAR_COUNT];
  uint8_t relayStates;
  #ifdef USE_PWM
  uint16_t pwmValues[16];
  #endif
} SensorData_t;
#pragma pack(pop)

const uint8_t relayPins[] = {PB6, PB7, PB8, PB9};

// Global objects
#ifdef USE_BME280
Adafruit_BME280 bme;
#endif
#ifdef USE_PWM
Adafruit_PWMServoDriver pwm(0x40);
#endif
#ifdef USE_QMC5883L
QMC5883LCompass mag;
#endif
#ifdef USE_MPU6050
Adafruit_MPU6050 mpu;
#endif

// FreeRTOS handles
TaskHandle_t xSensorTask = NULL;
TaskHandle_t xSonarTask = NULL;
TaskHandle_t xUartTask = NULL;
QueueHandle_t xCommandQueue = NULL;
SemaphoreHandle_t xSensorMutex = NULL;

// Shared data
SensorData_t sensorData;
#ifdef USE_PWM
uint16_t currentPwm[16] = {0};
#endif
uint8_t relayState = 0;

// SONARS
void initSonars() {
  for(int i = 0; i < SONAR_COUNT; i++) {
    pinMode(sonarTrigPins[i], OUTPUT);
    digitalWrite(sonarTrigPins[i], LOW);
    pinMode(sonarEchoPins[i], INPUT);
  }
}
uint16_t readSonar(uint8_t sonarIdx) {
  // Variables for pulse timing
  unsigned long duration;
  
  // Send trigger pulse
  digitalWrite(sonarTrigPins[sonarIdx], LOW);
  delayMicroseconds(2);
  digitalWrite(sonarTrigPins[sonarIdx], HIGH);
  delayMicroseconds(10);
  digitalWrite(sonarTrigPins[sonarIdx], LOW);
  
  // Measure echo pulse duration with timeout
  unsigned long startTime = millis();
  
  // Wait for echo pin to go HIGH (start of echo pulse)
  while(digitalRead(sonarEchoPins[sonarIdx]) == LOW) {
    if(millis() - startTime > SONAR_TIMEOUT) {
      return 0; // Timeout occurred, return 0
    }
  }
  
  // Record the start time of the echo pulse
  unsigned long echoStart = micros();
  
  // Wait for echo pin to go LOW (end of echo pulse)
  while(digitalRead(sonarEchoPins[sonarIdx]) == HIGH) {
    vTaskDelay(0);
    if(millis() - startTime > SONAR_TIMEOUT) {
      return 0; // Timeout occurred, return 0
    }
  }
  
  // Calculate pulse duration in microseconds
  duration = micros() - echoStart;
  
  // Convert to distance in cm (sound travels at ~330m/s, or 33cm/ms)
  // The signal travels to the object and back, so we divide by 2
  return duration * 0.033 / 2;
}

// Sonar task for FreeRTOS
void vSonarTask(void *pvParameters) {
  while(1) {
    // Read the active sonar
    uint16_t distance = readSonar(activeSonar);
    
    // Store result in shared data structure
    if(xSemaphoreTake(xSensorMutex, portMAX_DELAY) == pdTRUE) {
      sensorData.sonar[activeSonar] = distance;
      xSemaphoreGive(xSensorMutex);
    }
    
    // Move to next sonar
    activeSonar = (activeSonar + 1) % SONAR_COUNT;
    
    // Delay between measurements
    vTaskDelay(pdMS_TO_TICKS(25)); // 25ms between sonar measurements
  }
}
/// / SONARS


void setup() {
  Serial.begin(115200);
  while (!Serial) {
      // wait for Serial to become active
  }
  Serial.println("begin");

  initSonars();  
  Serial.println("sonar - ok");

  Wire2.begin();
  delay(100);
  Wire2.setClock(400000);
  delay(100);

  #ifdef USE_BME280
  // Initialize BME280
  if (!bme.begin(0x76, &Wire2)) {
    Serial.println("Не удалось найти BME280 на Wire2!");
    while(1);
  }
  Serial.println("bme - ok");
  #endif

  #ifdef USE_MPU6050
  // Initialize MPU6050
  if (!mpu.begin(0x68, &Wire2)) {
    Serial.println("Failed to find MPU6050 chip");
    while(1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.println("mpu6050 - ok");
  #endif

  #ifdef USE_PWM
  // Initialize PWM
  pwm.begin();
  pwm.setPWMFreq(50);
  Serial.println("pwm - ok");
  #endif

  #ifdef USE_QMC5883L
  // Initialize QMC5883L
  mag.init();
  Serial.println("qmc5883 - ok");
  #endif

  // Initialize GPIO
  for (int i = 0; i < 4; i++) {
    //pinMode(relayPins[i], OUTPUT);
    //digitalWrite(relayPins[i], LOW);
  }
  Serial.println("relay - ok");


  // Create RTOS objects
  xSensorMutex = xSemaphoreCreateMutex();
  xCommandQueue = xQueueCreate(10, sizeof(uint8_t[2]));

  // Create tasks
  xTaskCreate(vSensorTask, "Sensor", 1024, NULL, 3, &xSensorTask);
  xTaskCreate(vSonarTask, "Sonar", 512, NULL, 2, &xSonarTask);
  xTaskCreate(vUartTask, "UART", 768, NULL, 1, &xUartTask);

  Serial.println("rtos - ok");

  vTaskStartScheduler();
}

void loop() {}

void vSensorTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1) {
    // Process commands
    uint8_t command[2];
    while (xQueueReceive(xCommandQueue, &command, 0) == pdTRUE) {
      uint8_t channel = command[0];
      uint8_t value = command[1];

      if (channel <= 3) {
        digitalWrite(relayPins[channel], value ? HIGH : LOW);
        relayState = (relayState & ~(1 << channel)) | (value << channel);
      } 
      #ifdef USE_PWM
      else if (channel >= 16 && channel <= 31) {
        uint8_t pwmChan = channel - 16;
        currentPwm[pwmChan] = value * 16;
        pwm.setPWM(pwmChan, 0, currentPwm[pwmChan]);
      }
      #endif
    }

    // Read sensors
    if (xSemaphoreTake(xSensorMutex, portMAX_DELAY) == pdTRUE) {
      #ifdef USE_BME280
      sensorData.temperature = bme.readTemperature();
      sensorData.humidity = bme.readHumidity();
      sensorData.pressure = bme.readPressure() / 100.0F;
      #endif

      #ifdef USE_QMC5883L
      mag.read();
      sensorData.magX = mag.getX();
      sensorData.magY = mag.getY();
      sensorData.magZ = mag.getZ();
      #endif

      #ifdef USE_MPU6050
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      sensorData.accelX = a.acceleration.x;
      sensorData.accelY = a.acceleration.y;
      sensorData.accelZ = a.acceleration.z;
      sensorData.gyroX = g.gyro.x;
      sensorData.gyroY = g.gyro.y;
      sensorData.gyroZ = g.gyro.z;
      #endif

      sensorData.relayStates = relayState;
      #ifdef USE_PWM
      memcpy(sensorData.pwmValues, currentPwm, sizeof(currentPwm));
      #endif

      xSemaphoreGive(xSensorMutex);
    }

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
  }
}

void vUartTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1) {
    // Receive commands
    if (Serial.available() && Serial.read() == '#') {
      char buffer[3]; // Изменено на char вместо uint8_t
      if (Serial.readBytes(buffer, 3) == 3) {
        uint8_t crc = ('#' + (uint8_t)buffer[0] + (uint8_t)buffer[1]) % 256;
        if (crc == (uint8_t)buffer[2]) {
          uint8_t command[2] = {(uint8_t)buffer[0], (uint8_t)buffer[1]};
          xQueueSend(xCommandQueue, command, 0);
        }
      }
    }

    // Send sensor data
    if (xSemaphoreTake(xSensorMutex, portMAX_DELAY) == pdTRUE) {
      #ifdef DEBUG
      // Human-readable debug output
      Serial.println("\n=== SENSOR DATA ===");
      
      #ifdef USE_BME280
      Serial.print("BME280: ");
      Serial.print(sensorData.temperature, 1); Serial.print("°C ");
      Serial.print(sensorData.humidity, 1); Serial.print("% ");
      Serial.print(sensorData.pressure, 1); Serial.println("hPa");
      #endif
      
      #ifdef USE_QMC5883L
      Serial.print("MAG: ");
      Serial.print(sensorData.magX); Serial.print(", ");
      Serial.print(sensorData.magY); Serial.print(", ");
      Serial.print(sensorData.magZ); Serial.println(" uT");
      #endif
      
      #ifdef USE_MPU6050
      Serial.print("ACC: ");
      Serial.print(sensorData.accelX, 2); Serial.print(", ");
      Serial.print(sensorData.accelY, 2); Serial.print(", ");
      Serial.print(sensorData.accelZ, 2); Serial.println(" m/s²");
      Serial.print("GYR: ");
      Serial.print(sensorData.gyroX, 2); Serial.print(", ");
      Serial.print(sensorData.gyroY, 2); Serial.print(", ");
      Serial.print(sensorData.gyroZ, 2); Serial.println(" rad/s");
      #endif
      
      Serial.print("SONAR: ");
      for (int i = 0; i < SONAR_COUNT; i++) {
        Serial.print(sensorData.sonar[i]); Serial.print("cm ");
      }
      Serial.println();
      
      Serial.print("RELAY: 0b");
      for (int i = 3; i >= 0; i--) {
        Serial.print((sensorData.relayStates >> i) & 0x01);
      }
      Serial.println();
      
      #ifdef USE_PWM
      Serial.print("PWM: ");
      for (int i = 0; i < 16; i++) {
        if (i > 0) Serial.print(", ");
        Serial.print(sensorData.pwmValues[i]);
      }
      Serial.println();
      #endif
      
      #else
      // Compact binary-compatible output
      uint8_t crc = 0;
      uint8_t *dataPtr = (uint8_t*)&sensorData;

      Serial.write('@');
      for (size_t i = 0; i < sizeof(SensorData_t); i++) {
        Serial.write(dataPtr[i]);
        crc += dataPtr[i];
      }
      Serial.write(crc % 256);
      #endif // DEBUG

      xSemaphoreGive(xSensorMutex);
    }

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
  }
}


