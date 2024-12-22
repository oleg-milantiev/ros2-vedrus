# Blue Pill DC Motor Controller

This project implements a DC motor controller using the STM32F103C8T6 (Blue Pill) board. It features hardware quadrature encoder support, PID control, and serial communication for real-time control and monitoring.

## Features

- Hardware quadrature encoder support using STM32 Timer4
- 4x resolution encoder counting (both edges of both channels)
- PID-based motor speed control
- Serial communication at 115200 baud
- Real-time status reporting
- Configurable motor direction and speed
- Speed control in encoder ticks per second
- Hardware debouncing for reliable encoder readings

## Hardware Requirements

- STM32F103C8T6 (Blue Pill) board
- DC motor with quadrature encoder
- Motor driver (capable of DIR/PWM control)
- USB-to-Serial adapter (if not built into your programmer)

## Pin Connections

### Encoder
- Encoder A → PA6 (Timer3 Channel 1)
- Encoder B → PA7 (Timer3 Channel 2)

### Motor Control
- Direction → PA1
- PWM → PA2
- Brake → PA0 (TBD)

## Included Files

1. `Blink.ino` - Firmware for board testing
2. `BluePill.ino` - Main motor controller firmware
3. `Encoder.ino` - Test firmware for encoder verification
   - Supports one encoder:
     - Encoder 1: Timer3 (PA6/PA7)

## Software Requirements

1. Arduino IDE
2. STM32 board package for Arduino
3. PID_v1 library

## Installation

1. Install Arduino IDE from [arduino.cc](https://www.arduino.cc/en/software)
2. Install STM32 board package:
   - Open Arduino IDE
   - Go to Tools → Board → Boards Manager
   - Search for "STM32"
   - Install "STM32 MCU based boards"

3. Install PID Library:
   - Go to Tools → Manage Libraries
   - Search for "PID"
   - Install "PID" by Brett Beauregard

4. Configure Board Settings:
   - Board: "Generic STM32F103C series"
   - Upload Method: Your preferred method (ST-Link, Serial, etc.)
   - CPU Speed: "72MHz (Normal)"

5. Board Programming:
   - Set Boot0 jumper to 1
   - Connect board to computer
   - Press reset button on the board to prepare for upload
   - Click upload button in Arduino IDE
   - Set Boot0 jumper back to 0 (testing can be done without this step)

## Usage

### Serial Commands

The controller accepts the following commands through serial port at 115200 baud:

- `S` - Stop motor (sets target speed to 0)
- `M[speed]` - Move at specified speed (speed value depends on encoder resolution)
  - Positive values move forward
  - Negative values move reverse
  - Example: `M5` sets speed to 5 units forward
  - Example: `M-5` sets speed to 5 units reverse

### Python Control Script

The project includes a Python script (`serial_control.py`) for easy motor control:

- Press 'S' to stop the motor
- Press '0-9' to set different speed levels (multiplied by 5)
- Press 'Q' to quit the program
- Real-time status messages are displayed automatically

To run the script:
```bash
python3 serial_control.py
```

### Status Messages

The controller sends status messages every 50ms in the following format:
```
SIDE,POS:[position],SPD:[current_speed],TGT:[target_speed],PWR:[motor_power]
```

Where:
- `SIDE`: "VEDL" for left motor, "VEDR" for right motor
- `position`: Current encoder position (signed value, centered at 32767)
- `current_speed`: Actual speed in ticks/second
- `target_speed`: Commanded speed
- `motor_power`: PWM output (-PWR_MAX to PWR_MAX)

## Hardware Configuration

- Encoder resolution: 8192 ticks per wheel revolution
- Maximum PWM value: 10
- Speed calculation interval: 50ms
- LED indication: PC13 lights up when speed exceeds half of encoder resolution

## Testing Encoder Hardware

The `encoder_test.ino` firmware can be used to verify proper encoder operation:

1. Upload `encoder_test.ino` to the BluePill
2. Open Serial Monitor at 115200 baud
3. Rotate encoders manually or using motors
4. Monitor the position values for both encoders
   - Format: `encoder1_position,encoder2_position`
   - Positive values for forward rotation
   - Negative values for reverse rotation

## PID Tuning

The default PID values are:
- Kp = 0.5
- Ki = 0.3
- Kd = 0.1

You may need to adjust these values based on your specific setup. To tune:
1. Start with all values at 0
2. Increase Kp until you get a reasonable response
3. Increase Ki to eliminate steady-state error
4. Add Kd to reduce overshoot if necessary

## Troubleshooting

1. **Motor doesn't move**:
   - Check power connections
   - Verify motor driver connections
   - Ensure PWM and DIR pins are correctly connected

2. **Encoder readings are erratic**:
   - Verify encoder connections match the Timer channels
   - Check encoder power supply
   - Ensure proper grounding
   - Verify encoder pull-up resistors are enabled

3. **Position counts in wrong direction**:
   - Swap encoder A and B connections
   - Or invert the motor power polarity

## License

This project is released under the MIT License. Feel free to use, modify, and distribute as needed.

---

# Контроллер двигателя постоянного тока на базе Blue Pill

Этот проект реализует контроллер двигателя постоянного тока с использованием платы STM32F103C8T6 (Blue Pill). Он включает в себя аппаратную поддержку квадратурного энкодера, ПИД-регулирование и последовательную связь для управления и мониторинга в реальном времени.

## Возможности

- Аппаратная поддержка квадратурного энкодера с использованием таймера STM32 Timer4
- Счетчик энкодера с 4-кратным разрешением (обе грани обоих каналов)
- ПИД-регулирование скорости двигателя
- Последовательная связь на скорости 115200 бод
- Отчеты о состоянии в реальном времени
- Настраиваемое направление и скорость двигателя
- Управление скоростью в тиках энкодера в секунду
- Аппаратное подавление дребезга для надежного считывания энкодера

## Аппаратные требования

- Плата STM32F103C8T6 (Blue Pill)
- Двигатель постоянного тока с квадратурным энкодером
- Драйвер двигателя (с управлением DIR/PWM)
- USB-Serial адаптер (если не встроен в программатор)
  (подключение https://microkontroller.ru/stm32-projects/nachalo-raboty-s-stm32-blue-pill-s-pomoshhyu-arduino-ide/ )

## Подключение выводов

### Энкодер
- Энкодер A → PA6 (Timer3 Channel 1)
- Энкодер B → PA7 (Timer3 Channel 2)

### Управление двигателем
- Направление → PA1
- ШИМ → PA2
- Тормоз → PA0 (TBD)

## Файлы в проекте

1. `Blink.ino` - Прошивка для проверки работы платы
2. `BluePill.ino` - Основная прошивка контроллера двигателя
3. `Encoder.ino` - Тестовая прошивка для проверки работы энкодера
   - Поддерживает один энкодер:
     - Энкодер 1: Timer3 (PA6/PA7)

## Программные требования

1. Arduino IDE
2. Пакет плат STM32 для Arduino
3. Библиотека PID_v1

## Установка

1. Установите Arduino IDE с [arduino.cc](https://www.arduino.cc/en/software)
2. Установите пакет плат STM32:
   - Откройте Arduino IDE
   - Перейдите в Инструменты → Плата → Менеджер плат
   - Найдите "STM32"
   - Установите "STM32 MCU based boards"

3. Установите библиотеку PID:
   - Перейдите в Инструменты → Управление библиотеками
   - Найдите "PID"
   - Установите "PID" от Brett Beauregard

4. Настройте параметры платы:
   - Плата: "Generic STM32F103C series"
   - Метод загрузки: Предпочитаемый метод (ST-Link, Serial и т.д.)
   - Частота CPU: "72MHz (Normal)"

5. Прошивка платы:
   - Переключить джампер Boot0 в 1
   - Подключить плату к компьютеру
   - Кнопкой сброса на плате подготовить плату к загрузке
   - Нажать кнопку загрузки в Arduino IDE
   - Переключить джампер Boot0 в 0 (тестировать можно без этого шага)

## Использование

### Команды последовательного порта

Контроллер принимает следующие команды через последовательный порт на скорости 115200 бод:

- `S` - Остановка двигателя (устанавливает целевую скорость в 0)
- `M[скорость]` - Движение с указанной скоростью (значение зависит от разрешения энкодера)
  - Положительные значения - движение вперед
  - Отрицательные значения - движение назад
  - Пример: `M5` устанавливает скорость 5 единиц вперед
  - Пример: `M-5` устанавливает скорость 5 единиц назад

### Скрипт управления на Python

В проект включен Python-скрипт (`serial_control.py`) для удобного управления двигателем:

- Нажмите 'S' для остановки двигателя
- Нажмите '0-9' для установки различных уровней скорости (умножается на 5)
- Нажмите 'Q' для выхода из программы
- Сообщения о состоянии отображаются автоматически в реальном времени

Для запуска скрипта:
```bash
python3 serial_control.py
```

### Аппаратная конфигурация

- Разрешение энкодера: 8192 тика на оборот колеса
- Максимальное значение ШИМ: 10
- Интервал расчета скорости: 50мс
- Светодиодная индикация: PC13 загорается, когда скорость превышает половину разрешения энкодера

### Status Messages

The controller sends status messages every 50ms in the following format:
```
SIDE,POS:[position],SPD:[current_speed],TGT:[target_speed],PWR:[motor_power]
```

Где:
- `SIDE`: "VEDL" для левого мотора, "VEDR" для правого мотора
- `position`: Текущая позиция энкодера (знаковое значение, центрировано на 32767)
- `current_speed`: Фактическая скорость в тиках в секунду
- `target_speed`: Заданная скорость
- `motor_power`: Выход ШИМ (от -PWR_MAX до PWR_MAX)

## Тестирование энкодеров

Прошивка `encoder_test.ino` может быть использована для проверки правильной работы энкодеров:

1. Загрузите `encoder_test.ino` в BluePill
2. Откройте монитор порта на скорости 115200 бод
3. Вращайте энкодеры вручную или с помощью двигателей
4. Отслеживайте значения позиций обоих энкодеров
   - Формат: `позиция_энкодера1,позиция_энкодера2`
   - Положительные значения при вращении вперед
   - Отрицательные значения при вращении назад

## Настройка ПИД-регулятора

Значения ПИД-регулятора по умолчанию:
- Kp = 0.5
- Ki = 0.3
- Kd = 0.1

Возможно, потребуется настроить эти значения под вашу конкретную установку. Для настройки:
1. Начните со всех значений равных 0
2. Увеличивайте Kp, пока не получите приемлемый отклик
3. Увеличивайте Ki для устранения статической ошибки
4. Добавьте Kd для уменьшения перерегулирования, если необходимо

## Устранение неполадок

1. **Двигатель не вращается**:
   - Проверьте подключение питания
   - Проверьте подключение драйвера двигателя
   - Убедитесь, что выводы ШИМ и DIR правильно подключены

2. **Показания энкодера нестабильны**:
   - Проверьте соответствие подключения энкодера каналам таймера
   - Проверьте питание энкодера
   - Обеспечьте правильное заземление
   - Проверьте включение подтягивающих резисторов

3. **Счетчик позиции работает в неправильном направлении**:
   - Поменяйте местами подключение выводов A и B энкодера
   - Или измените полярность питания двигателя

## Лицензия

Этот проект распространяется под лицензией MIT. Вы можете свободно использовать, изменять и распространять его по необходимости.
