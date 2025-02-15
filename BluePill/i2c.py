import smbus2
import time

# Адрес STM32 I2C Slave
STM32_I2C_ADDR = 9

# Создаем экземпляр шины I2C
bus = smbus2.SMBus(3)

def write_to_stm32(data):
    """
    Отправка данных на STM32.
    :param data: Список байтов для передачи.
    """
    try:
        bus.write_i2c_block_data(STM32_I2C_ADDR, 48, data)
        print(f"Отправлено: {data}")
    except Exception as e:
        print(f"Ошибка при отправке: {e}")

def read_from_stm32(length):
    """
    Чтение данных от STM32.
    :param length: Количество байт для чтения.
    :return: Считанные данные.
    """
    try:
        data = bus.read_i2c_block_data(STM32_I2C_ADDR, 0, length)
        print(f"Получено: {data}")
        return data
    except Exception as e:
        print(f"Ошибка при чтении: {e}")
        return []

if __name__ == "__main__":
    try:
        while True:
            # Пример: Отправка данных на STM32
            message = [65, 66, 67, 68]  # Отправляем последовательность байтов
            write_to_stm32(message)

            # Пример: Чтение ответа от STM32
            response = read_from_stm32(6)  # Читаем 6 байт
            print(f"Ответ от STM32: {''.join(chr(b) for b in response)}")

            time.sleep(1)  # Пауза между циклами
    except KeyboardInterrupt:
        print("Выход из программы")
    finally:
        bus.close()
