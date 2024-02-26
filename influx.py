from influxdb import InfluxDBClient
import time

# Параметры подключения к InfluxDB
host = 'localhost'
port = 8086
user = 'r2'
password = 'r2'
database = 'r2'

client = InfluxDBClient(host, port, user, password, database)

# Пример данных датчика (замените на реальные значения)
sensor_data = [
    {
        "measurement": "temperature",
        "tags": {
            "location": "room"
        },
        "fields": {
            "value": 25.5
        }
    }
]

# Запись данных в базу данных
#while True:
client.write_points(sensor_data)
print("Данные успешно записаны в базу данных")
#time.sleep(5)  # Пауза в 5 секунд
