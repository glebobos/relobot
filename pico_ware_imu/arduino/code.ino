#include <Arduino.h>
#include <Wire.h>
#include "Arduino-ICM20948.h"

// ------------------ НАСТРОЙКИ ------------------
bool debugMode = false;  // Если true, будут выводиться данные в человекочитаемом виде

// Структура настроек для ICM20948
ArduinoICM20948Settings icmSettings = {
  .i2c_speed = 400000,
  .is_SPI = false,
  .cs_pin = 10,
  .spi_speed = 7000000,
  .mode = 1,
  .enable_gyroscope = true,
  .enable_accelerometer = true,
  .enable_magnetometer = false,
  .enable_gravity = false,
  .enable_linearAcceleration = false,
  .enable_quaternion6 = true,
  .enable_quaternion9 = false,
  .enable_har = false,
  .enable_steps = false,
  .gyroscope_frequency = 150,
  .accelerometer_frequency = 150,
  .magnetometer_frequency = 1,
  .gravity_frequency = 1,
  .linearAcceleration_frequency = 1,
  .quaternion6_frequency = 150,
  .quaternion9_frequency = 1,
  .har_frequency = 1,
  .steps_frequency = 1
};

// ------------------ КОНСТАНТЫ ПРОТОКОЛА ------------------
static const uint8_t HEADER_BYTE          = 0x55;
static const uint8_t ACCELERATION_BYTE    = 0x51;
static const uint8_t ANGULAR_VELOCITY_BYTE= 0x52;
static const uint8_t QUATERNION_BYTE      = 0x53;

// ------------------ ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ------------------
ArduinoICM20948 icm20948;
const uint8_t number_i2c_addr = 2;
uint8_t poss_addresses[number_i2c_addr] = {0x69, 0x68};
uint8_t ICM_address;
bool ICM_found = false;

// ------------------ СЛУЖЕБНЫЕ ФУНКЦИИ ------------------
void i2c_scan() {
  for (uint8_t i = 0; i < number_i2c_addr; i++) {
    Serial.printf("Scanning 0x%02X for slave...", poss_addresses[i]);
    Wire.beginTransmission(poss_addresses[i]);
    uint8_t error = Wire.endTransmission();
    if (error == 0) {
      Serial.println("found.");
      if (poss_addresses[i] == 0x69 || poss_addresses[i] == 0x68) {
        Serial.println("\t- address is ICM.");
        ICM_address = poss_addresses[i];
        ICM_found = true;
      }
    } else {
      Serial.println("not found.");
    }
  }
}

// Вычисление контрольной суммы для массива (просто сумма байт, взятая по модулю 256)
uint8_t calcChecksum(const uint8_t *data, size_t length) {
  uint8_t sum = 0;
  for (size_t i = 0; i < length; i++) {
    sum += data[i];
  }
  return sum;
}

// ------------------ ФУНКЦИИ ОТПРАВКИ ДАННЫХ ------------------
/*
  Отправка ускорений в двоичном формате:
  [HEADER_BYTE] [ACCELERATION_BYTE] [ax_L] [ax_H] [ay_L] [ay_H] [az_L] [az_H] [CHK]
  Где ax, ay, az - это int16, соответствующие ускорениям в диапазоне ±4g,
  интерпретируемым узлом как: float = raw / 32768.0 * 4 * 9.80665
*/
void sendAcceleration(float x_g, float y_g, float z_g, bool debug) {
  // Узел умножает raw/32768.0 * 4*g для получения м/с^2.
  // Т.к. здесь x_g, y_g, z_g в "g", то 1g = 8192 (т.к. 32768 соответствует 4g).
  int16_t rawX = (int16_t)(x_g * 8192.0f);
  int16_t rawY = (int16_t)(y_g * 8192.0f);
  int16_t rawZ = (int16_t)(z_g * 8192.0f);

  uint8_t buffer[9];
  buffer[0] = HEADER_BYTE;
  buffer[1] = ACCELERATION_BYTE;

  buffer[2] = (uint8_t)(rawX & 0xFF);
  buffer[3] = (uint8_t)((rawX >> 8) & 0xFF);
  buffer[4] = (uint8_t)(rawY & 0xFF);
  buffer[5] = (uint8_t)((rawY >> 8) & 0xFF);
  buffer[6] = (uint8_t)(rawZ & 0xFF);
  buffer[7] = (uint8_t)((rawZ >> 8) & 0xFF);

  // Контрольная сумма по первым 8 байтам
  buffer[8] = calcChecksum(buffer, 8);

  // Отправляем пакет в порт
  Serial.write(buffer, 9);

  // Вывод в Serial при включенном debug
  if (debug) {
    Serial.print("ACC (g): [");
    Serial.print(x_g, 6); Serial.print(", ");
    Serial.print(y_g, 6); Serial.print(", ");
    Serial.print(z_g, 6);
    Serial.print("] => raw: [");
    Serial.print(rawX); Serial.print(", ");
    Serial.print(rawY); Serial.print(", ");
    Serial.print(rawZ); 
    Serial.println("]");
  }
}

/*
  Отправка угловых скоростей в двоичном формате:
  [HEADER_BYTE] [ANGULAR_VELOCITY_BYTE] [gx_L] [gx_H] [gy_L] [gy_H] [gz_L] [gz_H] [CHK]
  Где gx, gy, gz - это int16, соответствующие гироскопу в диапазоне ±2000°/с,
  переводимому узлом в радианы/с так:
    angular_velocity = raw / 32768.0 * 2000 * (pi/180).
*/
void sendGyro(float x_rad, float y_rad, float z_rad, bool debug) {
  // Сначала переведём рад/с в °/с:
  float x_deg = x_rad * (180.0f / PI);
  float y_deg = y_rad * (180.0f / PI);
  float z_deg = z_rad * (180.0f / PI);

  // 32768 соответствует 2000 °/с => 1 °/с = 32768/2000 = 16.384
  int16_t rawX = (int16_t)((x_deg / 2000.0f) * 32768.0f);
  int16_t rawY = (int16_t)((y_deg / 2000.0f) * 32768.0f);
  int16_t rawZ = (int16_t)((z_deg / 2000.0f) * 32768.0f);

  uint8_t buffer[9];
  buffer[0] = HEADER_BYTE;
  buffer[1] = ANGULAR_VELOCITY_BYTE;

  buffer[2] = (uint8_t)(rawX & 0xFF);
  buffer[3] = (uint8_t)((rawX >> 8) & 0xFF);
  buffer[4] = (uint8_t)(rawY & 0xFF);
  buffer[5] = (uint8_t)((rawY >> 8) & 0xFF);
  buffer[6] = (uint8_t)(rawZ & 0xFF);
  buffer[7] = (uint8_t)((rawZ >> 8) & 0xFF);

  // Контрольная сумма по первым 8 байтам
  buffer[8] = calcChecksum(buffer, 8);

  // Отправляем пакет в порт
  Serial.write(buffer, 9);

  if (debug) {
    Serial.print("GYRO (rad/s): [");
    Serial.print(x_rad, 6); Serial.print(", ");
    Serial.print(y_rad, 6); Serial.print(", ");
    Serial.print(z_rad, 6);
    Serial.print("] => raw: [");
    Serial.print(rawX); Serial.print(", ");
    Serial.print(rawY); Serial.print(", ");
    Serial.print(rawZ);
    Serial.println("]");
  }
}

/*
  Отправка кватерниона:
  [HEADER_BYTE] [QUATERNION_BYTE] [w_L] [w_H] [x_L] [x_H] [y_L] [y_H] [z_L] [z_H] [CHK]
  (итого 11 байт), где w,x,y,z - int16, соответствующие float = raw / 32768.0
*/
void sendQuaternion(float w, float x, float y, float z, bool debug) {
  int16_t rawW = (int16_t)(w * 32768.0f);
  int16_t rawX = (int16_t)(x * 32768.0f);
  int16_t rawY = (int16_t)(y * 32768.0f);
  int16_t rawZ = (int16_t)(z * 32768.0f);

  uint8_t buffer[11];
  buffer[0] = HEADER_BYTE;
  buffer[1] = QUATERNION_BYTE;

  buffer[2] = (uint8_t)(rawW & 0xFF);
  buffer[3] = (uint8_t)((rawW >> 8) & 0xFF);
  buffer[4] = (uint8_t)(rawX & 0xFF);
  buffer[5] = (uint8_t)((rawX >> 8) & 0xFF);
  buffer[6] = (uint8_t)(rawY & 0xFF);
  buffer[7] = (uint8_t)((rawY >> 8) & 0xFF);
  buffer[8] = (uint8_t)(rawZ & 0xFF);
  buffer[9] = (uint8_t)((rawZ >> 8) & 0xFF);

  // Контрольная сумма по первым 10 байтам
  buffer[10] = calcChecksum(buffer, 10);

  // Отправляем пакет в порт
  Serial.write(buffer, 11);

  if (debug) {
    Serial.print("QUAT (w,x,y,z): [");
    Serial.print(w, 6); Serial.print(", ");
    Serial.print(x, 6); Serial.print(", ");
    Serial.print(y, 6); Serial.print(", ");
    Serial.print(z, 6);
    Serial.print("] => raw: [");
    Serial.print(rawW); Serial.print(", ");
    Serial.print(rawX); Serial.print(", ");
    Serial.print(rawY); Serial.print(", ");
    Serial.print(rawZ);
    Serial.println("]");
  }
}

// ------------------ SETUP & LOOP ------------------
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  Serial.println("Starting ICM...");
  delay(10);

  i2c_scan();
  if (ICM_found) {
    icm20948.init(icmSettings);
    Serial.println("ICM initialized.");
  } else {
    Serial.println("ICM not found.");
  }
}

void loop() {
  if (ICM_found) {
    // Обновляем внутренние данные
    icm20948.task();

    // Если готовы данные акселерометра — отправляем пакет
    if (icm20948.accelDataIsReady()) {
      float ax, ay, az;
      icm20948.readAccelData(&ax, &ay, &az);
      sendAcceleration(ax, ay, az, debugMode);
    }

    // Если готовы данные гироскопа — отправляем пакет
    if (icm20948.gyroDataIsReady()) {
      float gx, gy, gz;
      icm20948.readGyroData(&gx, &gy, &gz);
      sendGyro(gx, gy, gz, debugMode);
    }

    // Если готовы данные кватерниона (6DOF) — отправляем пакет
    if (icm20948.quat6DataIsReady()) {
      float qw, qx, qy, qz;
      icm20948.readQuat6Data(&qw, &qx, &qy, &qz);
      sendQuaternion(qw, qx, qy, qz, debugMode);
    }
  }

  // Небольшая пауза, чтобы не заспамить порт
  delay(10);
}
