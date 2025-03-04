#include "Arduino-ICM20948.h"
#include <Wire.h>

ArduinoICM20948 icm20948;

ArduinoICM20948Settings icmSettings =
{
  .i2c_speed = 400000,
  .is_SPI = false,
  .cs_pin = 10,
  .spi_speed = 7000000,
  .mode = 1,
  .enable_gyroscope = true,
  .enable_accelerometer = false,
  .enable_magnetometer = false,
  .enable_gravity = false,
  .enable_linearAcceleration = true,
  .enable_quaternion6 = true,
  .enable_quaternion9 = false,
  .enable_har = false,
  .enable_steps = false,
  .gyroscope_frequency = 150,
  .accelerometer_frequency = 150,
  .linearAcceleration_frequency = 150,
  .quaternion6_frequency = 150
};

const uint8_t number_i2c_addr = 2;
uint8_t poss_addresses[number_i2c_addr] = {0X69, 0X68};
uint8_t ICM_address;
bool ICM_found = false;

void i2c_scan() {
    for (uint8_t add_int = 0; add_int < number_i2c_addr; add_int++) {
        Wire.beginTransmission(poss_addresses[add_int]);
        if (Wire.endTransmission() == 0) {
            ICM_address = poss_addresses[add_int];
            ICM_found = true;
            break;
        }
    }
}

void send_packet(byte packet_type, int16_t *data) {
    byte checksum = 0x55 + packet_type;
    Serial.write(0x55);
    Serial.write(packet_type);
    for (int i = 0; i < 4; i++) {
        byte low_byte = data[i] & 0xff;
        byte high_byte = (data[i] >> 8) & 0xff;
        Serial.write(low_byte);
        Serial.write(high_byte);
        checksum += low_byte + high_byte;
    }
    Serial.write(checksum);
}

void run_icm20948_sensors() {
    int16_t data[4] = {0, 0, 0, 0};
    
    if (icm20948.linearAccelDataIsReady()) {
        float x, y, z;
        icm20948.readLinearAccelData(&x, &y, &z);
        data[0] = (int16_t)(x * 8192.0);
        data[1] = (int16_t)(y * 8192.0);
        data[2] = (int16_t)(z * 8192.0);
        send_packet(0x51, data);
    }
    
    if (icm20948.gyroDataIsReady()) {
        float x, y, z;
        icm20948.readGyroData(&x, &y, &z);
        data[0] = (int16_t)(x * 16.4);
        data[1] = (int16_t)(y * 16.4);
        data[2] = (int16_t)(z * 16.4);
        send_packet(0x52, data);
    }
    
    if (icm20948.quat6DataIsReady()) {
        float qw, qx, qy, qz;
        icm20948.readQuat6Data(&qw, &qx, &qy, &qz);
        double q0 = sqrt(1.0 - ((qx * qx) + (qy * qy) + (qz * qz)));
        data[0] = (int16_t)(qx * 32767.0);
        data[1] = (int16_t)(qy * 32767.0);
        data[2] = (int16_t)(qz * 32767.0);
        data[3] = (int16_t)(q0 * 32767.0);
        send_packet(0x53, data);
    }
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000);
    delay(10);
    i2c_scan();
    if (ICM_found) {
        icm20948.init(icmSettings);
    }
}

void loop() {
    if (ICM_found) {
        icm20948.task();
        run_icm20948_sensors();
    }
    delay(1);
}