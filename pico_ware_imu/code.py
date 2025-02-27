import board
import busio
import time
import usb_cdc
from adafruit_icm20x import ICM20948
from fusion import Fusion

# Initialize I2C bus for XIAO RP2040
i2c = busio.I2C(board.SCL, board.SDA)  # SCL=GP7, SDA=GP6
fuse = Fusion()

# Initialize ICM-20948
imu = ICM20948(i2c, address=0x68)

def send_packet(packet_type, data):
    checksum = 0x55 + packet_type
    packet = bytearray()
    packet.append(0x55)
    packet.append(packet_type)
    for value in data:
        low_byte = value & 0xff
        high_byte = (value >> 8) & 0xff
        packet.append(low_byte)
        packet.append(high_byte)
        checksum += low_byte + high_byte
    packet.append(checksum & 0xff)
    usb_cdc.data.write(packet)  # Send the packet over USB CDC

while True:
    # Read sensor data
    accel_x, accel_y, accel_z = imu.acceleration
    gyro_x, gyro_y, gyro_z = imu.gyro
    mag_x, mag_y, mag_z = imu.magnetic
    
    # Convert float readings to int16_t equivalent
    raw_acc = [int(accel_x * 32767), int(accel_y * 32767), int(accel_z * 32767), 0]
    raw_gyr = [int(gyro_x * 32767), int(gyro_y * 32767), int(gyro_z * 32767), 0]
    raw_mag = [int(mag_x * 32767), int(mag_y * 32767), int(mag_z * 32767), 0]
    
    # Send packets
    send_packet(0x51, raw_acc)  # Send raw Acc values
    send_packet(0x52, raw_gyr)  # Send raw Gyr values
    send_packet(0x54, raw_mag)  # Send raw Mag values
    
    # Print readings for debugging
    print(f"Accel: {accel_x:.2f}, {accel_y:.2f}, {accel_z:.2f} m/s²")
    print(f"Gyro: {gyro_x:.2f}, {gyro_y:.2f}, {gyro_z:.2f} dps")
    print(f"Mag: {mag_x:.2f}, {mag_y:.2f}, {mag_z:.2f} uT")
    # print("-" * 40)
    time.sleep(0.00444)

