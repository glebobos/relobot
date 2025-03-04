from asciimatics.screen import Screen
import numpy as np
import serial
import time
import struct
def quaternion_to_rotation_matrix(w, x, y, z):
    """Convert a quaternion into a 3x3 rotation matrix."""
    return np.array([
        [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w,     2*x*z + 2*y*w],
        [2*x*y + 2*z*w,       1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w,       2*y*z + 2*x*w,     1 - 2*x**2 - 2*y**2]
    ])

def draw_cube(screen, rotation_matrix):
    # Define the 8 vertices of the cube
    vertices = np.array([
        [-1, -1, -1],
        [ 1, -1, -1],
        [ 1,  1, -1],
        [-1,  1, -1],
        [-1, -1,  1],
        [ 1, -1,  1],
        [ 1,  1,  1],
        [-1,  1,  1]
    ])

    # Define the edges of the cube (vertex numbers)
    edges = [
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7)
    ]

    # Apply the rotation matrix
    rotated_vertices = vertices @ rotation_matrix.T

    # Project 3D coordinates into 2D (take only the first two coordinates)
    projected_vertices = rotated_vertices[:, :2]

    # Scale and offset for display on the screen
    scale = 10
    offset = np.array([screen.width // 2, screen.height // 2])
    projected_vertices = projected_vertices * scale + offset

    # Draw the edges
    for start, end in edges:
        x1, y1 = projected_vertices[start]
        x2, y2 = projected_vertices[end]
        screen.move(int(x1), int(y1))
        screen.draw(int(x2), int(y2), char="*")

def demo(screen):
    # Open the serial port with the required speed (115200 baud)
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    buffer = bytearray()

    while True:
        screen.clear()

        # Read all available bytes and add them to the buffer
        if ser.in_waiting:
            buffer.extend(ser.read(ser.in_waiting))

        # Process packets if there are at least 11 bytes in the buffer
        while len(buffer) >= 11:
            # Check the packet header
            if buffer[0] != 0x55:
                buffer.pop(0)
                continue

            # Extract a potential packet of 11 bytes
            packet = buffer[:11]

            # Check the checksum:
            # The sum of the first 10 bytes (modulo 256) should match the 11th byte
            checksum = sum(packet[0:10]) & 0xFF
            if checksum != packet[10]:
                # If the checksum does not match, remove the first byte and try again
                buffer.pop(0)
                continue

            # Parse the packet depending on the type
            packet_type = packet[1]
            if packet_type == 0x53:  # quaternion packet
                # Extract 4 int16_t values (little-endian) from bytes 2-9
                qx, qy, qz, qw = struct.unpack('<hhhh', packet[2:10])
                # Convert to normalized values (divide by 32768.0)
                qx = qx / 32768.0
                qy = qy / 32768.0
                qz = qz / 32768.0
                qw = qw / 32768.0
                # To compute the rotation matrix, pass the quaternion as (w, x, y, z)
                rotation_matrix = quaternion_to_rotation_matrix(qw, qx, qy, qz)
                draw_cube(screen, rotation_matrix)

            # Remove the processed packet from the buffer
            del buffer[:11]

        screen.refresh()
        time.sleep(0.01)

Screen.wrapper(demo)