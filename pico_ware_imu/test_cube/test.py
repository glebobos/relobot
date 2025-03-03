from asciimatics.screen import Screen
import numpy as np
import serial
import time

def quaternion_to_rotation_matrix(w, x, y, z):
    """Convert a quaternion into a 3x3 rotation matrix."""
    return np.array([
        [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]
    ])

def draw_cube(screen, rotation_matrix):
    # Define the 8 vertices of a cube
    vertices = np.array([
        [-1, -1, -1],
        [1, -1, -1],
        [1, 1, -1],
        [-1, 1, -1],
        [-1, -1, 1],
        [1, -1, 1],
        [1, 1, 1],
        [-1, 1, 1]
    ])

    # Define the edges of the cube
    edges = [
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7)
    ]

    # Apply rotation
    rotated_vertices = vertices @ rotation_matrix.T

    # Project 3D to 2D
    projected_vertices = rotated_vertices[:, :2]

    # Scale and translate to fit the screen
    scale = 10
    offset = np.array([screen.width // 2, screen.height // 2])
    projected_vertices = projected_vertices * scale + offset

    # Draw edges
    for edge in edges:
        start, end = edge
        x1, y1 = projected_vertices[start]
        x2, y2 = projected_vertices[end]
        screen.move(int(x1), int(y1))
        screen.draw(int(x2), int(y2), char="*")

def demo(screen):
    ser = serial.Serial('/dev/ttyACM0', 9600)
    while True:
        screen.clear()

        # Read quaternion data from serial
        line = ser.readline().decode('utf-8').strip()
        if line.startswith("Quat6"):
            try:
                quat_data = line.split(':')[1].strip().strip('[]').split(',')
                w, x, y, z = map(float, quat_data)
                rotation_matrix = quaternion_to_rotation_matrix(w, x, y, z)
                draw_cube(screen, rotation_matrix)
            except Exception as e:
                screen.print_at(f"Error parsing quaternion: {e}", 0, 0)

        screen.refresh()
        time.sleep(0.01)

Screen.wrapper(demo)