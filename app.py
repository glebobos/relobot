from flask import Flask, render_template, request, jsonify
import serial
import threading
import time

app = Flask(__name__)

SERIAL_PORT = '/dev/ttyACM1'
BAUD_RATE = 115200

# Motor control parameters
ACCELERATION = 0.1  # Acceleration rate (adjust as needed)
UPDATE_RATE = 0.02  # Update rate in seconds
current_left = 0.0
current_right = 0.0
target_left = 0.0
target_right = 0.0

# Initialize the serial connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

def send_command(command):
    ser.write((command + '\n').encode())

def smooth_motor_control():
    global current_left, current_right, target_left, target_right
    
    while True:
        # Calculate difference between current and target speeds
        diff_left = target_left - current_left
        diff_right = target_right - current_right
        
        # Apply acceleration/deceleration
        if abs(diff_left) > ACCELERATION:
            current_left += ACCELERATION if diff_left > 0 else -ACCELERATION
        else:
            current_left = target_left
            
        if abs(diff_right) > ACCELERATION:
            current_right += ACCELERATION if diff_right > 0 else -ACCELERATION
        else:
            current_right = target_right
        
        # Send the smoothed values to the motors
        command = f"{current_left},{current_right}"
        send_command(command)
        
        time.sleep(UPDATE_RATE)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/set_motors', methods=['POST'])
def set_motors():
    global target_left, target_right
    
    data = request.json
    x = float(data['x'])
    y = float(data['y'])
    
    # Calculate motor speeds based on joystick position
    left_motor = y + x
    right_motor = y - x
    
    # Clamp values to [-1, 1] range
    target_left = max(-1, min(1, left_motor))
    target_right = max(-1, min(1, right_motor))
    
    return jsonify(success=True)

def main_loop():
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Stopping motors")
        global target_left, target_right
        target_left = 0.0
        target_right = 0.0
        # Wait for motors to stop smoothly
        time.sleep(1)
        send_command("0,0")
        ser.close()

if __name__ == '__main__':
    # Start the smooth motor control thread
    motor_thread = threading.Thread(target=smooth_motor_control)
    motor_thread.daemon = True
    motor_thread.start()
    
    # Run the Flask app in a separate thread
    flask_thread = threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False))
    flask_thread.daemon = True
    flask_thread.start()

    # Run the main loop
    main_loop()