from flask import Flask, render_template, request, jsonify
import serial
import threading
import time

app = Flask(__name__)

SERIAL_PORT = '/dev/ttyACM1'  # Update this with the correct port
BAUD_RATE = 115200

# Initialize the serial connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

def send_command(command):
    ser.write((command + '\n').encode())

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/set_motors', methods=['POST'])
def set_motors():
    data = request.json
    x = float(data['x'])
    y = float(data['y'])
    
    # Calculate motor speeds based on joystick position
    left_motor = y + x
    right_motor = y - x
    
    # Clamp values to [-1, 1] range
    left_motor = max(-1, min(1, left_motor))
    right_motor = max(-1, min(1, right_motor))
    
    # Send motor speeds to the Pico
    command = f"{left_motor},{right_motor}"
    send_command(command)
    
    return jsonify(success=True)

def main_loop():
    try:
        while True:
            # Add a small sleep to prevent high CPU usage
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Stopping motors")
        send_command("0,0")  # Stop motors
        ser.close()

if __name__ == '__main__':
    # Run the Flask app in a separate thread
    flask_thread = threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False))
    flask_thread.daemon = True  # Make the thread daemon so it exits when the main program exits
    flask_thread.start()

    # Run the main loop
    main_loop()