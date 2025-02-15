import serial
import serial.tools.list_ports
import time

def find_knives_interface():
    # List all available serial ports
    ports = serial.tools.list_ports.comports()
    for port in ports:
        try:
            print(f"Checking port: {port.device}")
            # Open the serial port
            ser = serial.Serial(port.device, baudrate=115200, timeout=2)
            if ser.is_open:
                print(f"Port {port.device} opened successfully.")
            else:
                print(f"Failed to open port {port.device}.")
                continue

            # Flush any existing input/output
            ser.reset_input_buffer()
            ser.reset_output_buffer()

            # Send a test command
            try:
                print(f"Sending command to port {port.device}")
                ser.write(b"whoyouare\r\n")  # Try using \r\n for line ending
                print(f"Command sent to port {port.device}")
            except Exception as e:
                print(f"Error writing to serial port {port.device}: {e}")
                ser.close()
                continue

            # Wait for a response
            print(f"Waiting for response from port {port.device}")
            time.sleep(1)  # Increase sleep time to ensure device has time to respond

            # Read the response
            try:
                response = ser.readline().decode(errors='ignore').strip()
                print(f"Received response from port {port.device}: {response}")
            except Exception as e:
                print(f"Error reading from serial port {port.device}: {e}")
                ser.close()
                continue

            # Check if the response is "knives"
            if response.lower() == "knives":
                print(f"Found 'knives' response on port: {port.device}")
                ser.close()
                return port.device
            else:
                print(f"No 'knives' response on port: {port.device}, got: {response}")

            # Close the port
            ser.close()

        except serial.SerialException as e:
            print(f"SerialException on port {port.device}: {e}")
        except Exception as e:
            print(f"Error checking port {port.device}: {e}")

    print("No interface responded with 'knives'.")
    return None

if __name__ == "__main__":
    find_knives_interface()