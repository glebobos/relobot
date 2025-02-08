import serial
import glob

def find_device(expected_response="wheels", command="whoyouare", baudrate=9600, timeout=2):
    # List all potential ACM devices
    ports = glob.glob('/dev/ttyACM*')
    
    for port in ports:
        try:
            # Open the serial port
            with serial.Serial(port, baudrate, timeout=timeout) as ser:                
                # Send the identification command
                ser.write((command + '\n').encode())
                
                # Read the response
                response = ser.readline().decode().strip()
                
                # Check if the response matches the expected response
                if response == expected_response:
                    print(f"Device found at: {port}")
                    return port
                else:
                    print(f"Unexpected response from {port}: {response}")
        except Exception as e:
            print(f"Error communicating with {port}: {e}")
    
    print("No matching device found.")
    return None

if __name__ == "__main__":
    find_device()