import serial
import serial.tools.list_ports
import time

def identify_devices():
    ports = serial.tools.list_ports.comports()
    print(f"Total ports found: {len(ports)}")
    for port in ports:
        print(f"\nChecking port: {port.device}")
        try:
            ser = serial.Serial(port.device, baudrate=115200, timeout=1)
            if not ser.is_open:
                print(f"Failed to open {port.device}")
                continue
                
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            
            # Send whoyouare
            ser.write(b"whoyouare\n")
            time.sleep(0.5)
            
            response = ser.readline().decode(errors='ignore').strip()
            print(f"Response: '{response}'")
            
            if "wheels" in response.lower():
                print(f"Found 'wheels' on {port.device}")
            elif "knives" in response.lower():
                print(f"Found 'knives' on {port.device}")
            else:
                # Try with carriage return
                ser.write(b"whoyouare\r\n")
                time.sleep(0.5)
                response = ser.readline().decode(errors='ignore').strip()
                print(f"Response (with \r\n): '{response}'")
                if "wheels" in response.lower():
                    print(f"Found 'wheels' on {port.device}")
                elif "knives" in response.lower():
                    print(f"Found 'knives' on {port.device}")
            
            ser.close()
        except Exception as e:
            print(f"Error checking {port.device}: {e}")

if __name__ == '__main__':
    identify_devices()
