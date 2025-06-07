import time
import serial

class SerialKnifeMotorController:
    """Interface for controlling a mower knife motor via serial port."""
    
    def __init__(self, port_name='/dev/ttyACM4', baud_rate=115200):
        """Initialize with specified port and baud rate."""
        self.port_name = port_name
        self.baud_rate = baud_rate
        self.serial = None
        self.connect()
        
    def connect(self):
        """Connect to the serial port."""
        try:
            self.serial = serial.Serial(
                port=self.port_name,
                baudrate=self.baud_rate,
                timeout=1.0
            )
            time.sleep(2)  # Allow time for the connection to establish
            print(f"Connected to {self.port_name}")
            return True
        except serial.SerialException as e:
            print(f"Failed to connect to {self.port_name}: {e}")
            return False
    
    def set_rpm(self, rpm):
        """Send RPM command to the knife motor controller."""
        if not self.serial:
            if not self.connect():
                return False
                
        try:
            # Send the RPM value as a command
            command = f"{rpm}\n"
            self.serial.write(command.encode())
            return True
        except serial.SerialException as e:
            print(f"Error sending command: {e}")
            self.serial = None
            return False
    
    def read_rpm(self):
        """Read current RPM from the knife motor controller."""
        if not self.serial:
            if not self.connect():
                return None
                
        try:
            if self.serial.in_waiting > 0:
                response = self.serial.readline().decode().strip()
                if response.startswith("RPM:"):
                    return float(response.split(':')[1].strip())
            return None
        except serial.SerialException as e:
            print(f"Error reading response: {e}")
            self.serial = None
            return None
    
    def identify_device(self):
        """Check if the device is the expected knife motor controller."""
        if not self.serial:
            if not self.connect():
                return False
        
        try:
            self.serial.write(b"whoyouare\n")
            time.sleep(0.1)
            if self.serial.in_waiting > 0:
                response = self.serial.readline().decode().strip()
                return response == "knives"
            return False
        except serial.SerialException:
            self.serial = None
            return False
    
    def close(self):
        """Close the serial connection."""
        if self.serial:
            self.serial.close()
            self.serial = None