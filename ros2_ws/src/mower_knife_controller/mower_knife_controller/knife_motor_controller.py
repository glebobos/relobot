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
            # Read all available data to ensure we get complete messages
            responses = []
            while self.serial.in_waiting > 0:
                response = self.serial.readline().decode().strip()
                responses.append(response)
                
            # Process all responses, looking for the latest RPM reading
            rpm_value = None
            for response in reversed(responses):  # Start with most recent
                if "RPM:" in response:
                    try:
                        # Extract the part after "RPM:"
                        value_part = response.split('RPM:', 1)[1].strip()
                        rpm_value = float(value_part)
                        break
                    except (ValueError, IndexError) as e:
                        print(f"Failed to parse RPM from '{response}': {e}")
            
            return rpm_value
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
# Add to SerialKnifeMotorController class

    def send_command(self, command):
        """Send a command to the motor controller and return the response."""
        try:
            self.serial.write(f"{command}\n".encode())
            time.sleep(0.1)  # Give time for the command to be processed
            
            # Read response
            response = ""
            start_time = time.time()
            while time.time() - start_time < 1.0:  # 1 second timeout
                if self.serial.in_waiting > 0:
                    data = self.serial.readline().decode().strip()
                    response += data + "\n"
                    if command == "pid_status" and "PID control" in data:
                        return data
                    elif (command == "pid_enable" or command == "pid_disable") and "PID control" in data:
                        return True
            
            return response if response else None
        except Exception as e:
            print(f"Error sending command {command}: {e}")
            return None
    # ─────────────────────────────────────────────────────────────────
    def read_telemetry(self):
        """
        Return (rpm, vin) floats or (None, None) if nothing new.

        Expects firmware lines like '123.4,11.9'
        but still accepts old-style 'RPM: 123.4'.
        """
        if not self.serial and not self.connect():
            return None, None

        try:
            lines = []
            while self.serial.in_waiting > 0:
                lines.append(self.serial.readline().decode().strip())

            rpm = vin = None
            for line in reversed(lines):               # newest first
                # new CSV format ------------------------------------------------
                if re.fullmatch(r'-?\d+(\.\d+)?,\d+(\.\d+)?', line):
                    try:
                        rpm_str, vin_str = line.split(',', 1)
                        rpm, vin = float(rpm_str), float(vin_str)
                        break
                    except ValueError:
                        continue

                # legacy 'RPM: xxx' --------------------------------------------
                if line.startswith("RPM:"):
                    try:
                        rpm = float(line.split("RPM:", 1)[1].strip())
                    except ValueError:
                        pass

            return rpm, vin
        except serial.SerialException as e:
            print(f"Telemetry read error: {e}")
            self.serial = None
            return None, None

    
    def close(self):
        """Close the serial connection."""
        if self.serial:
            self.serial.close()
            self.serial = None