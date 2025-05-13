import pygame
import sys
import threading
import time

# Initialize Pygame
pygame.init()

# Initialize the joystick
pygame.joystick.init()

# Check for joystick
if pygame.joystick.get_count() == 0:
    print("No joystick connected.")
    sys.exit()

# Get the first joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Joystick name: {joystick.get_name()}")
print(f"Number of axes: {joystick.get_numaxes()}")
print(f"Number of buttons: {joystick.get_numbuttons()}")

# Dictionary to track the last printed value for each axis
last_values = {}

# Configure idle zones and thresholds using hashmaps
IDLE_ZONE_CONFIG = {
    0: 0.07,
    3: 0.07,
}

# Variables to control vibration
vibration_active = False
vibration_thread = None
vibration_stop_event = threading.Event()
current_vibration_intensity = (0.5, 0.5)  # Default vibration intensity (low_freq, high_freq)

# Function to continuously vibrate the controller
def vibrate_continuously():
    """
    Continuously vibrate the controller until stopped
    """
    global current_vibration_intensity
    
    try:
        if hasattr(joystick, 'rumble'):
            while not vibration_stop_event.is_set():
                # Use the current intensity values which might be updated by axis movement
                low_freq, high_freq = current_vibration_intensity
                joystick.rumble(low_freq, high_freq, 100)  # Short duration to keep refreshing
                time.sleep(0.05)  # Small delay to prevent excessive calls
    except Exception as e:
        print(f"Error in vibration thread: {e}")
    finally:
        # Ensure vibration stops when thread ends
        if hasattr(joystick, 'rumble'):
            joystick.rumble(0, 0, 0)

# Function to start vibration
def start_vibration(low_frequency=0.5, high_frequency=0.5):
    global vibration_thread, vibration_active, vibration_stop_event, current_vibration_intensity
    
    # Set initial vibration intensity
    current_vibration_intensity = (low_frequency, high_frequency)
    
    # If vibration is already active, just update the intensity
    if vibration_active:
        return True
    
    # Start new vibration
    vibration_active = True
    vibration_stop_event.clear()
    vibration_thread = threading.Thread(
        target=vibrate_continuously
    )
    vibration_thread.daemon = True
    vibration_thread.start()
    return True

# Function to stop vibration
def stop_vibration():
    global vibration_thread, vibration_active, vibration_stop_event
    
    if vibration_active:
        vibration_stop_event.set()
        if vibration_thread and vibration_thread.is_alive():
            vibration_thread.join(0.2)  # Wait briefly for thread to end
        vibration_active = False
        
        # Ensure vibration stops
        if hasattr(joystick, 'rumble'):
            joystick.rumble(0, 0, 0)

# Function to update vibration based on axis value
def update_vibration_from_axis(axis_value):
    global current_vibration_intensity, vibration_active
    
    # Stop vibration if axis is at -1 or less
    if axis_value <= -1:
        if vibration_active:
            print("Axis 4 at minimum, stopping vibration")
            stop_vibration()
        return
    
    # Map axis value from (-1, 1] to vibration intensity [0, 1]
    normalized_value = (axis_value + 1) / 2
    intensity = max(0, min(1, normalized_value))  # Ensure value is between 0 and 1
    
    # Update vibration intensity
    current_vibration_intensity = (intensity, intensity)
    print(f"Axis 4: {axis_value:.2f}, Vibration intensity: {intensity:.2f}")
    
    # Start vibration if not already active and intensity > 0
    if not vibration_active and intensity > 0:
        print("Starting vibration from axis 4")
        start_vibration(intensity, intensity)
    # Stop vibration if intensity is 0
    elif vibration_active and intensity == 0:
        print("Stopping vibration (intensity 0)")
        stop_vibration()

# Main loop
try:
    print("Controls:")
    print("Button 0: Light vibration")
    print("Button 1: Medium vibration")
    print("Button 3: Strong vibration")
    print("Axis 4: Control vibration intensity (-1 stops, 1 is maximum)")
    
    # Dictionary to track button states
    button_states = {}
    
    while True:
        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                # Handle axis 4 for vibration control
                if event.axis == 4:
                    update_vibration_from_axis(event.value)
                    # Store the value for debugging
                    if event.axis not in last_values or event.value != last_values[event.axis]:
                        last_values[event.axis] = event.value
                
                # Handle other axes (except 1 and 2)
                elif event.axis != 1 and event.axis != 2:
                    # Check if this axis has an idle zone configuration
                    if event.axis in IDLE_ZONE_CONFIG:
                        threshold = IDLE_ZONE_CONFIG[event.axis]
                        if abs(event.value) < threshold:
                            value = 0.0
                        else:
                            # Rescale the value to maintain the full range
                            sign = 1 if event.value > 0 else -1
                            value = (abs(event.value) - threshold) / (1 - threshold) * sign
                    else:
                        value = event.value
                    
                    if event.axis not in last_values or value != last_values[event.axis]:
                        print(f"Axis {event.axis} value: {value}")
                        last_values[event.axis] = value
                    
            elif event.type == pygame.JOYBUTTONDOWN:
                print(f"Button {event.button} pressed")
                button_states[event.button] = True
                
                # Different vibration patterns based on button pressed
                if event.button == 0:
                    print("Light vibration started")
                    start_vibration(0.3, 0.3)
                elif event.button == 1:
                    print("Medium vibration started")
                    start_vibration(0.6, 0.6)
                elif event.button == 3:
                    print("Strong vibration started")
                    start_vibration(1.0, 1.0)
                    
            elif event.type == pygame.JOYBUTTONUP:
                print(f"Button {event.button} released")
                button_states[event.button] = False
                
                # If the released button was one that started vibration, stop it
                if event.button in [0, 1, 3]:
                    print("Vibration stopped")
                    stop_vibration()
        
        # Small delay to prevent maxing out CPU
        time.sleep(0.01)
            
except KeyboardInterrupt:
    print("Exiting...")

finally:
    # Make sure to stop any ongoing vibration before quitting
    stop_vibration()
    pygame.quit()