"""
Controller Feedback System - Handles joystick input and vibration feedback.
"""

import pygame
import sys
import threading
import time
from typing import Dict, Tuple, Optional, Set

class ControllerFeedbackSystem:
    """Manages controller input detection and haptic feedback."""
    
    # Configuration constants
    IDLE_ZONE_CONFIG = {0: 0.07, 3: 0.07}
    VIBRATION_PRESETS = {
        0: {"name": "Light", "intensity": 0.3},
        1: {"name": "Medium", "intensity": 0.6},
        3: {"name": "Strong", "intensity": 1.0}
    }
    # Axes to ignore (disabled axes)
    DISABLED_AXES: Set[int] = {1, 2}
    
    def __init__(self):
        """Initialize the controller feedback system."""
        pygame.init()
        pygame.joystick.init()
        
        # Check for joystick availability
        if pygame.joystick.get_count() == 0:
            print("No joystick connected.")
            sys.exit(1)
            
        # Initialize joystick
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        
        # Print controller information
        self._print_controller_info()
        
        # State tracking
        self.last_values: Dict[int, float] = {}
        self.button_states: Dict[int, bool] = {}
        
        # Vibration control
        self.vibration_active = False
        self.vibration_thread: Optional[threading.Thread] = None
        self.vibration_stop_event = threading.Event()
        self.current_vibration_intensity = (0.5, 0.5)  # (low_freq, high_freq)
    
    def _print_controller_info(self) -> None:
        """Display information about the connected controller."""
        print(f"Joystick name: {self.joystick.get_name()}")
        print(f"Number of axes: {self.joystick.get_numaxes()}")
        print(f"Number of buttons: {self.joystick.get_numbuttons()}")
        
        print("\nControls:")
        print("Button 0: Light vibration")
        print("Button 1: Medium vibration")
        print("Button 3: Strong vibration")
        print("Axis 4: Control vibration intensity (-1 stops, 1 is maximum)")
        print(f"Disabled axes: {', '.join(map(str, self.DISABLED_AXES))}")
    
    def vibrate_continuously(self) -> None:
        """Continuously vibrate the controller until stopped."""
        try:
            if hasattr(self.joystick, 'rumble'):
                while not self.vibration_stop_event.is_set():
                    low_freq, high_freq = self.current_vibration_intensity
                    self.joystick.rumble(low_freq, high_freq, 100)  # Short duration to keep refreshing
                    time.sleep(0.05)  # Small delay to prevent excessive calls
        except Exception as e:
            print(f"Error in vibration thread: {e}")
        finally:
            # Ensure vibration stops when thread ends
            if hasattr(self.joystick, 'rumble'):
                self.joystick.rumble(0, 0, 0)
    
    def start_vibration(self, low_frequency: float = 0.5, high_frequency: float = 0.5) -> bool:
        """
        Start controller vibration with specified intensity.
        
        Args:
            low_frequency: Intensity of low-frequency rumble (0.0 to 1.0)
            high_frequency: Intensity of high-frequency rumble (0.0 to 1.0)
            
        Returns:
            bool: True if vibration started or updated successfully
        """
        # Set vibration intensity
        self.current_vibration_intensity = (low_frequency, high_frequency)
        
        # If vibration is already active, just update the intensity
        if self.vibration_active:
            return True
        
        # Start new vibration
        self.vibration_active = True
        self.vibration_stop_event.clear()
        self.vibration_thread = threading.Thread(target=self.vibrate_continuously)
        self.vibration_thread.daemon = True
        self.vibration_thread.start()
        return True
    
    def stop_vibration(self) -> None:
        """Stop all controller vibration."""
        if self.vibration_active:
            self.vibration_stop_event.set()
            if self.vibration_thread and self.vibration_thread.is_alive():
                self.vibration_thread.join(0.2)  # Wait briefly for thread to end
            self.vibration_active = False
            
            # Ensure vibration stops
            if hasattr(self.joystick, 'rumble'):
                self.joystick.rumble(0, 0, 0)
    
    def update_vibration_from_axis(self, axis_value: float) -> None:
        """
        Update vibration intensity based on axis position.
        
        Args:
            axis_value: The current value of the axis (-1.0 to 1.0)
        """
        # Stop vibration if axis is at -1 or less
        if axis_value <= -1:
            if self.vibration_active:
                print("Axis 4 at minimum, stopping vibration")
                self.stop_vibration()
            return
        
        # Map axis value from (-1, 1] to vibration intensity [0, 1]
        normalized_value = (axis_value + 1) / 2
        intensity = max(0, min(1, normalized_value))  # Ensure value is between 0 and 1
        
        # Update vibration intensity
        self.current_vibration_intensity = (intensity, intensity)
        print(f"Axis 4: {axis_value:.2f}, Vibration intensity: {intensity:.2f}")
        
        # Start vibration if not already active and intensity > 0
        if not self.vibration_active and intensity > 0:
            print("Starting vibration from axis 4")
            self.start_vibration(intensity, intensity)
        # Stop vibration if intensity is 0
        elif self.vibration_active and intensity == 0:
            print("Stopping vibration (intensity 0)")
            self.stop_vibration()
    
    def handle_axis_motion(self, axis: int, value: float) -> None:
        """
        Process axis movement events.
        
        Args:
            axis: The axis number that changed
            value: The new position value
        """
        # Skip disabled axes
        if axis in self.DISABLED_AXES:
            return
            
        # Handle axis 4 for vibration control
        if axis == 4:
            self.update_vibration_from_axis(value)
            # Store the value for debugging
            if axis not in self.last_values or value != self.last_values[axis]:
                self.last_values[axis] = value
        
        # Handle other axes
        else:
            # Apply idle zone configuration if available
            if axis in self.IDLE_ZONE_CONFIG:
                threshold = self.IDLE_ZONE_CONFIG[axis]
                if abs(value) < threshold:
                    processed_value = 0.0
                else:
                    # Rescale the value to maintain the full range
                    sign = 1 if value > 0 else -1
                    processed_value = (abs(value) - threshold) / (1 - threshold) * sign
            else:
                processed_value = value
            
            # Only print if the value has changed
            if axis not in self.last_values or processed_value != self.last_values[axis]:
                print(f"Axis {axis} value: {processed_value:.2f}")
                self.last_values[axis] = processed_value
    
    def handle_button_press(self, button: int) -> None:
        """
        Process button press events.
        
        Args:
            button: The button number that was pressed
        """
        print(f"Button {button} pressed")
        self.button_states[button] = True
        
        # Apply vibration preset if this button has one
        if button in self.VIBRATION_PRESETS:
            preset = self.VIBRATION_PRESETS[button]
            intensity = preset["intensity"]
            print(f"{preset['name']} vibration started")
            self.start_vibration(intensity, intensity)
    
    def handle_button_release(self, button: int) -> None:
        """
        Process button release events.
        
        Args:
            button: The button number that was released
        """
        print(f"Button {button} released")
        self.button_states[button] = False
        
        # If the released button was one that started vibration, stop it
        if button in self.VIBRATION_PRESETS:
            print("Vibration stopped")
            self.stop_vibration()
    
    def run(self) -> None:
        """Run the main event loop."""
        try:
            while True:
                for event in pygame.event.get():
                    if event.type == pygame.JOYAXISMOTION:
                        self.handle_axis_motion(event.axis, event.value)
                    elif event.type == pygame.JOYBUTTONDOWN:
                        self.handle_button_press(event.button)
                    elif event.type == pygame.JOYBUTTONUP:
                        self.handle_button_release(event.button)
                
                # Small delay to prevent maxing out CPU
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            print("Exiting...")
        finally:
            # Make sure to stop any ongoing vibration before quitting
            self.stop_vibration()
            pygame.quit()


if __name__ == "__main__":
    controller = ControllerFeedbackSystem()
    controller.run()