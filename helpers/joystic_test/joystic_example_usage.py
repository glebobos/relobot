# example_usage.py

from controller import Controller

def on_axis(axis_id: int, value: float):
    print(f"[AXIS {axis_id}] -> {value:.2f}")
    # here you can control a motor, camera, slider, etc.

def on_button_down(btn: int):
    print(f"[BUTTON {btn}] DOWN")
    # start some action

def on_button_up(btn: int):
    print(f"[BUTTON {btn}] UP")
    # stop action

if __name__ == "__main__":
    ctrl = Controller()

    # register axis 0
    ctrl.register_axis(0, on_axis)
    # register button 1
    ctrl.register_button_down(1, on_button_down)
    ctrl.register_button_up(1,   on_button_up)

    # add as many registrations as needed
    ctrl.run()
