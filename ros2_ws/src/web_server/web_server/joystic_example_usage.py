# example_usage.py

from controller import Controller

def on_axis(axis_id: int, value: float):
    print(f"[AXIS {axis_id}] -> {value:.2f}")
    # здесь вы можете управлять мотором, камерой, слайдером и т.д.

def on_button_down(btn: int):
    print(f"[BUTTON {btn}] DOWN")
    # старт какого-то действия

def on_button_up(btn: int):
    print(f"[BUTTON {btn}] UP")
    # стоп действия

if __name__ == "__main__":
    ctrl = Controller()

    # регистрируем ось 0
    ctrl.register_axis(0, on_axis)
    # регистрируем кнопку 1
    ctrl.register_button_down(1, on_button_down)
    ctrl.register_button_up(1,   on_button_up)

    # добавьте столько регистраций, сколько нужно
    ctrl.run()
