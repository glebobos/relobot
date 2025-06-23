# ──────────────────────────────────────────────────────────────────────────────
# CONFIGURATION  ❱  tune all values here
# ──────────────────────────────────────────────────────────────────────────────
# Motor / PWM
FREQUENCY                = 10_000     # Hz
COUNTS_PER_REVOLUTION    = 1
RPM_UPDATE_INTERVAL      = 0.60       # s
INACTIVITY_TIMEOUT       = 5.0        # s
SPEED_SMOOTHING_FACTOR   = 0.20

# Motor – empirical calibration  (see calibration/test.py)
K                        = 0.001381   # PWM = K·ω + C
C                        = 0.048623

# PID
PID_SMOOTH_FACTOR        = 0.20
PID_KP                   = 0.00005
PID_KI                   = 0.00001
PID_KD                   = 0.00001

# Voltage divider  (A2 → R1 : R2)
R1                       = 104_000    # Ω
R2                       = 10_000     # Ω
DIVIDER_RATIO            = (R1 + R2) / R2
CAL                      = 1.022        # extra calibration factor

# ADC
ADC_MAX                  = 65_535
VREF                     = 3.3        # V

# EMA filtering
ALPHA                    = 0.057      # 0 < α ≤ 1  (lower = smoother)
THRESHOLD                = 0.08       # custom rounding threshold

# Main-loop cadence
LOOP_PERIOD              = 0.10       # seconds per outer loop
# ──────────────────────────────────────────────────────────────────────────────


import time, math, board, pwmio, countio, digitalio, usb_cdc, analogio


# ─────────────────────────────── Helper: voltage rounding ────────────────────
def round_to_tenth_custom(value: float, threshold: float = THRESHOLD) -> float:
    """
    Floor to one decimal place, then round up only if the residual
    is >= threshold (so 26.22→26.2, 26.23→26.3 for threshold = 0.03).
    """
    lower_tenth = math.floor(value * 10) / 10
    if value - lower_tenth >= threshold:
        lower_tenth += 0.1
    return round(lower_tenth, 1)


# ─────────────────────────── Hardware initialisation ─────────────────────────
# Motor outputs
FWD_PWM = pwmio.PWMOut(board.D0, frequency=FREQUENCY)
REV_PWM = pwmio.PWMOut(board.D3, frequency=FREQUENCY)

FWD_EN  = digitalio.DigitalInOut(board.D1)
REV_EN  = digitalio.DigitalInOut(board.D4)
for pin in (FWD_EN, REV_EN):
    pin.direction = digitalio.Direction.OUTPUT

# Encoder
encoder = countio.Counter(board.D5, pull=digitalio.Pull.UP)

# ADC for VIN
adc = analogio.AnalogIn(board.A2)
_filtered_adc = None                    # EMA state (module-level for speed)

# ─────────────────────────────── Motor controller ────────────────────────────
class MotorController:
    """Closed-loop speed controller with optional PID refinement."""
    def __init__(self):
        self.prev_count      = 0
        self.prev_time       = time.monotonic()
        self.measured_rpm    = 0.0
        self.current_rpm     = 0.0
        self.target_rpm      = 0.0
        self.last_cmd_time   = time.monotonic()

        # PID
        self.kp, self.ki, self.kd = PID_KP, PID_KI, PID_KD
        self.integral         = 0.0
        self.prev_error       = 0.0
        self.last_pid_time    = time.monotonic()
        self.pid_active       = False
        self.pid_allowed      = True
        self.pwm_adj          = 0.0

    # ───────────── low-level PWM helper
    def _set_pwm(self, norm, forward=True):
        duty = int(65_535 * max(0.0, min(1.0, norm)))
        if duty == 0:                       # stop
            FWD_EN.value = REV_EN.value = False
            FWD_PWM.duty_cycle = REV_PWM.duty_cycle = 0
            return
        FWD_EN.value = REV_EN.value = True
        if forward:
            FWD_PWM.duty_cycle, REV_PWM.duty_cycle = duty, 0
        else:
            FWD_PWM.duty_cycle, REV_PWM.duty_cycle = 0, duty

    # ───────────── model-based feed-forward conversion
    def _rpm_to_pwm(self, rpm):
        forward = rpm >= 0
        omega   = abs(rpm) * 2 * math.pi / 60
        pwm     = max(0.0, min(1.0, K * omega + C))
        return pwm, forward

    # ───────────── public interface
    def set_rpm(self, rpm):
        if rpm != self.target_rpm:
            self.integral = self.prev_error = 0.0
            self.pid_active = False
            self.pwm_adj = 0.0
        self.target_rpm = rpm
        self.last_cmd_time = time.monotonic()

    def update(self):
        now = time.monotonic()

        # inactivity safety cutoff
        if now - self.last_cmd_time > INACTIVITY_TIMEOUT and self.target_rpm:
            print("Inactivity timeout → stop")
            self.set_rpm(0)

        # fresh measurement
        self.measured_rpm = self._calc_rpm()

        # idle handling
        if self.target_rpm == 0:
            self.current_rpm = 0
            self.pid_active = False
            self.pwm_adj = 0
            self._set_pwm(0, True)
            return self.measured_rpm

        # decide whether to enable PID
        if self.target_rpm > 0 and not self.pid_active and self.pid_allowed:
            if abs(self.measured_rpm) >= 0.9 * abs(self.target_rpm):
                print("PID activated")
                self.pid_active = True
                self.integral = self.prev_error = 0.0
                self.last_pid_time = now
        if self.target_rpm < 0:            # reverse → no PID
            self.pid_active = False

        # ───── control law
        if self.pid_active and self.pid_allowed:
            base_pwm, forward = self._rpm_to_pwm(self.target_rpm)
            self._update_pid()
            pwm = max(0.0, min(1.0, base_pwm + self.pwm_adj))
            self._set_pwm(pwm, forward)
        else:
            # smooth ramp toward target
            if abs(self.current_rpm - self.target_rpm) > 0.1:
                self.current_rpm += SPEED_SMOOTHING_FACTOR * (self.target_rpm - self.current_rpm)
                pwm, fwd = self._rpm_to_pwm(self.current_rpm)
                self._set_pwm(pwm, fwd)

        return self.measured_rpm

    # ───────────── PID math
    def _update_pid(self):
        now = time.monotonic()
        dt  = now - self.last_pid_time
        if dt < 0.001:
            return
        err = self.target_rpm - self.measured_rpm
        self.integral = max(-500, min(500, self.integral + err * dt))
        deriv = (err - self.prev_error) / dt
        pid_out = (self.kp * err) + (self.ki * self.integral) + (self.kd * deriv)
        self.pwm_adj = (1 - PID_SMOOTH_FACTOR) * self.pwm_adj + PID_SMOOTH_FACTOR * pid_out
        self.pwm_adj = max(-0.20, min(0.20, self.pwm_adj))
        self.prev_error = err
        self.last_pid_time = now

    # ───────────── encoder RPM
    def _calc_rpm(self):
        now   = time.monotonic()
        count = encoder.count
        dt    = now - self.prev_time
        if dt >= RPM_UPDATE_INTERVAL:
            delta = count - self.prev_count
            rpm   = (delta / COUNTS_PER_REVOLUTION) * (60 / dt)
            self.prev_count, self.prev_time = count, now
            return rpm
        return self.measured_rpm


# ────────────────────────────── Voltage sampler ──────────────────────────────
def sample_voltage():
    """
    Take one ADC sample, update the EMA, and return (vin_raw, vin_display)
    """
    global _filtered_adc
    raw = adc.value
    _filtered_adc = raw if _filtered_adc is None else ALPHA * raw + (1 - ALPHA) * _filtered_adc
    adc_v = _filtered_adc * VREF / ADC_MAX
    vin   = adc_v * DIVIDER_RATIO * CAL
    return vin, round_to_tenth_custom(vin)


# ───────────────────────────────── Main loop ─────────────────────────────────
def process_command(cmd, m: MotorController):
    cmd = cmd.strip().lower()
    if   cmd == 'whoyouare': usb_cdc.data.write(b'knives\n')
    elif cmd == 'test':      m.set_rpm(700)
    elif cmd == 'pid_enable':
        m.pid_allowed = True;  usb_cdc.data.write(b'PID enabled\n')
    elif cmd == 'pid_disable':
        m.pid_allowed = m.pid_active = False;  usb_cdc.data.write(b'PID disabled\n')
    elif cmd == 'pid_status':
        st = "enabled" if m.pid_allowed else "disabled"
        ac = "active"  if m.pid_active  else "inactive"
        msg = f'PID is {st} ({ac})\n'; usb_cdc.data.write(msg.encode())
    else:
        try:  m.set_rpm(float(cmd))
        except ValueError: m.set_rpm(0)

def main():
    motor = MotorController()      # start stopped
    motor.set_rpm(0) 
    while True:
        # ─── USB-CDC command handler
        if usb_cdc.data.in_waiting:
            process_command(usb_cdc.data.readline().decode(), motor)

        # ─── speed loop
        rpm = motor.update()

        # ─── voltage sample (one EMA update per outer loop)
        vin_raw, vin_display = sample_voltage()

        # ─── concurrent telemetry  (CSV:  rpm,vin)
        line = f"{rpm:.1f},{vin_display:.1f}\n"
        usb_cdc.data.write(line.encode())
        print(line, end="")        # mirror to serial console

        time.sleep(LOOP_PERIOD)

if __name__ == "__main__":
    try:
        main()
    finally:
        # fail-safe shutdown
        FWD_EN.value = REV_EN.value = False
        FWD_PWM.duty_cycle = REV_PWM.duty_cycle = 0
