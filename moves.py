import time
import sys
from simple_pid import PID
import busio
from adafruit_pca9685 import PCA9685
import RPi.GPIO as GPIO

SCL = 3
SDA = 2

GPIO.setmode(GPIO.BCM)
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 1000

# Motors — PCA channels
ENAFR = 0
IN1FR = 1
IN2FR = 2
IN3FL = 5
IN4FL = 6
ENBFL = 4
ENARR = 8
IN1RR = 9
IN2RR = 10
IN3RL = 13
IN4RL = 14
ENBRL = 12

# Encoders — GPIO pins
S1FR = 17
S2FR = 27
S1FL = 22
S2FL = 10
S1RR = 9
S2RR = 11
S1RL = 5
S2RL = 6
perRev = 750

PWMOEN = 4
pwmOEn = GPIO.setup(PWMOEN, GPIO.OUT)
pushButton = 26
GPIO.setup(pushButton, GPIO.IN)
oldPushb = 0


def readPush():
    global oldPushb
    pushb = GPIO.input(pushButton)
    if pushb != oldPushb:
        oldPushb = pushb
        return True, pushb
    return False, pushb


def valmap(value, istart, istop, ostart, ostop):
    return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))


def getPWMPer(value):
    return int(valmap(value, 0, 100, 0, 0xFFFF))


high = 0xFFFF
low = 0


# ── Encoder ──────────────────────────────────────────────────────────────────
class Encoder:
    def __init__(self, name, S1, S2, side):
        self.name = name
        self.s1 = S1
        self.s2 = S2
        GPIO.setup(S1, GPIO.IN)
        GPIO.setup(S2, GPIO.IN)
        self.aState = 0
        self.bState = 0
        self.aLastState = 0
        self.bLastState = 0
        self.counter = 0
        self.lastCounter = 0
        self.aturn = 0
        self.bturn = 0
        self.speed = 0
        self.time = time.perf_counter_ns()
        self.lastTime = self.time
        self.side = side

    def read(self):
        self.aState = GPIO.input(self.s1)
        self.bState = GPIO.input(self.s2)
        self.time = time.perf_counter_ns()
        return self.aState, self.bState

    def readEncoder(self):
        aState, bState = self.read()
        if aState != self.aLastState:
            self.counter += 1 if bState != aState else -1
        if bState != self.bLastState:
            self.bturn += 1
        self.aLastState = aState
        self.bLastState = bState

    def readEncoderTest(self):
        self.readEncoder()

    def callback_encoder(self, channel):
        self.readEncoder()

    def readSpeed(self):
        """Returns speed in clicks/ns, corrected for wheel side."""
        if self.time != 0 and self.time != self.lastTime:
            self.speed = self.side * (self.counter -
                                      self.lastCounter) / (self.time -
                                                           self.lastTime)
        else:
            self.speed = 0
        self.lastTime = self.time
        self.lastCounter = self.counter
        return self.speed

    def readSpeedRPS(self):
        """Returns speed in revolutions per second (positive = forward)."""
        return self.readSpeed() * 1e9 / perRev

    def resetSpeed(self):
        self.speed = 0
        self.counter = 0
        self.lastCounter = 0
        self.time = time.perf_counter_ns()
        self.lastTime = time.perf_counter_ns()


# ── Wheel (with PID) ─────────────────────────────────────────────────────────
class Wheel:
    """
    Controls a single wheel motor.

    PID loop:
      - setpoint  : target speed in rev/s (set via set_target_speed)
      - measured  : encoder.readSpeedRPS() called inside update()
      - output    : power adjustment clamped to [-100, 100]

    Call update() in your main loop at a consistent rate (e.g. every 20 ms).
    """

    # Default PID gains — tune these for your motors
    DEFAULT_KP = 1.5
    DEFAULT_KI = 0.8
    DEFAULT_KD = 0.05

    def __init__(self, name, enCh, in1Ch, in2Ch, encoder: Encoder):
        self.name = name
        self.en = pca.channels[enCh]
        self.in1 = pca.channels[in1Ch]
        self.in2 = pca.channels[in2Ch]
        self.encoder = encoder

        # PID: input = measured speed (rev/s), output = power delta (-100..100)
        self.pid = PID(
            self.DEFAULT_KP,
            self.DEFAULT_KI,
            self.DEFAULT_KD,
            setpoint=0,
        )
        self.pid.output_limits = (-100, 100)
        self.pid.sample_time = 0.02  # 20 ms minimum between PID recalculations

        self._current_power = 0         # last commanded power (−100 … 100)
        self._pid_enabled = False

    # ── raw hardware write ──────────────────────────────────────────────────
    def _apply_power(self, power: float):
        """Write power directly to the motor driver (-100 … 100)."""
        power = max(-100, min(100, power))
        self._current_power = power
        self.in1.duty_cycle = high if power > 0 else low
        self.in2.duty_cycle = low if power > 0 else high
        self.en.duty_cycle = getPWMPer(abs(power))

    def brake(self):
        """Electric brake — stops and disables PID."""
        self._pid_enabled = False
        self.in1.duty_cycle = low
        self.in2.duty_cycle = low
        self._current_power = 0

    # ── open-loop move (original behaviour, PID off) ─────────────────────────
    def move(self, power: float):
        """
        Open-loop move (original API).  Disables PID and sets power directly.
        Positive = forward, negative = reverse, 0 = coast.
        """
        self._pid_enabled = False
        self.pid.setpoint = 0          # clear setpoint so resume is clean
        self._apply_power(power)

    # ── closed-loop speed control ───────────────────────────────────────────
    def set_target_speed(self, target_rps: float):
        """
        Enable PID speed control.
        target_rps: desired speed in revolutions per second.
                    Positive = forward, negative = reverse, 0 = hold still.
        """
        self.pid.setpoint = target_rps
        self._pid_enabled = True
        # Reset integrator when direction changes to avoid windup
        self.pid.auto_mode = True

    def update(self):
        """
        Call this in your main loop (ideally every ~20 ms).
        Reads encoder speed, runs PID, and adjusts motor power.
        Only acts when PID is enabled via set_target_speed().
        """
        if not self._pid_enabled:
            return

        measured_speed = self.encoder.readSpeedRPS()
        correction = self.pid(measured_speed)   # PID output = power change

        if correction is None:          # sample_time not elapsed yet
            return

        new_power = self._current_power + correction
        self._apply_power(new_power)

    def tune(self, kp: float, ki: float, kd: float):
        """Adjust PID gains at runtime."""
        self.pid.tunings = (kp, ki, kd)

    @property
    def speed_rps(self) -> float:
        """Current measured wheel speed in rev/s."""
        return self.encoder.readSpeedRPS()


# ── Instances ───────────────────────────────────────────────────────────────

sfl = Encoder("sfl", S1FL, S2FL, -1)
sfr = Encoder("sfr", S1FR, S2FR,  1)
srl = Encoder("srl", S1RL, S2RL, -1)
srr = Encoder("srr", S1RR, S2RR,  1)

fl = Wheel("fl", ENBFL, IN3FL, IN4FL, sfl)
fr = Wheel("fr", ENAFR, IN1FR, IN2FR, sfr)
rl = Wheel("rl", ENBRL, IN3RL, IN4RL, srl)
rr = Wheel("rr", ENARR, IN1RR, IN2RR, srr)

# Encoder interrupts
GPIO.add_event_detect(sfl.s1, GPIO.BOTH, callback=sfl.callback_encoder)
GPIO.add_event_detect(sfr.s1, GPIO.BOTH, callback=sfr.callback_encoder)
GPIO.add_event_detect(srl.s1, GPIO.BOTH, callback=srl.callback_encoder)
GPIO.add_event_detect(srr.s1, GPIO.BOTH, callback=srr.callback_encoder)


# ── Car-level helpers ───────────────────────────────────────────────────────
def stop_car():
    for w in (fl, fr, rl, rr):
        w.brake()
    time.sleep(0.5)
    for enc in (sfl, sfr, srl, srr):
        enc.resetSpeed()


def go_ahead_pid(target_rps: float):
    """Drive all wheels forward at target_rps using PID."""
    for w in (fl, fr, rl, rr):
        w.set_target_speed(target_rps)


def go_back_pid(target_rps: float):
    """Drive all wheels backward at target_rps using PID."""
    for w in (fl, fr, rl, rr):
        w.set_target_speed(-target_rps)


def update_all():
    """Call this in your main loop to tick all four PID controllers."""
    for w in (fl, fr, rl, rr):
        w.update()


# ── Original open-loop helpers (unchanged) ──────────────────────────────────
def go_ahead(power, forSecs):
    for w in (fl, fr, rl, rr):
        w.move(power)
    time.sleep(forSecs)


def go_back(power, forSecs):
    for w in (fl, fr, rl, rr):
        w.move(-power)
    time.sleep(forSecs)


def turn_right(power, forSecs):
    rl.move(power)
    rr.move(-power)
    fl.move(power)
    fr.move(-power)
    time.sleep(forSecs)


def turn_left(power, forSecs):
    rr.move(power)
    rl.move(-power)
    fr.move(power)
    fl.move(-power)
    time.sleep(forSecs)


def shift_left(power, forSecs):
    fr.move(power)
    rr.move(-power)
    rl.move(power)
    fl.move(-power)
    time.sleep(forSecs)


def shift_right(power, forSecs):
    fr.move(-power)
    rr.move(power)
    rl.move(-power)
    fl.move(power)
    time.sleep(forSecs)


def destroy():
    GPIO.output(PWMOEN, 1)
    GPIO.cleanup()


# ── Main ────────────────────────────────────────────────────────────────────
def main():
    print("Starting — PID speed control demo")
    GPIO.output(PWMOEN, 0)

    TARGET_RPS = 2.0   # revolutions per second — adjust to your motor specs
    RUN_SECS = 3.0
    LOOP_HZ = 50    # update loop frequency
    dt = 1.0 / LOOP_HZ

    print(f"Driving forward at {TARGET_RPS} rev/s for {RUN_SECS}s ...")
    go_ahead_pid(TARGET_RPS)

    t_start = time.perf_counter()
    while time.perf_counter() - t_start < RUN_SECS:
        update_all()
        # Optional: print live speeds
        speeds = {w.name: f"{w.speed_rps:+.2f}" for w in (fl, fr, rl, rr)}
        print("speeds (rev/s):", speeds, end="\r")
        time.sleep(dt)

    print("\nStopping.")
    stop_car()
    destroy()
    print("Done.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        stop_car()
        destroy()
        print("\nStopped and cleanup done")
