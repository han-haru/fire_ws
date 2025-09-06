import sys, termios, tty, select, time, threading
import lgpio as lg

############################################
# Blade (DC) - lgpio PWM
############################################
GPIOCHIP = 4          # 보통 Raspberry Pi 5는 gpiochip0
PWM_PIN = 19          # PWM 핀
DIR_PIN = 26          # 방향 핀
PWM_FREQ = 1000       # Hz

blade_percent = 0     # -100 ~ +100
blade_on = False

h = None  # gpiochip 핸들

def set_blade(percent):
    """Blade 속도/방향 설정"""
    global blade_percent
    blade_percent = max(-100, min(100, percent))
    duty = abs(blade_percent) / 100.0  # 0.0~1.0
    # 방향 설정
    lg.gpio_write(h, DIR_PIN, 1 if blade_percent >= 0 else 0)
    # PWM 출력
    if blade_on and duty > 0:
        lg.tx_pwm(h, PWM_PIN, PWM_FREQ, duty)
    else:
        lg.tx_pwm(h, PWM_PIN, PWM_FREQ, 0.0)

def blade_toggle():
    global blade_on
    blade_on = not blade_on
    if blade_on:
        set_blade(blade_percent)
        print(f"[Blade] ON (speed={blade_percent}%)")
    else:
        lg.tx_pwm(h, PWM_PIN, PWM_FREQ, 0.0)
        print("[Blade] OFF")

def blade_speed_up():
    global blade_percent
    blade_percent = min(blade_percent + 10, 100)
    if blade_on: set_blade(blade_percent)
    print(f"[Blade] speed ↑ → {blade_percent}%")

def blade_speed_down():
    global blade_percent
    blade_percent = max(blade_percent - 10, -100)
    if blade_on: set_blade(blade_percent)
    print(f"[Blade] speed ↓ → {blade_percent}%")

def blade_reverse():
    global blade_percent
    blade_percent = -blade_percent
    if blade_on: set_blade(blade_percent)
    print(f"[Blade] reverse → {blade_percent}%")

############################################
# Arm (Stepper) - lgpio pulse
############################################
PIN_RELAY_IN = 5     # ENA- 릴레이 IN
PIN_PUL_MINUS = 6    # PUL-
PIN_DIR_MINUS = 13   # DIR-

HIGH_ACTIVE = True
ON_LEVEL = 1 if HIGH_ACTIVE else 0
OFF_LEVEL = 1 - ON_LEVEL

PULSE_LOW_US = 20
MIN_GAP_US   = 6
ARM_FREQ_HZ  = 800

arm_thread = None
arm_stop_event = threading.Event()

def relay_enable(on=True):
    lg.gpio_write(h, PIN_RELAY_IN, ON_LEVEL if on else OFF_LEVEL)

def set_dir_arm(cw: bool):
    lg.gpio_write(h, PIN_DIR_MINUS, 0 if cw else 1)

def _pulse_once(freq_hz):
    period = 1.0 / float(freq_hz)
    low_s  = max(PULSE_LOW_US * 1e-6, 10e-6)
    high_s = max(period - low_s, MIN_GAP_US * 1e-6)
    lg.gpio_write(h, PIN_PUL_MINUS, 0)
    time.sleep(low_s)
    lg.gpio_write(h, PIN_PUL_MINUS, 1)
    time.sleep(high_s)

def arm_run_continuous(cw, stop_event, freq_hz=ARM_FREQ_HZ):
    relay_enable(True)
    set_dir_arm(cw)
    time.sleep(0.001)

    print(f"[Arm] START {'UP' if cw else 'DOWN'} (→{freq_hz}Hz)")
    while not stop_event.is_set():
        _pulse_once(freq_hz)

    print("[Arm] STOP (holding torque active)")

def arm_start(cw=True):
    global arm_thread, arm_stop_event
    if arm_thread is None or not arm_thread.is_alive():
        arm_stop_event.clear()
        arm_thread = threading.Thread(target=arm_run_continuous,
                                      args=(cw, arm_stop_event),
                                      daemon=True)
        arm_thread.start()

def arm_stop():
    arm_stop_event.set()
    print("[Arm] stop signal")

def arm_release():
    relay_enable(False)
    print("[Arm] RELEASED (free mode)")

############################################
# SSH용 입력 처리
############################################
HOLD_TIMEOUT = 0.15
POLL_DT      = 0.02

def raw_mode():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    return fd, old

def restore_mode(fd, old):
    termios.tcsetattr(fd, termios.TCSADRAIN, old)

############################################
# 메인
############################################
def main():
    global h
    h = lg.gpiochip_open(GPIOCHIP)

    # 핀 출력 모드 설정
    lg.gpio_claim_output(h, DIR_PIN, 0)
    lg.gpio_claim_output(h, PWM_PIN, 0)
    lg.gpio_claim_output(h, PIN_RELAY_IN, OFF_LEVEL)
    lg.gpio_claim_output(h, PIN_DIR_MINUS, 0)
    lg.gpio_claim_output(h, PIN_PUL_MINUS, 1)

    print("=== Control Keys (SSH-friendly) ===")
    print("Blade: [w]=+speed, [s]=-speed, [d]=toggle ON/OFF, [a]=reverse dir")
    print("Arm:   [i]=up (hold), [k]=down (hold), [l]=release (disable torque)")
    print("Stop:  [space] (Arm immediate stop)")
    print("Exit:  [q]")
    print("===================================")

    last_i = 0.0
    last_k = 0.0
    moving = None

    fd, old = raw_mode()
    try:
        while True:
            r, _, _ = select.select([sys.stdin], [], [], POLL_DT)
            now = time.time()

            if r:
                ch = sys.stdin.read(1)
                if ch == "q":
                    print("종료합니다.")
                    break

                # Blade
                if ch == "d":
                    blade_toggle()
                elif ch == "w":
                    blade_speed_up()
                elif ch == "s":
                    blade_speed_down()
                elif ch == "a":
                    blade_reverse()

                # Arm
                elif ch == "i":
                    last_i = now
                elif ch == "k":
                    last_k = now
                elif ch == " ":
                    arm_stop()
                elif ch == "l":
                    arm_release()

            if now - last_i < HOLD_TIMEOUT and not (now - last_k < HOLD_TIMEOUT):
                if moving != 'up':
                    arm_start(cw=True)
                    moving = 'up'
            elif now - last_k < HOLD_TIMEOUT and not (now - last_i < HOLD_TIMEOUT):
                if moving != 'down':
                    arm_start(cw=False)
                    moving = 'down'
            else:
                if moving is not None:
                    arm_stop()
                    moving = None

    finally:
        restore_mode(fd, old)
        lg.tx_pwm(h, PWM_PIN, PWM_FREQ, 0.0)
        lg.gpiochip_close(h)

if __name__ == "__main__":
    main()
