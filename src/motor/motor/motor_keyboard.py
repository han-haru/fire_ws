import sys, termios, tty, select, time, threading
import lgpio as lg
import RPi.GPIO as GPIO

############################################
# Blade (DC) - RPi.GPIO PWM
############################################
PWM_PIN = 19        # 하드웨어 PWM 핀
DIR_PIN = 26        # 방향 핀
PWM_FREQ = 1000     # Hz

GPIO.setmode(GPIO.BCM)
GPIO.setup(PWM_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)
pwm = GPIO.PWM(PWM_PIN, PWM_FREQ)
pwm.start(0)

blade_percent = 0   # -100 ~ +100
blade_on = False

def set_blade(percent):
    """Blade 속도/방향 설정 (비블로킹, 하드웨어 PWM)"""
    global blade_percent
    blade_percent = max(-100, min(100, percent))
    duty = abs(blade_percent)
    GPIO.output(DIR_PIN, GPIO.HIGH if blade_percent >= 0 else GPIO.LOW)
    pwm.ChangeDutyCycle(duty)

def blade_toggle():
    global blade_on
    blade_on = not blade_on
    if blade_on:
        set_blade(blade_percent)
        print(f"[Blade] ON (speed={blade_percent}%)")
    else:
        pwm.ChangeDutyCycle(0)
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
GPIOCHIP = 4
PIN_RELAY_IN = 5     # ENA- 릴레이 IN
PIN_PUL_MINUS = 6    # PUL-
PIN_DIR_MINUS = 13   # DIR-

HIGH_ACTIVE = True
ON_LEVEL = 1 if HIGH_ACTIVE else 0
OFF_LEVEL = 1 - ON_LEVEL

# 펄스 타이밍
PULSE_LOW_US = 20
MIN_GAP_US   = 6

# SSH/파이썬 환경 현실적인 기본 주파수(안정 범위에서 시작)
ARM_FREQ_HZ  = 800     # 필요시 800~1500 사이에서 실험적으로 조정

arm_thread = None
arm_stop_event = threading.Event()

def relay_enable(h, on=True):
    lg.gpio_write(h, PIN_RELAY_IN, ON_LEVEL if on else OFF_LEVEL)

def set_dir_arm(h, cw: bool):
    # CW → DIR- LOW 싱크, CCW → High-Z
    if cw:
        lg.gpio_claim_output(h, PIN_DIR_MINUS, 0)
    else:
        lg.gpio_claim_input(h, PIN_DIR_MINUS)

def _pulse_once(h, freq_hz):
    period = 1.0 / float(freq_hz)
    low_s  = max(PULSE_LOW_US * 1e-6, 10e-6)
    high_s = max(period - low_s, MIN_GAP_US * 1e-6)
    lg.gpio_claim_output(h, PIN_PUL_MINUS, 0)   # LOW
    time.sleep(low_s)
    lg.gpio_claim_input(h, PIN_PUL_MINUS)       # High-Z
    time.sleep(high_s)

def arm_run_continuous(h, cw, stop_event, freq_hz=ARM_FREQ_HZ, ramp_s=0.25):
    """가속 램프 후 일정 주파수 유지. stop_event set 시 정지(Enable 유지)."""
    relay_enable(h, True)
    set_dir_arm(h, cw)
    time.sleep(0.001)

    # 간단 가속 램프
    f0 = 200.0
    steps = max(int(ramp_s / 0.01), 1)
    freqs = [f0 + (freq_hz - f0) * i / steps for i in range(1, steps + 1)]

    print(f"[Arm] START {'UP' if cw else 'DOWN'} (→{freq_hz}Hz)")
    for f in freqs:
        if stop_event.is_set(): break
        _pulse_once(h, f)

    while not stop_event.is_set():
        _pulse_once(h, freq_hz)

    print("[Arm] STOP (holding torque active)")

def arm_start(h, cw=True):
    """이미 동작 중이면 무시, 아니면 시작"""
    global arm_thread, arm_stop_event
    if arm_thread is None or not arm_thread.is_alive():
        arm_stop_event.clear()
        arm_thread = threading.Thread(target=arm_run_continuous,
                                      args=(h, cw, arm_stop_event),
                                      daemon=True)
        arm_thread.start()

def arm_stop():
    """정지(Enable 유지)"""
    arm_stop_event.set()
    print("[Arm] stop signal")

def arm_release(h):
    """토크 완전 해제 (free)"""
    relay_enable(h, False)
    print("[Arm] RELEASED (free mode)")

############################################
# SSH용 입력 처리 (stdin, raw, select)
############################################
HOLD_TIMEOUT = 0.15   # 초. 이 시간 내로 같은 키가 안 오면 '뗀 것'으로 간주
POLL_DT      = 0.02   # 메인 루프 주기

def raw_mode():
    """터미널 raw 모드 컨텍스트"""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)        # 에코 억제 + 즉시 입력
    return fd, old

def restore_mode(fd, old):
    termios.tcsetattr(fd, termios.TCSADRAIN, old)

############################################
# 메인
############################################
def main():
    h = lg.gpiochip_open(GPIOCHIP)
    # 안전 초기화
    lg.gpio_claim_output(h, PIN_RELAY_IN, OFF_LEVEL)  # Disable(발열↓)
    lg.gpio_claim_input(h, PIN_DIR_MINUS)
    lg.gpio_claim_input(h, PIN_PUL_MINUS)

    print("=== Control Keys (SSH-friendly) ===")
    print("Blade: [w]=+speed, [s]=-speed, [d]=toggle ON/OFF, [a]=reverse dir")
    print("Arm:   [i]=up (hold), [k]=down (hold), [l]=release (disable torque)")
    print("Stop:  [space] (Arm immediate stop)")
    print("Exit:  [q]")
    print("===================================")

    last_i = 0.0
    last_k = 0.0
    moving = None     # 'up' / 'down' / None

    fd, old = raw_mode()
    try:
        while True:
            # 키 읽기 (논블로킹)
            r, _, _ = select.select([sys.stdin], [], [], POLL_DT)
            now = time.time()

            if r:
                ch = sys.stdin.read(1)
                if ch == "q":
                    print("종료합니다.")
                    break

                # ----- Blade -----
                if ch == "d":
                    blade_toggle()
                elif ch == "w":
                    blade_speed_up()
                elif ch == "s":
                    blade_speed_down()
                elif ch == "a":
                    blade_reverse()

                # ----- Arm: 누르고 있는 동안만 -----
                elif ch == "i":
                    last_i = now    # auto-repeat로 주기적으로 업데이트됨
                elif ch == "k":
                    last_k = now
                elif ch == " ":
                    arm_stop()
                elif ch == "l":
                    arm_release(h)

            # 홀드 판단 & 제어 (i 우선 → k 우선은 정책에 따라 바꿀 수 있음)
            if now - last_i < HOLD_TIMEOUT and not (now - last_k < HOLD_TIMEOUT):
                if moving != 'up':
                    arm_start(h, cw=True)
                    moving = 'up'
            elif now - last_k < HOLD_TIMEOUT and not (now - last_i < HOLD_TIMEOUT):
                if moving != 'down':
                    arm_start(h, cw=False)
                    moving = 'down'
            else:
                # 둘 다 시간 초과 → 멈춤
                if moving is not None:
                    arm_stop()
                    moving = None

    finally:
        restore_mode(fd, old)
        # 자원 정리
        try:
            pwm.stop()
            GPIO.cleanup()
        except:
            pass
        try:
            lg.gpiochip_close(h)
        except:
            pass

if __name__ == "__main__":
    main()
