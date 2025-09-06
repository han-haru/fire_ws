import gpiod, time
import lgpio as lg
import threading

############################################
################ Blade Part ################
############################################
# Raspberry Pi 5 & MDD10A & DC motor(S6D15-24A) 연결 설정
CHIP = "/dev/gpiochip4"
PWM1 = 5
DIR1 = 6
FREQ = 1000
PERIOD = 1.0 / FREQ

chip = gpiod.Chip(CHIP)
settings = gpiod.LineSettings(direction=gpiod.line.Direction.OUTPUT,
                              output_value=gpiod.line.Value.INACTIVE)
req = chip.request_lines(config={PWM1: settings, DIR1: settings},
                         consumer='mdd10a_sw')



############################################
################ Arm Part ##################
############################################
# Raspberry Pi 5 & TB6600 & Stepper motor 연결 설정

### TB6600, 릴레이 스위치, 라즈베리파이, stepper motor 연결된 방식 설명
# ======= Common Anode 방식 (High-Z & LOW sink 모드) =======
# ==== High-Z는 TB6600의 채널들의 +, - 채널들을 둘 다 5.1V로 맞춰준다.
# ==== 따라서 채널들 간 전위차가 없어 전류 흐르지 않는 Disable(Free mode, 제어 X)
# ==== LOW sink는 TB6600의 채널들 중 - 채널들을 0.5V로 끌어내려서
# ==== 각 +, - 채널들 사이 전위차가 생겨 전류가 흐른다. 이를 통해 Enable(제어로만 조작 가능)가 된다.
# ==== tip: High-Z, LOW sink의 논리가 ENA와 DIR&PUL이 다르다.
# ==== ENA는 High-Z 상태여야 ENA +,- 사이에 전위차가 없어지고 그래야 Enable(제어 가능 상태)가 된다.
# ==== -> ENA를 릴레이 스위치 N.C.(Normally Closed)를 사용해서 사용할 때만 모터에 전원 공급
# ==== DIR/PUL의 경우 High-Z 상태에서는 전위차가 없어서 전류가 흐르지 않아 제어 되지 않는다.

############################################
# ===== GPIO 핀 설정 (BCM #) =====
GPIOCHIP = 4            # RPi 5
PIN_RELAY_IN = 13       # Relay Switch IN. (Vcc는 +5V) 모터 구동 필요할 때마다 Open하여 ENA +,- 전위차 0
PIN_PUL_MINUS = 19     # TB6600 PUL-  (PUL+는 +5V)
PIN_DIR_MINUS = 26      # TB6600 DIR-  (DIR+는 +5V)

############################################
# ===== 릴레이 트리거 타입 설정 =====
HIGH_ACTIVE = True # 고레벨 트리거여서 (HIGH=on)면 True
ON_LEVEL = 1 if HIGH_ACTIVE else 0
OFF_LEVEL = 1- ON_LEVEL

############################################
# ===== 타이밍 =====
PULSE_LOW_US = 20       # LOW 펄스폭 >= 10us
MIN_GAP_US   = 6        # OFF 최소 여유
DIR_SETUP_S = 0.0002    # DIR 바뀐 후 첫 펄스까지 대기
RELAY_SETTLE_S = 0.02   # 접점 안정 대기(10~20ms)


############################################
# ===== 모터/기어 =====
# TB6600의 실제 하드웨어 상 레버를 조작하여 조정할 수 있음. 소프트웨어로는 불가
# 모터드라이버로 microstep 수 조절
STEPS_PER_REV = 200 # 감속기 없는 모터 1바퀴당 스텝수
MICROSTEP     = 1
GEAR_RATIO    = 45
USTEPS_PER_OUT_REV = STEPS_PER_REV * MICROSTEP * GEAR_RATIO # 감속기 출력축 1바퀴를 돌리기 위한 스텝수


############################################
############블레이드 방향, 속도 설정#############
############################################
def set_blade(percent, seconds=2.0):

    # 블레이드 속도(duty cycle) 범위 설정
    percent = max(-100.0, min(100.0, percent)) 
    duty = abs(percent) / 100.0 # 양수 혹은 0 [1]

    # 모터 회전 방향 설정
    # M1A - 빨강/ M1B - 검정일 때 ACTIVE는 블레이드 pitch 방향 +(로봇 몸체 쪽으로 비산물 보냄), INACTIVE는 pitch 방향 -(로봇 바깥 쪽으로 비산물 보냄)
    direction = gpiod.line.Value.ACTIVE if percent >= 0 else gpiod.line.Value.INACTIVE 
    req.set_value(DIR1, direction)

    t_end = time.time() + seconds
    # duty cycle이 잘못 설정되면 작동 x
    while time.time() < t_end:
        # duty cycle이 음수이면 작동 x
        if duty <= 0.0:
            req.set_value(PWM1, gpiod.line.Value.INACTIVE)
            time.sleep(PERIOD)
        # duty 100 % 이상이면 작동 x 
        elif duty >= 1.0:
            req.set_value(PWM1, gpiod.line.Value.ACTIVE)
            time.sleep(PERIOD)
        else:
            req.set_value(PWM1, gpiod.line.Value.ACTIVE)
            time.sleep(PERIOD * duty)
            req.set_value(PWM1, gpiod.line.Value.INACTIVE)
            time.sleep(PERIOD * (1.0 - duty))


# 출력축 rpm_out을 freq_out으로 변환
def freq_from_out_rpm(rpm_out): return rpm_out * USTEPS_PER_OUT_REV /60 # FREQ(Hz) 계산

# 출력축 freq_out을 rpm_out으로 변환
def out_rpm_from_freq(freq_hz): return (freq_hz * 60.0) / USTEPS_PER_OUT_REV

############################################
# 원하는 출력 속도 지정
RPM_OUT = 1.5 # 출력축 rpm
#REV_TURNS     = 10.0
PAUSE_SEC     = 0.1
FREQ_HZ      = freq_from_out_rpm(RPM_OUT) # 출력축 freq
FREQ_LONG_HZ = 1000

############################################
# TB6600 Enable/Disable 제어
# ===== ENA 극성 제어 by Relay switch =====
def relay_enable(h, on=True):
    """
    릴레이: 저레벨 트리거. ON_LEVEL=코일 ON.
    접점 배선: COM→ENA−, NC→GND(스타)
      - 코일 OFF(기본) : NC 닫힘 → ENA−=0V → Disable(발열↓)
      - 코일 ON(구동시): NC 열림 → ENA− High-Z(~+5V) → Enable
    """
    lg.gpio_write(h, PIN_RELAY_IN, ON_LEVEL if on else OFF_LEVEL)

############################################
#?????????????????????????/
# 한 스텝의 주기 계산
# pulse on/off를 스테퍼 모터에 입력, pulse 주기 짧을수록 더 정밀한 제어 가능
def pulse_stream(h, n_steps, freq_hz):
    """고정 주파수로 n_steps만큼 펄스 출력 (High-Z idle / LOW pulse)"""
    period = 1.0 / float(freq_hz)
    low_s  = max(PULSE_LOW_US * 1e-6, 10e-6)
    high_s = max(period - low_s, MIN_GAP_US * 1e-6)

    # idle : High-Z
    lg.gpio_claim_input(h, PIN_PUL_MINUS)

    for _ in range(int(n_steps)):
        # 펄스 ON = LOW 싱크
        lg.gpio_claim_output(h, PIN_PUL_MINUS, 0)
        time.sleep(low_s)
        # 펄스 OFF = 다시 High-Z
        lg.gpio_claim_input(h, PIN_PUL_MINUS)
        time.sleep(high_s)

def inf_pulse_stream(h, n_steps, freq_hz):
    """고정 주파수로 n_steps만큼 펄스 출력 (High-Z idle / LOW pulse)"""
    period = 1.0 / float(freq_hz)
    low_s  = max(PULSE_LOW_US * 1e-6, 10e-6)
    high_s = max(period - low_s, MIN_GAP_US * 1e-6)

    # idle : High-Z
    lg.gpio_claim_input(h, PIN_PUL_MINUS)

    for _ in range(int(n_steps)):
        # 펄스 ON = LOW 싱크
        lg.gpio_claim_output(h, PIN_PUL_MINUS, 0)
        time.sleep(low_s)
        # 펄스 OFF = 다시 High-Z
        #lg.gpio_claim_input(h, PIN_PUL_MINUS)
        #time.sleep(high_s)


############################################
#############Arm 방향 설정####################
def set_dir_arm(h, cw:bool):
    """CW일 때 DIR- LOW(ON), CCW일 때 High-Z(OFF≈+5V)"""
    if cw:
        lg.gpio_claim_output(h, PIN_DIR_MINUS, 0)     # LOW 싱크
    else:
        lg.gpio_claim_input(h,  PIN_DIR_MINUS)        # High-Z

############################################
# Arm 회전 방향, 회전 주기 조절
def spin_turns_arm(h, turns, cw=True, freq_hz=FREQ_HZ):
    relay_enable(h, True)     # 코일 ON → NC 열림 → ENA- 분리 → Enable
    time.sleep(RELAY_SETTLE_S)          # 릴레이 접점 안정 대기
    set_dir_arm(h, cw)
    time.sleep(DIR_SETUP_S)   # DIR setup time
    total_steps = int(USTEPS_PER_OUT_REV * turns) # 출력축 기준 스텝 수
    pulse_stream(h, total_steps, freq_hz)
    relay_enable(h, False)    # 코일 OFF → NC 닫힘 → ENA- 결합 → Disable
    time.sleep(RELAY_SETTLE_S)          # 릴레이 접점 안정 대기

############################################
# Arm 정지 상태 유지
def resist_turns_arm(h,turns=100000, cw=True, freq_hz=FREQ_LONG_HZ):
    relay_enable(h, True) # stepper Enable
    time.sleep(RELAY_SETTLE_S)
    set_dir_arm(h, cw)
    total_steps = int(USTEPS_PER_OUT_REV * turns)
    inf_pulse_stream(h, total_steps, freq_hz)
    relay_enable(h, True)
    time.sleep(RELAY_SETTLE_S)

############################################
# ====== 유틸: 스텝 구동 예상 시간 계산(릴레이/세팅 시간 포함) ======
def estimate_spin_time(turns, freq_hz):
    steps = int(USTEPS_PER_OUT_REV * float(turns))
    move_s = steps / float(freq_hz)
    # 릴레이 안정 + DIR setup + 릴레이 안정 포함
    return RELAY_SETTLE_S + DIR_SETUP_S + move_s + RELAY_SETTLE_S

#???????????????????
# ====== 스텝퍼: 펄스 시작 순간을 외부에 알리는 버전 ======
def spin_turns_arm_with_event(h, turns, cw=True, freq_hz=FREQ_HZ, start_evt: threading.Event | None = None):
    """spin_turns_arm와 동일하지만 실제 펄스 출력 직전에 start_evt를 set()해서 동기화에 사용."""
    relay_enable(h, True)               # ENA High-Z → Enable
    time.sleep(RELAY_SETTLE_S)
    set_dir_arm(h, cw)
    time.sleep(DIR_SETUP_S)

    # 여기서부터 실제 펄스 쏘기 직전 — 동기화 이벤트 set()
    if start_evt is not None:
        start_evt.set()

    total_steps = int(USTEPS_PER_OUT_REV * turns)
    pulse_stream(h, total_steps, freq_hz)

    relay_enable(h, False)              # ENA 0V → Disable/Free
    time.sleep(RELAY_SETTLE_S)


# ====== DC 블레이드: '정해진 시간' 동안 돌리기(동기화 이벤트 지원) ======
def _run_blade_for_duration(percent, duration_s, start_evt: threading.Event | None = None):
    """start_evt가 주어지면 set()될 때까지 대기했다가 그 시점부터 duration_s만큼 DC 모터 PWM 유지."""
    if start_evt is not None:
        start_evt.wait()
    set_blade(percent, seconds=duration_s)

############################################
# ====== 함께 구동: 턴수 기준 ======
def run_blade_and_arm(h, blade_percent: float, arm_turns: float,
                      arm_cw: bool = True, arm_freq_hz: float = FREQ_HZ, extra_hold_sec: float = 0.0):
    """
    스텝퍼가 펄스를 내기 시작하는 '그 순간'에 맞춰 블레이드 PWM을 시작해서
    스텝 동작이 끝날 때까지 동시에 유지.
    """
    # 스텝 동작 전체에 걸쳐 DC 모터를 켜둘 시간 추정
    duration_s = estimate_spin_time(arm_turns, arm_freq_hz) + max(0.0, extra_hold_sec)

    # 동기화 이벤트 준비
    start_evt = threading.Event()

    # DC 블레이드 스레드 시작 (시작 신호를 기다렸다가 동작)
    t = threading.Thread(target=_run_blade_for_duration,
                         args=(blade_percent, duration_s, start_evt),
                         daemon=True)
    t.start()

    # 스텝퍼는 실제 펄스를 내기 직전에 start_evt.set()을 호출
    spin_turns_arm_with_event(h, arm_turns, cw=arm_cw, freq_hz=arm_freq_hz, start_evt=start_evt)

    # 블레이드 스레드 종료 대기
    t.join()

############################################
# ====== 함께 구동: 각도(deg) 입력 버전 ======
def together(h, blade_percent: float, arm_deg: float,
             arm_cw: bool = True, arm_rpm_out: float = RPM_OUT, extra_hold_sec: float = 0.0):
    """
    arm_deg 도 만큼 회전하는 동안 블레이드(percent)를 동시에 구동.
    arm_rpm_out 는 출력축 기준 rpm (기존 RPM_OUT과 같은 정의).
    """
    turns = float(arm_deg) / 360.0
    freq_hz = freq_from_out_rpm(arm_rpm_out)
    run_blade_and_arm(h, blade_percent, turns, arm_cw=arm_cw, arm_freq_hz=freq_hz, extra_hold_sec=extra_hold_sec)


def main():
    h = lg.gpiochip_open(GPIOCHIP)
    try:
        # 릴레이 IN: 기본 OFF(Disable 유지)
        lg.gpio_claim_output(h, PIN_RELAY_IN, OFF_LEVEL)

        # 초기: ENA = LOW(0V), DIR = High-Z(=CCW), PUL=High-Z
        lg.gpio_claim_input(h, PIN_DIR_MINUS)
        lg.gpio_claim_input(h, PIN_PUL_MINUS)

        # ===== 동시 구동 예시 1: 블레이드 60%, 암 +90도 (CW) @ 1.5 rpm =====
        #print(f"▶ Together: Blade 60% + Arm +90° (CW) @ {RPM_OUT} rpm")
        #together(h, blade_percent=60.0, arm_deg=90.0, arm_cw=True, arm_rpm_out=RPM_OUT, extra_hold_sec=0.0)

        #time.sleep(PAUSE_SEC)

        # ===== 동시 구동 예시 2: 블레이드 50%, 암 -90도 (CCW) @ 2.0 rpm =====
        # === blade percent -는 바깥으로 흙 버림, arm_cw= True는 들어올림
        print("▶ Together: Blade 50% + Arm -90° (CCW) @ 2.0 rpm")
        together(h, blade_percent=-100.0, arm_deg=90.0, arm_cw=True, arm_rpm_out=0.9, extra_hold_sec=10.0)

        print("끝.")

    finally:
        # 안전 종료
        try:
            req.set_value(PWM1, gpiod.line.Value.INACTIVE)
            req.set_value(DIR1, gpiod.line.Value.INACTIVE)
            req.release()
            chip.close()
        except:
            pass
        try:
            lg.gpio_write(h, PIN_RELAY_IN, OFF_LEVEL)
        except:
            pass
        try:
            lg.gpiochip_close(h)
        except:
            pass

if __name__ == "__main__":
    main()