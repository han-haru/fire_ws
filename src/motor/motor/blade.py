import gpiod, time

CHIP = "/dev/gpiochip4"
PWM1 = 5 # MDD10A에서 PWM1 핀이 RPI5의 어떤 GPIO 핀과 연결되는지 
DIR1 = 6 # MDD10A에서 DIR1 핀이 RPI5의 어떤 GPIO 핀과 연결되는지 
FREQ = 1000
PERIOD = 1.0 / FREQ

chip = gpiod.Chip(CHIP)
settings = gpiod.LineSettings(direction=gpiod.line.Direction.OUTPUT,
                              output_value=gpiod.line.Value.INACTIVE)
req = chip.request_lines(config={PWM1: settings, DIR1: settings},
                         consumer='mdd10a_sw')

def set_motor(percent, seconds=2.0):
    percent = max(-100.0, min(100.0, percent))
    direction = gpiod.line.Value.ACTIVE if percent >= 0 else gpiod.line.Value.INACTIVE
    duty = abs(percent) / 100.0
    req.set_value(DIR1, direction)

    t_end = time.time() + seconds
    while time.time() < t_end:
        if duty <= 0.0:
            req.set_value(PWM1, gpiod.line.Value.INACTIVE)
            time.sleep(PERIOD)
        elif duty >= 1.0:
            req.set_value(PWM1, gpiod.line.Value.ACTIVE)
            time.sleep(PERIOD)
        else:
            req.set_value(PWM1, gpiod.line.Value.ACTIVE)
            time.sleep(PERIOD * duty)
            req.set_value(PWM1, gpiod.line.Value.INACTIVE)
            time.sleep(PERIOD * (1.0 - duty))

try:
    # speed: -100.0 ~ +100.0(%), 음수 = 반대방향, 양수 = 정방향, 0 = 정지
    # set_motor(모터 속도, 작동시간)
    
    #set_motor(+100, 3) # 정방향 duty cycle 100.0 % 로 3초간 작동
    set_motor(-100, 20) # 역방향 duty cycle 70.0 % 로 3초간 작동
    set_motor(0,   1) # 다음과 같이 쓰면 1초간 정지하는 명령의 의미를 가진다
finally:
    req.set_value(PWM1, gpiod.line.Value.INACTIVE)
    req.set_value(DIR1, gpiod.line.Value.INACTIVE)
    req.release()
    chip.close()