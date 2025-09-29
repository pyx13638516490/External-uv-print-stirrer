# main.py (最终性能版 - PWM驱动, 12mm导程)
import machine
import time
import sh1106
from machine import Pin, I2C, ADC
import uasyncio as asyncio
import json
import math

# --- 硬件引脚配置 (无变化) ---
PIN_OLED_SCL = 22
PIN_OLED_SDA = 21
PIN_ENC_A = 18
PIN_ENC_B = 19
PIN_ENC_SW = 5
PIN_BTN_START = 13 
PIN_BTN_STOP = 4
PIN_LDR = 34
PIN_STEP = 17
PIN_DIR = 16
PIN_RELAY = 25
CONFIG_FILE = 'config.json'

# --- 物理参数配置 ---
STEPS_PER_REV = 200
MICROSTEPPING = 8
LEAD_SCREW_PITCH = 12.0 # <--- 已更新为12mm导程
STEPS_PER_MM = (STEPS_PER_REV * MICROSTEPPING) / LEAD_SCREW_PITCH

# --- 软件参数配置 ---
MAX_DISTANCE_MM = 200.0
MAX_SPEED_MMS = 100.0 
MIN_SPEED_MMS = 1.0
ACCEL_DURATION_S = 0.2 
ACCEL_SLICES = 20 

MAX_DELAY_S = 60.0
LIGHT_THRESHOLD = 2000

# --- 全局状态变量 (无变化) ---
target_distance_mm = 10.0
target_speed_mms = 100.0
trigger_delay_s = 2.0
is_moving = False
continuous_run_enabled = False
emergency_stop_flag = False
edit_mode = 0
light_sequence_state = 0
last_debounce_time = 0
encoder_last_state = 0

# --- 初始化硬件 (无变化) ---
i2c = I2C(0, scl=Pin(PIN_OLED_SCL), sda=Pin(PIN_OLED_SDA), freq=400000)
# ... 其余初始化代码无变化 ...
display = sh1106.SH1106_I2C(128, 64, i2c, Pin(PIN_DIR), 0x3c)
display.sleep(False); display.fill(0); display.text('Loading...', 0, 0); display.show()
adc = ADC(Pin(PIN_LDR)); adc.atten(ADC.ATTN_11DB)
enc_a = Pin(PIN_ENC_A, Pin.IN, Pin.PULL_UP)
enc_b = Pin(PIN_ENC_B, Pin.IN, Pin.PULL_UP)
enc_sw = Pin(PIN_ENC_SW, Pin.IN, Pin.PULL_UP)
btn_start = Pin(PIN_BTN_START, Pin.IN, Pin.PULL_UP)
btn_stop = Pin(PIN_BTN_STOP, Pin.IN, Pin.PULL_UP)
step_pin = Pin(PIN_STEP, Pin.OUT)
dir_pin = Pin(PIN_DIR, Pin.OUT)
relay_pin = Pin(PIN_RELAY, Pin.OUT, value=0)

# --- 配置保存与加载 (无变化) ---
def save_config(): #...
    config_data = {'distance': target_distance_mm, 'speed': target_speed_mms, 'delay': trigger_delay_s}
    try:
        with open(CONFIG_FILE, 'w') as f: json.dump(config_data, f)
    except OSError: pass
def load_config(): #...
    global target_distance_mm, target_speed_mms, trigger_delay_s
    try:
        with open(CONFIG_FILE, 'r') as f:
            config_data = json.load(f)
            target_distance_mm = config_data.get('distance', 10.0)
            target_speed_mms = config_data.get('speed', 100.0)
            trigger_delay_s = config_data.get('delay', 2.0)
    except (OSError, ValueError):
        save_config()

# --- 中断处理函数 (无变化) ---
def debounce(pin, delay=300): #...
    global last_debounce_time
    current_time = time.ticks_ms()
    if time.ticks_diff(current_time, last_debounce_time) < delay: return False
    last_debounce_time = current_time
    return True
def handle_encoder_turn(pin): #...
    global encoder_last_state
    global target_distance_mm, target_speed_mms, trigger_delay_s
    currentState = (enc_a.value() << 1) | enc_b.value()
    if currentState == encoder_last_state: return
    transitions = {(0, 1): 1, (1, 3): 1, (3, 2): 1, (2, 0): 1, (0, 2): -1, (2, 3): -1, (3, 1): -1, (1, 0): -1}
    direction = transitions.get((encoder_last_state, currentState), 0)
    if direction != 0:
        if edit_mode == 0:
            target_distance_mm = max(0.0, min(MAX_DISTANCE_MM, target_distance_mm + direction * 0.5))
        elif edit_mode == 1:
            target_speed_mms = max(MIN_SPEED_MMS, min(MAX_SPEED_MMS, target_speed_mms + direction * 1.0))
        else:
            trigger_delay_s = max(0.0, min(MAX_DELAY_S, trigger_delay_s + direction * 0.5))
        save_config()
    encoder_last_state = currentState
def encoder_switch_pressed(pin): #...
    global edit_mode
    if debounce(pin): edit_mode = (edit_mode + 1) % 3
def start_button_pressed(pin): #...
    global continuous_run_enabled, emergency_stop_flag
    if debounce(pin):
        if emergency_stop_flag:
            emergency_stop_flag = False
            return
        if not is_moving:
            continuous_run_enabled = True
            asyncio.create_task(move_stepper_continuous())
        else:
            if continuous_run_enabled:
                continuous_run_enabled = False
def emergency_stop_pressed(pin): #...
    global emergency_stop_flag, continuous_run_enabled
    if debounce(pin, 500):
        if is_moving:
            emergency_stop_flag = True
            continuous_run_enabled = False

# --- 中断绑定 (无变化) ---
enc_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=handle_encoder_turn)
# ...
enc_b.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=handle_encoder_turn)
enc_sw.irq(trigger=Pin.IRQ_FALLING, handler=encoder_switch_pressed)
btn_start.irq(trigger=Pin.IRQ_FALLING, handler=start_button_pressed)
btn_stop.irq(trigger=Pin.IRQ_FALLING, handler=emergency_stop_pressed)

# --- 核心功能函数：PWM版本 (无变化) ---
async def _execute_move(distance_mm, speed_mms, direction):
    if distance_mm <= 0 or emergency_stop_flag:
        return
    dir_pin.value(direction)
    total_steps = int(distance_mm * STEPS_PER_MM)
    start_freq = int(MIN_SPEED_MMS * STEPS_PER_MM)
    target_freq = int(min(speed_mms, MAX_SPEED_MMS) * STEPS_PER_MM)
    if start_freq <= 0: start_freq = 1
    if target_freq < start_freq: target_freq = start_freq
    accel_dist = (MIN_SPEED_MMS + speed_mms) / 2 * ACCEL_DURATION_S
    decel_dist = accel_dist
    if accel_dist + decel_dist > distance_mm:
        ratio = distance_mm / (accel_dist + decel_dist)
        accel_duration = ACCEL_DURATION_S * ratio
        decel_duration = ACCEL_DURATION_S * ratio
        const_duration = 0
    else:
        accel_duration = ACCEL_DURATION_S
        decel_duration = ACCEL_DURATION_S
        const_dist = distance_mm - (accel_dist + decel_dist)
        const_duration = const_dist / speed_mms
    pwm = machine.PWM(step_pin, freq=start_freq, duty_u16=32768)
    try:
        slice_duration_ms = int((accel_duration * 1000) / ACCEL_SLICES)
        if slice_duration_ms > 0:
            for i in range(1, ACCEL_SLICES + 1):
                if emergency_stop_flag: raise KeyboardInterrupt
                current_freq = start_freq + int((target_freq - start_freq) * (i / ACCEL_SLICES))
                pwm.freq(current_freq)
                await asyncio.sleep_ms(slice_duration_ms)
        if const_duration > 0:
            pwm.freq(target_freq)
            await asyncio.sleep(const_duration)
            if emergency_stop_flag: raise KeyboardInterrupt
        slice_duration_ms = int((decel_duration * 1000) / ACCEL_SLICES)
        if slice_duration_ms > 0:
            for i in range(1, ACCEL_SLICES + 1):
                if emergency_stop_flag: raise KeyboardInterrupt
                current_freq = target_freq - int((target_freq - start_freq) * (i / ACCEL_SLICES))
                if current_freq <= 0: current_freq = 1
                pwm.freq(current_freq)
                await asyncio.sleep_ms(slice_duration_ms)
    except KeyboardInterrupt:
        print("Move interrupted by E-STOP.")
    finally:
        pwm.deinit()
        step_pin.value(0)

# --- (其余所有异步任务均无需修改) ---
async def move_stepper_roundtrip():
    global is_moving
    if is_moving or emergency_stop_flag: return
    is_moving = True
    await _execute_move(target_distance_mm, target_speed_mms, 0)
    if not emergency_stop_flag:
        await asyncio.sleep_ms(100)
        await _execute_move(target_distance_mm, target_speed_mms, 1)
    is_moving = False
# ...
async def move_stepper_continuous():
    while continuous_run_enabled and not emergency_stop_flag:
        await move_stepper_roundtrip()
        if not emergency_stop_flag: await asyncio.sleep_ms(200)
# ...
async def timed_relay_and_motor_trigger():
    global is_moving
    if is_moving: return
    relay_pin.value(1)
    await asyncio.sleep(trigger_delay_s)
    relay_pin.value(0)
    asyncio.create_task(move_stepper_roundtrip())
# ...
async def update_display_task():
    while True:
        display.fill(0)
        status_str = "S: "
        if emergency_stop_flag: status_str += "E-STOP!"
        elif is_moving: status_str += "MOVING" + ("..." if continuous_run_enabled else "(!)")
        elif continuous_run_enabled: status_str += "ARMED"
        else: status_str += "IDLE"
        if not is_moving and not continuous_run_enabled and not emergency_stop_flag: display.text(">", 0, edit_mode * 16)
        display.text(f"Dist: {target_distance_mm:.1f}mm", 10, 1)
        display.text(f"Spd: {target_speed_mms:.1f}mm/s", 10, 17)
        display.text(f"Delay: {trigger_delay_s:.1f}s", 10, 33)
        light_value = adc.read()
        display.text(status_str, 0, 49)
        display.text(f"L:{light_value}", 80, 49)
        display.show()
        await asyncio.sleep_ms(100)
# ...
async def light_sensor_monitor_task():
    global light_sequence_state
    initial_light_value = adc.read()
    light_sequence_state = -1 if initial_light_value < LIGHT_THRESHOLD else 0
    while True:
        if continuous_run_enabled or emergency_stop_flag or is_moving:
            await asyncio.sleep_ms(200)
            continue
        light_value = adc.read()
        is_lit_now = light_value < LIGHT_THRESHOLD
        if light_sequence_state == -1 and not is_lit_now:
            light_sequence_state = 0
        elif light_sequence_state == 0 and is_lit_now:
            light_sequence_state = 1
        elif light_sequence_state == 1 and not is_lit_now:
            asyncio.create_task(timed_relay_and_motor_trigger())
            light_sequence_state = 0
        await asyncio.sleep_ms(100)
# ...        
async def main():
    load_config()
    global encoder_last_state
    encoder_last_state = (enc_a.value() << 1) | enc_b.value()
    print("System Ready.")
    asyncio.create_task(update_display_task())
    asyncio.create_task(light_sensor_monitor_task())
    while True:
        if emergency_stop_flag:
            is_moving = False
            continuous_run_enabled = False
        await asyncio.sleep(1)

# 运行主程序
try:
    asyncio.run(main())
except KeyboardInterrupt:
    print("Program stopped.")
finally:
    display.fill(0); display.show()