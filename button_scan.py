"""
button_scan.py — 找出按钮接在哪个脚（45 秒）
============================================
你只说了"接好了"，没说接哪个脚。这个脚本把所有空闲脚都当成"上拉到 3V3 的输入"
同时监视，谁的电平变了就记下来——你不用告诉我脚号。

请在 45 秒内按 3 次按钮。屏幕会显示最近变化的脚号和按下次数。
结束后把事件写入 buttons.csv（时间, 脚号, 电平）。
"""

from machine import Pin, I2C
import ssd1306
import time

CANDIDATES = [21, 47, 16, 15, 14, 8, 9, 1, 2, 39, 40, 41, 42]
RUN_S = 45

i2c = I2C(0, scl=Pin(18), sda=Pin(17), freq=400000)
oled = ssd1306.SSD1306(i2c, 128, 64, 0x3C)

pins = {}
for n in CANDIDATES:
    try:
        pins[n] = Pin(n, Pin.IN, Pin.PULL_UP)
    except Exception:
        pass

prev = {n: p.value() for n, p in pins.items()}
events = []
last_pin = 0
last_val = 1
count = 0

t0 = time.ticks_ms()
next_t = time.ticks_us()

while time.ticks_diff(time.ticks_ms(), t0) < RUN_S * 1000:
    for n, p in pins.items():
        v = p.value()
        if v != prev[n]:
            prev[n] = v
            events.append((time.ticks_diff(time.ticks_ms(), t0), n, v))
            last_pin, last_val = n, v
            if v == 0:
                count += 1

    oled.fill(0)
    oled.text("GPIO %d" % last_pin, 0, 0)
    oled.text("val %d" % last_val, 0, 14)
    oled.text("press %d" % count, 0, 28)
    oled.text("t %ds" % (time.ticks_diff(time.ticks_ms(), t0) // 1000), 0, 44)
    oled.show()

    next_t = time.ticks_add(next_t, 20000)     # 50Hz
    gap = time.ticks_diff(next_t, time.ticks_us())
    if gap > 0:
        time.sleep_us(gap)
    else:
        next_t = time.ticks_us()

with open("buttons.csv", "w") as f:
    f.write("t_ms,pin,val\n")
    for e in events:
        f.write("%d,%d,%d\n" % e)

oled.fill(0)
oled.text("DONE", 0, 0)
oled.text("press %d" % count, 0, 16)
oled.show()
print("# events=%d presses=%d" % (len(events), count))
