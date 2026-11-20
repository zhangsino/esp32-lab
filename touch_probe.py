"""
touch_probe.py — 触摸点(GPIO13)可行性测试，60 秒
================================================
屏幕只显示一个数字和一个随数值变长的横条。
请用手指按住 GPIO13 那根线/铜箔 3 秒，再松开，重复两三次。

判读：
  * 按下去时数字明显变大、横条变长 → TouchPad 可用；
  * 数字纹丝不动（一直是同一个值）      → 这个固件的 TouchPad 在 S3 上不可用，
                                            急停改用实体按键。
结束时会把 min/max 存到 touch.csv。
"""

from machine import Pin, TouchPad
import ssd1306
from machine import I2C
import time

i2c = I2C(0, scl=Pin(18), sda=Pin(17), freq=400000)
oled = ssd1306.SSD1306(i2c, 128, 64, 0x3C)
t = TouchPad(Pin(13))

RUN_S = 60
lo = 10 ** 9
hi = -1
rows = []
t0 = time.ticks_ms()
next_t = t0

while time.ticks_diff(time.ticks_ms(), t0) < RUN_S * 1000:
    try:
        v = t.read()
    except Exception as e:
        oled.fill(0); oled.text("ERR", 0, 0); oled.text(str(e)[:20], 0, 16); oled.show()
        time.sleep(1); continue
    if v < lo: lo = v
    if v > hi: hi = v
    rows.append(v)

    oled.fill(0)
    oled.text(str(v)[:11], 0, 0)
    span = max(1, hi - lo)
    w = int((v - lo) * 120 / span) if hi > lo else 0
    oled.rect(0, 22, 128, 12, 1)
    if w > 0:
        oled.fill_rect(2, 24, min(124, w), 8, 1)
    oled.text("min %d" % lo, 0, 40)
    oled.text("max %d" % hi, 0, 52)
    oled.show()

    next_t = time.ticks_add(next_t, 50)
    gap = time.ticks_diff(next_t, time.ticks_us())
    if gap > 0:
        time.sleep_us(gap)
    else:
        next_t = time.ticks_us()

with open("touch.csv", "w") as f:
    f.write("value\n")
    for v in rows:
        f.write("%d\n" % v)

oled.fill(0)
oled.text("min %d" % lo, 0, 0)
oled.text("max %d" % hi, 0, 14)
oled.show()
print("# touch min=%d max=%d range=%d n=%d" % (lo, hi, hi - lo, len(rows)))
