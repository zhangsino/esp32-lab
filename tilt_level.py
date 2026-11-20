"""
tilt_level.py — 水平仪（图形）+ 数据记录
========================================
屏幕不再有任何英文提示：中间一个圆圈是水平仪，圆点会跟着板子的倾斜方向跑。
你随便倾斜板子玩 45 秒，脚本在板上记录 10Hz 的 CSV，跑完我读回来分析。

圆点方向约定（先按直觉画，趋势对不对我后面从数据核对）：
    板子左边抬起  -> 圆点往左
    板子远端抬起  -> 圆点往上
"""

from machine import Pin, I2C
import ssd1306
import time
import math
import struct

SDA, SCL = 17, 18
MPU_ADDR = 0x68
OLED_ADDR = 0x3C
LOOP_HZ = 100
RUN_S = 45
CX, CY, RAD = 64, 28, 24

i2c = I2C(0, scl=Pin(SCL), sda=Pin(SDA), freq=400000)
oled = ssd1306.SSD1306(i2c, 128, 64, OLED_ADDR)


def angle_from_accel(ax, ay, az):
    return (math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az))),
            math.degrees(math.atan2(ay, az)))


def draw_level(p, r):
    oled.fill(0)
    oled.ellipse(CX, CY, RAD, RAD, 1)
    oled.hline(CX - RAD, CY, 2 * RAD, 1)
    oled.vline(CX, CY - RAD, 2 * RAD, 1)
    k = 2.0                      # 1° -> 2 像素，超出范围就贴边
    dx = int(max(-RAD, min(RAD, r * k)))
    dy = int(max(-RAD, min(RAD, -p * k)))
    oled.fill_rect(CX + dx - 2, CY + dy - 2, 4, 4, 1)
    oled.text("%+5.1f %+5.1f" % (p, r), 26, 56)


i2c.writeto_mem(MPU_ADDR, 0x6B, b"\x00")
i2c.writeto_mem(MPU_ADDR, 0x1C, b"\x00")
i2c.writeto_mem(MPU_ADDR, 0x1B, b"\x00")
time.sleep_ms(100)

# 标定
n = 0; sx = sy = sz = 0; flip = None
t_end = time.ticks_add(time.ticks_ms(), 2000)
while time.ticks_diff(t_end, time.ticks_ms()) > 0:
    ax, ay, az, _t, gx, gy, gz = struct.unpack(">7h", i2c.readfrom_mem(MPU_ADDR, 0x3B, 14))
    if flip is None:
        flip = -1 if az < 0 else 1
    if flip < 0:
        ay, az = -ay, -az
    sx += ax; sy += ay; sz += az; n += 1
    time.sleep_ms(10)
p0, r0 = angle_from_accel(sx / n, sy / n, sz / n)
print("# calib flip=%d p0=%.2f r0=%.2f" % (flip, p0, r0))

period = 1000000 // LOOP_HZ
next_t = time.ticks_us()
pf = rf = 0.0
rows = []
k = 0
t_start = time.ticks_ms()

while time.ticks_diff(time.ticks_ms(), t_start) < RUN_S * 1000:
    ax, ay, az, _t, gx, gy, gz = struct.unpack(">7h", i2c.readfrom_mem(MPU_ADDR, 0x3B, 14))
    if flip < 0:
        ay, az, gy, gz = -ay, -az, -gy, -gz
    p, r = angle_from_accel(ax, ay, az)
    pf += ((p - p0) - pf) * 0.25
    rf += ((r - r0) - rf) * 0.25

    if k % 10 == 0:
        draw_level(pf, rf)
    if k % 10 == 0:
        rows.append("%d,%.2f,%.2f,%d,%d,%d" % (time.ticks_diff(time.ticks_ms(), t_start),
                                               pf, rf, ax, ay, az))
    k += 1
    next_t = time.ticks_add(next_t, period)
    gap = time.ticks_diff(next_t, time.ticks_us())
    if gap > 0:
        time.sleep_us(gap)
    else:
        next_t = time.ticks_us()

with open("tilt.csv", "w") as f:
    f.write("t_ms,pitch,roll,ax,ay,az\n")
    f.write("\n".join(rows))

oled.fill(0); oled.text("DONE", 0, 0); oled.show()
print("# saved tilt.csv rows=%d loops=%d" % (len(rows), k))
