"""
tilt_graph.py — 图形化引导测试（屏幕上用"方框 + 高亮边 + 箭头"代替英文）
=====================================================================
你上次没看懂屏幕上的英文，这次全部改成图形：
  * 中间一个方框 = 平板（下边 = 靠你最近、出线的那边）
  * 高亮加粗的那条边 = 让你抬起来的边
  * 边上向上的箭头 = 往上抬
不再需要读任何英文单词，只看图和数字。

阶段：STILL 8 → LEFT 4 → FLAT 3 → RIGHT 4 → FLAT 3 → FAR 4 → FLAT 3 → NEAR 4 → FLAT 5
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

# 平板矩形在屏幕上的位置（下边 = NEAR = 靠你出线一侧）
PX, PY, PW, PH = 30, 6, 68, 36
CX, CY = PX + PW // 2, PY + PH // 2

PHASES = [
    (None, 8),
    ("LEFT", 4),
    (None, 3),
    ("RIGHT", 4),
    (None, 3),
    ("FAR", 4),
    (None, 3),
    ("NEAR", 4),
    (None, 5),
]


def arrow_up(x, y, h=7):
    oled.line(x, y, x, y - h, 1)
    oled.line(x, y - h, x - 2, y - h + 3, 1)
    oled.line(x, y - h, x + 2, y - h + 3, 1)


def draw_scene(edge, secs, total, idx, p, r):
    oled.fill(0)
    oled.rect(PX, PY, PW, PH, 1)
    if edge == "LEFT":
        oled.fill_rect(PX - 2, PY, 3, PH, 1)
        arrow_up(PX - 8, CY)
    elif edge == "RIGHT":
        oled.fill_rect(PX + PW - 1, PY, 3, PH, 1)
        arrow_up(PX + PW + 8, CY)
    elif edge == "FAR":                      # 屏幕上边 = 远端
        oled.fill_rect(PX, PY - 2, PW, 3, 1)
        arrow_up(CX, PY + 10)
    elif edge == "NEAR":                     # 屏幕下边 = 近端
        oled.fill_rect(PX, PY + PH - 1, PW, 3, 1)
        arrow_up(CX, PY + PH - 4)
    else:
        oled.hline(PX + 6, CY, PW - 12, 1)   # 放平提示：中间一条横线

    oled.text("%d/%d %ds" % (idx, total, secs), 0, 48)
    oled.text("P%+5.1f R%+5.1f" % (p, r), 0, 56)


def angle_from_accel(ax, ay, az):
    return (math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az))),
            math.degrees(math.atan2(ay, az)))


i2c = I2C(0, scl=Pin(SCL), sda=Pin(SDA), freq=400000)
oled = None


def open_bus():
    global i2c, oled
    for freq in (400000, 100000):
        try:
            bus = I2C(0, scl=Pin(SCL), sda=Pin(SDA), freq=freq)
            addrs = bus.scan()
        except Exception:
            continue
        if OLED_ADDR in addrs and MPU_ADDR in addrs:
            i2c = bus
            oled = ssd1306.SSD1306(bus, 128, 64, OLED_ADDR)
            return freq
    return None


def main():
    freq = open_bus()
    if freq is None:
        print("# ABORT: 0x3C/0x68 not both present")
        return
    print("# bus ok", freq)
    chunk = 128 if freq >= 400000 else 64
    nchunk = 8 if chunk == 128 else 16
    off = 0

    i2c.writeto_mem(MPU_ADDR, 0x6B, b"\x00")
    i2c.writeto_mem(MPU_ADDR, 0x1C, b"\x00")
    i2c.writeto_mem(MPU_ADDR, 0x1B, b"\x00")
    time.sleep_ms(100)

    # 标定
    oled.fill(0); oled.text("CAL", 0, 0); oled.show()
    n = 0; sx = sy = sz = 0; flip = None
    t_end = time.ticks_add(time.ticks_ms(), 2000)
    while time.ticks_diff(t_end, time.ticks_ms()) > 0:
        ax, ay, az, _t, gx, gy, gz = struct.unpack(
            ">7h", i2c.readfrom_mem(MPU_ADDR, 0x3B, 14))
        if flip is None:
            flip = -1 if az < 0 else 1
        if flip < 0:
            ay, az = -ay, -az
        sx += ax; sy += ay; sz += az; n += 1
        time.sleep_ms(10)
    p0, r0 = angle_from_accel(sx / n, sy / n, sz / n)

    period = 1000000 // LOOP_HZ
    next_t = time.ticks_us()
    pf = rf = 0.0
    k = 0
    idx = 0

    for edge, secs in PHASES:
        idx += 1
        t_phase = time.ticks_add(time.ticks_ms(), secs * 1000)
        while time.ticks_diff(t_phase, time.ticks_ms()) > 0:
            ax, ay, az, _t, gx, gy, gz = struct.unpack(
                ">7h", i2c.readfrom_mem(MPU_ADDR, 0x3B, 14))
            if flip < 0:
                ay, az = -ay, -az
            p, r = angle_from_accel(ax, ay, az)
            pf += ((p - p0) - pf) * 0.25
            rf += ((r - r0) - rf) * 0.25

            if k % 10 == 0:
                left = int((time.ticks_diff(t_phase, time.ticks_ms()) + 999) / 1000)
                draw_scene(edge, left, len(PHASES), idx, pf, rf)
            # 分块推屏
            row = off * chunk // 128
            col = (off * chunk) % 128
            oled.write_cmd(0xB0 | row)
            oled.write_cmd(0x00 | (col & 0x0F))
            oled.write_cmd(0x10 | (col >> 4))
            i2c.writeto(OLED_ADDR, b"\x40" + oled.buffer[off * chunk:(off + 1) * chunk])
            off = (off + 1) % nchunk

            k += 1
            next_t = time.ticks_add(next_t, period)
            gap = time.ticks_diff(next_t, time.ticks_us())
            if gap > 0:
                time.sleep_us(gap)
            else:
                next_t = time.ticks_us()

    oled.fill(0); oled.text("DONE", 0, 0); oled.show()
    print("# done loops=%d" % k)


main()
