"""
tilt_stream.py — 板端推流（v2：不会自己死）
===========================================
v1 死因（两个，都得治）：
  1. 循环里任何一次 I2C 异常都会抛出去 → 程序直接退出，屏幕定格；
  2. 50Hz print 若 Mac 端读得慢，USB CDC 缓冲满 → print 阻塞 → 卡住。
v2：
  * 读失败重试 3 次，再失败丢弃该帧继续；连续 50 帧失败重建 I2C 总线；
  * 整个循环体包 try/except，任何异常都不许逃出去；
  * 推流 25Hz、屏幕 5Hz；
  * 每秒上报一次错误数：S,<errs>。

数据：D,<t_ms>,<pitch>,<roll>    状态：S,<errs>
"""

from machine import Pin, I2C
import ssd1306
import time
import math
import struct

SDA, SCL = 17, 18
MPU_ADDR = 0x68
OLED_ADDR = 0x3C
RATE_HZ = 50
CX, CY, RAD = 64, 26, 22

i2c = None
oled = None
errs = 0
btn = Pin(21, Pin.IN, Pin.PULL_UP)     # 急停按钮：按下为 0


def init_bus():
    global i2c, oled
    for freq in (400000, 100000):
        try:
            bus = I2C(0, scl=Pin(SCL), sda=Pin(SDA), freq=freq)
            addrs = bus.scan()
        except Exception:
            continue
        if MPU_ADDR in addrs:
            i2c = bus
            if oled is None and OLED_ADDR in addrs:
                try:
                    oled = ssd1306.SSD1306(bus, 128, 64, OLED_ADDR)
                except Exception:
                    oled = None
            return True
    return False


def read_mpu():
    global errs
    for _ in range(3):
        try:
            d = i2c.readfrom_mem(MPU_ADDR, 0x3B, 14)
            if len(d) == 14:
                ax, ay, az, _t, gx, gy, gz = struct.unpack(">7h", d)
                return ax, ay, az, gx, gy, gz
        except Exception:
            pass
    errs += 1
    return None


def ang(ax, ay, az):
    return (math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az))),
            math.degrees(math.atan2(ay, az)))


def draw(pf, rf):
    if oled is None:
        return
    try:
        oled.fill(0)
        oled.ellipse(CX, CY, RAD, RAD, 1)
        oled.hline(CX - RAD, CY, 2 * RAD, 1)
        oled.vline(CX, CY - RAD, 2 * RAD, 1)
        dx = int(max(-RAD, min(RAD, rf * 2)))
        dy = int(max(-RAD, min(RAD, -pf * 2)))
        oled.fill_rect(CX + dx - 2, CY + dy - 2, 4, 4, 1)
        oled.text("%+5.1f" % pf, 12, 51)
        oled.text("%+5.1f" % rf, 72, 51)
        oled.show()
    except Exception:
        pass


if not init_bus():
    print("E,no_bus")
    while True:
        time.sleep(1)

try:
    i2c.writeto_mem(MPU_ADDR, 0x6B, b"\x00")
    i2c.writeto_mem(MPU_ADDR, 0x1C, b"\x00")
    i2c.writeto_mem(MPU_ADDR, 0x1B, b"\x00")
except Exception:
    pass
time.sleep_ms(100)

n = 0; sx = sy = sz = 0; flip = None
t_end = time.ticks_add(time.ticks_ms(), 2000)
while time.ticks_diff(t_end, time.ticks_ms()) > 0:
    v = read_mpu()
    if v is None:
        continue
    ax, ay, az = v[0], v[1], v[2]
    if flip is None:
        flip = -1 if az < 0 else 1
    if flip < 0:
        ay, az = -ay, -az
    sx += ax; sy += ay; sz += az; n += 1
    time.sleep_ms(10)
p0, r0 = ang(sx / n, sy / n, sz / n)
print("# calib flip=%d p0=%.2f r0=%.2f" % (flip, p0, r0))

period = 1000000 // RATE_HZ
next_t = time.ticks_us()
pf = rf = 0.0
k = 0
t0 = time.ticks_ms()
last_status = 0
fail_streak = 0
low_cnt = 0

while True:
    try:
        v = read_mpu()
        if v is None:
            fail_streak += 1
            if fail_streak % 50 == 0:
                init_bus()
            time.sleep_ms(2)
            continue
        fail_streak = 0
        ax, ay, az, gx, gy, gz = v
        if flip < 0:
            ay, az = -ay, -az
        p, r = ang(ax, ay, az)
        pf += ((p - p0) - pf) * 0.25
        rf += ((r - r0) - rf) * 0.25

        # 急停按钮：连续 5 次（约 100ms）为低才算按下
        if btn.value() == 0:
            low_cnt += 1
        else:
            low_cnt = 0
        if low_cnt >= 5:
            print("T,1")
            print("S,%d" % errs)
            if oled is not None:
                try:
                    oled.fill(0)
                    oled.text("STOP", 0, 0)
                    oled.text("btn GPIO21", 0, 16)
                    oled.show()
                except Exception:
                    pass
            break

        if k % 2 == 0:
            print("D,%d,%.2f,%.2f" % (time.ticks_diff(time.ticks_ms(), t0), pf, rf))
        if k % 10 == 0:
            draw(pf, rf)

        now = time.ticks_ms()
        if time.ticks_diff(now, last_status) > 1000:
            last_status = now
            print("S,%d" % errs)

        k += 1
        next_t = time.ticks_add(next_t, period)
        gap = time.ticks_diff(next_t, time.ticks_us())
        if gap > 0:
            time.sleep_us(gap)
        else:
            next_t = time.ticks_us()
    except Exception as e:
        errs += 1
        if errs <= 3:
            print("E,%s" % e)
        time.sleep_ms(2)
