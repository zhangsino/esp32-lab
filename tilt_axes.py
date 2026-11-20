"""
tilt_axes.py — 轴向/符号标定（抬边法，IMU 已粘在平板上）
=========================================================
目的：确定"抬哪条边 → Pitch/Roll 变正还是变负"。这个映射一旦定下来，
以后舵机往哪边顶、PID 输出取什么符号就都定了。

方向约定（以你坐着看板子的视角）：
    LEFT  = 你的左手边
    RIGHT = 你的右手边
    FAR   = 离你最远的那条边
    NEAR  = 靠你最近的那条边（杜邦线出来的一侧）

阶段：READY 8 → LEFT 4 → FLAT 3 → RIGHT 4 → FLAT 3 → FAR 4 → FLAT 3 → NEAR 4 → FLAT 5
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
HEALTH_N = 100
HEALTH_MAX_ERR = 5
WAIT_LIMIT_S = 240

PHASES = [
    ("READY", 8, "GET READY"),
    ("LEFT", 4, "LIFT LEFT"),
    ("FLAT1", 3, "FLAT"),
    ("RIGHT", 4, "LIFT RIGHT"),
    ("FLAT2", 3, "FLAT"),
    ("FAR", 4, "LIFT FAR"),
    ("FLAT3", 3, "FLAT"),
    ("NEAR", 4, "LIFT NEAR"),
    ("FLAT4", 5, "FLAT"),
]


def angle_from_accel(ax, ay, az):
    pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
    roll = math.degrees(math.atan2(ay, az))
    return pitch, roll


class IMU:
    def __init__(self):
        self.i2c = None
        self.freq = None
        self.errs = 0

    def open(self):
        for freq in (400000, 100000):
            try:
                i2c = I2C(0, scl=Pin(SCL), sda=Pin(SDA), freq=freq)
                addrs = i2c.scan()
            except Exception:
                continue
            if OLED_ADDR in addrs and MPU_ADDR in addrs:
                self.i2c, self.freq = i2c, freq
                return True
        return False

    def health(self):
        ok = err = 0
        for _ in range(HEALTH_N):
            try:
                d = self.i2c.readfrom_mem(MPU_ADDR, 0x3B, 14)
                ok += 1 if len(d) == 14 else 0
                err += 0 if len(d) == 14 else 1
            except Exception:
                err += 1
        return -1 if ok == 0 else err

    def read(self):
        for _ in range(3):
            try:
                d = self.i2c.readfrom_mem(MPU_ADDR, 0x3B, 14)
                if len(d) == 14:
                    ax, ay, az, _t, gx, gy, gz = struct.unpack(">7h", d)
                    return ax, ay, az, gx, gy, gz
            except Exception:
                pass
            time.sleep_ms(1)
        self.errs += 1
        return None

    def write(self, reg, val):
        for _ in range(3):
            try:
                self.i2c.writeto_mem(MPU_ADDR, reg, bytes([val]))
                return True
            except Exception:
                time.sleep_ms(1)
        return False


def main():
    imu = IMU()
    oled = None
    t0 = time.ticks_ms()
    last = None

    while time.ticks_diff(time.ticks_ms(), t0) < WAIT_LIMIT_S * 1000:
        msg = None
        if imu.i2c is None and not imu.open():
            msg = ("CHECK WIRES", "no 0x3C/0x68")
        else:
            if oled is None:
                try:
                    oled = ssd1306.SSD1306(imu.i2c, 128, 64, OLED_ADDR)
                except Exception:
                    imu.i2c = None
                    continue
            err = imu.health()
            if err == -1:
                msg = ("WIRING UNSTABLE", "no response")
            elif err > HEALTH_MAX_ERR:
                msg = ("WIRING UNSTABLE", "%d/100 err" % err)
        if msg is None:
            break
        if msg != last:
            print("# waiting:", msg)
            last = msg
        try:
            oled.fill(0); oled.text(msg[0], 0, 0); oled.text(msg[1], 0, 20)
            oled.text("check 4 wires", 0, 36); oled.show()
        except Exception:
            imu.i2c = None; oled = None
        time.sleep(1)

    if imu.i2c is None:
        print("# ABORT: bus never healthy")
        return

    print("# bus ok freq=%d health=%d" % (imu.freq, imu.health()))
    chunk = 128 if imu.freq >= 400000 else 64
    nchunk = 8 if chunk == 128 else 16

    imu.write(0x6B, 0x00); imu.write(0x1C, 0x00); imu.write(0x1B, 0x00)
    time.sleep_ms(100)

    # 标定
    oled.fill(0); oled.text("CALIBRATING", 0, 0); oled.text("keep still 2s", 0, 16); oled.show()
    n = 0; sx = sy = sz = sgx = sgy = sgz = 0; flip = None
    t_end = time.ticks_add(time.ticks_ms(), 2000)
    while time.ticks_diff(t_end, time.ticks_ms()) > 0:
        v = imu.read()
        if v is None:
            continue
        ax, ay, az, gx, gy, gz = v
        if flip is None:
            flip = -1 if az < 0 else 1
        if flip < 0:
            ay, az, gy, gz = -ay, -az, -gy, -gz
        sx += ax; sy += ay; sz += az; sgx += gx; sgy += gy; sgz += gz; n += 1
        time.sleep_ms(10)
    if n == 0:
        print("# ABORT: no samples during calibration"); return
    p0, r0 = angle_from_accel(sx / n, sy / n, sz / n)
    print("# calib flip=%d pitch0=%.2f roll0=%.2f bias=(%.0f,%.0f,%.0f) n=%d"
          % (flip, p0, r0, sgx / n, sgy / n, sgz / n, n))

    period_us = 1000000 // LOOP_HZ
    next_t = time.ticks_us()
    pitch_f = roll_f = 0.0
    rows = []
    stats = {}
    k = 0
    off = 0

    for name, secs, label in PHASES:
        stats[name] = [999, -999, 999, -999]
        t_phase = time.ticks_add(time.ticks_ms(), secs * 1000)
        while time.ticks_diff(t_phase, time.ticks_ms()) > 0:
            v = imu.read()
            if v is None:
                time.sleep_ms(2); continue
            ax, ay, az, gx, gy, gz = v
            if flip < 0:
                ay, az, gy, gz = -ay, -az, -gy, -gz
            p, r = angle_from_accel(ax, ay, az)
            pitch_f += ((p - p0) - pitch_f) * 0.25
            roll_f += ((r - r0) - roll_f) * 0.25

            s = stats[name]
            if pitch_f < s[0]: s[0] = pitch_f
            if pitch_f > s[1]: s[1] = pitch_f
            if roll_f < s[2]: s[2] = roll_f
            if roll_f > s[3]: s[3] = roll_f

            if k % 8 == 0:
                left = time.ticks_diff(t_phase, time.ticks_ms()) / 1000.0
                oled.fill(0)
                oled.text(label, 0, 0)
                oled.text("%.0fs  P%+.1f R%+.1f" % (left, pitch_f, roll_f), 0, 14)
                oled.text("peak P %+.1f" % (s[1] if abs(s[1]) > abs(s[0]) else s[0]), 0, 30)
                oled.text("peak R %+.1f" % (s[3] if abs(s[3]) > abs(s[2]) else s[2]), 0, 42)
                oled.text(name, 0, 56)

            row = off * chunk // 128
            col = (off * chunk) % 128
            try:
                oled.write_cmd(0xB0 | row)
                oled.write_cmd(0x00 | (col & 0x0F))
                oled.write_cmd(0x10 | (col >> 4))
                imu.i2c.writeto(OLED_ADDR, b"\x40" + oled.buffer[off * chunk:(off + 1) * chunk])
            except Exception:
                pass
            off = (off + 1) % nchunk

            if k % 10 == 0:
                rows.append((name, pitch_f, roll_f))
            k += 1
            next_t = time.ticks_add(next_t, period_us)
            gap = time.ticks_diff(next_t, time.ticks_us())
            if gap > 0:
                time.sleep_us(gap)
            else:
                next_t = time.ticks_us()

    print("# loops=%d i2c_errs=%d freq=%d" % (k, imu.errs, imu.freq))
    print("# PHASE: pitch[min,max]  roll[min,max]")
    for name, secs, label in PHASES:
        s = stats[name]
        print("# %-6s P[%+.1f,%+.1f] R[%+.1f,%+.1f]" % (name, s[0], s[1], s[2], s[3]))
    print("# phase,pitch,roll")
    for r in rows:
        print("%s,%.2f,%.2f" % r)

    oled.fill(0); oled.text("DONE", 0, 0); oled.show()


main()
