"""
tilt.py — MPU6050 倾角 + SSD1306 显示（MicroPython / ESP32-S3 N16R8）
====================================================================
第一步目标：只验证"IMU 读数可信 + 屏幕能实时显示"，不接舵机、不做 PID。

设计要点
--------
1. 上电自动标定（2 秒静止）：记录 pitch0 / roll0 / 陀螺零偏，之后角度都减去零点。
   模块没放平、甚至上下颠倒，都能被零点吸收。
2. 自动识别"倒装"：若 az < 0（芯片朝下），就绕 X 轴翻 180° 把坐标纠正成正 Z 朝上。
3. 只用加速度计算角度 + 指数平滑（EMA）。先不叠加陀螺积分，避免轴向符号没确认时
   互补滤波自己打架。等"按压测试"确认了方向，再加陀螺。
4. 屏幕分页刷新：整屏 show() 要 24ms（实测），会顶掉控制周期；改成每周期刷 1 页
   （128B ≈ 3ms），8 个周期刷完一整屏。
5. 循环锁定 100Hz，统计周期抖动（这对以后上 PID 很关键）。

用法
----
Thonny 里直接运行；串口会打印 CSV 和统计。Ctrl-C 结束。
"""

from machine import Pin, I2C
import ssd1306
import time
import math
import struct

# ---------------- 配置 ----------------
SDA, SCL = 17, 18
MPU_ADDR = 0x68
OLED_ADDR = 0x3C
LOOP_HZ = 100
RUN_SECONDS = globals().get("RUN_SECONDS", 20)   # 0 = 一直跑
ZERO_SECONDS = 2.0

# ---------------- 硬件初始化 ----------------
i2c = I2C(0, scl=Pin(SCL), sda=Pin(SDA), freq=400000)
oled = ssd1306.SSD1306(i2c, 128, 64, OLED_ADDR)


def mpu_write(reg, val):
    i2c.writeto_mem(MPU_ADDR, reg, bytes([val]))


def mpu_init():
    mpu_write(0x6B, 0x00)   # PWR_MGMT_1: 退出 sleep
    mpu_write(0x1C, 0x00)   # ACCEL_CONFIG: ±2g
    mpu_write(0x1B, 0x00)   # GYRO_CONFIG: ±250 dps
    time.sleep_ms(100)


def mpu_read():
    d = i2c.readfrom_mem(MPU_ADDR, 0x3B, 14)
    ax, ay, az, _t, gx, gy, gz = struct.unpack(">7h", d)
    return ax, ay, az, gx, gy, gz


def angle_from_accel(ax, ay, az):
    pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
    roll = math.degrees(math.atan2(ay, az))
    return pitch, roll


# ---------------- 屏幕分页刷新 ----------------
def flush_page(p):
    """只推第 p 页（128 字节），约 3ms。"""
    oled.write_cmd(0xB0 | p)
    oled.write_cmd(0x00)
    oled.write_cmd(0x10)
    i2c.writeto(OLED_ADDR, b"\x40" + oled.buffer[p * 128:(p + 1) * 128])


# ---------------- 标定 ----------------
def calibrate():
    oled.fill(0)
    oled.text("Calibrating...", 0, 0)
    oled.text("keep still 2s", 0, 16)
    oled.show()

    n = 0
    sx = sy = sz = sgx = sgy = sgz = 0
    t_end = time.ticks_add(time.ticks_ms(), int(ZERO_SECONDS * 1000))
    flip = None
    while time.ticks_diff(t_end, time.ticks_ms()) > 0:
        ax, ay, az, gx, gy, gz = mpu_read()
        if flip is None:
            flip = -1 if az < 0 else 1          # 自动识别倒装
        if flip < 0:                             # 绕 X 轴翻 180°: y,z 取反
            ay, az, gy, gz = -ay, -az, -gy, -gz
        sx += ax; sy += ay; sz += az
        sgx += gx; sgy += gy; sgz += gz
        n += 1
        time.sleep_ms(10)

    ax, ay, az = sx / n, sy / n, sz / n
    pitch0, roll0 = angle_from_accel(ax, ay, az)
    return flip, pitch0, roll0, sgx / n, sgy / n, sgz / n


# ---------------- 主循环 ----------------
def main():
    mpu_init()
    flip, pitch0, roll0, gbx, gby, gbz = calibrate()
    print("# calib flip=%d pitch0=%.2f roll0=%.2f gyro_bias=(%.0f,%.0f,%.0f)"
          % (flip, pitch0, roll0, gbx, gby, gbz))

    period_us = 1000000 // LOOP_HZ
    next_t = time.ticks_us()
    t_start = time.ticks_ms()
    pitch_f = roll_f = 0.0
    samples = []
    overruns = 0
    max_gap = 0
    k = 0
    page = 0

    while RUN_SECONDS == 0 or time.ticks_diff(time.ticks_ms(), t_start) < RUN_SECONDS * 1000:
        ax, ay, az, gx, gy, gz = mpu_read()
        if flip < 0:
            ay, az, gy, gz = -ay, -az, -gy, -gz

        p, r = angle_from_accel(ax, ay, az)
        p -= pitch0
        r -= roll0
        # EMA 平滑（对方波/振动更稳）
        pitch_f += (p - pitch_f) * 0.25
        roll_f += (r - roll_f) * 0.25
        amag = math.sqrt(ax * ax + ay * ay + az * az) / 16384.0

        # 显示：每 8 个周期整屏更新一次，中间分页推送
        if k % 8 == 0:
            oled.fill(0)
            oled.text("Pitch %+6.1f" % pitch_f, 0, 0)
            oled.text("Roll  %+6.1f" % roll_f, 0, 12)
            oled.text("|a|   %.3fg" % amag, 0, 24)
            oled.text("gyro %d %d %d" % (gx, gy, gz), 0, 40)
            oled.text("100Hz", 0, 54)
        flush_page(page)
        page = (page + 1) % 8

        # 5Hz 抽样留档，跑完统一打印
        if k % 20 == 0:
            samples.append((pitch_f, roll_f, ax, ay, az, gx, gy, gz))

        k += 1
        next_t = time.ticks_add(next_t, period_us)
        gap = time.ticks_diff(next_t, time.ticks_us())
        if gap > 0:
            time.sleep_us(gap)
        else:
            overruns += 1
            if -gap > max_gap:
                max_gap = -gap
            next_t = time.ticks_us()   # 严重滞后就重新对齐

    print("# loops=%d overruns=%d max_late=%.1fms" % (k, overruns, max_gap / 1000.0))
    print("# t_ms,pitch,roll,ax,ay,az,gx,gy,gz")
    for i, s in enumerate(samples):
        print("%d,%.2f,%.2f,%d,%d,%d,%d,%d,%d" % ((i * 200,) + s))

    oled.fill(0)
    oled.text("done", 0, 0)
    oled.show()


if __name__ == "__main__":
    main()
