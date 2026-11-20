# -*- coding: utf-8 -*-
"""
main.py —— ESP32-S3 WiFi HMI 服务端（MicroPython）
=================================================
作用：把手机的浏览器变成一个控制面板。

  ESP32-S3  ──WiFi(STA)──  家里路由器  ──WiFi──  乐视手机浏览器

板端提供：
  GET /            → 控制面板页面（index.html）
  GET /api         → JSON 实时数据 {"p":pitch,"r":roll,"t":ms,"n":seq,"e":err,"ok":1}
  GET /cmd?x=NAME  → 接收下行指令（目前只打印 + 记录最后一条，留给舵机用）
  GET /favicon.ico → 204，避免日志刷屏

设计原则（吃过亏的地方）：
  1. 传感器坏了也要能跑 —— IMU 初始化失败时 /api 返回 ok:0，页面照样能开，
     这样你可以先验证"网页这条路通不通"，再管传感器。
  2. 任何异常都不许逃出协程 —— 单次 I2C 读失败就丢帧，连续多次才重建总线。
  3. 只有一个 REPL —— 别在串口里 print 高频数据，会拖慢循环。

接线（沿用 tilt.py）：
  I2C(0)  SDA=GPIO17  SCL=GPIO18  400kHz
  MPU6050 @0x68

用法：
  Thonny 里把 secrets.py / main.py / index.html 传到板子根目录，复位即可。
"""

import network
import socket
import time
import math
import json
import struct
import asyncio
from machine import Pin, I2C

try:
    from secrets import WIFI_SSID, WIFI_PASS, STATIC_IP
except ImportError:
    WIFI_SSID, WIFI_PASS, STATIC_IP = "SSID", "PASS", None

# ---------------- 配置 ----------------
SDA, SCL = 17, 18
MPU_ADDR = 0x68
ALPHA = 0.20          # EMA 平滑系数，越小越稳但越滞后
LOOP_HZ = 100
HTTP_PORT = 80

# ---------------- 全局状态 ----------------
state = {"p": 0.0, "r": 0.0, "n": 0, "e": 0, "ok": 0, "cmd": ""}
paused = False
i2c = None
flip = 1
pitch0 = roll0 = 0.0
i2c_fail = 0


# ---------------- WiFi ----------------
def wifi_connect():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print("# WiFi 连接中:", WIFI_SSID)
        wlan.connect(WIFI_SSID, WIFI_PASS)
        for _ in range(60):                 # 最多等 15 秒
            if wlan.isconnected():
                break
            time.sleep_ms(250)
    if not wlan.isconnected():
        print("# WiFi 连接失败，进入热点模式（ESP32-HMI / 12345678）")
        ap = network.WLAN(network.AP_IF)
        ap.active(True)
        ap.config(essid="ESP32-HMI", password="12345678")
        return ap.ifconfig()[0]
    # 关闭 WiFi 省电模式：默认的 modem sleep 会引入 100~200ms 延迟，
    # 对 5Hz 轮询影响很大。插电使用不必省这几个毫安。
    try:
        wlan.config(pm=0)
    except Exception as e:
        print("# 关闭省电失败:", e)
    if STATIC_IP:
        try:
            wlan.ifconfig(STATIC_IP)
        except Exception as e:
            print("# 静态 IP 设置失败:", e)
    ip = wlan.ifconfig()[0]
    print("# WiFi OK  IP =", ip)
    return ip


# ---------------- IMU ----------------
def imu_init(retries=5):
    """带重试的初始化。
    WiFi 射频启动时 3.3V 会轻微波动，MPU6050 偶尔首次访问失败，
    实测表现为 ENODEV；重试几次即可。"""
    global i2c
    for i in range(retries):
        try:
            i2c = I2C(0, scl=Pin(SCL), sda=Pin(SDA), freq=400000)
            time.sleep_ms(60)
            who = i2c.readfrom_mem(MPU_ADDR, 0x75, 1)[0]      # WHO_AM_I
            i2c.writeto_mem(MPU_ADDR, 0x6B, b"\x80")          # 软复位
            time.sleep_ms(100)
            i2c.writeto_mem(MPU_ADDR, 0x6B, b"\x00")          # 唤醒
            time.sleep_ms(50)
            i2c.writeto_mem(MPU_ADDR, 0x1C, b"\x00")          # ±2g
            i2c.writeto_mem(MPU_ADDR, 0x1B, b"\x00")          # ±250 dps
            i2c.writeto_mem(MPU_ADDR, 0x19, b"\x04")          # DLPF 20Hz
            i2c.readfrom_mem(MPU_ADDR, 0x3B, 6)                # 验证能读数据
            print("# IMU OK  第%d次尝试  WHO_AM_I=0x%02x" % (i + 1, who))
            return True
        except Exception as e:
            print("# IMU 第%d次失败: %s" % (i + 1, e))
            time.sleep_ms(300)
    return False


def imu_read():
    d = i2c.readfrom_mem(MPU_ADDR, 0x3B, 6)
    ax, ay, az = struct.unpack(">hhh", d)
    return ax / 16384.0, ay / 16384.0, az / 16384.0


def angle(ax, ay, az):
    p = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
    r = math.degrees(math.atan2(ay, az))
    return p, r


def calibrate(ok):
    global flip, pitch0, roll0
    if not ok:
        return
    n = 0
    sp = sr = 0.0
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < 2000:
        try:
            ax, ay, az = imu_read()
            if az < 0:          # 芯片倒装
                ax, ay, az = -ax, -ay, -az
            p, r = angle(ax, ay, az)
            sp += p
            sr += r
            n += 1
        except Exception:
            pass
        time.sleep_ms(10)
    if n:
        pitch0, roll0 = sp / n, sr / n
    print("# 标定完成 n=%d pitch0=%.2f roll0=%.2f" % (n, pitch0, roll0))


# ---------------- 采集协程 ----------------
async def imu_loop():
    global i2c_fail, paused
    pf = rf = 0.0
    last_try = 0
    period = 1.0 / LOOP_HZ
    while True:
        t_start = time.ticks_us()

        # ---- 处理下行指令 ----
        cmd = state["cmd"]
        if cmd:
            state["cmd"] = ""
            if cmd == "calib":
                print("# 收到 calib，重新标定（保持静止 2 秒）…")
                calibrate(state["ok"])
                pf = rf = 0.0
                state["p"] = state["r"] = 0.0
            elif cmd == "stop":
                paused = True
                print("# 已暂停")
            elif cmd == "start":
                paused = False
                print("# 已恢复")

        if paused:
            await asyncio.sleep_ms(50)
            continue

        # ---- 传感器自愈：没认到就每 2 秒重试一次 ----
        if not state["ok"]:
            if time.ticks_diff(time.ticks_ms(), last_try) > 2000:
                last_try = time.ticks_ms()
                print("# 传感器未就绪，重试初始化…")
                if imu_init():
                    state["ok"] = 1
                    calibrate(True)
            await asyncio.sleep_ms(50)
            continue

        if state["ok"]:
            try:
                ax, ay, az = imu_read()
                i2c_fail = 0
                if az < 0:
                    ax, ay, az = -ax, -ay, -az
                p, r = angle(ax, ay, az)
                p -= pitch0
                r -= roll0
                pf = pf + ALPHA * (p - pf)
                rf = rf + ALPHA * (r - rf)
                state["p"] = round(pf, 2)
                state["r"] = round(rf, 2)
                state["n"] += 1
            except Exception:
                i2c_fail += 1
                state["e"] += 1
                if i2c_fail == 30:          # 连续 30 次失败 → 重建总线
                    print("# I2C 重建")
                    try:
                        imu_init()
                    except Exception:
                        pass
                    i2c_fail = 0
        # 100Hz 节拍
        dt = time.ticks_diff(time.ticks_us(), t_start)
        gap = int(period * 1000000) - dt
        if gap > 0:
            await asyncio.sleep_ms(gap // 1000 or 1)
        else:
            await asyncio.sleep_ms(1)


# ---------------- HTTP ----------------
_page_cache = None


def get_page():
    global _page_cache
    if _page_cache is None:
        try:
            with open("index.html", "r") as f:
                _page_cache = f.read()
        except Exception as e:
            print("# 读 index.html 失败:", e)
            _page_cache = "<h1>index.html 没上传</h1>"
    return _page_cache


async def http_handle(reader, writer):
    try:
        # 一次读完整请求（HTTP 请求头通常 < 512 字节，会落在同一个 TCP 段里）。
        # 原来逐行 readline() 每行一次 await，实测把单请求拖到 120ms。
        req = await reader.read(512)
        if not req:
            return
        parts = req.split(b"\r\n")[0].split(b" ")
        path = parts[1].decode() if len(parts) > 1 else "/"
        if "?" in path:
            path, qs = path.split("?", 1)
        else:
            qs = ""

        if path == "/":
            body = get_page().encode()
            await writer.awrite(b"HTTP/1.0 200 OK\r\nContent-Type: text/html; charset=utf-8\r\n"
                                b"Cache-Control: no-store\r\nConnection: close\r\n"
                                b"Content-Length: %d\r\n\r\n" % len(body) + body)
        elif path == "/api":
            body = json.dumps({"p": state["p"], "r": state["r"], "t": time.ticks_ms(),
                               "n": state["n"], "e": state["e"], "ok": state["ok"],
                               "c": state["cmd"], "z": 1 if paused else 0}).encode()
            await writer.awrite(b"HTTP/1.0 200 OK\r\nContent-Type: application/json\r\n"
                                b"Cache-Control: no-store\r\nConnection: close\r\n"
                                b"Content-Length: %d\r\n\r\n" % len(body) + body)
        elif path == "/cmd":
            cmd = ""
            for kv in qs.split("&"):
                if kv.startswith("x="):
                    cmd = kv[2:].replace("%20", " ")
            state["cmd"] = cmd
            print("# CMD:", cmd)
            await writer.awrite(b"HTTP/1.0 200 OK\r\nContent-Length: 2\r\n"
                                b"Connection: close\r\n\r\nOK")
        else:
            await writer.awrite(b"HTTP/1.0 204 No Content\r\nConnection: close\r\n\r\n")
        await writer.aclose()
    except Exception as e:
        print("# HTTP 错误:", e)
        try:
            await writer.aclose()
        except Exception:
            pass


async def http_loop():
    # 注意：MicroPython 的 start_server 启动后立刻返回、不阻塞。
    # 必须自己挂住事件循环，否则 asyncio.run() 会结束、服务跟着消失。
    await asyncio.start_server(http_handle, "0.0.0.0", HTTP_PORT)
    while True:
        await asyncio.sleep(3600)


# ---------------- 入口 ----------------
async def main():
    ip = wifi_connect()
    time.sleep_ms(500)          # 等射频稳定，避开启动瞬间的电压跌落
    ok = imu_init()
    state["ok"] = 1 if ok else 0
    calibrate(ok)
    print("# ==============================")
    print("#  手机浏览器打开: http://%s" % ip)
    print("# ==============================")
    asyncio.create_task(imu_loop())
    await http_loop()


try:
    asyncio.run(main())
except KeyboardInterrupt:
    print("# 停止")
