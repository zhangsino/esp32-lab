#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mic_doa.py —— 逐个"响声"测方向（双麦 TDOA）

用法:
    python3 mic_doa.py --sec 20 --spacing 0.06

做什么:
    抓一段实时流，找出所有明显的响声（拍手/敲击/说话的重音），
    对每个响声用 GCC-PHAT 求两路时差，再换算成"偏离中轴多少度"。

看什么:
    在左边拍 -> 应报"左"；在右边拍 -> 应报"右"；正前方 -> "中间"。
    如果方向一直是反的，就是两颗麦装反了（或把 --flip 加上）。
"""

import argparse
import socket
import time

import numpy as np

SPEED = 343.0


def fetch(ip, port, sec):
    s = socket.create_connection((ip, port), timeout=5)
    s.sendall(b"GET /stream HTTP/1.1\r\nHost: x\r\n\r\n")
    buf = b""
    while b"\r\n\r\n" not in buf:
        buf += s.recv(4096)
    head, rest = buf.split(b"\r\n\r\n", 1)
    hdr = {}
    for line in head.split(b"\r\n")[1:]:
        if b":" in line:
            k, v = line.split(b":", 1)
            hdr[k.strip().lower().decode()] = v.strip().decode()
    rate = int(hdr.get("x-rate", "32000"))
    nch = int(hdr.get("x-channels", "2"))
    data = bytearray(rest)
    pcm = bytearray()
    t0 = time.time()
    while time.time() - t0 < sec:
        while b"\r\n" not in data:
            d = s.recv(65536)
            if not d:
                raise SystemExit("连接被关闭")
            data += d
        line, data = data.split(b"\r\n", 1)
        n = int(line, 16)
        while len(data) < n + 2:
            d = s.recv(65536)
            if not d:
                raise SystemExit("连接被关闭")
            data += d
        pcm += data[:n]
        data = data[n + 2:]
    s.close()
    return bytes(pcm), rate, nch


def gcc_phat(a, b, max_lag):
    n = 1
    while n < a.size * 2:
        n <<= 1
    A = np.fft.rfft(a, n)
    B = np.fft.rfft(b, n)
    R = A * np.conj(B)
    R /= np.abs(R) + 1e-12
    cc = np.fft.irfft(R, n)
    cc = np.concatenate((cc[-max_lag:], cc[:max_lag + 1]))
    k = int(np.argmax(cc))
    lag = k - max_lag
    # 抛物线插值
    fine = float(lag)
    if 0 < k < len(cc) - 1:
        cm, c0, cp = cc[k - 1], cc[k], cc[k + 1]
        den = cm - 2 * c0 + cp
        if abs(den) > 1e-12:
            fine += 0.5 * (cm - cp) / den
    return fine, float(cc[k])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", default="192.168.101.158")
    ap.add_argument("--port", type=int, default=80)
    ap.add_argument("--sec", type=float, default=20.0)
    ap.add_argument("--spacing", type=float, default=0.06, help="两麦间距（米）")
    ap.add_argument("--flip", action="store_true", help="方向左右对称翻转")
    ap.add_argument("--thr", type=float, default=700.0, help="响声的最小峰值（底噪约 200~300）")
    ap.add_argument("-o", "--out", default="doa.wav")
    a = ap.parse_args()

    print("抓 %.0f 秒… 现在开始：先在左边拍几下，再在右边拍几下" % a.sec)
    pcm, rate, nch = fetch(a.ip, a.port, a.sec)
    x = np.frombuffer(pcm, dtype="<i2").astype(np.float32)
    if nch == 2:
        L = x[0::2].copy()
        R = x[1::2].copy()
    else:
        L = R = x.copy()
    print("收到 %.2f 秒，%d Hz" % (L.size / rate, rate))

    import wave
    with wave.open(a.out, "wb") as w:
        w.setnchannels(nch)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes(pcm)

    # 事件检测：5ms 一帧的 RMS，超过门限且隔开 150ms 就算一次响声
    hop = max(1, rate // 200)
    frames = L.size // hop
    rms = np.array([np.sqrt(np.mean(L[i * hop:(i + 1) * hop] ** 2) + 1e-9) for i in range(frames)])
    thr = max(np.percentile(rms, 99) * 0.15, a.thr * 0.15)
    events = []
    last = -999
    for i in range(frames):
        if rms[i] > thr and (i - last) * hop / rate > 0.4:
            events.append(i)
            last = i
    print("门限 %.0f，找到 %d 个响声\n" % (thr, len(events)))
    if not events:
        print("没听到明显响声（阈值太高或全程太安静）")
        return

    max_lag = int(a.spacing / SPEED * rate * 1.5) + 2
    print("  时间      峰值    亚样本时延    方向")
    for i in events:
        c = i * hop
        seg = int(0.02 * rate)
        s0 = max(0, c - seg // 4)
        Lw = L[s0:s0 + seg]
        Rw = R[s0:s0 + seg]
        if Lw.size < seg:
            continue
        if np.abs(Lw).max() < a.thr:
            continue
        lag, score = gcc_phat(Lw, Rw, max_lag)
        if a.flip:
            lag = -lag
        d = lag / rate * SPEED
        deg = float(np.degrees(np.arcsin(max(-1.0, min(1.0, d / a.spacing)))))
        side = "左" if deg > 8 else ("右" if deg < -8 else "中间")
        clip = "  [削顶]" if np.abs(Lw).max() >= 32000 else ""
        print("  %5.2fs  %6.0f  %+7.2f 样本  %s %+5.0f°%s"
              % (c / rate, np.abs(Lw).max(), lag, side, deg, clip))

    print("\n判读：左边响应该出现「左」，右边响应该出现「右」。")
    print("      如果整体反过来，加 --flip（或把两颗麦对调位置）。")


if __name__ == "__main__":
    main()
