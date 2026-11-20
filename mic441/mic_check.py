#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mic_check.py —— 在 Mac 上客观核验双麦（不靠耳朵）

用法:
    python3 mic_check.py                      # 抓 5 秒，存 mic_check.wav
    python3 mic_check.py --sec 8
    python3 mic_check.py --rec                # 用 /rec.wav 完整文件
    python3 mic_check.py --spacing 0.08       # 两麦间距（米），用于估方向

判断标准（"能听见"不算数，看数）:
    1. 有效速率 = 16000 ±5%，时长 ≈ N 秒         -> 采样率对、没丢帧
    2. 两路都不是数字 0（RMS > 3）               -> 两颗麦都在工作
    3. 两路互相关 < 0.95                         -> 是两个独立声道，不是抢同一根线
    4. 说话时 >5kHz 能量 < 30%、谱平坦度 < 0.3   -> 是人声不是噪音
"""

import argparse
import math
import socket
import struct
import time
import wave

import numpy as np

SPEED_OF_SOUND = 343.0


def _parse_headers(head):
    h = {}
    for line in head.split(b"\r\n")[1:]:
        if b":" in line:
            k, v = line.split(b":", 1)
            h[k.strip().lower().decode()] = v.strip().decode()
    return h


def fetch_stream(ip, port, sec):
    s = socket.create_connection((ip, port), timeout=5)
    s.sendall(b"GET /stream HTTP/1.1\r\nHost: x\r\n\r\n")
    buf = b""
    while b"\r\n\r\n" not in buf:
        buf += s.recv(4096)
    head, rest = buf.split(b"\r\n\r\n", 1)
    if b"200" not in head.split(b"\r\n")[0]:
        raise SystemExit("板子没回 200:\n" + head.decode(errors="replace"))
    hdr = _parse_headers(head)
    nch = int(hdr.get("x-channels", "1"))
    rate = int(hdr.get("x-rate", "16000"))
    data = bytearray(rest)
    pcm = bytearray()
    marks = []
    t0 = time.time()
    while time.time() - t0 < sec:
        while b"\r\n" not in data:
            d = s.recv(8192)
            if not d:
                raise SystemExit("板子中途关闭了连接（可能被别的页面占着流）")
            data += d
        line, data = data.split(b"\r\n", 1)
        n = int(line, 16)
        while len(data) < n + 2:
            d = s.recv(8192)
            if not d:
                raise SystemExit("板子中途关闭了连接（可能被别的页面占着流）")
            data += d
        pcm += data[:n]
        data = data[n + 2:]
        marks.append((time.time() - t0, len(pcm)))
    s.close()
    return bytes(pcm), marks, nch, rate


def fetch_rec(ip, port, sec):
    s = socket.create_connection((ip, port), timeout=10)
    s.sendall(b"GET /rec.wav?sec=%d HTTP/1.1\r\nHost: x\r\n\r\n" % sec)
    buf = b""
    while True:
        d = s.recv(16384)
        if not d:
            break
        buf += d
    s.close()
    head, body = buf.split(b"\r\n\r\n", 1)
    nch = struct.unpack("<H", body[22:24])[0]
    rate = struct.unpack("<I", body[24:28])[0]
    return body[44:], [], nch, rate


def rate_from_marks(marks, nch):
    pts = [(t, b) for t, b in marks if t > 0.5 and b > 0]
    if len(pts) < 4:
        return None
    ts = np.array([p[0] for p in pts])
    bs = np.array([p[1] for p in pts])
    return float(np.polyfit(ts, bs, 1)[0] / (2.0 * nch))


def chan_stats(x, name):
    pk = float(np.abs(x).max())
    rms = float(x.std())
    z = float((x == 0).mean() * 100)
    print("  %s  峰值 %5d (%6.1f dBFS)   RMS %5.0f   全0 %4.1f%%"
          % (name, pk, 20 * np.log10(max(pk, 1) / 32768), rms, z))
    return pk, rms


def spectrum(x, rate, name):
    seg = x[: 1 << 15] if x.size >= (1 << 15) else x
    if seg.size < 1024:
        return
    spec = np.abs(np.fft.rfft(seg * np.hanning(seg.size))) + 1e-12
    freqs = np.fft.rfftfreq(seg.size, 1 / rate)
    ps = spec ** 2
    total = ps.sum()
    print("  %s  质心 %.0f Hz | >5kHz %.1f%% | 平坦度 %.3f | 主峰 %.0f Hz"
          % (name, (freqs * ps).sum() / total, 100 * ps[freqs > 5000].sum() / total,
             float(np.exp(np.log(spec).mean()) / spec.mean()), freqs[np.argmax(spec)]))


def inter_channel(L, R, rate, spacing):
    """互相关求两路时延 -> 声源方向。

    时延是整数样本，但 1 个样本 = 2.14cm 路程差，直接用分辨率太粗（±90° 只有几档），
    所以在峰值附近做抛物线插值拿亚样本时延。
    符号约定：lag > 0 表示 R 比 L 晚，即声音先到左麦 -> 声源偏左。
    """
    Lf = L.astype(np.float64)
    Rf = R.astype(np.float64)
    norm = math.sqrt(float(np.dot(Lf, Lf)) * float(np.dot(Rf, Rf))) or 1.0
    cc = {}
    for lag in range(-40, 41):
        if lag >= 0:
            c = float(np.dot(Lf[: L.size - lag], Rf[lag:]))
        else:
            c = float(np.dot(Lf[-lag:], Rf[: R.size + lag]))
        cc[lag] = c / norm
    best_lag = max(cc, key=cc.get)
    best_c = cc[best_lag]

    same = float((L == R).mean() * 100)
    print("  两路完全相同样本 %.1f%%   最大互相关 %.3f @ 时延 %d 样本 (%.2f ms)"
          % (same, best_c, best_lag, best_lag * 1000.0 / rate))
    if best_c >= 0.95:
        print("  ✗ 两路几乎一样 —— 两颗麦可能在抢同一个 slot（查 L/R 是否一高一低）")
        return False

    fine = float(best_lag)
    if -39 < best_lag < 39:
        cm, c0, cp = cc[best_lag - 1], cc[best_lag], cc[best_lag + 1]
        den = cm - 2 * c0 + cp
        if abs(den) > 1e-12:
            fine += 0.5 * (cm - cp) / den          # 抛物线顶点 = 亚样本时延
    print("  亚样本时延 %.2f 样本 (%.1f µs)" % (fine, fine / rate * 1e6))

    d = (fine / rate) * SPEED_OF_SOUND
    deg = math.degrees(math.asin(max(-1.0, min(1.0, d / spacing)))) if spacing > 0 else 0
    side = "左" if deg > 5 else ("右" if deg < -5 else "中间")
    print("  路径差 %.1f mm（间距 %.0f mm）-> 声源约 %s，偏离中轴 %+.0f°"
          % (d * 1000, spacing * 1000, side, deg))
    if abs(fine) >= 39:
        print("  （时延顶到搜索边界，声源可能很近或在侧面，间距/近场另算）")
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", default="192.168.101.158")
    ap.add_argument("--port", type=int, default=80)
    ap.add_argument("--sec", type=float, default=5.0)
    ap.add_argument("--spacing", type=float, default=0.08, help="两麦间距（米）")
    ap.add_argument("-o", "--out", default="mic_check.wav")
    ap.add_argument("--rec", action="store_true")
    a = ap.parse_args()

    print("从 %s 抓 %.1f 秒…" % (a.ip, a.sec))
    sps = None
    if a.rec:
        pcm, _, nch, rate = fetch_rec(a.ip, a.port, int(a.sec))
        wall = a.sec
    else:
        pcm, marks, nch, rate = fetch_stream(a.ip, a.port, a.sec)
        wall = marks[-1][0] if marks else a.sec
        sps = rate_from_marks(marks, nch)

    x = np.frombuffer(pcm, dtype="<i2").astype(np.float32)
    frames = x.size // nch
    print("收到 %d 字节 -> %.2f 秒音频，%d 声道，%d Hz" % (len(pcm), frames / float(rate), nch, rate))

    with wave.open(a.out, "wb") as w:
        w.setnchannels(nch)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes(pcm)
    print("已保存 %s" % a.out)

    print("\n===== 客观指标 =====")
    if sps is None:
        sps = frames / wall if wall else 0
    print("有效速率  %.0f 样本/秒 (误差 %+.1f%%)" % (sps, (sps / float(rate) - 1) * 100))

    ok = True
    if nch == 2:
        L = x[0::2].copy()
        R = x[1::2].copy()
        ok &= chan_stats(L, "左")[1] > 3
        ok &= chan_stats(R, "右")[1] > 3
        spectrum(L, rate, "左")
        spectrum(R, rate, "右")
        print()
        ok &= inter_channel(L, R, rate, a.spacing)
    else:
        ok &= chan_stats(x, "单")[1] > 3
        spectrum(x, rate, "单")

    if abs(sps / float(rate) - 1) > 0.05:
        print("✗ 有效速率偏差超过 5%（%.0f / %d 样本/秒）" % (sps, rate))
        if sps < rate:
            print("  提示：48k 立体声每个客户端要 192 KB/s，板子上行约 430 KB/s。")
            print("        如果浏览器还开着同一个页面，两路就把带宽吃满了 —— 先关掉网页再测。")
        ok = False

    print("\n总体：%s" % ("通过" if ok else "有问题，看上面的 ✗"))


if __name__ == "__main__":
    main()
