#!/usr/bin/env python3
"""
live_plot.py — Mac 端：小球 + 两条曲线（v3）
===========================================
v2 的两个问题：
  1. 窗口里只有曲线，没有你熟悉的"小球"，看着像没反应；
  2. FuncAnimation 没保存引用，可能被 GC 掉 → 窗口空白、不刷新。
v3：左边一个水平仪小球（和 OLED 一致），右边 Pitch/Roll 曲线；
    animation 保存引用；没数据时标题显示"等待数据"。

用法：
    bash esp32/plot.sh                       # 实时窗口
    bash esp32/plot.sh --selftest            # 不接板子，直接看窗口长什么样（存 PNG）
    bash esp32/plot.sh --headless --seconds 10 --out run.csv --png run.png
"""

import argparse
import collections
import datetime as dt
import glob
import math
import sys
import threading
import time

try:
    import serial
except ImportError:
    sys.exit(
        "缺少 pyserial。\n"
        "  最省事：改用启动脚本  ->  bash esp32/plot.sh\n"
        "  或手动安装：/opt/homebrew/Caskroom/miniconda/base/bin/python3 -m pip install pyserial matplotlib"
    )

try:
    import matplotlib
    if "--headless" in sys.argv or "--selftest" in sys.argv:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
except ImportError:
    sys.exit("需要 matplotlib：bash esp32/plot.sh 会自动找装了它的 python")

plt.rcParams["font.sans-serif"] = ["PingFang SC", "Heiti TC", "Arial Unicode MS"]
plt.rcParams["axes.unicode_minus"] = False


def pick_port(explicit=None):
    if explicit:
        return explicit
    ports = sorted(glob.glob("/dev/cu.usbmodem*"))
    if not ports:
        sys.exit("没找到 /dev/cu.usbmodem*，先确认板子插好")
    return ports[0]


class BoardLink:
    def __init__(self, port, script="tilt_stream.py"):
        self.ser = serial.Serial(port, 115200, timeout=0.2)
        self.ser.dtr = True
        self.ser.rts = False
        time.sleep(0.3)
        self.ser.reset_input_buffer()
        self.script = script
        self.q = collections.deque(maxlen=20000)
        self.stop_flag = threading.Event()
        self.status = 0
        self.stopped = False
        self.thread = None

    def start(self):
        self.ser.write(b"\r\x03\x03")
        time.sleep(0.4)
        self.ser.write(b"\r\x01")
        time.sleep(0.3)
        self.ser.reset_input_buffer()
        self.ser.write(("exec(open('%s').read())" % self.script).encode() + b"\x04")
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def _reader(self):
        while not self.stop_flag.is_set():
            try:
                raw = self.ser.readline()
            except Exception:
                break
            if not raw:
                continue
            line = raw.decode("utf-8", "replace").strip()
            if line.startswith("D,"):
                self.q.append(line)
            elif line.startswith("T,"):
                self.stopped = True
            elif line.startswith("S,"):
                try:
                    self.status = int(line.split(",")[1])
                except Exception:
                    pass
            elif line.startswith("E,"):
                print("# 板端错误:", line)

    def stop(self):
        self.stop_flag.set()
        try:
            self.ser.write(b"\x03\x03")
            time.sleep(0.3)
            self.ser.write(b"\x02")
        except Exception:
            pass
        try:
            self.ser.close()
        except Exception:
            pass


def build_figure(window=10.0):
    fig = plt.figure(figsize=(11, 4.6))
    gs = fig.add_gridspec(1, 2, width_ratios=[1, 2.3])
    axb = fig.add_subplot(gs[0])
    axp = fig.add_subplot(gs[1])

    axb.set_aspect("equal")
    axb.set_xlim(-1.25, 1.25)
    axb.set_ylim(-1.45, 1.25)
    axb.axis("off")
    axb.add_patch(plt.Circle((0, 0), 1.0, fill=False, lw=1.6, color="#999"))
    axb.add_patch(plt.Circle((0, 0), 0.33, fill=False, lw=0.8, color="#ddd"))
    axb.axhline(0, color="#eee", lw=0.8)
    axb.axvline(0, color="#eee", lw=0.8)
    ball, = axb.plot([0], [0], "o", ms=20, color="#1f77b4", zorder=5)
    label = axb.text(0, -1.38, "等待数据", ha="center", fontsize=9, color="#666")

    line_p, = axp.plot([], [], lw=1.4, label="Pitch 俯仰")
    line_r, = axp.plot([], [], lw=1.4, label="Roll 横滚")
    axp.set_xlabel("时间 (s)")
    axp.set_ylabel("角度 (°)")
    axp.grid(alpha=0.3)
    axp.legend(loc="upper right")
    axp.set_title("等待数据…", loc="right", fontsize=9)
    fig.tight_layout()
    return fig, (axb, axp), ball, label, line_p, line_r


def selftest():
    fig, (axb, axp), ball, label, lp, lr = build_figure()
    t = [i * 0.05 for i in range(200)]
    p = [12 * math.sin(2 * math.pi * 0.3 * x) for x in t]
    r = [9 * math.cos(2 * math.pi * 0.2 * x) for x in t]
    lp.set_data(t, p)
    lr.set_data(t, r)
    axp.set_xlim(0, 10)
    axp.set_ylim(-16, 16)
    ball.set_data([math.sin(1.0) * 0.5], [math.cos(0.7) * 0.4])
    label.set_text("Pitch +12.3°  Roll -4.5°")
    axp.set_title("示例数据（未接板）", loc="right", fontsize=9)
    fig.savefig("esp32/plot_preview.png", dpi=130)
    print("# 已生成 esp32/plot_preview.png")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--port")
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--seconds", type=float, default=0)
    ap.add_argument("--out", default=None)
    ap.add_argument("--png", default=None)
    ap.add_argument("--window", type=float, default=10.0)
    args = ap.parse_args()

    if args.selftest:
        selftest()
        return

    port = pick_port(args.port)
    out = args.out or ("esp32_run_%s.csv" % dt.datetime.now().strftime("%H%M%S"))
    link = BoardLink(port)
    link.start()
    print(f"# 端口 {port}，数据 -> {out}，Ctrl-C 结束")

    ts, ps, rs = [], [], []
    fh = open(out, "w")
    fh.write("t_s,pitch,roll\n")
    t0 = time.time()

    def drain():
        while link.q:
            line = link.q.popleft()
            try:
                _, t, p, r = line.split(",")
                ts.append(int(t) / 1000.0)
                ps.append(float(p))
                rs.append(float(r))
                fh.write("%.3f,%s,%s\n" % (ts[-1], p, r))
            except ValueError:
                continue

    if args.headless:
        while not args.seconds or time.time() - t0 < args.seconds:
            drain()
            time.sleep(0.05)
        link.stop()
        fh.close()
        fig, ax = plt.subplots(figsize=(9, 4.5))
        ax.plot(ts, ps, label="Pitch 俯仰")
        ax.plot(ts, rs, label="Roll 横滚")
        ax.set_xlabel("时间 (s)")
        ax.set_ylabel("角度 (°)")
        ax.grid(alpha=0.3)
        ax.legend()
        fig.tight_layout()
        png = args.png or "live_plot.png"
        fig.savefig(png, dpi=130)
        print(f"# 保存 {png}（{len(ts)} 点，板端错误 {link.status}）")
        return

    fig, (axb, axp), ball, label, lp, lr = build_figure(args.window)

    def update(_):
        drain()
        if ts:
            n = min(400, len(ps))
            lp.set_data(ts[-n:], ps[-n:])
            lr.set_data(ts[-n:], rs[-n:])
            axp.set_xlim(max(0, ts[-1] - args.window), ts[-1] + 0.5)
            lo = min(min(ps[-n:]), min(rs[-n:])) - 3
            hi = max(max(ps[-n:]), max(rs[-n:])) + 3
            axp.set_ylim(lo, hi)
            bx = max(-1, min(1, rs[-1] / 30.0))
            by = max(-1, min(1, ps[-1] / 30.0))   # matplotlib 的 y 轴朝上，符号与 OLED 相反
            ball.set_data([bx], [by])
            label.set_text("Pitch %+.1f°  Roll %+.1f°" % (ps[-1], rs[-1]))
            ttl = "急停！" if link.stopped else ("板端错误 %d" % link.status)
            axp.set_title(ttl, loc="right", fontsize=9,
                          color=("#d62728" if link.stopped else "#333"))
        return ball, label, lp, lr

    ani = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        link.stop()
        fh.close()
        print(f"# 结束，数据在 {out}（板端错误 {link.status}）")


if __name__ == "__main__":
    main()
