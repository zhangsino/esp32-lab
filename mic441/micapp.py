# -*- coding: utf-8 -*-
"""
micapp.py —— ESP32-S3 + 双 INMP441 立体声采集，WiFi 网页拉流
================================================================
设计要点（都可复核，不是"试出来的"）：

1. 为什么走 WiFi 而不是串口
   16kHz / 16bit / 单声道 = 32 KB/s。串口 115200 只有 11.5 KB/s，不够；
   本机实测这块板子 WiFi 上行 429 KB/s（4MB / 9.55s），余量 13 倍。

2. 双麦怎么分左右（2026-11-01 实测 + 用户拍板，B 级，可复现）
   两颗麦 SD/SCK/WS 并联，靠 L/R 脚分声道。本机**实际映射**是：
        L/R 接 **3V3**  -> 数据落在**左** slot（ch=0）  -> 装在物理左边
        L/R 接 **GND**  -> 数据落在**右** slot（ch=1）  -> 装在物理右边
   注意：手册/网上通说"L/R 接 GND = 左声道"，在本机**不成立**。
   证据：只留 GND 那颗时左声道 100% 全 0；插上 3V3 那颗后左声道立刻有能量，
   两路能量相当（865607 / 873804），样本仅 4% 相同、互相关 0.497
   （说明是两条独立通道，不是一颗麦抢两个 slot）。
   结论：**按实测定物理方向，不要按手册推**。
   约定（2026-11-01 用户确认）：物理左 = 3V3 那颗 = ch 0，物理右 = GND 那颗 = ch 1。
   程序仍然开机实测每路能量（detect()），换线接反了也能立刻看出来。

3. 24bit 左对齐 → int16
   32 位 slot 的**高 16 位**就是样本本身，所以 `ptr16[i] = ptr32[i] >> 16`
   一步到位（conv()，viper 实现，本机实测 0.21µs/样本 ≈ 16kHz 下 0.3% CPU）。

4. 采集永远不停
   主循环每 32ms 读一次 I2S（这才是真正的节拍源），没有客户端时读出来就丢。
   这样浏览器一连上，听到的就是"此刻"的声音，而不是 DMA 里攒了几百毫秒的旧数据。

5. 同一时刻只服务一个流
   单麦、单听者；串行比并发更好预测。页面拉流期间不要去请求 /status。

接口：
  GET /                页面
  GET /stream          48kHz/16bit/**立体声**(L,R 交织) 实时 PCM（chunked，带 X-Channels / X-Rate）
  GET /rec.wav?sec=3   录 N 秒，返回双声道 WAV（用于客观核验）
  GET /status          JSON：左右两路各自的峰值/RMS，以及 sps / frames
"""

import gc
import select
import socket
import struct
import time

import network
from machine import I2S, Pin

try:
    from secrets import WIFI_SSID, WIFI_PASS
except ImportError:
    WIFI_SSID, WIFI_PASS = "SSID", "PASS"

# ================= 配置（接线改了只改这三行） =================
PIN_SCK = 10
PIN_WS = 11
PIN_SD = 9
# =============================================================

RATE = 16000          # 采样率 = 送给 INMP441 的 WS 频率（它只是从设备）。16k 是本机实测最稳的一档（单连接 100%，零丢帧）
HTTP_PORT = 80
READ_FRAMES = 512     # 每次读 512 帧 = 2048 字节 = 32ms（@16k）
DMA_BYTES = 65536     # DMA 环 512ms（@16k）
NOISE_FLOOR = 30      # 判断"有没有麦克风"的保守门限（底噪通常个位数）


# ================= 位对齐（热路径，viper） =================
@micropython.viper
def conv(src, dst, nframes: int, ch: int):
    """32bit 帧 / 24bit 左对齐 -> int16。dst 装 nframes 个样本的裸字节。"""
    s = ptr32(src)
    d = ptr16(dst)
    for i in range(nframes):
        d[i] = s[2 * i + ch] >> 16


@micropython.viper
def conv2(src, dst, nframes: int):
    """32bit 帧 -> 交织的 int16 立体声 (L,R,L,R...)。一次把两路都取出来。"""
    s = ptr32(src)
    d = ptr16(dst)
    for i in range(nframes):
        d[2 * i] = s[2 * i] >> 16
        d[2 * i + 1] = s[2 * i + 1] >> 16


@micropython.viper
def stereo_mix(src, dst, nframes: int):
    """单声道 -> 交织立体声（左右同一声），只给 tone 自测用。"""
    s = ptr16(src)
    d = ptr16(dst)
    for i in range(nframes):
        d[2 * i] = s[i]
        d[2 * i + 1] = s[i]


@micropython.viper
def energy(src, nframes: int, ch: int) -> int:
    """一路声道的平均绝对幅度：用来自动判断麦克风落在左还是右。"""
    s = ptr32(src)
    acc = 0
    for i in range(nframes):
        v = int(s[2 * i + ch]) >> 16
        if v & 0x8000:
            v -= 0x10000
        if v < 0:
            v = 0 - v          # viper 不支持一元负号
        acc += v
    return acc // nframes if nframes else 0


# ================= 采集 =================
class Mic:
    def __init__(self, tone_hz=0):
        self.tone_hz = tone_hz
        self.buf = bytearray(READ_FRAMES * 8)
        self.pcm = bytearray(READ_FRAMES * 4)      # 交织立体声 int16: L,R,L,R...
        self.tmp = bytearray(READ_FRAMES * 2)      # tone 模式单声道暂存
        self.ch = 0                                # 信号更强的声道（仅供 /status 参考）
        self.stereo = False                        # 两路都有信号 = 双麦
        self.ok = False
        self.frames = 0
        self.peak = [0, 0]
        self.rms = [0, 0]
        self.t_start = time.ticks_ms()
        self.tone = None
        self.tone_pos = 0
        self.tone_due = 0
        self.i2s = None
        # 预分配发送缓冲：分片头按块长算（48k 下是 "1000\r\n"），长度由代码算，别写死
        self.hdr = b"%x\r\n" % (READ_FRAMES * 4)
        self.hl = len(self.hdr)
        self.tx = bytearray(self.hl + READ_FRAMES * 4 + 2)
        self.tx[0:self.hl] = self.hdr
        self.tx[-2:] = b"\r\n"
        if tone_hz:
            self.tone = self._make_tone(tone_hz)
            self.tone_due = time.ticks_ms()
            self.ok = True
            print("# TONE 模式：%dHz 合成音（不读麦克风）" % tone_hz)
        else:
            self.i2s = I2S(0, sck=Pin(PIN_SCK), ws=Pin(PIN_WS), sd=Pin(PIN_SD),
                           mode=I2S.RX, bits=32, format=I2S.STEREO,
                           rate=RATE, ibuf=DMA_BYTES)

    @staticmethod
    def _make_tone(hz):
        import math
        out = bytearray(RATE * 2)
        for i in range(RATE):
            v = int(10000 * math.sin(2 * math.pi * hz * i / RATE))
            out[i * 2] = v & 0xFF
            out[i * 2 + 1] = (v >> 8) & 0xFF
        return bytes(out)

    def detect(self, ms=250, tries=5):
        """实测哪一路有信号，不猜 L/R 接线。"""
        if self.tone:
            return
        els = [0, 0]
        for _ in range(tries):
            e = [0, 0]
            t0 = time.ticks_ms()
            while time.ticks_diff(time.ticks_ms(), t0) < ms:
                n = self.i2s.readinto(self.buf)
                if n:
                    nf = n // 8
                    e[0] += energy(self.buf, nf, 0)
                    e[1] += energy(self.buf, nf, 1)
            els = e
            if max(e) > NOISE_FLOOR:
                break
        self.ch = 0 if els[0] >= els[1] else 1
        self.ok = max(els) > NOISE_FLOOR
        self.stereo = min(els) > NOISE_FLOOR
        if self.stereo:
            print("# 声道能量 L=%d R=%d -> 双麦立体声（两颗都在）" % (els[0], els[1]))
        elif self.ok:
            print("# 声道能量 L=%d R=%d -> 只有%s声道有信号（单麦）"
                  % (els[0], els[1], "左" if self.ch == 0 else "右"))
        else:
            print("# 声道能量 L=%d R=%d -> 无信号（查 SD/SCK/WS/VDD/GND）"
                  % (els[0], els[1]))

    def read_block(self):
        """把一块音频读进 self.pcm，返回样本数 n（0 = 没读到）。

        注意：这里**不分配任何新对象**。热路径上一旦频繁分配，
        自动 GC 的停顿会让 DMA 环溢出丢样本（这正是旧方案里不容易察觉的丢帧来源）。
        """
        if self.tone:
            gap = time.ticks_diff(self.tone_due, time.ticks_ms())
            if gap > 0:
                time.sleep_ms(gap)
            self.tone_due = time.ticks_add(self.tone_due, READ_FRAMES * 1000 // RATE)
            t = self.tone
            p = self.tone_pos
            n = READ_FRAMES * 2
            tmp = self.tmp
            avail = len(t) - p
            if n <= avail:
                tmp[:n] = t[p:p + n]
                self.tone_pos = p + n
            else:                      # 表长正好 1 秒 = 整数个周期，跨过去相位仍然连续
                tmp[:avail] = t[p:]
                tmp[avail:n] = t[:n - avail]
                self.tone_pos = n - avail
            stereo_mix(tmp, self.pcm, READ_FRAMES)   # 同一声给左右两路
            self.frames += READ_FRAMES
            if self.frames == READ_FRAMES:
                self.t_start = time.ticks_ms()
            return READ_FRAMES

        n = self.i2s.readinto(self.buf)
        if not n:
            return 0
        nf = n // 8
        conv2(self.buf, self.pcm, nf)
        self.frames += nf
        if self.frames == nf:
            self.t_start = time.ticks_ms()
        return nf

    def frame_tx(self, nf):
        """把当前块包成 chunked 分片；块长正常时完全复用预分配缓冲。"""
        n = nf * 4                      # 立体声：2 声道 × 2 字节
        if n == READ_FRAMES * 4:
            self.tx[self.hl:self.hl + n] = self.pcm
            return self.tx
        hdr = b"%x\r\n" % n
        out = bytearray(len(hdr) + n + 2)
        out[:len(hdr)] = hdr
        out[len(hdr):len(hdr) + n] = self.pcm[:n]
        out[-2:] = b"\r\n"
        return out

    def stats(self):
        """左右两路各自的峰值/RMS：只在没人监听时算，省下推流时的 CPU。"""
        n = READ_FRAMES
        pcm = self.pcm
        for c in (0, 1):
            pk = 0
            ss = 0
            for i in range(0, n * 4, 4):
                o = i + c * 2
                v = pcm[o] | (pcm[o + 1] << 8)
                if v & 0x8000:
                    v -= 0x10000
                a = v if v >= 0 else -v
                if a > pk:
                    pk = a
                ss += v * v
            self.peak[c] = pk
            self.rms[c] = int((ss / n) ** 0.5)

    def sps(self):
        dt = time.ticks_diff(time.ticks_ms(), self.t_start) / 1000.0
        return int(self.frames / dt) if dt > 0 else 0


# ================= WiFi =================
def wifi_up():
    # 沿用板 A 的老约定：主机名 esp32a，浏览器/路由器支持下可直接 http://esp32a.local
    try:
        network.hostname("esp32a")
    except Exception as e:
        print("# hostname 设置失败:", e)
    w = network.WLAN(network.STA_IF)
    w.active(True)
    if not w.isconnected():
        print("# WiFi 连接 %s ..." % WIFI_SSID)
        w.connect(WIFI_SSID, WIFI_PASS)
        for _ in range(80):
            if w.isconnected():
                break
            time.sleep_ms(250)
    if not w.isconnected():
        print("# WiFi 失败，起热点 ESP32-MIC / 12345678")
        ap = network.WLAN(network.AP_IF)
        ap.active(True)
        ap.config(essid="ESP32-MIC", password="12345678")
        return ap.ifconfig()[0]
    try:
        w.config(pm=0)          # 关 modem sleep：否则包会被攒成 100ms 一批
    except Exception as e:
        print("# 关省电失败:", e)
    return w.ifconfig()[0]


# ================= HTTP =================
_page = None


def get_page():
    global _page
    if _page is None:
        try:
            with open("mic.html") as f:
                _page = f.read()
        except Exception as e:
            _page = "<h1>mic.html 没上传</h1><pre>%s</pre>" % e
    return _page


def wav_header(n_samples, rate=RATE, channels=2):
    n = n_samples * 2 * channels
    return (b"RIFF" + struct.pack("<I", 36 + n) + b"WAVE"
            + b"fmt " + struct.pack("<IHHIIHH", 16, 1, channels, rate,
                                    rate * 2 * channels, 2 * channels, 16)
            + b"data" + struct.pack("<I", n))


def send_body(conn, ctype, body):
    if not isinstance(body, bytes):
        body = body.encode()
    conn.send(b"HTTP/1.1 200 OK\r\nContent-Type: " + ctype +
              b"\r\nCache-Control: no-store\r\nContent-Length: " +
              str(len(body)).encode() + b"\r\nConnection: close\r\n\r\n" + body)


def serve_rec(conn, mic, path):
    sec = 3
    if "sec=" in path:
        try:
            sec = int(path.split("sec=")[1].split("&")[0])
        except Exception:
            sec = 3
    sec = max(1, min(sec, 30))
    total = RATE * sec
    head = wav_header(total)
    conn.send(b"HTTP/1.1 200 OK\r\nContent-Type: audio/wav\r\n"
              b"Cache-Control: no-store\r\nContent-Length: " +
              str(len(head) + total * 4).encode() +
              b"\r\nConnection: close\r\n\r\n" + head)
    done = 0
    while done < total:
        nf = mic.read_block()
        if not nf:
            continue
        n = nf * 4
        left = (total - done) * 4
        if n > left:
            n = left
        conn.send(memoryview(mic.pcm)[:n])
        done += n // 4
    print("# rec.wav %ds 完成" % sec)


def read_request(conn):
    conn.settimeout(1.0)
    try:
        req = conn.recv(1024)
    except Exception:
        return None
    if not req:
        return None
    try:
        return req.split(b"\r\n", 1)[0].split(b" ")[1].decode()
    except Exception:
        return "/"


def start_stream(conn, mic):
    # 响应头用阻塞方式发（几百字节，一定会一次发完），之后**立刻转非阻塞**。
    # 关键教训：阻塞 send 遇到"只连不读"的客户端（切到后台的标签页就会这样）
    # 会把整个采集线程卡住，I2S 的 DMA 环一满就丢样本，表现为"画面卡住、然后停"。
    # 现在宁可丢块，也绝不拖住采集。
    conn.settimeout(2.0)
    conn.send(b"HTTP/1.1 200 OK\r\nContent-Type: application/octet-stream\r\n"
              b"Cache-Control: no-store\r\nX-Rate: " + str(RATE).encode() +
              b"\r\nX-Channels: 2\r\n"
              b"Transfer-Encoding: chunked\r\nConnection: close\r\n\r\n")
    # 发送策略（试了四轮才定下来，过程记在 README 里）：
    #   * 纯非阻塞、每轮只 send 一次：lwIP 的 TCP 发送缓冲只有约 2.9KB（2×MSS），
    #     而一块 4KB，一次根本塞不下 -> 实测只有 139/192 KB/s
    #   * 阻塞发送：能跑满线速，但"只连不读"的客户端会把采集线程拖死（实测掉到 1019 帧/秒）
    # 最终方案：**非阻塞 + 每轮最多 8 次小步写、每次先 poll(0) 且绝不等待**。
    # 一轮里把缓冲当前能装下的都写掉，装不下就留给下一轮（21ms 后缓冲又空了）。
    # 这样既不会超过缓冲上限，也绝不会阻塞采集。
    conn.setblocking(False)
    print("# 开始推流")
    # [socket, 自有发送缓冲, 待发 memoryview, 已发字节, 连续卡住轮数, poll]
    po = select.poll()
    po.register(conn, select.POLLOUT)
    return [conn, bytearray(len(mic.tx)), None, 0, 0, po]


# ================= 入口 =================


def main(tone_hz=0):
    ip = wifi_up()
    mic = Mic(tone_hz)
    time.sleep_ms(300)          # 等射频稳定，避开上电瞬间的电压跌落
    mic.detect()

    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", HTTP_PORT))
    srv.listen(4)
    srv.setblocking(False)
    poll = select.poll()
    poll.register(srv, select.POLLIN)

    print("# ==============================")
    print("#  浏览器打开: http://%s/" % ip)
    print("#  接线 SCK=%d WS=%d SD=%d ；L/R 接 GND 或 3V3 都行（自动识别）"
          % (PIN_SCK, PIN_WS, PIN_SD))
    print("# ==============================")

    # 每个听众一条： [socket, 自己的发送缓冲, 待发 memoryview, 已发字节, 连续 EAGAIN]
    # 用"每人一份缓冲"是为了互不干扰：慢的那个发不完，不会把别人的数据覆盖掉。
    streams = []
    t_log = time.ticks_ms()
    n_log = 0
    t_detect = time.ticks_ms()
    while True:
        # ---- 先接连接（非阻塞查询）----
        if poll.poll(0):
            conn = None
            try:
                conn, addr = srv.accept()
            except OSError as e:
                print("# accept:", e)
            if conn:
                path = read_request(conn)
                if path is None:
                    conn.close()
                elif path.startswith("/stream"):
                    streams.append(start_stream(conn, mic))
                    print("# 当前听众 %d 个" % len(streams))
                else:
                    try:
                        if path.startswith("/rec"):
                            serve_rec(conn, mic, path)
                        elif path.startswith("/status"):
                            send_body(conn, b"application/json",
                                      '{"ok":%d,"stereo":%d,"ch":%d,"sps":%d,'
                                      '"frames":%d,"lpk":%d,"rpk":%d,"lrms":%d,'
                                      '"rrms":%d,"rate":%d,"ip":"%s","free":%d}'
                                      % (1 if mic.ok else 0, 1 if mic.stereo else 0,
                                         mic.ch, mic.sps(), mic.frames,
                                         mic.peak[0], mic.peak[1],
                                         mic.rms[0], mic.rms[1], RATE, ip,
                                         gc.mem_free()))
                        elif path.startswith("/favicon"):
                            conn.send(b"HTTP/1.1 204 No Content\r\n"
                                      b"Connection: close\r\n\r\n")
                        else:
                            send_body(conn, b"text/html; charset=utf-8", get_page())
                    except Exception as e:
                        print("# 请求出错:", e)
                    finally:
                        try:
                            conn.close()
                        except Exception:
                            pass

        # ---- 采集一块（真正的节拍源）----
        try:
            nf = mic.read_block()
        except Exception as e:
            print("# I2S 读失败:", e)
            time.sleep_ms(100)
            continue

        # ---- 有听众就转发；没人听就只算一下电平给 /status ----
        if nf:
            if not streams:
                mic.stats()
            else:
                chunk = mic.frame_tx(nf)          # 每块只打包一次，各客户端各拷一份
                n = len(chunk)
                dead = None
                for i in range(len(streams)):
                    c = streams[i]
                    if c[2] is None:              # 上一块发完了，才拷新的一块
                        c[1][:n] = chunk
                        c[2] = memoryview(c[1])[:n]
                        c[3] = 0
                    # 注意：lwIP 的发送缓冲通常只有几 KB，而一块就有 4KB，
                    # 所以 EAGAIN 很常见。一次试探失败就等到下一个音频块（21ms）再补，
                    # 等于每挑一次就丢一整块（实测只能跑到 139/192 KB/s）。
                    # 这里当场小步重试，最多 8 次 × 2ms。
                    # 一轮最多 8 次小步写，写不进去立刻停，绝不等待
                    for _ in range(8):
                        if c[2] is None or not c[5].poll(0):
                            break
                        try:
                            c[3] += c[0].send(c[2][c[3]:])
                        except OSError as e:
                            code = e.args[0] if e.args else 0
                            if code in (11, 115, 110):
                                break
                            print("# 听众断开:", e)
                            dead = c
                            break
                        if c[3] >= n:
                            c[2] = None
                    if dead is not None:
                        break
                    if c[2] is None:
                        c[4] = 0
                    else:
                        c[4] += 1
                        if c[4] > 100:               # 连续 100 轮(约 2 秒)都不通 -> 断开
                            print("# 有听众跟不上，断开它")
                            dead = c
                            break
                if dead is not None:
                    try:
                        dead[0].close()
                    except Exception:
                        pass
                    streams.remove(dead)

        # ---- 没信号就每 3 秒快速重测一次：支持开电后再接线 ----
        if not mic.ok and not mic.tone:
            if time.ticks_diff(time.ticks_ms(), t_detect) > 3000:
                t_detect = time.ticks_ms()
                try:
                    mic.detect(ms=60, tries=1)
                except Exception as e:
                    print("# 重测失败:", e)

        # ---- 每 5 秒报一次实况 ----
        if time.ticks_diff(time.ticks_ms(), t_log) > 5000:
            dt = time.ticks_diff(time.ticks_ms(), t_log) / 1000.0
            got = mic.frames - n_log
            print("# %.1fs %d 帧 (%.0f 帧/s) L:pk=%d rms=%d  R:pk=%d rms=%d %s"
                  % (dt, got, got / dt, mic.peak[0], mic.rms[0],
                     mic.peak[1], mic.rms[1],
                     ("推流中(%d)" % len(streams)) if streams else "待机"))
            n_log = mic.frames
            t_log = time.ticks_ms()
            if not streams:
                gc.collect()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("# 停止")
