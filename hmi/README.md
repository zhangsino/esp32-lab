# ESP32-S3-DevKitC-1 → 乐视手机 HMI 部署手册

> 板子：ESP32-S3-DevKitC-1（N16R8，16MB Flash / 8MB PSRAM）
> 手机：乐视 X520（Android 9 + Clash + 浏览器）
> 目标：手机浏览器当控制面板，Mac 同时用 UART 口烧录调试

---

## 零、板子接线确认

DevKitC-1 有两个 USB-C 口，**别插错**：

| 丝印 | 芯片 | 用途 | 引脚 |
|---|---|---|---|
| **UART** | CP2102N | Mac 烧录 + REPL | GPIO43(TX) / GPIO44(RX) |
| **USB** | S3 原生 USB | USB CDC / JTAG | **GPIO19 / GPIO20（专用，别挪用）** |

本项目两者都要：
- **Mac → UART 口**：烧固件、看 print
- **手机 → WiFi**：显示

传感器接线（沿用 `tilt.py`）：

```
MPU6050   SDA → GPIO17
          SCL → GPIO18
          VCC → 3V3
          GND → GND
SSD1306   （同上 I2C 总线，地址 0x3C）
```

---

## 一、固件（如果还没刷）

需要 **带 SPIRAM 的 ESP32-S3 固件**（N16R8 必须选 8MB PSRAM 版本），否则起不来。

从 micropython.org 下载 `ESP32_GENERIC_S3-SPIRAM_OCT-*.bin`，然后：

```bash
# Mac 上，UART 口
esptool.py --chip esp32s3 -p /dev/cu.usbserial-* erase_flash
esptool.py --chip esp32s3 -p /dev/cu.usbserial-* --baud 460800 \
  write_flash -z 0 firmware.bin
```

---

## 二、上传三个文件

**Thonny** → 右下角选对串口 → 菜单 `视图 / View` → `文件 / Files` 打开双栏，
把这三个文件拖到板子根目录：

```
secrets.py     ← 先改里面的 WiFi 名和密码！
main.py
index.html
```

⚠️ `main.py` 必须叫这个名字，MicroPython 开机自动执行它。

---

## 三、填 WiFi

打开 `secrets.py`，改两处：

```python
WIFI_SSID = "HUAWEI-805"        # 你的 WiFi 名（2.4G）
WIFI_PASS = "你的密码"
STATIC_IP = None                 # 想固定地址就填，见下
```

**建议填固定 IP**，省得每次去路由器里找板子：

```python
# 家里网段是 192.168.101.x，就挑一个没人用的
STATIC_IP = ("192.168.101.60", "255.255.255.0", "192.168.101.1", "192.168.101.1")
#           板子自己的IP      子网掩码          网关                DNS
```

---

## 四、复位，看串口

按板上 **RST** 键，Thonny 的 Shell 里会打印：

```
# WiFi 连接中: HUAWEI-805
# WiFi OK  IP = 192.168.101.60
# 标定完成 n=200 pitch0=0.12 roll0=-0.35
# ==============================
#  手机浏览器打开: http://192.168.101.60
# ==============================
```

**这一步就能判断成败**：有 IP 说明 WiFi 通了；`标定完成` 说明 MPU6050 认到了。
如果显示 `IMU 初始化失败`，先不用管——网页照样能开（`/api` 会返回 `ok:0`）。

---

## 五、手机打开

1. 手机连**同一个 WiFi**（`HUAWEI-805`）
2. 浏览器输入 `http://192.168.101.60`
3. 看到仪表盘 → 成功

**建议**：
- 加到书签，或者用「添加到主屏幕」
- 浏览器菜单里选「全屏 / 全屏模式」
- 手机设置里把息屏时间调长，或者直接：

```bash
adb shell settings put system screen_off_timeout 2147483647
adb shell svc power stayon true
# 用完恢复：settings put system screen_off_timeout 120000
```

---

## 六、验证顺序（别一次全上）

| 步骤 | 验证什么 | 怎么判断 |
|---|---|---|
| 1 | WiFi 通不通 | 串口打印出 IP |
| 2 | 页面能不能开 | 手机显示仪表盘（气泡在中间） |
| 3 | 数据动没动 | 倾斜板子，气泡跟着走、数字变化 |
| 4 | 声音 | 点「开声音」，倾斜超过 25° 会「嘀」一声 |
| 5 | 下行指令 | 输入框打字回车 → 串口打印 `# CMD: xxx` |

---

## 七、接口说明（给后续扩展用）

| 路径 | 方法 | 说明 |
|---|---|---|
| `/` | GET | 控制面板页面 |
| `/api` | GET | `{"p":pitch,"r":roll,"t":ms,"n":seq,"e":err,"ok":1,"c":cmd}` |
| `/cmd?x=xxx` | GET | 下行指令，板端存到 `state["cmd"]` 并打印 |

**要接舵机时**，在 `main.py` 里改两处：

1. `/cmd` 的分支里，把 `state["cmd"]` 换成真正的动作（设置目标角度、启停 PID）
2. `imu_loop()` 里加 PID 计算 → 输出 PWM

⚠️ **PID 必须跑在板端**，手机只负责显示和下发参数。WiFi 有 10~50ms 抖动，闭环放在手机上会振荡。

---

## 八、常见问题

**页面打不开**
- 手机和板子是不是同一个 WiFi？（ESP32-S3 只支持 2.4G，别连 5G）
- 串口打印的 IP 和浏览器里输的一致吗？
- 路由器有没有开「AP 隔离」？开了的话设备之间互相访问不了

**页面打开了但气泡不动**
- 串口看有没有 `IMU 初始化失败`
- 用 `tilt.py` 单独验证一次 MPU6050 接线

**改了 index.html 但页面没变**
- `main.py` 缓存了页面内容，**要按 RST 重启**才生效

**连接经常断开**
- 路由器信号弱；把板子靠近一点
- 或者页面刷新间隔从 200ms 改成 500ms（`setInterval(poll, 200)`）

---

## 九、文件说明

| 文件 | 作用 |
|---|---|
| `secrets.py` | WiFi 账号密码、可选固定 IP（**别提交到公开仓库**） |
| `main.py` | 板端：WiFi 连接 + IMU 采集 + HTTP 服务 |
| `index.html` | 手机端页面：仪表盘 + 按钮 + 声音 |

页面用的是**原生 JS + Canvas + XHR**，没有任何框架——因为 Android 9 的 WebView 停在 2019 年，跑不动现代前端。
