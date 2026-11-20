# -*- coding: utf-8 -*-
"""
main.py —— 开机自启入口
========================
现在开机跑的是 INMP441 单麦人声采集 + WiFi 网页（micapp.py / mic.html）。

原来的 IMU 六轴网页服务已原样保存为 **main_imu.py**，想切回去：
    mpremote connect /dev/cu.xxx cp :main_imu.py :main.py
（或者把本文件改回 `import main_imu` 也行，注意 main_imu.py 里是直接跑 main loop 的模块级代码）
"""

import micapp

try:
    micapp.main()
except KeyboardInterrupt:
    print("# 已停止")
