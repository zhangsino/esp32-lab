#!/bin/bash
# 启动实时绘图：自动挑一个同时装了 pyserial + matplotlib 的 python3
set -u
cd "$(dirname "$0")/.." || exit 1

CANDIDATES=(
  "/opt/homebrew/Caskroom/miniconda/base/bin/python3"
  "/opt/homebrew/bin/python3"
  "/usr/bin/python3"
  "python3"
)

for py in "${CANDIDATES[@]}"; do
  if command -v "$py" >/dev/null 2>&1 || [ -x "$py" ]; then
    if "$py" -c "import serial, matplotlib" >/dev/null 2>&1; then
      exec "$py" esp32/live_plot.py "$@"
    fi
  fi
done

echo "没找到同时装了 pyserial 和 matplotlib 的 python3。"
echo "任选一条安装后再运行："
echo "  /opt/homebrew/Caskroom/miniconda/base/bin/python3 -m pip install pyserial matplotlib"
echo "  /opt/homebrew/bin/python3 -m pip install --user pyserial matplotlib"
exit 1
