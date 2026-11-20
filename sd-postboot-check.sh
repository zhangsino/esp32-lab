#!/bin/bash
# SD 读卡器 重启后取证脚本
# 用法：双击或在终端运行；结果同时打印并保存到本目录

out="$HOME/Documents/deepseek-harness/default-workspace/sd-postboot-$(date +%Y%m%d-%H%M%S).txt"

{
  echo "================ SD 读卡器重启后取证 ================"
  echo "时间: $(date)"
  echo "开机: $(uptime)"
  echo
  echo "--- 插槽状态 ---"
  ioreg -r -c AppleSDXCSlot -l -w0 2>/dev/null | grep -E \
    '"Card Present"|"ConnectionCount"|"Card Type"|"Specification Version"|"Speed Class"|"TRM_State"|"AuthorizationStatus" =|"Calculated Clock Frequency'
  echo
  echo "--- 控制器状态 ---"
  ioreg -r -c AppleSDXC -w0 2>/dev/null | grep -E '"SDXC Powered"|"CurrentPowerState"'
  echo
  echo "--- 磁盘 ---"
  diskutil list 2>&1
  echo
  echo "--- 卷 ---"
  ls -la /Volumes 2>&1
  echo
  echo "--- 日志库是否恢复 ---"
  if [ -d /var/db/diagnostics ]; then echo "OK: /var/db/diagnostics 存在"; else echo "缺失: /var/db/diagnostics 仍不存在"; fi
  log show --last 2m --style compact 2>&1 | tail -3
  echo "===================================================="
} 2>&1 | tee "$out"

echo
echo "结果已保存到: $out"
echo "判读要点：空槽时 Card Present 应为 No。"
