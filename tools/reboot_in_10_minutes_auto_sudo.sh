#!/bin/bash

# 如果没有以 sudo 运行，则重新以 sudo 运行此脚本
if [ "$EUID" -ne 0 ]; then
    exec sudo "$0" "$@"
fi

# 设置 10 分钟后重启
shutdown -r +10

echo "System will reboot in 10 minutes."
echo "To cancel, run 'sudo shutdown -c'"

