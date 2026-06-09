#  船舶检测(Ship Motion Attitude Detection System)

## 功能

实时检测当前船的六个坐标值，x,y,z的差值，x,y,z方向的偏移角度差值。

将获取到的数据，存入数据库中，并且推送mqtt，供前端展示。

## 基本原理

可以将点云数据理解成一帧图像，计算两帧图像直接的旋转和平移，可以简单抽象成一个刚体运动，空间中的一帧图像，经过旋转和平移，得到另一帧图像。

得到旋转矩阵和平移矩阵，就可以得到两帧图像之间的相对位姿。

## 开源库

pcl

libhv：实现了mqtt协议，作为mqtt的客户端，推送数据。

mysqlclient：实现写mysql数据库。

## 编译

```bash
cd build 
cmake ..
make -j4
```

## 运行 
```bash
./smads
```
运行后，请检测参数是否正确。
```bash
2024-09-23 17:30:37 - Software Version: 1.0.0
2024-09-23 17:30:37 - Build Date and Time: Sep 12 2024 10:21:17
2024-09-23 17:30:37 - lidar dst_port: 2368, src_port: 2369, dst_ip: 192.168.1.102
2024-09-23 17:30:37 - mysql user: df_dk_yyladmin, pwd: EwdRSFTZ2zrRYcny, ip: 106.15.238.108, port: 3306, db: df_dk_yyladmin
2024-09-23 17:30:37 - mqtt ip: 47.103.101.18, port: 1883, user: admin, pwd: public, topic: D22/6FS
2024-09-23 17:30:37 - savecloud: 0
2024-09-23 17:30:37 - rectangle min_h: -8.500000, max_h: 11.000000rectangle min_v: -5.900000, max_v: 12.100000
2024-09-23 17:30:37 - pcl: max_correspondence_distance:0.100000, max_iterations: 100, euclidean_fitness_epsilon: 0.050000
```
## 配置文件

config.yaml

```bash
# 雷达参数
lidar:
  dst_port: 2368
  src_port: 2369
  dst_ip: 192.168.1.102

# mysql参数
mysql:
  user: df_dk_yyladmin
  pwd: EwdRSFTZ2zrRYcny
  ip: 106.15.238.108
  port: 3306
  db: df_dk_yyladmin

# mqtt参数
mqtt:
  ip: 47.103.101.18
  port: 1883
  user: admin
  pwd: public
  topic: D22/6FS

# 是否保存点云 0: 不保存 1: 保存
# 保存路径为执行目录中的pcd文件夹，默认为 build/pcd，用来进行测试和验证。
savecloud: 0

# 雷达点云参数，用于点云裁剪
rectangle:
  min_h: -8.5
  max_h: 11
  min_v: -5.9
  max_v: 12.1

# 点云配准参数
pcl:
  # 对于静止或缓慢移动的物体，可以设置较小的值，如0.05-0.1m。
  # 对于快速移动的物体，可能需要设置更大的值，如0.5-1m。
  # 通常从一个较大的值开始，然后逐步减小直到获得最佳结果。
  max_correspondence_distance: 0.1
  # 通常设置在 30 到 100 之间
  # 对于实时应用，可能需要设置较小的值，如 30-50
  # 对于离线处理且需要高精度的场景，可以设置更大的值，如 100-200
  max_iterations: 100
  # 通常设置在 0.001 到 0.1 之间
  # 对于需要高精度的应用，可以设置较小的值，如 0.001 或 0.005
  # 对于实时应用或对精度要求不是特别高的场景，可以设置较大的值，如 0.05 或 0.1
  euclidean_fitness_epsilon: 0.05

```

## 日志文件

build/logs 目录下，自动保存3天日志

# 雷达抓包
注意：必须同时抓2368和2369的数据，否则工具无法解析
tcpdump -i eno1 -vv udp port '(2368 or 2369)' -w lidar_2368_2369_1104.pcap


journalctl -u smads-server.service -f
sudo journalctl -u smads-server.service --since "2026-03-03 20:00:00" --until "2026-03-03 23:00:00"
journalctl -u smads-server.service --since "2026-03-03 20:00:00" --until "2026-03-03 23:00:00" > server.log

## 开机重启
```bash
# 重新加载 systemd 配置
sudo systemctl daemon-reload

# 启用开机自启动
sudo systemctl enable smads-server.service

# 立即启动服务
sudo systemctl start smads-server.service

# 查看状态
sudo systemctl status smads-server.service

# 查看日志
sudo journalctl -u smads-server.service -f

# 禁用开机自启动
sudo systemctl disable smads-server.service

# 可选：停止当前运行的服务
sudo systemctl stop smads-server.service

