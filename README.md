
## 源码编译

### 依赖库安装

flann 库比阿姨报错处理：

报错：

CMake Error at /src/cpp/CMakeLists.txt:33 (add_library):
    No SOURCES given to target: flann_cpp

CMake Error at /src/cpp/CMakeLists.txt:91 (add_library):
    No SOURCES given to target: flann

解决方案：

cd flann
touch src/cpp/empty.cpp

gedit src/cpp/CMakeLists.txt

33行修改
add_library(flann_cpp SHARED " ")
add_library(flann_cpp SHARED empty.cpp)

91行修改
add_library(flann SHARED " ")
add_library(flann SHARED empty.cpp)

清空build文件夹再次编译


### yaml-cpp

mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DYAML_BUILD_SHARED_LIBS=ON

### libhv

./configure --with-mqtt --with-openssl && make clean && make

### cbyd

1、代码结构

-include
--common_base.h    
--data_outputter.h  
--industrial_mqtt_client.h  
--lock_free_queue.h  
--LSSDK                   
--point_cloud_fuser.h   
--system_state_manager.h
--config_loader.h  
--easylogging++.h   
--lidar_bg_diff.h           
--Logger.h           
--point_cloud_acquirer.h  
--six_dof_calculator.h  
--wharf_dock_checker.h

-src
--config_loader.cpp   
--easylogging++.cc            
--lidar_bg_diff.cpp  
--main.cpp                  
--point_cloud_fuser.cpp   
--system_state_manager.cpp
--data_outputter.cpp  
--industrial_mqtt_client.cpp  
--Logger.cpp         
--point_cloud_acquirer.cpp  
--six_dof_calculator.cpp  
--wharf_dock_checker.cpp

2、编译

mkdir build && cd build
cmake ..
make -j8


## LidarBgDiff 船舶背景差分监测模块全参数说明和调参指南

### 一、YAML 全局算法参数总览
```yaml
# 背景差分邻域距离阈值（米）
dist_threshold: 0.15
# 欧式聚类邻域半径（米）
cluster_eps: 0.3
# 船舶最小有效聚类点数
cluster_min_points: 20
# 判定停稳需要连续稳定帧数
stable_frame_count: 12
# 船舶中心位移阈值（米），小于该值判定静止
stable_dist_threshold: 0.5
# 船舶最小有效点云总数，低于则认为无船
min_valid_ship_points: 150
```
### 二、参数含义、影响、调整方案

1. dist_threshold 背景差分距离阈值（m）
含义
背景点云构建 KDTree，对当前帧每个降采样点搜索最近背景点；
两点空间距离大于该阈值则判定为前景（船舶），小于则判定为码头静态背景直接剔除。
内部提前计算距离平方对比，无 sqrt 开销。
配套体素降采样固定 0.02m。
业务影响
值过小：大量船体点被误判为背景，船体残缺、识别不到船；
值过大：码头栏杆、浮标、墙体噪点全部保留，前景混入大量静态干扰，聚类后出现虚假船舶。
调参策略
近距毫米波 / 激光雷达（0.01~0.03m 精度）：0.10 ~ 0.18
远距离码头雷达、远距离扫描：0.20 ~ 0.30
雨天 / 雾天噪点多：适当加大；船体与码头间隙极小（贴岸停靠）：适当缩小。

2. cluster_eps 欧式聚类半径（m）
含义
前景点云做欧式连通聚类：两点间距小于该值视为同一物体；
算法只保留最大一簇作为船舶，其余小簇全部滤除。
业务影响
过小：船体被拆分成多个小簇，最大簇仅一小块船体，包围盒失真；
过大：远处浮漂、岸边杂物与船体粘连合并，包围盒偏大、中心偏移。
调参策略
小型作业船、近距离雷达：0.2 ~ 0.3；
大型货船、远距离雷达点云稀疏：0.35 ~ 0.6；
水面漂浮杂物多：适当减小，分离船体与漂浮噪点。

3. cluster_min_points 单簇最小点数
含义
欧式聚类时，点数少于该值的簇直接丢弃，过滤微小噪点簇。
业务影响
数值太大：远处小船、船体局部点云稀疏时直接被过滤，识别丢失；
数值太小：水花、水面噪点形成微小簇，产生虚假船舶。
调参策略
近距离高密度点云：15 ~ 30；
远距离稀疏点云：8 ~ 15；
水面浪花、水花严重场景适当加大。

4. min_valid_ship_points 船舶全局最小有效点数
含义
聚类输出完整船体点云总点数，低于该值直接判定 SHIP_LEAVE 无船舶。
业务影响
数值偏大：远处刚驶入泊位的船舶点数不足，延迟检测；
数值偏小：残留噪点小簇误判为船舶，持续输出虚假船位。
调参策略
雷达近距离覆盖泊位：120 ~ 200；
远距离、斜向扫描点云稀疏：60 ~ 120。

5. stable_frame_count 停稳判定连续帧数
含义
滑动窗口记录船舶中心坐标，只有历史缓存帧数达到该数值，才会计算位移判断是否静止；
窗口超出长度自动丢弃最早帧。
业务影响
数值大：停稳判定延迟，船舶停稳后需要多等若干帧才输出稳定状态；抗抖动更强；
数值小：极易受雷达点云抖动、水面波动干扰，频繁在「移动 / 稳定」来回跳变。
调参策略
对停稳实时性要求高：6 ~ 10；
港口风浪大、点云抖动剧烈：12 ~ 20。

6. stable_dist_threshold 船舶中心位移阈值（m）
含义
滑动窗口内当前帧船舶中心与窗口第一帧中心三维欧式距离；
距离小于该阈值判定 SHIP_STABLE 船舶停稳，否则持续移动。
业务影响
阈值大：轻微晃动、水流漂移不会判定移动，容易把缓慢漂移的船误判为停稳；
阈值小：微小点云抖动就判定船舶移动，稳定状态频繁跳变。
调参策略
要求严格判定船舶完全停靠锁死：0.2 ~ 0.4；
允许微小水流漂移、对稳定宽容度高：0.5 ~ 0.8。

### 三、内部固定硬编码参数（无 yaml 配置，如需修改改源码）

VOXEL_RES = 0.02f 体素降采样分辨率 (m)
作用：背景差分前对点云统一降采样，减少 KDTree 遍历耗时，提升实时性。
场景点云密集、算力充足：缩小至 0.01；
点云稀疏、追求低延迟：放大至 0.03~0.05。

### 四、完整执行链路 + 参数联动逻辑

原始点云 → 体素降采样 (VOXEL_RES)
背景 KDTree 差分，使用 dist_threshold 分割前景 / 背景
前景点云欧式聚类：cluster_eps 连通半径 + cluster_min_points 过滤小簇，保留最大船体
船体总点数校验：低于 min_valid_ship_points → 直接判定船舶离开
船体包围盒中心存入滑动历史队列
队列帧数达到 stable_frame_count 才计算位移
中心位移小于 stable_dist_threshold → 船舶停稳，否则移动

### 五、分场景成套推荐参数

场景 1：近距离激光雷达，泊位内船体点云密集，风浪小
```yaml
dist_threshold: 0.12
cluster_eps: 0.25
cluster_min_points: 15
stable_frame_count: 10
stable_dist_threshold: 0.3
min_valid_ship_points: 120
```

场景 2：远距离岸基雷达，点云稀疏，水面浪花多（当前默认方案）
```yaml
dist_threshold: 0.15
cluster_eps: 0.3
cluster_min_points: 20
stable_frame_count: 12
stable_dist_threshold: 0.5
min_valid_ship_points: 150
```

场景 3：风浪大、水面持续波动，抗抖动优先
```yaml
dist_threshold: 0.18
cluster_eps: 0.35
cluster_min_points: 25
stable_frame_count: 18
stable_dist_threshold: 0.6
min_valid_ship_points: 180
```

### 六、故障排查调参速查

检测不到船舶、船体残缺
增大 dist_threshold；减小 cluster_min_points；减小 min_valid_ship_points
大量虚假船舶、噪点被识别成船
减小 dist_threshold；增大 cluster_min_points、min_valid_ship_points
船舶明明停稳，状态持续显示移动
增大 stable_dist_threshold；适当增大 stable_frame_count
船舶轻微漂移就判定停稳
缩小 stable_dist_threshold
状态切换延迟很高，船停稳很久才输出稳定
减小 stable_frame_count

## 船舶点云 NDT 粗配准 + GICP 精配准 YAML 参数完整说明

适用场景：岸基雷达 / 双目相机采集船舶模型点云、船载多传感器点云融合、船模水池 6DoF 定位、双雷达点云拼接；NDT 粗匹配消除大位移 / 大角度偏差，GICP 精细收敛得到毫米级位姿。

一、NDT 粗配准模块（ndt）
作用：对点云分体素正态分布建模，鲁棒性强，适合初始位姿偏差大、点云残缺、海面噪点多、远距离船舶点云，输出粗略变换矩阵给 GICP 做初值。
```yaml
ndt:
  voxel:
    x: 0.1
    y: 0.1
    z: 0.1
  max_iter: 30
  trans_eps: 1e-6
  step_size: 0.1
  resolution: 0.5
  fit_thresh: 1.5
```
1. voxel x/y/z：NDT 体素网格尺寸，单位 m
当前值：0.1m 立方体网格
作用：把点云切分成三维格子，每个格子统计点云均值、协方差构建正态分布；网格越大抗噪越强、速度越快，精度下降；网格越小细节匹配越好，易受海面浪花、杂点干扰。
修改规则 & 场景：
1）远距离船舶、海面噪点多、点云稀疏 → 调大至 0.15~0.2m，抑制浪花噪点；
2）近距离船模、船体轮廓清晰、噪点少 → 缩小至 0.05~0.08m，提升粗匹配精度；
3）网格过大（>0.25）：船体细节丢失，容易匹配错位；过小（<0.05）：杂点造成匹配发散。

2. max_iter：NDT 迭代最大次数
当前 30 次。
作用：迭代优化刚体变换，达到收敛条件提前退出，否则跑完最大次数停止。
修改：
船舶初始偏差极大（多雷达跨帧、船舶大幅摆动）→ 加大到 40~50；
RTK 提供精准初值、偏差很小 → 减小至 15~20，提速；
数值过高增加耗时，过低容易没收敛就终止，粗匹配失效。

3. trans_eps：平移收敛阈值 1e-6 m
作用：连续两次迭代平移变化量小于该值，判定平移收敛，停止迭代。
船舶场景不用频繁修改：1e-6~1e-5 区间通用；放宽到 1e-4 可提速，但粗匹配精度下降。

4. step_size：线搜索步长
当前 0.1
作用：每次迭代沿下降方向的更新步长，控制收敛速度。
修改：
船舶大位移场景调大 0.15~0.2，快速跳出局部最优；
船体精细轮廓、初值较好调小 0.05，收敛更平稳。

5. resolution：NDT 分辨率（核心滤波阈值）0.5m
作用：过滤距离超过该值的对应点对，剔除跨船体、海面无关匹配点。
场景修改：
大船、远距离点云 → 增大 0.6~0.8；
小型船模、近距离双目点云 → 缩小 0.3~0.4；
数值过大容易把海面噪点和船体匹配，过小丢失有效对应点。

6. fit_thresh：匹配代价阈值 1.5
作用：迭代代价函数低于阈值判定匹配有效，高于则认为配准失败。
修改：
海面浪花多、点云残缺 → 提高至 2.0~2.5，容忍更高匹配误差；
干净船模点云、无杂点 → 降低至 1.0，过滤劣质匹配结果；
阈值过高会保留错误错位变换，过低容易直接判定匹配失败。

二、GICP 精配准模块（gicp）

作用：基于广义迭代最近点，同时约束点云法向 + 距离，收敛精度远高于 NDT；接收 NDT 粗变换初值，输出高精度 6DoF 位姿，适合船体轮廓精细对齐、毫米级定位、多帧点云融合。
```yaml
gicp:
  voxel:
    x: 0.05
    y: 0.05
    z: 0.05
  use_reciprocal: true
  max_corr_dist: 0.3
  max_iter: 150
  trans_eps: 1e-6
  euclid_eps: 1e-6
  rot_eps: 1e-3
  rand_num: 15
  fit_thresh: 1.0
  score_radius: 0.5
```
1. voxel x/y/z：GICP 下采样体素 0.05m
作用：精匹配前对点云降采样，平衡精度与计算耗时，比 NDT 体素更小，保留船体细节（甲板、舷边、船首轮廓）。
修改场景：
1）实时嵌入式（Jetson Orin）、多船并行配准 → 放大至 0.08~0.1m，降低计算量；
2）离线高精度船模重建、标定场景 → 缩小至 0.02~0.03m，极致精度；
注意：不可大于 NDT 体素，否则细节丢失，精匹配无法修正粗匹配误差。

2. use_reciprocal: true 互斥匹配开关
作用：开启双向对应点查找（源点找目标 + 目标反向找源点），大幅减少海面杂点、船体不对称结构造成的错误匹配对；船舶场景建议永远 true。
仅当点云完全单面缺失、严重遮挡时临时关闭 false，一般不修改。

3. max_corr_dist：最大匹配点对距离阈值 0.3m
作用：两点距离超过该值则舍弃，剔除船体与海面、远距离无关点匹配。
场景修改：
大型船舶、甲板跨度大 → 0.4~0.5m；
小型船模、近距离双目点云 → 0.15~0.2m；
数值过大容易引入浪花噪点，过小船体边缘点丢失，匹配漂移。

4. max_iter：精配准最大迭代次数 150
GICP 收敛慢于 NDT，默认给较高迭代上限。
修改：
初值精准（NDT 匹配效果好 + RTK 辅助）→ 80~100，提速；
船体曲面复杂、遮挡严重、点云噪声大 → 200~300，保证充分收敛；
嵌入式实时场景严格压低迭代次数，避免帧率下跌。

5. trans_eps /euclid_eps：平移收敛阈值 1e-6 m
trans_eps：变换平移增量阈值；euclid_eps：点对欧氏距离收敛阈值。
双阈值联合判定平移收敛，船舶高精度定位场景保持 1e-6 不变；
纯可视化、低精度跟踪场景可放宽至 1e-4，减少迭代耗时。

6. rot_eps：旋转收敛阈值 1e-3 rad
作用：旋转角增量小于该值判定角度收敛。
高精度姿态测量（船舶俯仰横滚监测）保持 1e-3；
仅平移定位、不关注角度可放宽至 5e-3。

7. rand_num：随机采样点数 15
作用：每次迭代随机抽取 N 个点计算匹配代价，加速求解。
嵌入式算力弱 → 减小至 8~10；
离线高精度标定 → 增大至 20~30，匹配稳定性提升。

8. fit_thresh：精匹配代价阈值 1.0
低于阈值代表匹配成功，高于判定匹配发散。
海面杂点多、船体遮挡：提高 1.2~1.5；
干净实验室船模点云：降低 0.6~0.8，过滤低质量收敛结果。

9. score_radius：协方差计算邻域半径 0.5m
作用：在该半径内搜索邻域点计算点云法向、协方差矩阵，是 GICP 核心参数，决定曲面约束效果。
修改：
船体大面积平整甲板 → 放大 0.6~0.7，法向更稳定；
船首、栏杆、细小曲面结构 → 缩小 0.3~0.4，捕捉局部曲率；
半径过大平滑掉船体棱角，过小法向受单点噪点剧烈波动。

三、分场景成套调参方案（直接复制使用）
场景 1：水池小型船模、双目近距离采集、低噪、离线高精度标定
```yaml
# NDT粗配准
ndt:
  voxel: {x:0.05,y:0.05,z:0.05}
  max_iter: 40
  trans_eps: 1e-6
  step_size: 0.08
  resolution: 0.3
  fit_thresh: 2.0
# GICP精配准
gicp:
  voxel: {x:0.02,y:0.02,z:0.02}
  use_reciprocal: true
  max_corr_dist: 0.2
  max_iter: 250
  trans_eps: 1e-6
  euclid_eps: 1e-6
  rot_eps: 5e-4
  rand_num: 25
  fit_thresh: 0.7
  score_radius: 0.3
```
场景 2：岸基雷达远距离船舶、海面浪花噪点大、实时 Jetson 嵌入式运行
```yaml
# NDT粗配准（抗噪优先）
ndt:
  voxel: {x:0.18,y:0.18,z:0.18}
  max_iter: 20
  trans_eps: 1e-5
  step_size: 0.15
  resolution: 0.7
  fit_thresh: 2.5
# GICP精配准（轻量化提速）
gicp:
  voxel: {x:0.08,y:0.08,z:0.08}
  use_reciprocal: true
  max_corr_dist: 0.45
  max_iter: 80
  trans_eps: 1e-5
  euclid_eps: 1e-5
  rot_eps: 3e-3
  rand_num: 8
  fit_thresh: 1.4
  score_radius: 0.6
```
场景 3：双雷达跨帧融合、船舶大幅摆动、初始位姿偏差大
```yaml
ndt:
  voxel: {x:0.12,y:0.12,z:0.12}
  max_iter: 50
  step_size: 0.2
  resolution: 0.6
  fit_thresh: 2.2
gicp:
  max_corr_dist: 0.4
  max_iter: 200
  score_radius: 0.55
```

