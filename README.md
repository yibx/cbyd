
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