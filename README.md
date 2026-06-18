# LIO-LoopClosure（Scan Context 位姿图优化）

一个基于 ROS2 的纯 SLAM 后端功能包，专注于**回环检测**和**位姿图优化**。它与前端 LiDAR 里程计**松耦合**——前端独立运行里程计，LIO-LoopClosure 通过话题订阅前端数据进行后端全局校正。

## 概览

```
┌──────────────────────┐                                          ┌───────────────────────┐
│  前端 LiDAR 里程计   │ /aft_mapped_to_init (Odometry)           │                       │
│  (FAST-LIO,          │ ───────────────────────────────────────► │                       │
│   Point-LIO 等)      │ /velodyne_cloud_registered_local         │   LIO-LoopClosure     │
│                      │ ───────────────────────────────────────► │   (纯 SLAM 后端)     │
│                      │ /cloud_for_scancontext (可选)            │  ┌─────────────────┐  │
│                      │ ───────────────────────────────────────► │  │ Scan Context    │  │
│                      │                                          │  │ 回环检测        │  │
│                      │                                          │  └────────┬────────┘  │
│                      │                                          │           │           │
│                      │                                          │  ┌────────▼────────┐  │
│                      │                                          │  │ ICP 精化        │  │
│                      │                                          │  └────────┬────────┘  │
│                      │                                          │           │           │
│                      │                                          │  ┌────────▼────────┐  │
│                      │                                          │  │ iSAM2 位姿      │  │
│                      │                                          │  │ 图优化器        │  │
│                      │                                          │  └────────┬────────┘  │
│                      │                                          │           │           │
│                      │ /aft_pgo_odom, /aft_pgo_path             │  ┌────────▼────────┐  │
│                      │ ◄─────────────────────────────────────── │  │ 可视化 &        │  │
└──────────────────────┘                                          │  │ 地图保存        │  │
                                                                  └──┴─────────────────┴──┘
```

## 主要特点

- **Scan Context 回环检测** — 基于 [Scan Context](https://github.com/irapkaist/scancontext)（IROS 2018）的全局地点识别，对视角变化鲁棒的激光雷达全局描述子
- **ICP 引导的回环约束** — Scan Context 给出回环候选后，ICP 精化 6 自由度相对位姿变换，确保约束的准确性
- **iSAM2 位姿图优化** — 使用 GTSAM 的 iSAM2 增量式优化器，实时接入回环约束后全局校正轨迹
- **等间隔关键帧选取** — 按累积平移（`keyframe_meter_gap`）或累积旋转（`keyframe_deg_gap`）阈值新增关键帧
- **松耦合架构** — 独立于前端里程计，通过标准话题订阅数据，兼容 FAST-LIO、Point-LIO、A-LOAM 等任意 LiDAR 里程计系统
- **GPS 因子支持** — 可选 GPS 高度约束，用于长距离场景的高度漂移校正（x/y 噪声极大，仅校正 z 轴）
- **校正地图生成** — 结束时自动保存校正后地图（PGO 后）和原始地图（纯里程计）为 PCD 文件，也可通过 `/save_map` 服务按需保存
- **多线程流水线** — 独立线程处理位姿图构建、回环检测、ICP、iSAM2 优化、地图可视化和路径可视化
- **SIGSEGV 恢复机制** — 坏点云导致 ICP 段错误时，通过 `sigsetjmp`/`siglongjmp` 安全恢复

## 依赖

### ROS2 包
- `rclcpp`, `std_msgs`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`
- `tf2`, `tf2_ros`, `tf2_geometry_msgs`
- `pcl_conversions`, `PCL`（点云库）
- `std_srvs`

### 第三方库
- **GTSAM** — 因子图优化（iSAM2）
- **Boost** — timer, chrono, filesystem, system, thread, serialization
- **Eigen3** — 线性代数（常随 PCL / GTSAM 一起安装）
- **OpenMP** — 并行点云变换加速

### Python（离线地图拼接用）
- `pypcd`, `numpy`, `open3d`（详见 `utils/python/makeMergedMap.py`）

## 编译

```bash
# 克隆到 ROS2 工作空间
cd ~/ros2_ws/src
git clone <本仓库> lio_loopclosure

# 编译
cd ~/ros2_ws
colcon build --packages-select lio_loopclosure
source install/setup.bash
```

## 使用方式

### 订阅的话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/aft_mapped_to_init` | `nav_msgs/Odometry` | 前端里程计（`camera_init` 坐标系下的位置+姿态） |
| `/velodyne_cloud_registered_local` | `sensor_msgs/PointCloud2` | 前端注册后的全分辨率点云，用于关键帧存储 |
| `/cloud_for_scancontext` | `sensor_msgs/PointCloud2` | （可选）用于 Scan Context 生成的降采样点云；若未提供则回退使用全分辨率点云 |
| `/gps/fix` | `sensor_msgs/NavSatFix` | （可选，默认关闭）GPS 数据，用于高度校正 |

### 发布的话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/aft_pgo_odom` | `nav_msgs/Odometry` | 校正后的里程计（最新优化位姿） |
| `/aft_pgo_path` | `nav_msgs/Path` | 校正后的完整轨迹（所有关键帧优化位姿） |
| `/aft_pgo_map` | `sensor_msgs/PointCloud2` | 体素滤波后的全局点云地图（回环插入后更新） |
| `/loop_scan_local` | `sensor_msgs/PointCloud2` | 当前关键帧点云，用于 ICP 回环验证可视化 |
| `/loop_submap_local` | `sensor_msgs/PointCloud2` | 匹配的历史子图，用于 ICP 回环验证可视化 |
| TF `camera_init` → `/aft_pgo` | `tf2` | 优化后的位姿变换（带去重逻辑） |

### 服务

| 服务 | 类型 | 说明 |
|------|------|------|
| `/save_map` | `std_srvs/srv/Empty` | 按需保存校正地图和原始地图。LIO-LoopClosure 空闲时会打印提示，告知可以安全保存 |

### 启动文件

**FAST-LIO2 + Ouster64：**
```bash
ros2 launch lio_loopclosure fastlio_ouster64.launch.py
```

**Point-LIO + Mid-360：**
```bash
ros2 launch lio_loopclosure pointlio_mid360.launch.py
```

**启动参数：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `keyframe_meter_gap` | 0.5 | 触发新关键帧的最小累积平移（米） |
| `keyframe_deg_gap` | 10.0 | 触发新关键帧的最小累积旋转（度） |
| `sc_dist_thres` | 0.2 / 0.3 | Scan Context 余弦距离阈值，低于该值视为回环 |
| `sc_max_radius` | 80.0 / 40.0 | Scan Context 分格的最大有效距离（米） |
| `mapviz_filter_size` | 0.4 | 地图可视化体素滤波的叶子大小（米） |
| `save_directory` | `$HOME/.../data/` | 保存优化位姿和逐帧点云的目录 |
| `map_save_directory` | `$HOME/.../map/` | 保存 PCD 地图文件的目录 |
| `rviz_lio_loopclosure` | `true` / `false` | 是否自动启动 RViz2 可视化 |

### 生成地图的典型流程

1. 同时启动前端里程计（如 FAST-LIO2）和 LIO-LoopClosure
2. 操控机器人在环境中运行，确保经过回环区域
3. 等待 LIO-LoopClosure 打印空闲提示：
   ```
   [LIO-LoopClosure] idle: no new odom/cloud for 10.0 s, loop/ICP queue drained, graph optimized.
   Safe to call: ros2 service call /save_map std_srvs/srv/Empty
   ```
4. 调用保存服务：
   ```bash
   ros2 service call /save_map std_srvs/srv/Empty
   ```
5. 生成的文件：
   - `corrected_map.pcd` — 经 iSAM2 优化位姿变换后的校正地图
   - `corrected_map_dense.pcd` — 同上，但 0.1 米体素降采样（适合 CloudCompare 查看）
   - `uncorrected_map.pcd` — 原始里程计位姿变换的地图（供对比）
   - `optimized_poses.txt` — KITTI 格式的优化后位姿
   - `odom_poses.txt` — KITTI 格式的原始里程计位姿
   - `times.txt` — 各关键帧的时间戳
   - `Scans/` — 逐帧 PCD 点云目录

6. 离线地图拼接（带强度着色）：
   ```bash
   cd utils/python
   # 修改 makeMergedMap.py 中的 data_dir、扫描范围等
   python3 makeMergedMap.py
   ```

## 架构

系统运行 **8 个并发线程**：

| 线程 | 功能 | 频率 |
|------|------|------|
| **`process_pg`** | 位姿图构建：订阅里程计+点云，筛选关键帧，构建含里程计约束（及可选 GPS 因子）的因子图 | 事件驱动（~10 Hz） |
| **`process_lcd`** | 回环检测：定时运行 Scan Context 匹配 | 1 Hz |
| **`process_icp`** | 回环约束计算：从 ICP 队列取出候选，执行 ICP 精化 | 事件驱动 |
| **`process_isam`** | iSAM2 增量优化：新因子加入图后触发 | 1 Hz |
| **`process_viz_map`** | 地图可视化：重建并发布全局点云地图 | 0.5 Hz |
| **`process_viz_path`** | 路径可视化：发布优化后的里程计+轨迹 | 10 Hz |
| **`process_idle_status`** | 空闲检测：队列排空后提示可以安全保存地图 | 1 Hz |
| **`rclcpp::spin`** | ROS2 主事件循环 | N/A |

### 数据流（关键帧流水线）

```
里程计+点云 → 时间戳对齐 → 检查 Δt/ΔR → 是否为关键帧？
                                               │
                                       ┌───────┴───────┐
                                       │     是         │ 否（跳过）
                                       ▼
                      ┌───────────────────────────────────┐
                      │ 1. 体素降采样（VoxelGrid 0.4m）    │
                      │ 2. 存储：点云、位姿、时间戳         │
                      │ 3. 向 GTSAM 图中添加里程计因子      │
                      │ 4. 生成 Scan Context 描述子         │
                      └───────────────┬───────────────────┘
                                      │
                                      ▼
                      ┌───────────────────────────────────┐
                      │ 回环检测（1 Hz）                   │
                      │ ScanContext::detectLoopClosure     │
                      │   → ringkey kNN 搜索               │
                      │   → 列对齐余弦距离                 │
                      │   → 阈值判断                       │
                      └───────────────┬───────────────────┘
                                      │（发现回环）
                                      ▼
                      ┌───────────────────────────────────┐
                      │ ICP 精化                           │
                      │ 源点云：当前关键帧                  │
                      │ 目标点云：历史子图（前后±5帧）      │
                      │ 初值：仅平移（不使用 SC 的偏航角）  │
                      │ 适应度阈值：0.3                    │
                      └───────────────┬───────────────────┘
                                      │（ICP 收敛）
                                      ▼
                      ┌───────────────────────────────────┐
                      │ 插入回环闭合因子                    │
                      │ （鲁棒核函数：Cauchy）              │
                      │ → 触发 iSAM2 优化                  │
                      └───────────────────────────────────┘
```

## Scan Context 回环检测

回环检测模块实现了 [Scan Context](https://github.com/irapkaist/scancontext)（G. Kim 和 A. Kim, IROS 2018）激光雷达全局描述子：

- **网格结构**：20 环 × 60 扇区，覆盖距离 `sc_max_radius` 米
- **格值**：每个格子中的最大点高度（z）
- **环键（Ring Key）**：行均值向量（20×1），用于快速 kNN 搜索
- **扇区键（Sector Key）**：列均值向量（1×60），用于偏航角对齐
- **距离度量**：对应扇区间的余弦距离，穷举偏航搜索（±9 扇区 = ±54°）
- **kNN 候选**：从 KD-Tree 中取 Top 3（使用 nanoflann 实现），排除最近 30 个关键帧
- **阈值**：通过 `sc_dist_thres` 配置（默认 0.2~0.3）

## 位姿图优化

基于 GTSAM 的 **iSAM2** 增量式优化器：

| 因子类型 | 噪声模型 | 说明 |
|---------|---------|------|
| **先验因子（Prior）** | 对角（1e-12） | 固定原点（第一个关键帧） |
| **里程计因子（Between）** | 对角（1e-6 平移, 1e-4 旋转） | 相邻关键帧间的相对位姿（来自前端） |
| **回环因子（Between）** | 鲁棒 Cauchy（0.5） | ICP 精化后的非相邻关键帧相对位姿 |
| **GPS 因子（可选）** | 鲁棒 Cauchy（1e9 xy, 250 z） | 仅约束高度（默认关闭） |

## 参数调优

可通过启动文件调整的关键参数：

| 参数 | 效果 |
|------|------|
| `keyframe_meter_gap` | 越小关键帧越密（精度更高但更慢）。建议 0.5~2.0 米 |
| `sc_dist_thres` | 越低虚警越少，但可能漏检。0.15 严格，0.3 宽松 |
| `sc_max_radius` | 与激光雷达有效距离匹配。Ouster 用 80 米，Mid-360 用 40 米 |
| `mapviz_filter_size` | 越大地图越轻。0.4 米是较好平衡 |

## 文件结构

```
lio_loopclosure/
├── CMakeLists.txt
├── package.xml
├── include/
│   ├── lio_loopclosure/
│   │   ├── common.h              # PointType, Pose6D, 角度工具函数
│   │   └── tic_toc.h             # TicToc 计时工具
│   └── scancontext/
│       ├── Scancontext.h          # SCManager 类声明
│       ├── Scancontext.cpp        # Scan Context 实现
│       ├── nanoflann.hpp          # KD-Tree 库（纯头文件）
│       └── KDTreeVectorOfVectorsAdaptor.h  # nanoflann 适配器
├── src/
│   └── laserPosegraphOptimization.cpp   # 主节点（约 1450 行）
├── launch/
│   ├── fastlio_ouster64.launch.py       # FAST-LIO2 + Ouster64 启动文件
│   └── pointlio_mid360.launch.py        # Point-LIO + Mid-360 启动文件
├── rviz_cfg/
│   └── lio_loopclosure.rviz                      # RViz2 配置
└── utils/
    ├── python/
    │   ├── makeMergedMap.py             # 离线地图拼接（带强度着色）
    │   ├── pypcdMyUtils.py              # PCD 读写工具
    │   ├── bone_table.npy
    │   └── jet_table.npy                # 颜色查找表
    └── sample_data/
        └── KAIST03/                     # KAIST 03 序列样本数据
            ├── optimized_poses.txt      # 3179 帧 KITTI 格式位姿
            ├── times.txt                # 时间戳
            └── Scans/                   # 逐帧 PCD 点云
```

## 已测试的组合

- **FAST-LIO2**（Ouster OS1-64）— 启动文件：`fastlio_ouster64.launch.py`
- **Point-LIO**（Livox Mid-360）— 启动文件：`pointlio_mid360.launch.py`
- LIO-LoopClosure 与前端里程计松耦合，理论上支持任意发布所需话题的 LiDAR SLAM 前端

## 致谢

- **Scan Context**：G. Kim 和 A. Kim, "Scan Context: Egocentric 3D Place Recognition for Large-Scale Localization with Point Cloud", IROS 2018
- **GTSAM**：Georgia Tech Smoothing and Mapping 库
- **LOAM**：J. Zhang 和 S. Singh, "LOAM: Lidar Odometry and Mapping in Real-time", RSS 2014
- 原始 ROS1 实现：**Tong Qin**、**Shaozu Cao**；ROS2 移植及增强版

## 许可证

BSD
