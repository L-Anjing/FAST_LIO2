# FAST-LIO2 论文 + 工程代码深度分析报告

## 📘 论文核心算法概述

**FAST-LIO2** (Fast Direct Lidar-Inertial Odometry) 的核心创新是：
- **直接ICP (Direct Matching)**: 不提取特征，而是使用原始点云直接配准
- **紧耦合EKF**: 将LiDAR和IMU深度融合
- **动态KD-Tree**: 高效管理地图点云

---

## 🔴 主要逻辑实现分析

### **1️⃣ 数据预处理 (预处理阶段)**

#### 论文思想：
- 将激光点云从发射时刻变换到扫描结束时刻 (运动补偿)
- 移除不可靠的点 (盲区、距离过远等)

#### 代码实现对应：

**文件**: `src/preprocess.cpp` 和 `src/IMU_Processing.hpp`

```cpp
// 核心流程
1. avia_handler() / oust64_handler() / velodyne_handler()
   ↓
2. 特征提取 (如果启用)
   - give_feature() 分类点云: 平面点 vs 边缘点
   - plane_judge(): 判断是否为平面
   ↓
3. 采样和滤波
   - 距离阈值过滤 (blind)
   - 间隔采样 (point_filter_num)
   ↓
4. 输出: pl_surf (平面点)
```

**关键参数**:
- `blind`: 盲区 (近点过滤)
- `disA, disB`: 平面判断的距离阈值
- `p2l_ratio`: 点到线的比值阈值

---

### **2️⃣ IMU积分与运动补偿 (关键！)**

#### 论文思想：
$$\mathbf{R}(t) = \mathbf{R}_k \exp((\boldsymbol{\omega} - \mathbf{b}_g)^{\wedge} (t - t_k))$$

激光点云进行**运动去畸变**，变换到扫描结束时刻的坐标系。

#### 代码实现：

**文件**: `include/IMU_Processing.hpp` 中的 `Process()` 函数

```cpp
class IMUProcess {
    void Process(MeasureGroup &meas, KalmanFilter &kf, PointCloudXYZI &feats_undistort) {
        // 1. 陀螺仪积分：计算旋转矩阵 R(t)
        //   关键公式: R(t) = R_k * exp(integral(omega) dt)
        
        // 2. 加速度计积分：计算速度和位置
        //   v(t) = v_k + integral(acc) dt
        //   p(t) = p_k + integral(v) dt
        
        // 3. 去除点云畸变 (Undistortion)
        //   将每个点从发射时刻变换到扫描结束时刻
        for each point {
            t_point = point.time;  // 该点的发射时刻
            R_point = SO3_exp(integral_omega[t_point]);  // 该时刻的旋转
            p_point = integral_v[t_point];  // 该时刻的位移
            
            // 变换点到扫描结束时刻
            point_undistorted = R(t_end).T * (R_point * point + p_point)
        }
    }
};
```

**关键变量**:
- `vx, vy, vz`: 速度积分
- `ax, ay, az`: 加速度积分
- 通过陀螺仪积分计算旋转矩阵

**在代码中的位置**: `laserMapping.cpp` 第1020行附近：
```cpp
p_imu->Process(Measures, kf, feats_undistort);
```

---

### **3️⃣ ICP匹配与特征搜索 (核心！)**

#### 论文思想：
直接使用**点到平面**的距离作为观测量，而不是特征点匹配。

$$\mathbf{h} = \mathbf{n}^T (\mathbf{R} \mathbf{p}_{body} + \mathbf{t} - \mathbf{p}_{map})$$

其中 $\mathbf{n}$ 是地图中最近邻点的法向量。

#### 代码实现：

**文件**: `src/laserMapping.cpp` 主循环 (第1000-1050行)

```cpp
// 步骤 1: KDTree 搜索最近邻
for(int i = 0; i < feats_down_size; i++) {
    // 搜索 body frame 中的点在 world frame 地图中的近邻
    Nearest_Points[i] = ikdtree.Nearest_Search(feats_down_world[i], NUM_MATCH_POINTS);
    //                     ↑ 地图的 KDTree     ↑ 查询点       ↑ 搜索k个最近邻点
}

// 步骤 2: 计算 Jacobian 矩阵
// 在 h_share_model 函数中 (laserMapping.cpp L800-820)
const PointType &norm_p = corr_normvect->points[i];
V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);  // 地图中点的法向量

// 计算观测量: 点到平面的距离
// h = -norm_p.intensity (存储的是到平面的距离)

// 计算 Jacobian H
// H = [n_x, n_y, n_z, ...rotation_terms..., ...extrinsic_terms...]
```

**观测方程的含义**:
- 将激光点变换到世界坐标系后，计算它到附近点的法向量方向上的距离
- 这个距离应该很小 (理想情况为 0)
- EKF 通过最小化这个距离来优化位姿

**特殊处理**: `point_selected_surf` 数组
- 记录哪些点被选中用于优化
- 迭代过程中会动态调整

---

### **4️⃣ 迭代EKF优化 (Extended Kalman Filter)**

#### 论文思想：
标准的 **Extended Kalman Filter** 框架：
$$\mathbf{x}_k = [\mathbf{p}, \mathbf{v}, \mathbf{R}, \mathbf{b}_g, \mathbf{b}_a, \mathbf{g}, \mathbf{T}_{L \to I}]$$

包含状态：位置、速度、旋转、IMU偏差、外参等。

#### 代码实现：

**文件**: `src/laserMapping.cpp` L1050 附近

```cpp
// EKF 更新（迭代版本）
kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
         ↑
    这是核心的滤波器更新函数
    在 include/use-ikfom.hpp 中定义
    
// 过程：
// 1. 预测 (Predict): 根据IMU数据预测新的状态
// 2. 更新 (Update): 根据LiDAR观测修正状态
//    - 多次迭代以达到收敛 (NUM_MAX_ITERATIONS)
//    - 每次迭代：计算残差 → 计算增益矩阵 → 更新状态
```

**状态结构** (`state_ikfom`):
```cpp
struct state_ikfom {
    Eigen::Vector3d pos;           // 位置 p
    Eigen::Vector3d vel;           // 速度 v
    Eigen::Quaterniond rot;        // 旋转 R (四元数)
    Eigen::Vector3d bg;            // 陀螺仪 bias
    Eigen::Vector3d ba;            // 加速度计 bias
    double grav;                   // 重力加速度
    Eigen::Vector3d offset_T_L_I;  // LiDAR 相对 IMU 的平移
    Eigen::Quaterniond offset_R_L_I; // LiDAR 相对 IMU 的旋转
};
```

---

### **5️⃣ 地图更新与维护 (KD-Tree)**

#### 论文思想：
维护一个**动态KD-Tree**结构，随着机器人运动：
- 添加新的点云
- 删除距离过远的点
- 定期重建以保持树的平衡

#### 代码实现：

**文件**: `src/laserMapping.cpp`

```cpp
// 步骤 1: FOV 分割 (野外视场管理)
lasermap_fov_segment()  // L250-290
  ↓
  // 定义局部地图立方体 (cube)
  // 当机器人接近立方体边界时，移动立方体
  // 删除超出新立方体的点

// 步骤 2: 点云下采样
downSizeFilterSurf.filter(*feats_down_body)
  ↓
  // 使用体素网格下采样，减少计算量

// 步骤 3: 地图增量更新
map_incremental()  // L420-470
  {
    for each point in feats_down_body {
      // 判断是否应该添加到地图
      if(need_add) {
        PointToAdd.push_back(point);
      }
    }
    ikdtree.Add_Points(PointToAdd, true);  // 添加到KD-Tree
  }
```

**KD-Tree 操作**:
- `Build()`: 初始化地图
- `Add_Points()`: 增量添加点
- `Delete_Point_Boxes()`: 删除立方体内的点
- `Nearest_Search()`: 搜索最近邻

---

## 🎯 应用时需要注意的关键点

### **1. 运动补偿的时间戳处理** ⚠️

```cpp
// 关键问题：不同雷达的时间戳格式不同
enum TIME_UNIT {
    SEC = 0,     // 秒级
    MS = 1,      // 毫秒
    US = 2,      // 微秒
    NS = 3       // 纳秒
};

// 配置中必须设置正确的时间单位
// 否则运动补偿会完全错误！
```

**应用建议**:
- 确认你的激光雷达时间戳格式
- 在配置文件中正确设置 `timestamp_unit`
- 检查 IMU 时间戳和激光时间戳的同步问题

---

### **2. 外参估计 (Extrinsic Calibration)** ⚠️

```cpp
// LiDAR 相对于 IMU 的旋转和平移
// 这是系统性能的关键！

nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());
nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>());

extrinsic_est_en = true;  // 自动估计外参
```

**应用建议**:
- 如果启用 `extrinsic_est_en=true`，系统会自动优化外参
- 提供初值会加速收敛
- 外参误差会导致严重的漂移

---

### **3. EKF噪声参数调优** ⚠️

```cpp
nh.param<double>("mapping/gyr_cov", gyr_cov, 0.1);      // 陀螺仪噪声
nh.param<double>("mapping/acc_cov", acc_cov, 0.1);      // 加速度计噪声
nh.param<double>("mapping/b_gyr_cov", b_gyr_cov, 0.0001);  // 陀螺仪 bias 噪声
nh.param<double>("mapping/b_acc_cov", b_acc_cov, 0.0001);  // 加速度计 bias 噪声
```

**应用建议**:
- 这些参数决定了EKF对IMU的信任程度
- 如果噪声参数太小 → 过度信任IMU，漂移快
- 如果噪声参数太大 → 过度信任LiDAR，发散
- 根据实际IMU的spec进行调整

---

### **4. 特征点提取vs直接ICP** ⚠️

```cpp
nh.param<bool>("feature_extract_enable", p_pre->feature_enabled, false);

// 论文推荐值: false
// 原因: FAST-LIO2 使用直接点云匹配，不需要特征提取
```

**应用建议**:
- **不建议启用特征提取**，这会降低算法精度
- 直接使用原始点云是 FAST-LIO2 的优势所在

---

### **5. 平面检测参数调优** ⚠️ (如果启用特征提取)

```cpp
// 这些参数只有在 feature_enabled=true 时才会使用

disA = 0.1;          // 距离阈值
disB = 0.1;
p2l_ratio = 225;     // 点到线比值阈值
limit_maxmid = 6.25; // 中点距离变化率范围
limit_midmin = 6.25;
limit_maxmin = 3.24;

// 这些参数控制了"什么样的点被认为是平面"
```

**应用建议**:
- 这些参数对特征提取质量影响很大
- 根据你的应用场景和环境调整
- 例如，在特征丰富的室内环境vs平坦的室外环境，参数完全不同

---

### **6. 地图管理** ⚠️

```cpp
nh.param<double>("cube_side_length", cube_len, 200);
      ↑ 局部地图立方体的边长

// 如果 cube_len 太小：
//   - 内存低，计算快
//   - 但回环检测困难，不能构建全局地图
//
// 如果 cube_len 太大：
//   - 可以保留更多地图信息
//   - 但内存占用和计算量增加
```

**应用建议**:
- 根据你的应用场景选择
- 室内小场景: 50-100m
- 室外大场景: 200-500m
- 需要全局一致性: 更大值，但需要足够的计算资源

---

### **7. 时间同步问题** ⚠️

```cpp
// 关键参数
time_sync_en = false;  // 是否启用自动时间同步
time_diff_lidar_to_imu = 0.0;  // 时间偏移

// 如果LiDAR和IMU时间戳不同步，会导致严重问题！
```

**应用建议**:
- 检查你的硬件时间同步方案
- 如果使用 GPIO 同步（推荐），设 `time_sync_en=false`
- 如果没有同步，设 `time_sync_en=true`，但精度会降低

---

## 📊 核心算法流程图

```
┌─ 激光点云 ───────────────────────────────────────────────┐
│                                                             │
├─ 预处理 (preprocess.cpp)                                 │
│  ├─ Handler 选择 (avia/oust/velo/sim)                   │
│  ├─ 特征提取 (可选)                                      │
│  └─ 输出: pl_surf (平面点)                              │
│                                                             │
├─ IMU 处理 (IMU_Processing.hpp)                           │
│  ├─ 陀螺仪积分 → 旋转矩阵                                │
│  ├─ 加速度计积分 → 位移                                  │
│  └─ 运动去畸变 (Undistortion)                           │
│                                                             │
├─ ICP 匹配核心                                            │
│  ├─ KDTree 搜索最近邻                                    │
│  ├─ 计算点到平面距离                                     │
│  └─ 构建 Jacobian 矩阵                                   │
│                                                             │
├─ EKF 优化 (use-ikfom.hpp)                               │
│  ├─ 状态预测                                             │
│  ├─ 多次迭代                                             │
│  │  ├─ 计算残差                                         │
│  │  ├─ 计算增益                                         │
│  │  └─ 更新状态                                         │
│  └─ 输出: 优化后的位姿                                  │
│                                                             │
├─ 地图管理 (laserMapping.cpp)                           │
│  ├─ FOV 分割 (立方体管理)                                │
│  ├─ 点云下采样                                           │
│  ├─ 点云体素化                                           │
│  └─ KD-Tree 更新                                        │
│                                                             │
└─ 发布结果 (Odometry, Map, Path)                         │
```

---

## 🔍 调试建议

1. **检查时间戳**:
   ```bash
   rostopic hz /livox/lidar /livox/imu
   # 应该看到一致的频率
   ```

2. **验证点云质量**:
   - RViz 中查看点云是否畸变
   - 检查是否有异常点

3. **监控 EKF 状态**:
   - 在日志中添加 DEBUG 输出
   - 观察协方差矩阵的收敛

4. **参数敏感性分析**:
   - 逐个调整参数
   - 观察里程计精度的变化

