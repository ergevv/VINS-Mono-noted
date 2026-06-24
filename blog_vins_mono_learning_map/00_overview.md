# 00 从零开始读 VINS-Mono：系统全景图

这篇是整套教程的入口。它的目标不是先推公式，而是先回答一个更基础的问题：

$$
\text{VINS-Mono 到底由哪些模块组成？每个模块解决什么问题？}
$$

VINS-Mono 可以理解为一个把相机和 IMU 测量变成连续轨迹的系统。它的主链路是：

$$
\text{图像与 IMU 输入} \rightarrow
\text{前端特征跟踪} \rightarrow
\text{IMU 预积分} \rightarrow
\text{初始化} \rightarrow
\text{滑窗非线性优化} \rightarrow
\text{边缘化} \rightarrow
\text{闭环检测与全局优化}
$$

## 1. 数据预处理

相机给出图像，IMU 给出角速度和加速度。二者频率不同、噪声不同、时间戳也可能存在偏差。

图像前端主要做：

- 检测角点；
- 跟踪相邻帧特征；
- 剔除外点；
- 输出归一化相机坐标、像素坐标和特征速度。

IMU 前端主要做：

- 用 IMU 高频数据预测当前帧的初始状态；
- 在相邻关键帧之间做预积分；
- 传播预积分协方差和 bias Jacobian。

## 2. 初始化

VIO 不是一上来就能直接优化。单目视觉没有尺度，IMU 有 bias，世界系重力方向也需要确定。

初始化要估计：

$$
\left\{
s,\mathbf{g},\mathbf{v}_k,\mathbf{b}_g,\mathbf{b}_a,\mathbf{T}_{ic}
\right\}
$$

其中：

- $s$ 表示单目视觉尺度；
- $\mathbf{g}$ 表示重力向量；
- $\mathbf{v}_k$ 表示第 $k$ 帧速度；
- $\mathbf{b}_g$ 表示陀螺仪零偏；
- $\mathbf{b}_a$ 表示加速度计零偏；
- $\mathbf{T}_{ic}$ 表示相机和 IMU 的外参。

初始化的本质是：

$$
\text{把无尺度视觉 SFM 和有物理单位的 IMU 预积分对齐}
$$

## 3. 后端滑窗优化

初始化成功后，系统进入正常滑窗优化。窗口内状态包括：

$$
\mathcal{X} =
\left\{
\mathbf{x}_0,\dots,\mathbf{x}_N,\lambda_1,\dots,\lambda_M,\mathbf{T}_{ic},t_d
\right\}
$$

其中：

- $\mathbf{x}_k$ 表示第 $k$ 帧 IMU 状态；
- $\lambda_l$ 表示第 $l$ 个特征点逆深度；
- $t_d$ 表示相机和 IMU 的时间偏移。

目标函数由三类主要残差组成：

$$
\min_{\mathcal{X}}
\left(
\left\|\mathbf{r}_{prior}\right\|^2 +
\sum_k
\left\|\mathbf{r}_{imu,k}\right\|^2 +
\sum_l
\left\|\mathbf{r}_{proj,l}\right\|^2
\right)
$$

其中：

- $\mathbf{r}_{prior}$ 表示边缘化先验；
- $\mathbf{r}_{imu,k}$ 表示 IMU 预积分残差；
- $\mathbf{r}_{proj,l}$ 表示视觉重投影残差。

## 4. 边缘化

滑窗不能无限增长。旧帧必须被移除，但历史信息不能直接丢掉。

边缘化做的事情是：

$$
\text{删除旧变量}
\quad
\text{但把它的信息转成当前变量上的 prior}
$$

核心工具是 Schur 补：

$$
\mathbf{H}^{prior} =
\mathbf{H}_{rr} -
\mathbf{H}_{rm}\mathbf{H}_{mm}^{-1}\mathbf{H}_{mr}
$$

其中 $m$ 表示被边缘化变量，$r$ 表示保留变量。

## 5. 可观性与一致性

VINS-Mono 没有 GPS 和磁力计，所以它不可能知道绝对世界原点和绝对 yaw。典型不可观方向是：

$$
3\text{ DOF 平移} +
1\text{ DOF yaw}
$$

一个一致的估计器不应该在这些方向上产生虚假自信。边缘化、线性化点、FEJ-like prior 都和这个问题有关。

## 6. 闭环

滑窗 VIO 会漂移。闭环模块用词袋检测历史相似帧，通过 PnP 得到闭环约束，再做 4DoF pose graph 优化。

闭环优化通常只优化：

$$
\left[
x,y,z,\psi
\right]
$$

其中 $\psi$ 表示 yaw。因为 VIO 已经通过重力对齐确定了 roll 和 pitch，而长期漂移主要体现在位置和 yaw。

## 7. 推荐学习顺序

建议按下面顺序读：

1. 先读因子图和 GTSAM 对照，建立优化问题视角；
2. 再读 IMU 预积分，理解 VIO 和纯视觉的根本区别；
3. 然后读后端残差与雅克比，知道优化器到底在最小化什么；
4. 接着读初始化，理解系统如何启动；
5. 再读边缘化、FEJ-like prior 和可观性；
6. 最后读前端和闭环，把工程闭环补齐。

