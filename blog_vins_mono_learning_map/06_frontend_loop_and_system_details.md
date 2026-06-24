# 06 前端、关键帧、闭环与系统细节

前几篇主要讲后端数学。这篇补上 VINS-Mono 的工程闭环：前端如何给后端提供观测，关键帧如何选择，闭环如何修正长期漂移。

## 1. 前端视觉处理做什么

前端视觉处理的目标是输出稳定的特征轨迹。它通常包括：

$$
\text{角点检测} \rightarrow
\text{光流跟踪} \rightarrow
\text{RANSAC 外点剔除} \rightarrow
\text{归一化坐标与速度计算}
$$

角点检测用于获得可跟踪点。光流跟踪用于在相邻帧之间建立同一个特征点的对应关系。RANSAC 用于剔除错误匹配。归一化坐标用于后端投影模型。

## 2. 特征点数据如何进入后端

一个特征观测通常包含：

$$
\left[
x,y,z,u,v,v_x,v_y
\right]
$$

其中：

- $x,y,z$ 表示归一化相机坐标；
- $u,v$ 表示像素坐标；
- $v_x,v_y$ 表示图像平面速度。

图像平面速度在时间同步优化中会用到：

$$
\mathbf{u}^{corr} =
\mathbf{u} +
t_d\dot{\mathbf{u}}
$$

其中 $t_d$ 是相机和 IMU 的时间偏移。

## 3. FeatureManager 的概念模型

后端并不是只看单帧特征，而是维护特征轨迹。可以抽象成：

$$
\text{FeaturePerId} \rightarrow
\left[
\text{FeaturePerFrame}_0,
\text{FeaturePerFrame}_1,
\dots
\right]
$$

其中：

- FeaturePerId 表示一个路标点；
- FeaturePerFrame 表示这个路标点在某一帧中的一次观测。

只有被多帧观测到的特征点，才能形成有效的几何约束。

## 4. 关键帧选择

VINS-Mono 不一定把每一帧都当作关键帧。常见判断依据包括：

- 当前帧和前面关键帧之间的视差；
- 当前帧跟踪到的特征点数量；
- 系统是否需要保持足够的运动基线。

如果视差足够大，当前帧更有价值，通常边缘化最老帧。如果视差不足，当前帧信息增量小，可能边缘化次新帧。

可以理解为：

$$
\text{视差大} \Rightarrow
\text{保留当前帧，移除最老帧}
$$

$$
\text{视差小} \Rightarrow
\text{当前帧不够关键，移除次新帧}
$$

## 5. 闭环检测为什么需要

滑窗 VIO 只保留局部窗口，长期运行一定会漂移。闭环检测用于发现：

$$
\text{当前帧} \approx
\text{历史某一帧}
$$

VINS-Mono 常用词袋方法进行图像检索。为了闭环检测更稳定，关键帧会重新提取更多角点并计算 BRIEF 描述子。

## 6. 闭环相对位姿

检测到候选闭环后，需要建立当前帧和历史帧之间的几何关系。常见流程是：

$$
\text{描述子匹配} \rightarrow
\text{RANSAC 剔除外点} \rightarrow
\text{PnP 求相对位姿}
$$

PnP 得到的相对位姿可作为闭环约束，进入后续优化。

## 7. 快速重定位

当闭环成功时，可以把闭环帧相关的视觉约束临时加入当前滑窗后端优化中，使当前窗口快速对齐到历史地图。

它的作用是：

$$
\text{用局部滑窗优化快速估计当前窗口相对历史帧的 drift}
$$

其中 drift 表示当前局部世界系相对于旧世界系的累积偏移。

## 8. 4DoF pose graph 优化

因为 VIO 已经通过重力确定了 roll 和 pitch，闭环全局优化通常只优化：

$$
\left[
\mathbf{p}_k,\psi_k
\right]
$$

其中：

- $\mathbf{p}_k$ 表示关键帧位置；
- $\psi_k$ 表示 yaw。

相邻关键帧之间有序列边，闭环帧之间有闭环边。目标函数可以写成：

$$
\min
\left(
\sum_{(i,j)\in\mathcal{S}}
\left\|\mathbf{r}_{ij}\right\|^2 +
\sum_{(i,j)\in\mathcal{L}}
\rho
\left(
\left\|\mathbf{r}_{ij}\right\|^2
\right)
\right)
$$

其中：

- $\mathcal{S}$ 表示序列边集合；
- $\mathcal{L}$ 表示闭环边集合；
- $\rho(\cdot)$ 表示鲁棒核。

一条 4DoF 边通常约束两个关键帧之间的相对位置和相对 yaw。设关键帧 $i$ 和 $j$ 的待优化量为：

$$
\mathbf{p}_i,\psi_i,\mathbf{p}_j,\psi_j
$$

其中：

- $\mathbf{p}_i$ 和 $\mathbf{p}_j$ 表示两个关键帧在全局 pose graph 坐标系下的位置；
- $\psi_i$ 和 $\psi_j$ 表示两个关键帧的 yaw；
- roll 和 pitch 不优化，因为它们已经由重力方向提供约束。

设边测量为：

$$
\hat{\mathbf{p}}_{ij},\quad \hat{\psi}_{ij}
$$

其中：

- $\hat{\mathbf{p}}_{ij}$ 表示从关键帧 $i$ 到关键帧 $j$ 的相对平移测量；
- $\hat{\psi}_{ij}$ 表示从关键帧 $i$ 到关键帧 $j$ 的相对 yaw 测量。

则残差可以写成：

$$
\mathbf{r}_{ij} =
\begin{bmatrix}
\mathbf{R}_{z}(\psi_i)^{\top}
\left(
\mathbf{p}_j-\mathbf{p}_i
\right) -
\hat{\mathbf{p}}_{ij}
\\
\psi_j-\psi_i-\hat{\psi}_{ij}
\end{bmatrix}
$$

其中 $\mathbf{R}_{z}(\psi_i)$ 表示由 yaw 角 $\psi_i$ 构造的绕重力方向旋转矩阵。这个残差表达的是：当前 pose graph 中两帧的相对 4DoF 关系，应该和序列边或闭环边测到的相对关系一致。

## 9. 后端优化后的 yaw 修正

由于全局 yaw 不可观，滑窗优化后可能出现整体 yaw 漂移。系统通常会把优化后的窗口重新旋回到一个一致的 yaw 参考上。

这不是传感器观测到了绝对 yaw，而是选择 gauge：

$$
\text{不可观方向需要人为选择表达方式}
$$

## 10. 从工程模块回到数学主线

前端、关键帧、闭环看起来是工程模块，但它们都服务于同一个数学目标：

$$
\text{给后端提供足够好、足够稳定、足够有约束力的因子}
$$

前端提供视觉因子，IMU 预积分提供惯性因子，初始化提供初值，边缘化保证实时性，闭环提供长期一致性。
