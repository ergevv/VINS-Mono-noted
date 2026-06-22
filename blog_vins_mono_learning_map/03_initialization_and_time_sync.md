# 03 初始化与时间同步：让 VIO 优化从正确的问题开始

## 1. 为什么初始化和时间同步要单独讲

前两篇已经说明，VIO 可以被写成一个因子图优化问题：

$$
\mathcal{X}^{*}
=
\arg\min_{\mathcal{X}}
\sum_i
\left\|
\mathbf{r}_i(\mathcal{X})
\right\|^2_{\mathbf{\Sigma}_i^{-1}}
$$

其中：

- $\mathcal{X}$ 表示所有待估计变量；
- $\mathbf{r}_i(\mathcal{X})$ 表示第 $i$ 个残差；
- $\mathbf{\Sigma}_i$ 表示第 $i$ 个残差的协方差；
- $\mathcal{X}^{*}$ 表示最优估计。

但是，非线性优化有一个前提：初值不能太差。VIO 的残差包含旋转、投影、IMU 预积分、逆深度等强非线性关系。如果一开始的尺度、重力方向、速度、IMU 零偏、相机-IMU 外参或时间同步严重错误，优化器很可能收敛到错误结果，甚至完全发散。

因此，初始化要解决的问题是：

$$
\text{在进入滑窗非线性优化前，构造一个足够合理的初始状态。}
$$

时间同步要解决的问题是：

$$
\text{当相机和 IMU 时间戳存在偏差时，让视觉观测对应到正确的物理时刻。}
$$

这两个问题都不是锦上添花。初始化决定优化能不能起步，时间同步决定残差模型是不是在解释同一个时刻的运动。

## 2. VIO 初始化到底要初始化什么

单目 VIO 的初始化通常要估计以下量：

$$
\left\{
s,\,
\mathbf{R}_{wg},\,
\mathbf{v}_k,\,
\mathbf{b}_g,\,
\mathbf{b}_a,\,
\mathbf{T}_{ic}
\right\}
$$

其中：

- $s$ 表示视觉结构恢复出来的尺度因子；
- $\mathbf{R}_{wg}$ 表示把重力方向对齐到世界坐标系的旋转；
- $\mathbf{v}_k$ 表示第 $k$ 帧的速度；
- $\mathbf{b}_g$ 表示陀螺仪零偏；
- $\mathbf{b}_a$ 表示加速度计零偏；
- $\mathbf{T}_{ic}$ 表示相机到 IMU 的外参。

为什么这些量重要？

纯单目视觉能恢复轨迹形状，但尺度未知。IMU 有米、秒、弧度这些物理单位，可以帮助恢复尺度和重力方向。可是 IMU 自己又有零偏，外参也可能未知。因此初始化本质上是把“无尺度视觉几何”和“有尺度惯性测量”对齐。

可以把初始化理解成下面这条链：

$$
\text{视觉相对运动}
\rightarrow
\text{无尺度 SFM}
\rightarrow
\text{陀螺仪零偏校正}
\rightarrow
\text{视觉-惯性对齐}
\rightarrow
\text{尺度、重力、速度}
\rightarrow
\text{进入非线性优化}
$$

## 3. 第一步：用视觉建立无尺度几何

单目相机首先可以通过多帧特征匹配建立几何关系。对于两帧图像，如果有足够多的匹配点，可以估计本质矩阵：

$$
\mathbf{E}
=
[\mathbf{t}]_{\times}\mathbf{R}
$$

其中：

- $\mathbf{E}$ 表示本质矩阵；
- $\mathbf{R}\in SO(3)$ 表示两帧相机之间的相对旋转；
- $\mathbf{t}\in\mathbb{R}^3$ 表示两帧相机之间的相对平移方向；
- $[\mathbf{t}]_{\times}$ 表示由向量 $\mathbf{t}$ 构成的反对称矩阵。

本质矩阵约束为：

$$
\bar{\mathbf{u}}_2^{\top}
\mathbf{E}
\bar{\mathbf{u}}_1
=
0
$$

其中：

- $\bar{\mathbf{u}}_1$ 表示特征点在第一帧的归一化齐次坐标；
- $\bar{\mathbf{u}}_2$ 表示同一特征点在第二帧的归一化齐次坐标。

单目几何能恢复 $\mathbf{R}$ 和 $\mathbf{t}$ 的方向，但不能恢复 $\mathbf{t}$ 的真实长度。也就是说，如果视觉恢复出的平移为 $\tilde{\mathbf{t}}$，真实平移可以写成：

$$
\mathbf{t}
=
s\tilde{\mathbf{t}}
$$

其中：

- $\tilde{\mathbf{t}}$ 表示无尺度视觉平移；
- $s$ 表示未知尺度。

通过多帧结构恢复，可以得到一组无尺度相机位姿和特征点：

$$
\left\{
\tilde{\mathbf{R}}_k,\,
\tilde{\mathbf{p}}_k,\,
\tilde{\mathbf{P}}_l
\right\}
$$

其中：

- $\tilde{\mathbf{R}}_k$ 表示第 $k$ 帧相机的视觉旋转估计；
- $\tilde{\mathbf{p}}_k$ 表示第 $k$ 帧相机的无尺度位置；
- $\tilde{\mathbf{P}}_l$ 表示第 $l$ 个特征点的无尺度三维坐标。

这些量还不能直接作为 VIO 状态，因为它们缺少尺度，也没有和 IMU 的重力方向、速度、零偏对齐。

## 4. 第二步：估计陀螺仪零偏

陀螺仪测量模型为：

$$
\hat{\boldsymbol{\omega}}_t
=
\boldsymbol{\omega}_t
+
\mathbf{b}_g
+
\mathbf{n}_g
$$

其中：

- $\hat{\boldsymbol{\omega}}_t$ 表示陀螺仪在时间 $t$ 的测量值；
- $\boldsymbol{\omega}_t$ 表示真实角速度；
- $\mathbf{b}_g$ 表示陀螺仪零偏；
- $\mathbf{n}_g$ 表示陀螺仪噪声。

IMU 预积分会根据当前零偏估计得到相邻帧之间的旋转：

$$
\Delta\mathbf{q}_{ij}(\mathbf{b}_g)
$$

其中 $\Delta\mathbf{q}_{ij}(\mathbf{b}_g)$ 表示从第 $i$ 帧到第 $j$ 帧的预积分旋转，它依赖陀螺仪零偏。

视觉也能提供相邻帧之间的相对旋转：

$$
\mathbf{q}_{ij}^{\text{vis}}
=
\left(\mathbf{q}_i^{\text{vis}}\right)^{-1}
\otimes
\mathbf{q}_j^{\text{vis}}
$$

其中：

- $\mathbf{q}_i^{\text{vis}}$ 表示视觉估计的第 $i$ 帧姿态；
- $\mathbf{q}_{ij}^{\text{vis}}$ 表示视觉估计的相对旋转。

陀螺仪零偏初始化的目标是让 IMU 预积分旋转和视觉相对旋转一致：

$$
\min_{\mathbf{b}_g}
\sum_{(i,j)}
\left\|
2\,\text{vec}
\left(
\Delta\mathbf{q}_{ij}(\mathbf{b}_g)^{-1}
\otimes
\mathbf{q}_{ij}^{\text{vis}}
\right)
\right\|^2
$$

其中：

- $\text{vec}(\cdot)$ 表示取四元数虚部；
- $2\,\text{vec}(\cdot)$ 是小角度姿态误差近似。

这一步的目的，是先把角度积分修正好。因为姿态错了，后面尺度、重力和速度都会被带偏。

## 5. 第三步：视觉-惯性对齐

完成视觉 SFM 和陀螺仪零偏估计后，需要把无尺度视觉轨迹和 IMU 运动方程对齐。

IMU 离散运动模型可以写成：

$$
\mathbf{p}_{j}
=
\mathbf{p}_{i}
+
\mathbf{v}_{i}\Delta t_{ij}
+
\frac{1}{2}\mathbf{g}\Delta t_{ij}^{2}
+
\mathbf{R}_{i}\Delta\mathbf{p}_{ij}
$$

$$
\mathbf{v}_{j}
=
\mathbf{v}_{i}
+
\mathbf{g}\Delta t_{ij}
+
\mathbf{R}_{i}\Delta\mathbf{v}_{ij}
$$

其中：

- $\mathbf{p}_i$ 和 $\mathbf{p}_j$ 表示第 $i$、$j$ 帧 IMU 在世界系下的位置；
- $\mathbf{v}_i$ 和 $\mathbf{v}_j$ 表示第 $i$、$j$ 帧 IMU 在世界系下的速度；
- $\Delta t_{ij}$ 表示两帧之间的时间间隔；
- $\mathbf{g}$ 表示世界坐标系下的重力向量；
- $\mathbf{R}_i$ 表示第 $i$ 帧 IMU 到世界坐标系的旋转；
- $\Delta\mathbf{p}_{ij}$ 表示 IMU 预积分相对位移；
- $\Delta\mathbf{v}_{ij}$ 表示 IMU 预积分相对速度。

视觉给出的是无尺度位置：

$$
\mathbf{p}_{c,k}
=
s\tilde{\mathbf{p}}_{c,k}
$$

其中：

- $\mathbf{p}_{c,k}$ 表示第 $k$ 帧相机的真实位置；
- $\tilde{\mathbf{p}}_{c,k}$ 表示视觉 SFM 得到的无尺度位置；
- $s$ 表示尺度。

如果考虑相机和 IMU 外参：

$$
\mathbf{p}_{c,k}
=
\mathbf{p}_{i,k}
+
\mathbf{R}_{i,k}\mathbf{p}_{ic}
$$

其中：

- $\mathbf{p}_{i,k}$ 表示第 $k$ 帧 IMU 的位置；
- $\mathbf{R}_{i,k}$ 表示第 $k$ 帧 IMU 的姿态；
- $\mathbf{p}_{ic}$ 表示相机原点在 IMU 坐标系下的位置。

现在开始推导视觉-惯性对齐的线性方程。为了让符号更清楚，先把相邻两帧记为 $k$ 和 $k+1$，并定义：

$$
\Delta t_k
=
t_{k+1}-t_k
$$

其中：

- $t_k$ 表示第 $k$ 帧图像或关键帧对应的时间；
- $\Delta t_k$ 表示第 $k$ 帧到第 $k+1$ 帧之间的时间间隔。

IMU 位置递推方程为：

$$
\mathbf{p}_{i,k+1}
=
\mathbf{p}_{i,k}
+
\mathbf{v}_k\Delta t_k
+
\frac{1}{2}\mathbf{g}\Delta t_k^2
+
\mathbf{R}_{i,k}\Delta\mathbf{p}_{k,k+1}
$$

其中：

- $\mathbf{p}_{i,k}$ 表示第 $k$ 帧 IMU 在世界坐标系下的位置；
- $\mathbf{v}_k$ 表示第 $k$ 帧 IMU 在世界坐标系下的速度；
- $\mathbf{R}_{i,k}$ 表示第 $k$ 帧 IMU 坐标系到世界坐标系的旋转；
- $\Delta\mathbf{p}_{k,k+1}$ 表示从第 $k$ 帧到第 $k+1$ 帧的 IMU 预积分位移；
- $\mathbf{g}$ 表示世界坐标系下的重力向量。

把未知项放在左边，已知预积分项放在右边：

$$
\mathbf{p}_{i,k+1}
-
\mathbf{p}_{i,k}
-
\mathbf{v}_k\Delta t_k
-
\frac{1}{2}\mathbf{g}\Delta t_k^2
=
\mathbf{R}_{i,k}\Delta\mathbf{p}_{k,k+1}
$$

视觉 SFM 给出的是相机的无尺度位置 $\tilde{\mathbf{p}}_{c,k}$，真实相机位置为：

$$
\mathbf{p}_{c,k}
=
s\tilde{\mathbf{p}}_{c,k}
$$

相机和 IMU 的位置关系为：

$$
\mathbf{p}_{c,k}
=
\mathbf{p}_{i,k}
+
\mathbf{R}_{i,k}\mathbf{p}_{ic}
$$

因此 IMU 位置可以由视觉位置和外参写成：

$$
\mathbf{p}_{i,k}
=
s\tilde{\mathbf{p}}_{c,k}
-
\mathbf{R}_{i,k}\mathbf{p}_{ic}
$$

同理：

$$
\mathbf{p}_{i,k+1}
=
s\tilde{\mathbf{p}}_{c,k+1}
-
\mathbf{R}_{i,k+1}\mathbf{p}_{ic}
$$

代入 IMU 位置递推方程：

$$
\left(
s\tilde{\mathbf{p}}_{c,k+1}
-
\mathbf{R}_{i,k+1}\mathbf{p}_{ic}
\right)
-
\left(
s\tilde{\mathbf{p}}_{c,k}
-
\mathbf{R}_{i,k}\mathbf{p}_{ic}
\right)
-
\mathbf{v}_k\Delta t_k
-
\frac{1}{2}\mathbf{g}\Delta t_k^2
=
\mathbf{R}_{i,k}\Delta\mathbf{p}_{k,k+1}
$$

整理视觉位置差：

$$
s
\left(
\tilde{\mathbf{p}}_{c,k+1}
-
\tilde{\mathbf{p}}_{c,k}
\right)
-
\mathbf{v}_k\Delta t_k
-
\frac{1}{2}\mathbf{g}\Delta t_k^2
=
\mathbf{R}_{i,k}\Delta\mathbf{p}_{k,k+1}
+
\left(
\mathbf{R}_{i,k+1}
-
\mathbf{R}_{i,k}
\right)
\mathbf{p}_{ic}
$$

这个式子已经是关于 $s$、$\mathbf{v}_k$、$\mathbf{g}$ 的线性方程。把未知量按顺序写成：

$$
\mathbf{y}
=
\left[
\mathbf{v}_0,\mathbf{v}_1,\dots,\mathbf{v}_N,\,
\mathbf{g},\,
s
\right]
$$

其中：

- $\mathbf{y}$ 表示初始化阶段的线性未知量集合；
- $\mathbf{v}_k$ 表示第 $k$ 帧速度；
- $\mathbf{g}$ 表示重力向量；
- $s$ 表示尺度。

对于第 $k$ 段 IMU 预积分，上面的方程可以写成块矩阵形式：

$$
\begin{bmatrix}
\mathbf{0}
&
\cdots
&
-\Delta t_k\mathbf{I}_3
&
\cdots
&
\mathbf{0}
&
-\frac{1}{2}\Delta t_k^2\mathbf{I}_3
&
\tilde{\mathbf{p}}_{c,k+1}-\tilde{\mathbf{p}}_{c,k}
\end{bmatrix}
\mathbf{y}
=
\mathbf{R}_{i,k}\Delta\mathbf{p}_{k,k+1}
+
\left(
\mathbf{R}_{i,k+1}
-
\mathbf{R}_{i,k}
\right)
\mathbf{p}_{ic}
$$

其中：

- $\mathbf{I}_3$ 表示 $3\times3$ 单位矩阵；
- $-\Delta t_k\mathbf{I}_3$ 放在速度变量 $\mathbf{v}_k$ 对应的列块上；
- $-\frac{1}{2}\Delta t_k^2\mathbf{I}_3$ 放在重力 $\mathbf{g}$ 对应的列块上；
- $\tilde{\mathbf{p}}_{c,k+1}-\tilde{\mathbf{p}}_{c,k}$ 放在尺度 $s$ 对应的列块上；
- 其他速度变量，例如 $\mathbf{v}_{k-1}$ 或 $\mathbf{v}_{k+1}$，在这个位置方程中系数为零。

注意最后一列看起来是一个三维向量，这是因为尺度 $s$ 是标量，乘上视觉无尺度位移：

$$
\left(
\tilde{\mathbf{p}}_{c,k+1}-\tilde{\mathbf{p}}_{c,k}
\right)s
$$

会得到一个三维向量。

仅使用位置递推方程，也可以约束速度、重力和尺度。为了让相邻速度之间也被约束起来，还可以使用 IMU 速度递推方程：

$$
\mathbf{v}_{k+1}
=
\mathbf{v}_k
+
\mathbf{g}\Delta t_k
+
\mathbf{R}_{i,k}\Delta\mathbf{v}_{k,k+1}
$$

整理为：

$$
\mathbf{v}_{k+1}
-
\mathbf{v}_k
-
\mathbf{g}\Delta t_k
=
\mathbf{R}_{i,k}\Delta\mathbf{v}_{k,k+1}
$$

对应的块矩阵为：

$$
\begin{bmatrix}
\mathbf{0}
&
\cdots
&
-\mathbf{I}_3
&
\mathbf{I}_3
&
\cdots
&
\mathbf{0}
&
-\Delta t_k\mathbf{I}_3
&
\mathbf{0}_{3\times1}
\end{bmatrix}
\mathbf{y}
=
\mathbf{R}_{i,k}\Delta\mathbf{v}_{k,k+1}
$$

其中：

- $-\mathbf{I}_3$ 放在 $\mathbf{v}_k$ 对应的列块上；
- $\mathbf{I}_3$ 放在 $\mathbf{v}_{k+1}$ 对应的列块上；
- $-\Delta t_k\mathbf{I}_3$ 放在 $\mathbf{g}$ 对应的列块上；
- $\mathbf{0}_{3\times1}$ 表示速度方程不直接约束尺度 $s$。

把所有相邻帧的这些方程堆叠起来，就得到整体线性方程：

$$
\mathbf{A}\mathbf{y}
=
\mathbf{b}
$$

其中：

- $\mathbf{A}$ 表示由时间间隔、视觉相对位移、姿态和预积分量组成的系数矩阵；
- $\mathbf{b}$ 表示由预积分位移、预积分速度和外参平移补偿项组成的右端向量。

更具体地说，每一段相邻帧 $k\rightarrow k+1$ 至少可以贡献两类 $3$ 维方程：

$$
\mathbf{A}_{p,k}\mathbf{y}
=
\mathbf{b}_{p,k}
$$

$$
\mathbf{A}_{v,k}\mathbf{y}
=
\mathbf{b}_{v,k}
$$

其中：

$$
\mathbf{b}_{p,k}
=
\mathbf{R}_{i,k}\Delta\mathbf{p}_{k,k+1}
+
\left(
\mathbf{R}_{i,k+1}
-
\mathbf{R}_{i,k}
\right)
\mathbf{p}_{ic}
$$

$$
\mathbf{b}_{v,k}
=
\mathbf{R}_{i,k}\Delta\mathbf{v}_{k,k+1}
$$

把所有 $k=0,1,\dots,N-1$ 的方程堆叠，就是：

$$
\begin{bmatrix}
\mathbf{A}_{p,0}\\
\mathbf{A}_{v,0}\\
\mathbf{A}_{p,1}\\
\mathbf{A}_{v,1}\\
\vdots\\
\mathbf{A}_{p,N-1}\\
\mathbf{A}_{v,N-1}
\end{bmatrix}
\mathbf{y}
=
\begin{bmatrix}
\mathbf{b}_{p,0}\\
\mathbf{b}_{v,0}\\
\mathbf{b}_{p,1}\\
\mathbf{b}_{v,1}\\
\vdots\\
\mathbf{b}_{p,N-1}\\
\mathbf{b}_{v,N-1}
\end{bmatrix}
$$

也就是：

$$
\mathbf{A}\mathbf{y}
=
\mathbf{b}
$$

不同资料中 $\mathbf{A}$ 的具体块形式可能略有差异。有的推导直接使用上面的“位置方程 + 速度方程”；有的推导会把连续两段位置方程组合起来，得到同时含有 $\mathbf{v}_k$、$\mathbf{v}_{k+1}$、$\mathbf{g}$ 和 $s$ 的约束。它们的本质相同：都在利用 IMU 预积分运动模型和无尺度视觉位置之间的一致性，把初始化问题整理成关于速度、重力和尺度的线性最小二乘。

通过最小二乘求解：

$$
\mathbf{y}^{*}
=
\arg\min_{\mathbf{y}}
\left\|
\mathbf{A}\mathbf{y}-\mathbf{b}
\right\|^2
$$

就可以得到初始速度、重力和尺度。

## 6. 第四步：重力方向细化

重力大小通常已知，例如：

$$
\|\mathbf{g}\|
=
g_0
$$

其中：

- $\|\mathbf{g}\|$ 表示重力向量的模长；
- $g_0$ 表示重力加速度大小，约为 $9.81\,\text{m}/\text{s}^2$。

线性最小二乘直接求出的 $\mathbf{g}$ 可能模长不完全等于 $g_0$。因此常见做法是固定重力模长，只优化重力方向。

设当前重力方向为：

$$
\bar{\mathbf{g}}
$$

其中 $\bar{\mathbf{g}}$ 表示当前估计的重力向量。它的切平面上有两个正交基：

$$
\mathbf{b}_1,\quad \mathbf{b}_2
$$

其中 $\mathbf{b}_1$ 和 $\mathbf{b}_2$ 都垂直于 $\bar{\mathbf{g}}$。重力的小扰动可以写成：

$$
\mathbf{g}
\approx
\bar{\mathbf{g}}
+
w_1\mathbf{b}_1
+
w_2\mathbf{b}_2
$$

其中：

- $w_1$ 和 $w_2$ 表示重力方向在切平面上的两个小扰动系数。

这样只优化两个自由度，而不是任意三维重力向量。优化后再把重力归一化到固定模长：

$$
\mathbf{g}
\leftarrow
g_0
\frac{\mathbf{g}}{\|\mathbf{g}\|}
$$

这一步的目的，是让初始世界系的 roll 和 pitch 与真实重力方向一致。

## 7. 外参初始化：相机和 IMU 的相对姿态

相机和 IMU 不在同一个坐标系。外参为：

$$
\mathbf{T}_{ic}
=
\left[
\mathbf{R}_{ic},\mathbf{p}_{ic}
\right]
$$

其中：

- $\mathbf{R}_{ic}$ 表示相机坐标系到 IMU 坐标系的旋转；
- $\mathbf{p}_{ic}$ 表示相机原点在 IMU 坐标系下的位置。

如果 $\mathbf{R}_{ic}$ 未知，可以利用相邻帧的视觉旋转和 IMU 旋转进行估计。

设视觉估计的相机相对旋转为：

$$
\mathbf{R}_{c_i c_j}^{\text{vis}}
$$

IMU 预积分得到的 IMU 相对旋转为：

$$
\mathbf{R}_{i_i i_j}^{\text{imu}}
$$

两者满足近似关系：

$$
\mathbf{R}_{i_i i_j}^{\text{imu}}
\mathbf{R}_{ic}
=
\mathbf{R}_{ic}
\mathbf{R}_{c_i c_j}^{\text{vis}}
$$

这个形式就是经典的手眼标定旋转问题：

$$
\mathbf{A}\mathbf{X}
=
\mathbf{X}\mathbf{B}
$$

其中：

- $\mathbf{A}$ 对应 IMU 相对旋转；
- $\mathbf{B}$ 对应相机相对旋转；
- $\mathbf{X}$ 对应未知外参旋转 $\mathbf{R}_{ic}$。

外参初始化的目的，是让视觉运动和 IMU 运动处在同一个刚体坐标关系下。如果外参初值错误，视觉重投影因子和 IMU 因子会互相矛盾。

## 8. 时间同步为什么重要

理想情况下，相机图像时间戳和 IMU 时间戳都准确，并且表示同一物理时钟下的时间。但真实系统中常见问题包括：

- 相机驱动延迟；
- IMU 和相机使用不同硬件时钟；
- ROS 时间戳不是曝光中点；
- 图像传输和处理带来延迟；
- 多线程系统中消息到达顺序不等于采样顺序。

如果相机和 IMU 存在时间偏移，视觉观测实际发生的时刻不是名义时间 $t$，而是：

$$
t_c^{\text{true}}
=
t_c + t_d
$$

其中：

- $t_c$ 表示图像消息给出的相机时间戳；
- $t_c^{\text{true}}$ 表示图像真实曝光对应的物理时刻；
- $t_d$ 表示相机相对于 IMU 的时间偏移。

如果 $t_d$ 没有正确处理，系统会把某一时刻的图像观测错误地关联到另一时刻的 IMU 状态。运动越快，误差越明显。

## 9. 时间偏移如何进入视觉观测模型

设某个特征点在图像中的归一化坐标为：

$$
\mathbf{u}(t)
=
\begin{bmatrix}
u(t)\\
v(t)
\end{bmatrix}
$$

其中：

- $\mathbf{u}(t)$ 表示时刻 $t$ 的二维归一化观测；
- $u(t)$ 和 $v(t)$ 表示横纵坐标。

如果图像时间需要修正 $t_d$，可以用一阶近似：

$$
\mathbf{u}(t+t_d)
\approx
\mathbf{u}(t)
+
\dot{\mathbf{u}}(t)t_d
$$

其中：

- $\dot{\mathbf{u}}(t)$ 表示特征点在归一化平面上的速度；
- $t_d$ 表示时间偏移。

如果跟踪器提供图像平面速度：

$$
\dot{\mathbf{u}}
=
\begin{bmatrix}
\dot{u}\\
\dot{v}
\end{bmatrix}
$$

则时间修正后的观测可以写成：

$$
\mathbf{u}^{\text{corr}}
=
\mathbf{u}
+
t_d\dot{\mathbf{u}}
$$

其中：

- $\mathbf{u}^{\text{corr}}$ 表示时间修正后的特征观测；
- $\mathbf{u}$ 表示原始特征观测。

这样，视觉重投影残差不再只是：

$$
\mathbf{r}_{\text{proj}}
=
\pi(\mathbf{P}_c)-\mathbf{u}
$$

而变成：

$$
\mathbf{r}_{\text{proj}}
=
\pi(\mathbf{P}_c)-\left(\mathbf{u}+t_d\dot{\mathbf{u}}\right)
$$

其中：

- $\pi(\mathbf{P}_c)$ 表示把相机坐标系下三维点 $\mathbf{P}_c$ 投影到归一化平面；
- $\mathbf{P}_c$ 表示特征点在相机坐标系下的三维坐标。

这意味着 $t_d$ 可以作为一个优化变量加入因子图。

## 10. 时间偏移的雅克比直觉

如果残差为：

$$
\mathbf{r}
=
\pi(\mathbf{P}_c)-\mathbf{u}-t_d\dot{\mathbf{u}}
$$

则它对时间偏移 $t_d$ 的雅克比为：

$$
\frac{\partial\mathbf{r}}{\partial t_d}
=
-\dot{\mathbf{u}}
$$

这个公式的含义很直观：如果特征点在图像上运动得很快，那么一点点时间偏移就会造成明显的观测误差；如果特征点几乎不动，那么时间偏移不容易被估计出来。

因此，时间偏移估计需要足够的图像运动激励。纯静止或非常慢的运动，很难可靠估计 $t_d$。

## 11. 时间同步和初始化的关系

时间偏移会影响初始化。原因是视觉相对运动和 IMU 预积分必须对应同一段真实时间。

如果图像时刻偏移为 $t_d$，那么名义上的帧间隔：

$$
\Delta t_{ij}
=
t_j-t_i
$$

不一定等于真实视觉曝光间隔和 IMU 积分区间的对应关系。轻微偏移在低速运动下可能影响不大，但在快速旋转或快速平移时，会导致：

- 视觉相对旋转和 IMU 相对旋转不一致；
- 陀螺仪零偏估计被污染；
- 视觉-惯性对齐得到错误尺度；
- 重力方向估计偏斜；
- 后续滑窗优化需要用错误初值起步。

因此，高质量 VIO 系统通常要么保证硬件同步，要么在线估计时间偏移。

## 12. 初始化失败的典型原因

初始化不是总能成功。常见失败原因包括：

第一，运动激励不足。如果设备几乎静止，视觉结构、尺度和速度都难以可靠估计。

第二，纯旋转或平移太小。纯旋转可以帮助估计旋转，但很难三角化稳定的特征点，也无法恢复尺度。

第三，特征匹配质量差。动态物体、低纹理、运动模糊都会破坏视觉 SFM。

第四，外参初值差。相机和 IMU 之间的坐标关系错误，会导致视觉和惯性约束难以对齐。

第五，时间偏移大。视觉观测和 IMU 预积分对应不上同一段运动。

第六，IMU 噪声或零偏过大。预积分量不可靠，会污染对齐结果。

这些失败原因背后的共同点是：初始化需要从短时间观测中同时恢复多个强耦合量，因此对运动、标定和数据质量都比较敏感。

## 13. 初始化之后发生什么

初始化成功后，系统会得到：

$$
\left\{
\mathbf{p}_k,\mathbf{q}_k,\mathbf{v}_k,\mathbf{b}_{a,k},\mathbf{b}_{g,k},\lambda_l,\mathbf{T}_{ic},t_d
\right\}
$$

其中：

- $\mathbf{p}_k$ 表示位置；
- $\mathbf{q}_k$ 表示姿态；
- $\mathbf{v}_k$ 表示速度；
- $\mathbf{b}_{a,k}$ 表示加速度计零偏；
- $\mathbf{b}_{g,k}$ 表示陀螺仪零偏；
- $\lambda_l$ 表示特征点逆深度；
- $\mathbf{T}_{ic}$ 表示外参；
- $t_d$ 表示时间偏移。

这些量会作为滑动窗口非线性优化的初始值。之后系统开始进入正常 VIO 模式：

$$
\text{新图像和 IMU}
\rightarrow
\text{构造因子}
\rightarrow
\text{滑窗优化}
\rightarrow
\text{边缘化旧变量}
\rightarrow
\text{继续运行}
$$

也就是说，初始化负责把系统带到一个合理的 basin of attraction，也就是非线性优化能够收敛到正确解附近的吸引域。

## 14. 和后续边缘化、FEJ 的关系

初始化、时间同步、边缘化和 FEJ 之间并不是孤立的。

初始化给滑窗优化提供初值。如果初值很差，边缘化会把错误的线性化信息压缩成先验，后续很难完全修正。

时间同步影响视觉残差。如果 $t_d$ 错误，视觉因子会系统性偏差，边缘化先验也会包含这种偏差。

FEJ 固定边缘化先验的线性化结构。如果 first estimate 本身很差，FEJ 虽然保持了一致性结构，但不能把错误初值变成正确初值。

所以，完整 VIO 系统的逻辑应当是：

$$
\text{可靠初始化}
\rightarrow
\text{正确时间建模}
\rightarrow
\text{稳定滑窗优化}
\rightarrow
\text{合理边缘化}
\rightarrow
\text{一致性保持}
$$

## 15. 最后一条直觉

初始化回答的是：

$$
\text{优化从哪里开始？}
$$

时间同步回答的是：

$$
\text{视觉和 IMU 是否在描述同一时刻的运动？}
$$

如果这两个问题没有处理好，后面的因子图、边缘化和 FEJ 都是在一个错误的问题上做精细优化。VIO 的困难不只在求解器，也在于一开始能否把视觉几何、惯性物理、外参标定和时间关系放到同一个一致的坐标和时间框架里。
