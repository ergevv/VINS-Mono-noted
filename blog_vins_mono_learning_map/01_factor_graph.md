# 01 从状态估计到因子图：VIO 为什么是一张图

## 1. 我们到底想估计什么

视觉惯性里程计，通常记作 VIO，即 Visual-Inertial Odometry。它的目标是在没有全局定位传感器的情况下，仅依靠相机和 IMU，估计载体在时间上的运动轨迹。

假设系统在离散时刻 $k=0,1,\dots,N$ 维护一组状态。这里 $k$ 表示第 $k$ 个关键时刻，通常可以理解为第 $k$ 个图像帧或关键帧。一个典型 VIO 状态可以写成：

$$
\mathbf{x}_k =
\left[
\mathbf{p}_k,\,
\mathbf{q}_k,\,
\mathbf{v}_k,\,
\mathbf{b}_{a,k},\,
\mathbf{b}_{g,k}
\right]
$$

其中：

- $\mathbf{x}_k$ 表示第 $k$ 个时刻的完整状态；
- $\mathbf{p}_k \in \mathbb{R}^3$ 表示 IMU 坐标系原点在世界坐标系下的位置；
- $\mathbf{q}_k \in SO(3)$ 或单位四元数表示 IMU 坐标系相对于世界坐标系的姿态；
- $\mathbf{v}_k \in \mathbb{R}^3$ 表示 IMU 在世界坐标系下的速度；
- $\mathbf{b}_{a,k} \in \mathbb{R}^3$ 表示加速度计零偏；
- $\mathbf{b}_{g,k} \in \mathbb{R}^3$ 表示陀螺仪零偏。

除了每一帧的运动状态，VIO 还可能估计相机和 IMU 之间的外参：

$$
\mathbf{T}_{ic} =
\left[
\mathbf{p}_{ic},\,
\mathbf{q}_{ic}
\right]
$$

其中：

- $\mathbf{T}_{ic}$ 表示从相机坐标系到 IMU 坐标系的刚体变换；
- $\mathbf{p}_{ic} \in \mathbb{R}^3$ 表示相机原点在 IMU 坐标系下的位置；
- $\mathbf{q}_{ic}$ 表示相机坐标系相对于 IMU 坐标系的姿态。

单目视觉还需要估计特征点的深度。常见做法不是直接估计深度 $d_l$，而是估计逆深度：

$$
\lambda_l = \frac{1}{d_l}
$$

其中：

- $l$ 表示第 $l$ 个特征点；
- $d_l$ 表示该特征点在其首次观测相机坐标系下的深度；
- $\lambda_l$ 表示该特征点的逆深度。

使用逆深度有两个重要好处。第一，远处点的深度趋向无穷大时，逆深度趋向 $0$，数值上更稳定。第二，单目系统在初始化阶段深度不确定性很大，逆深度通常比直接深度更适合优化。

于是，一个滑动窗口中的整体未知量可以抽象为：

$$
\mathcal{X}
=
\left\{
\mathbf{x}_0,\mathbf{x}_1,\dots,\mathbf{x}_N,\,
\lambda_1,\lambda_2,\dots,\lambda_M,\,
\mathbf{T}_{ic}
\right\}
$$

其中：

- $\mathcal{X}$ 表示当前优化问题中的全部未知变量；
- $N$ 表示窗口内状态的最后一个时刻索引；
- $M$ 表示当前窗口内参与优化的特征点数量。

VIO 的核心问题就是：给定 IMU 测量和图像特征观测，估计最可能的 $\mathcal{X}$。

## 2. 为什么估计问题可以写成最大后验

传感器测量有噪声，所以我们不能要求状态完全满足所有观测。更合理的说法是：寻找一组状态，使它在概率意义下最能解释已经发生的测量。

记所有测量为 $\mathcal{Z}$，包括 IMU 测量和视觉观测。我们想求：

$$
\mathcal{X}^{*}
=
\arg\max_{\mathcal{X}} p(\mathcal{X}\mid \mathcal{Z})
$$

其中：

- $\mathcal{X}^{*}$ 表示最优状态估计；
- $p(\mathcal{X}\mid \mathcal{Z})$ 表示在观测 $\mathcal{Z}$ 已知时，状态 $\mathcal{X}$ 的后验概率；
- $\arg\max$ 表示寻找使后验概率最大的变量取值。

根据贝叶斯公式：

$$
p(\mathcal{X}\mid \mathcal{Z})
=
\frac{p(\mathcal{Z}\mid \mathcal{X})p(\mathcal{X})}{p(\mathcal{Z})}
$$

其中：

- $p(\mathcal{Z}\mid \mathcal{X})$ 是似然，表示给定状态后产生这些测量的概率；
- $p(\mathcal{X})$ 是先验，表示没有看到当前测量前对状态的已有认知；
- $p(\mathcal{Z})$ 是观测本身出现的概率，它不依赖待优化变量 $\mathcal{X}$。

因为 $p(\mathcal{Z})$ 对优化变量是常数，所以最大化后验等价于：

$$
\mathcal{X}^{*}
=
\arg\max_{\mathcal{X}}
p(\mathcal{Z}\mid \mathcal{X})p(\mathcal{X})
$$

为了把乘法变成加法，通常取负对数：

先取对数。因为 $\log(\cdot)$ 是单调递增函数，所以最大值的位置不会改变：

$$
\arg\max_{\mathcal{X}}
p(\mathcal{Z}\mid \mathcal{X})p(\mathcal{X})
=
\arg\max_{\mathcal{X}}
\log
\left(
p(\mathcal{Z}\mid \mathcal{X})p(\mathcal{X})
\right)
$$

利用对数乘法展开公式：

$$
\log(ab)=\log a+\log b
$$

可以得到：

$$
\arg\max_{\mathcal{X}}
\log
\left(
p(\mathcal{Z}\mid \mathcal{X})p(\mathcal{X})
\right)
=
\arg\max_{\mathcal{X}}
\left[
\log p(\mathcal{Z}\mid \mathcal{X})
+
\log p(\mathcal{X})
\right]
$$

最大化一个函数，等价于最小化它的相反数：

$$
\arg\max_{\mathcal{X}} f(\mathcal{X})
=
\arg\min_{\mathcal{X}} -f(\mathcal{X})
$$

因此：

$$
\mathcal{X}^{*}
=
\arg\min_{\mathcal{X}}
-\log p(\mathcal{Z}\mid \mathcal{X})
-\log p(\mathcal{X})
$$

如果测量噪声服从高斯分布：

$$
\mathbf{n} \sim \mathcal{N}(\mathbf{0},\mathbf{\Sigma})
$$

其中：

- $\mathbf{n}$ 表示测量噪声；
- $\mathcal{N}$ 表示高斯分布；
- $\mathbf{0}$ 表示均值为零；
- $\mathbf{\Sigma}$ 表示协方差矩阵。

那么一个残差 $\mathbf{r}(\mathcal{X})$ 对应的负对数似然可以写成：

$$
-\log p(\mathbf{r}(\mathcal{X}))
\equiv
\frac{1}{2}
\mathbf{r}(\mathcal{X})^\top
\mathbf{\Sigma}^{-1}
\mathbf{r}(\mathcal{X})
$$

为了看清楚这个式子从哪里来，先写测量模型：

$$
\mathbf{z}
=
h(\mathcal{X})+\mathbf{n}
$$

其中：

- $\mathbf{z}$ 表示真实传感器测量；
- $h(\mathcal{X})$ 表示由状态 $\mathcal{X}$ 预测出来的测量；
- $\mathbf{n}$ 表示测量噪声。

定义残差：

$$
\mathbf{r}(\mathcal{X})
=
h(\mathcal{X})-\mathbf{z}
$$

如果噪声是零均值高斯噪声，则残差也可以按同样的协方差建模。设残差维度为 $d$，多元高斯概率密度为：

$$
p(\mathbf{r})
=
\frac{1}
{\sqrt{(2\pi)^d|\mathbf{\Sigma}|}}
\exp
\left(
-\frac{1}{2}
\mathbf{r}^{\top}
\mathbf{\Sigma}^{-1}
\mathbf{r}
\right)
$$

其中：

- $d$ 表示残差向量的维度；
- $|\mathbf{\Sigma}|$ 表示协方差矩阵 $\mathbf{\Sigma}$ 的行列式；
- $\exp(\cdot)$ 表示自然指数函数。

对这个概率密度取负对数：

$$
-\log p(\mathbf{r})
=
-\log
\left[
\frac{1}
{\sqrt{(2\pi)^d|\mathbf{\Sigma}|}}
\exp
\left(
-\frac{1}{2}
\mathbf{r}^{\top}
\mathbf{\Sigma}^{-1}
\mathbf{r}
\right)
\right]
$$

展开后得到：

$$
-\log p(\mathbf{r})
=
\frac{1}{2}
\mathbf{r}^{\top}
\mathbf{\Sigma}^{-1}
\mathbf{r}
+
\frac{1}{2}
\log
\left(
(2\pi)^d|\mathbf{\Sigma}|
\right)
$$

如果协方差 $\mathbf{\Sigma}$ 已知，那么：

$$
\frac{1}{2}
\log
\left(
(2\pi)^d|\mathbf{\Sigma}|
\right)
$$

对优化变量 $\mathcal{X}$ 是常数，不影响最优解的位置。因此在优化中可以忽略这个常数项，只保留与 $\mathcal{X}$ 有关的二次项：

$$
\frac{1}{2}
\mathbf{r}(\mathcal{X})^\top
\mathbf{\Sigma}^{-1}
\mathbf{r}(\mathcal{X})
$$

其中：

- $\mathbf{r}(\mathcal{X})$ 表示由状态 $\mathcal{X}$ 预测出来的测量与真实测量之间的误差；
- $\mathbf{\Sigma}^{-1}$ 是信息矩阵，协方差越小，信息越大；
- 上标 $\top$ 表示矩阵或向量转置。

因此，VIO 最终变成一个加权最小二乘问题：

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

- $i$ 表示第 $i$ 个约束或第 $i$ 个因子；
- $\mathbf{r}_i(\mathcal{X})$ 表示第 $i$ 个约束的残差；
- $\mathbf{\Sigma}_i$ 表示第 $i$ 个约束的噪声协方差；
- $\|\mathbf{r}\|^2_{\mathbf{\Sigma}^{-1}}$ 表示马氏距离，定义为：

$$
\left\|\mathbf{r}\right\|^2_{\mathbf{\Sigma}^{-1}}
=
\mathbf{r}^{\top}\mathbf{\Sigma}^{-1}\mathbf{r}
$$

这一步非常关键：因子图不是凭空出现的工程形式，而是最大后验估计在高斯噪声假设下的自然结果。

## 3. 什么是因子图

因子图是一种表达优化问题稀疏结构的图模型。它包含两类节点：

- 变量节点：表示未知量，例如位姿、速度、零偏、特征点逆深度、外参；
- 因子节点：表示约束，例如 IMU 预积分约束、视觉重投影约束、先验约束。

如果一个因子依赖某些变量，就在图中把这个因子和这些变量连起来。

这里要区分两个容易混用的概念：因子图和图优化。

因子图是问题的表达形式，它回答：

$$
\text{有哪些变量？有哪些约束？每个约束连接哪些变量？}
$$

图优化是求解过程，它回答：

$$
\text{给定这些变量和约束，怎样求出最优状态？}
$$

因此，更准确地说：

$$
\text{因子图}
=
\text{图优化问题的建模方式}
$$

$$
\text{图优化}
=
\text{在因子图上做非线性最小二乘求解}
$$

在 VIO 和 SLAM 语境里，大家经常把因子图优化、图优化、factor graph optimization 混着说，是因为建图和求解通常连续发生。但严格区分时，factor graph 更偏向模型结构，graph optimization 更偏向求解过程。

还要注意，因子图并不要求一定保留历史帧。只要有变量和约束，就可以构成因子图。比如单帧问题也可以写成：

$$
\mathcal{X}
=
\left\{
\mathbf{x}_0,\lambda_1,\lambda_2,\dots,\lambda_M
\right\}
$$

其中：

- $\mathbf{x}_0$ 表示当前帧状态；
- $\lambda_l$ 表示第 $l$ 个特征点的深度或逆深度；
- $M$ 表示当前帧中参与建模的特征点数量。

如果当前帧有先验、深度观测、已知地图点观测或其他传感器约束，就可以形成因子：

$$
f_l(\mathbf{x}_0,\lambda_l)
$$

这仍然是因子图。

不过，是否能构成因子图和是否有足够信息求解，是两个不同问题。以单目视觉为例，单帧观测通常只能提供 bearing，也就是特征点方向。若某个三维点只被当前帧看到，它可以写成：

$$
\mathbf{P}_l
=
d_l
\begin{bmatrix}
u_l\\
v_l\\
1
\end{bmatrix}
$$

其中：

- $\mathbf{P}_l$ 表示第 $l$ 个空间点在当前相机坐标系下的位置；
- $d_l$ 表示该点深度；
- $u_l$ 和 $v_l$ 表示归一化平面坐标。

此时深度 $d_l$ 仍然未知，所以单帧单目图虽然可以被写成因子图，但通常约束不足。滑动窗口 VIO 保留多帧历史，不是因为“有历史帧才叫因子图”，而是因为多帧视觉、IMU 预积分和边缘化先验能提供足够的信息来估计运动、尺度、速度和零偏。

可以这样记：

$$
\text{因子图}
\neq
\text{必须包含历史帧}
$$

$$
\text{滑窗因子图}
=
\text{包含有限历史帧的一类特殊因子图}
$$

对于 VIO，一个典型的因子图可以抽象为：

$$
\mathcal{G}
=
\left(
\mathcal{V},
\mathcal{F}
\right)
$$

其中：

- $\mathcal{G}$ 表示因子图；
- $\mathcal{V}$ 表示变量节点集合；
- $\mathcal{F}$ 表示因子节点集合。

变量集合可以写成：

$$
\mathcal{V}
=
\left\{
\mathbf{x}_k,\lambda_l,\mathbf{T}_{ic}
\right\}
$$

因子集合可以写成：

$$
\mathcal{F}
=
\left\{
f_{\text{imu}},
f_{\text{proj}},
f_{\text{prior}}
\right\}
$$

其中：

- $f_{\text{imu}}$ 表示 IMU 预积分因子；
- $f_{\text{proj}}$ 表示视觉重投影因子；
- $f_{\text{prior}}$ 表示先验因子，通常来自边缘化。

图优化的目标函数可以写成：

$$
\min_{\mathcal{X}}
\left(
\sum_k
\left\|
\mathbf{r}_{\text{imu},k}
\right\|^2_{\mathbf{\Sigma}_{\text{imu},k}^{-1}}
+
\sum_{l}\sum_{(i,j)\in\mathcal{O}_l}
\rho
\left(
\left\|
\mathbf{r}_{\text{proj},l}^{i,j}
\right\|^2_{\mathbf{\Sigma}_{\text{img}}^{-1}}
\right)
+
\left\|
\mathbf{r}_{\text{prior}}
\right\|^2
\right)
$$

其中：

- $\mathbf{r}_{\text{imu},k}$ 表示第 $k$ 和第 $k+1$ 个状态之间的 IMU 残差；
- $\mathbf{\Sigma}_{\text{imu},k}$ 表示该 IMU 残差的协方差；
- $\mathbf{r}_{\text{proj},l}^{i,j}$ 表示第 $l$ 个特征点在第 $i$ 帧和第 $j$ 帧之间形成的视觉重投影残差；
- $\mathcal{O}_l$ 表示能够观测到第 $l$ 个特征点的帧对集合；
- $\mathbf{\Sigma}_{\text{img}}$ 表示图像观测噪声协方差；
- $\rho(\cdot)$ 表示鲁棒核函数，用于降低外点的影响；
- $\mathbf{r}_{\text{prior}}$ 表示历史边缘化留下的先验残差。

## 4. IMU 因子：把连续运动压缩成相邻状态约束

IMU 的原始测量频率通常远高于相机。假设在第 $i$ 帧和第 $j$ 帧之间有大量 IMU 测量。直接把每个 IMU 测量都放进优化会让问题非常大。因此 VIO 通常使用 IMU 预积分，将一段时间内的 IMU 测量压缩成一个相邻关键帧之间的约束。

IMU 测量模型为：

$$
\hat{\mathbf{a}}_t
=
\mathbf{R}_t^\top
\left(
\mathbf{a}_t^w-\mathbf{g}
\right)
+
\mathbf{b}_{a,t}
+
\mathbf{n}_{a,t}
$$

$$
\hat{\boldsymbol{\omega}}_t
=
\boldsymbol{\omega}_t
+
\mathbf{b}_{g,t}
+
\mathbf{n}_{g,t}
$$

其中：

- $t$ 表示连续时间；
- $\hat{\mathbf{a}}_t$ 表示加速度计测量值；
- $\mathbf{a}_t^w=\ddot{\mathbf{p}}_t$ 表示载体在世界坐标系下的真实线加速度；
- $\mathbf{R}_t$ 表示从 IMU 坐标系到世界坐标系的旋转矩阵；
- $\mathbf{g}$ 表示世界坐标系下的重力加速度向量；
- $\mathbf{b}_{a,t}$ 表示加速度计零偏；
- $\mathbf{n}_{a,t}$ 表示加速度计白噪声；
- $\hat{\boldsymbol{\omega}}_t$ 表示陀螺仪测量值；
- $\boldsymbol{\omega}_t$ 表示真实角速度；
- $\mathbf{b}_{g,t}$ 表示陀螺仪零偏；
- $\mathbf{n}_{g,t}$ 表示陀螺仪白噪声。

这里的符号很容易弄反。加速度计测到的不是世界系线加速度 $\mathbf{a}_t^w$ 本身，而是比力，也就是“去掉重力后的加速度”在 IMU 坐标系下的表达：

$$
\mathbf{f}_t
=
\mathbf{R}_t^\top
\left(
\mathbf{a}_t^w-\mathbf{g}
\right)
$$

因此静止时 $\mathbf{a}_t^w=\mathbf{0}$。如果此时 IMU 坐标系和世界坐标系重合，即 $\mathbf{R}_t=\mathbf{I}$，则：

$$
\hat{\mathbf{a}}_t
\approx
-
\mathbf{g}
$$

这正是“加速度计静止时读到重力反方向”的含义。例如若世界系 $z$ 轴向上，重力向量定义为：

$$
\mathbf{g}
=
\begin{bmatrix}
0\\0\\-9.81
\end{bmatrix}
$$

则静止加速度计理想读数为：

$$
-
\mathbf{g}
=
\begin{bmatrix}
0\\0\\9.81
\end{bmatrix}
$$

反过来，运动方程写成：

$$
\mathbf{a}_t^w
=
\mathbf{R}_t
\left(
\hat{\mathbf{a}}_t-\mathbf{b}_{a,t}-\mathbf{n}_{a,t}
\right)
+
\mathbf{g}
$$

所以下面的 IMU 残差中会出现 $-\frac{1}{2}\mathbf{g}\Delta t_{ij}^2$ 和 $-\mathbf{g}\Delta t_{ij}$：它们是在把

$$
\mathbf{p}_j
=
\mathbf{p}_i
+
\mathbf{v}_i\Delta t_{ij}
+
\frac{1}{2}\mathbf{g}\Delta t_{ij}^2
+
\mathbf{R}_i\Delta\mathbf{p}_{ij}
$$

移项到残差左侧后得到的。

预积分给出从时刻 $i$ 到时刻 $j$ 的相对运动量：

$$
\Delta\mathbf{p}_{ij},\quad
\Delta\mathbf{q}_{ij},\quad
\Delta\mathbf{v}_{ij}
$$

其中：

- $\Delta\mathbf{p}_{ij}$ 表示从 $i$ 到 $j$ 的预积分相对位移；
- $\Delta\mathbf{q}_{ij}$ 表示从 $i$ 到 $j$ 的预积分相对旋转；
- $\Delta\mathbf{v}_{ij}$ 表示从 $i$ 到 $j$ 的预积分相对速度变化。

给定两个状态 $\mathbf{x}_i$ 和 $\mathbf{x}_j$，IMU 残差通常写成：

$$
\mathbf{r}_{\text{imu},ij}
=
\begin{bmatrix}
\mathbf{r}_{p,ij} \\
\mathbf{r}_{q,ij} \\
\mathbf{r}_{v,ij} \\
\mathbf{r}_{ba,ij} \\
\mathbf{r}_{bg,ij}
\end{bmatrix}
$$

其中：

$$
\mathbf{r}_{p,ij}
=
\mathbf{R}_i^\top
\left(
\mathbf{p}_j-\mathbf{p}_i-\mathbf{v}_i\Delta t_{ij}
-\frac{1}{2}\mathbf{g}\Delta t_{ij}^2
\right)
-
\Delta\mathbf{p}_{ij}
$$

$$
\mathbf{r}_{q,ij}
=
2\,\text{vec}
\left(
\Delta\mathbf{q}_{ij}^{-1}
\otimes
\mathbf{q}_i^{-1}
\otimes
\mathbf{q}_j
\right)
$$

$$
\mathbf{r}_{v,ij}
=
\mathbf{R}_i^\top
\left(
\mathbf{v}_j-\mathbf{v}_i-\mathbf{g}\Delta t_{ij}
\right)
-
\Delta\mathbf{v}_{ij}
$$

$$
\mathbf{r}_{ba,ij}
=
\mathbf{b}_{a,j}-\mathbf{b}_{a,i}
$$

$$
\mathbf{r}_{bg,ij}
=
\mathbf{b}_{g,j}-\mathbf{b}_{g,i}
$$

这里：

- $\mathbf{R}_i$ 是 $\mathbf{q}_i$ 对应的旋转矩阵；
- $\Delta t_{ij}$ 表示从时刻 $i$ 到时刻 $j$ 的时间间隔；
- $\otimes$ 表示四元数乘法；
- $\text{vec}(\cdot)$ 表示取四元数的虚部向量。

IMU 因子的目的，是让相邻状态之间的相对运动符合 IMU 测到的运动。它解决的问题是：相机帧率较低、单目视觉尺度和短时运动约束弱，而 IMU 能提供高频的运动连续性、尺度和重力方向信息。

## 5. 视觉因子：把同一个空间点的多次观测连起来

视觉测量来自图像中的特征点。假设第 $l$ 个空间点第一次在第 $i$ 帧被观测到，其归一化相机坐标为：

$$
\bar{\mathbf{u}}_{i,l}
=
\begin{bmatrix}
u_{i,l}\\
v_{i,l}\\
1
\end{bmatrix}
$$

其中：

- $\bar{\mathbf{u}}_{i,l}$ 表示第 $l$ 个特征点在第 $i$ 帧中的齐次归一化坐标；
- $u_{i,l}$ 和 $v_{i,l}$ 表示归一化平面上的横纵坐标。

如果该点的逆深度为 $\lambda_l$，则该点在第 $i$ 帧相机坐标系下的三维坐标为：

$$
\mathbf{P}_{c_i,l}
=
\frac{1}{\lambda_l}
\bar{\mathbf{u}}_{i,l}
$$

其中 $\mathbf{P}_{c_i,l}$ 表示特征点在第 $i$ 帧相机坐标系下的三维坐标。

通过外参和位姿，可以把这个点变换到世界坐标系：

$$
\mathbf{P}_{w,l}
=
\mathbf{R}_i
\left(
\mathbf{R}_{ic}\mathbf{P}_{c_i,l}
+
\mathbf{p}_{ic}
\right)
+
\mathbf{p}_i
$$

其中：

- $\mathbf{P}_{w,l}$ 表示第 $l$ 个特征点在世界坐标系下的位置；
- $\mathbf{R}_{ic}$ 表示从相机坐标系到 IMU 坐标系的旋转；
- $\mathbf{p}_{ic}$ 表示相机到 IMU 的平移外参。

再把这个世界点投影到第 $j$ 帧相机坐标系：

$$
\mathbf{P}_{c_j,l}
=
\mathbf{R}_{ic}^{\top}
\left(
\mathbf{R}_j^{\top}
\left(
\mathbf{P}_{w,l}-\mathbf{p}_j
\right)
-
\mathbf{p}_{ic}
\right)
$$

其中 $\mathbf{P}_{c_j,l}$ 表示该点在第 $j$ 帧相机坐标系下的坐标。

设：

$$
\mathbf{P}_{c_j,l}
=
\begin{bmatrix}
X_{j,l}\\
Y_{j,l}\\
Z_{j,l}
\end{bmatrix}
$$

则投影到归一化平面：

$$
\pi(\mathbf{P}_{c_j,l})
=
\begin{bmatrix}
X_{j,l}/Z_{j,l}\\
Y_{j,l}/Z_{j,l}
\end{bmatrix}
$$

如果第 $j$ 帧真实观测为：

$$
\mathbf{z}_{j,l}
=
\begin{bmatrix}
u_{j,l}\\
v_{j,l}
\end{bmatrix}
$$

则视觉重投影残差为：

$$
\mathbf{r}_{\text{proj},l}^{i,j}
=
\pi(\mathbf{P}_{c_j,l})
-
\mathbf{z}_{j,l}
$$

视觉因子的目的，是让同一个空间点在不同帧之间的几何关系一致。它解决的问题是：相机能提供丰富的几何约束，帮助估计轨迹形状、姿态变化和空间结构。

## 6. 鲁棒核：为什么不能完全相信每个视觉匹配

真实视觉系统中会有错误匹配、动态物体、光照变化、遮挡等问题。如果直接最小化所有重投影误差的平方，少量外点就可能严重拉偏结果。

因此通常引入鲁棒核函数：

$$
\rho(s)
$$

其中：

- $s$ 表示平方误差，例如 $s=\|\mathbf{r}\|^2$；
- $\rho(s)$ 表示对平方误差的鲁棒化代价。

普通最小二乘相当于：

$$
\rho(s)=s
$$

鲁棒核的特点是：当 $s$ 较小时，它近似 $s$；当 $s$ 很大时，它增长变慢，从而降低外点影响。

举个简单例子。假设大多数特征点重投影误差在 $1$ 像素以内，但一个误匹配点的误差达到 $50$ 像素。普通平方误差会让该点贡献 $2500$ 的代价，优化器可能为了照顾这个错误点而损害整体轨迹。鲁棒核会压低这个异常点的权重，使系统更关注多数一致的观测。

## 7. 非线性最小二乘：为什么要线性化

VIO 的残差包含旋转、投影、四元数乘法，因此是非线性的。为了求解：

$$
\min_{\mathcal{X}}
\sum_i
\left\|
\mathbf{r}_i(\mathcal{X})
\right\|^2
$$

我们通常在当前估计 $\mathcal{X}_0$ 附近做一阶泰勒展开：

$$
\mathbf{r}_i(\mathcal{X}_0 \boxplus \delta\mathcal{X})
\approx
\mathbf{r}_i(\mathcal{X}_0)
+
\mathbf{J}_i\delta\mathcal{X}
$$

其中：

- $\mathcal{X}_0$ 表示当前线性化点；
- $\delta\mathcal{X}$ 表示状态增量；
- $\boxplus$ 表示流形上的加法操作；
- $\mathbf{J}_i$ 表示第 $i$ 个残差对状态增量的雅克比矩阵。

把所有残差堆叠起来：

$$
\mathbf{r}
\approx
\mathbf{r}_0+\mathbf{J}\delta\mathcal{X}
$$

其中：

- $\mathbf{r}$ 表示总残差向量；
- $\mathbf{r}_0$ 表示当前线性化点处的残差；
- $\mathbf{J}$ 表示总雅克比矩阵。

最小化线性化后的平方误差：

$$
\min_{\delta\mathcal{X}}
\left\|
\mathbf{r}_0+\mathbf{J}\delta\mathcal{X}
\right\|^2
$$

令导数为零，得到正规方程：

$$
\mathbf{H}\delta\mathcal{X}
=
\mathbf{g}
$$

其中：

$$
\mathbf{H}
=
\mathbf{J}^{\top}\mathbf{J}
$$

$$
\mathbf{g}
=
-\mathbf{J}^{\top}\mathbf{r}_0
$$

这里：

- $\mathbf{H}$ 表示 Hessian 的 Gauss-Newton 近似；
- $\mathbf{g}$ 表示负梯度方向；
- $\delta\mathcal{X}$ 表示要求解的状态增量。

有些实现会把右端写成 $\mathbf{b}=\mathbf{J}^{\top}\mathbf{r}_0$，并在更新符号上做相应约定。关键不是正负号的形式，而是它来自同一个二次近似问题。

## 8. 为什么因子图适合 VIO

因子图适合 VIO，有三个根本原因。

第一，VIO 约束天然是局部的。IMU 因子只连接相邻两个时刻，视觉因子只连接观测到同一个特征点的若干相机位姿和该特征点逆深度。这意味着整体矩阵非常稀疏。

第二，VIO 是多传感器融合。IMU、视觉、先验、外参、时间延迟都可以统一成残差项放进同一个目标函数中。

第三，VIO 需要实时运行。因子图不仅表达问题，也暴露了稀疏结构，使 Schur 补、滑动窗口、边缘化等方法有了施展空间。

到这里，我们已经得到一张完整的 VIO 因子图。但新的问题马上出现：如果系统一直运行，状态和特征点会越来越多，优化问题会无限变大。实时系统不能无限保留历史变量。

这就引出下一篇的主题：滑动窗口和边缘化。
