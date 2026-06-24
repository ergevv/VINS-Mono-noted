# 03 IMU 预积分：把高频惯性测量压缩成帧间约束

IMU 预积分是 VINS-Mono 的核心。它解决的问题是：

$$
\text{如何把两帧图像之间大量 IMU 数据，压缩成一个相邻帧约束？}
$$

如果没有预积分，后端每次优化第 $i$ 帧的姿态、速度或 bias 后，都要重新积分第 $i$ 帧到第 $j$ 帧之间所有 IMU 数据。预积分的目标是把这段 IMU 信息整理成相对量：

$$
\boldsymbol{\alpha}_{ij},\quad
\boldsymbol{\beta}_{ij},\quad
\boldsymbol{\gamma}_{ij}
$$

其中：

- $\boldsymbol{\alpha}_{ij}$ 表示从第 $i$ 帧到第 $j$ 帧的相对位移预积分；
- $\boldsymbol{\beta}_{ij}$ 表示从第 $i$ 帧到第 $j$ 帧的相对速度预积分；
- $\boldsymbol{\gamma}_{ij}$ 表示从第 $i$ 帧到第 $j$ 帧的相对旋转预积分。

它们都表达在第 $i$ 帧 IMU 坐标系下，因此可以尽量和世界系位置、速度、姿态解耦。

## 1. 坐标和 IMU 测量模型

设世界坐标系为 $w$，IMU 机体系为 $b$。第 $t$ 时刻 IMU 状态为：

$$
\mathbf{x}_t =
\left[
\mathbf{p}_{w b_t},\,
\mathbf{v}_{w b_t},\,
\mathbf{R}_{w b_t},\,
\mathbf{b}_{a,t},\,
\mathbf{b}_{g,t}
\right]
$$

其中：

- $\mathbf{p}_{w b_t}$ 表示 IMU 在世界系中的位置；
- $\mathbf{v}_{w b_t}$ 表示 IMU 在世界系中的速度；
- $\mathbf{R}_{w b_t}$ 表示从 IMU 机体系到世界系的旋转；
- $\mathbf{b}_{a,t}$ 表示加速度计 bias；
- $\mathbf{b}_{g,t}$ 表示陀螺仪 bias。

IMU 测量为：

$$
\hat{\mathbf{a}}_t =
\mathbf{a}_t^b +
\mathbf{b}_{a,t} +
\mathbf{n}_{a,t}
$$

$$
\hat{\boldsymbol{\omega}}_t =
\boldsymbol{\omega}_t +
\mathbf{b}_{g,t} +
\mathbf{n}_{g,t}
$$

其中：

- $\hat{\mathbf{a}}_t$ 表示加速度计测量；
- $\mathbf{a}_t^b$ 表示机体系下真实比力；
- $\mathbf{n}_{a,t}$ 表示加速度计白噪声；
- $\hat{\boldsymbol{\omega}}_t$ 表示陀螺仪测量；
- $\boldsymbol{\omega}_t$ 表示真实角速度；
- $\mathbf{n}_{g,t}$ 表示陀螺仪白噪声。

去 bias 和噪声后：

$$
\mathbf{a}_t^b =
\hat{\mathbf{a}}_t-\mathbf{b}_{a,t}-\mathbf{n}_{a,t}
$$

$$
\boldsymbol{\omega}_t =
\hat{\boldsymbol{\omega}}_t-\mathbf{b}_{g,t}-\mathbf{n}_{g,t}
$$

连续运动方程为：

$$
\dot{\mathbf{p}}_{w b_t} =
\mathbf{v}_{w b_t}
$$

$$
\dot{\mathbf{v}}_{w b_t} =
\mathbf{R}_{w b_t}
\left(
\hat{\mathbf{a}}_t-\mathbf{b}_{a,t}-\mathbf{n}_{a,t}
\right) +
\mathbf{g}^{w}
$$

$$
\dot{\mathbf{R}}_{w b_t} =
\mathbf{R}_{w b_t}
\left(
\hat{\boldsymbol{\omega}}_t-\mathbf{b}_{g,t}-\mathbf{n}_{g,t}
\right)^{\wedge}
$$

其中：

- $\mathbf{g}^{w}$ 表示世界系下的重力向量；
- $(\cdot)^\wedge$ 表示把三维向量变成反对称矩阵。

## 2. 当前 PVQ 积分和预积分不是一回事

VINS 中有两类 IMU 积分，容易混淆。

第一类是当前状态预测，用来给新图像帧提供初值：

$$
\mathbf{p}_{k+1} =
\mathbf{p}_{k} +
\mathbf{v}_{k}\delta t +
\frac{1}{2}\mathbf{a}^{w}_{mid}\delta t^2
$$

$$
\mathbf{v}_{k+1} =
\mathbf{v}_{k} +
\mathbf{a}^{w}_{mid}\delta t
$$

$$
\mathbf{q}_{k+1} =
\mathbf{q}_{k}
\otimes
\delta\mathbf{q}
\left(
\boldsymbol{\omega}_{mid}\delta t
\right)
$$

其中：

- $\mathbf{a}^{w}_{mid}$ 表示世界系下中值加速度；
- $\boldsymbol{\omega}_{mid}$ 表示中值角速度；
- $\delta t$ 表示两个 IMU 采样之间的时间间隔。

这一类积分依赖当前估计的世界系状态，用于预测最新帧初值。

第二类才是后端优化使用的预积分。它把第 $i$ 帧到第 $j$ 帧之间的 IMU 数据压缩成相对量：

$$
\boldsymbol{\alpha}_{ij},\quad
\boldsymbol{\beta}_{ij},\quad
\boldsymbol{\gamma}_{ij}
$$

后端优化时，IMU 因子只需要这些相对量、协方差和 Jacobian，而不需要每次重新遍历原始 IMU 数据。

## 3. 从连续运动方程推到预积分形式

从运动方程积分得到：

$$
\mathbf{v}_j =
\mathbf{v}_i +
\mathbf{g}^{w}\Delta t_{ij} +
\int_{t_i}^{t_j}
\mathbf{R}_{w b_t}
\left(
\hat{\mathbf{a}}_t-\mathbf{b}_{a,t}-\mathbf{n}_{a,t}
\right)dt
$$

$$
\mathbf{p}_j =
\mathbf{p}_i +
\mathbf{v}_i\Delta t_{ij} +
\frac{1}{2}\mathbf{g}^{w}\Delta t_{ij}^2 +
\int_{t_i}^{t_j}
\int_{t_i}^{s}
\mathbf{R}_{w b_\tau}
\left(
\hat{\mathbf{a}}_\tau-\mathbf{b}_{a,\tau}-\mathbf{n}_{a,\tau}
\right)d\tau ds
$$

其中 $\Delta t_{ij}=t_j-t_i$。

关键步骤是把 $\mathbf{R}_{w b_t}$ 写成：

$$
\mathbf{R}_{w b_t} =
\mathbf{R}_{w b_i}\mathbf{R}_{b_i b_t}
$$

其中：

- $\mathbf{R}_{w b_i}$ 表示第 $i$ 帧 IMU 到世界系的旋转；
- $\mathbf{R}_{b_i b_t}$ 表示从第 $t$ 时刻 IMU 到第 $i$ 帧 IMU 的相对旋转。

将等式左乘 $\mathbf{R}_{w b_i}^{\top}$，定义：

$$
\boldsymbol{\beta}_{ij} =
\int_{t_i}^{t_j}
\mathbf{R}_{b_i b_t}
\left(
\hat{\mathbf{a}}_t-\mathbf{b}_{a,t}-\mathbf{n}_{a,t}
\right)dt
$$

$$
\boldsymbol{\alpha}_{ij} =
\int_{t_i}^{t_j}
\int_{t_i}^{s}
\mathbf{R}_{b_i b_\tau}
\left(
\hat{\mathbf{a}}_\tau-\mathbf{b}_{a,\tau}-\mathbf{n}_{a,\tau}
\right)d\tau ds
$$

$$
\boldsymbol{\gamma}_{ij} =
\mathbf{R}_{b_i b_j}
$$

则得到：

$$
\mathbf{p}_j =
\mathbf{p}_i +
\mathbf{v}_i\Delta t_{ij} +
\frac{1}{2}\mathbf{g}^{w}\Delta t_{ij}^2 +
\mathbf{R}_{w b_i}\boldsymbol{\alpha}_{ij}
$$

$$
\mathbf{v}_j =
\mathbf{v}_i +
\mathbf{g}^{w}\Delta t_{ij} +
\mathbf{R}_{w b_i}\boldsymbol{\beta}_{ij}
$$

$$
\mathbf{R}_{w b_j} =
\mathbf{R}_{w b_i}\boldsymbol{\gamma}_{ij}
$$

这就是预积分的本质：把和起始姿态相关的 $\mathbf{R}_{w b_i}$ 提到积分外面，让积分内部只描述第 $i$ 帧局部坐标系下的相对运动。

## 4. 中值法离散预积分

设 IMU 采样时刻为 $k$ 和 $k+1$，时间间隔为 $\delta t$。预积分状态为：

$$
\boldsymbol{\alpha}_k,\quad
\boldsymbol{\beta}_k,\quad
\boldsymbol{\gamma}_k
$$

初值为：

$$
\boldsymbol{\alpha}_i=\mathbf{0},\quad
\boldsymbol{\beta}_i=\mathbf{0},\quad
\boldsymbol{\gamma}_i=\mathbf{I}
$$

去 bias 后的中值角速度：

$$
\bar{\boldsymbol{\omega}}_k =
\frac{1}{2}
\left(
\hat{\boldsymbol{\omega}}_k+
\hat{\boldsymbol{\omega}}_{k+1}
\right) -
\mathbf{b}_g
$$

旋转增量：

$$
\delta\boldsymbol{\gamma}_k =
\exp
\left(
\bar{\boldsymbol{\omega}}_k\delta t
\right)
$$

旋转预积分更新：

$$
\boldsymbol{\gamma}_{k+1} =
\boldsymbol{\gamma}_k
\delta\boldsymbol{\gamma}_k
$$

第 $k$ 时刻和 $k+1$ 时刻的局部加速度：

$$
\mathbf{a}^{i}_k =
\boldsymbol{\gamma}_k
\left(
\hat{\mathbf{a}}_k-\mathbf{b}_a
\right)
$$

$$
\mathbf{a}^{i}_{k+1} =
\boldsymbol{\gamma}_{k+1}
\left(
\hat{\mathbf{a}}_{k+1}-\mathbf{b}_a
\right)
$$

中值加速度：

$$
\bar{\mathbf{a}}^i_k =
\frac{1}{2}
\left(
\mathbf{a}^{i}_k+\mathbf{a}^{i}_{k+1}
\right)
$$

位移和速度预积分更新：

$$
\boldsymbol{\alpha}_{k+1} =
\boldsymbol{\alpha}_{k} +
\boldsymbol{\beta}_{k}\delta t +
\frac{1}{2}
\bar{\mathbf{a}}^i_k\delta t^2
$$

$$
\boldsymbol{\beta}_{k+1} =
\boldsymbol{\beta}_{k} +
\bar{\mathbf{a}}^i_k\delta t
$$

这组公式就是中值法预积分的核心。

## 5. 为什么 bias 改变时不用重积分

预积分是在某个 bias 线性化点上计算的，记为：

$$
\bar{\mathbf{b}}_a,\quad
\bar{\mathbf{b}}_g
$$

优化过程中 bias 会变成：

$$
\mathbf{b}_a =
\bar{\mathbf{b}}_a+\delta\mathbf{b}_a
$$

$$
\mathbf{b}_g =
\bar{\mathbf{b}}_g+\delta\mathbf{b}_g
$$

如果每次 bias 改变都重新积分，代价很大。于是对预积分量做一阶近似：

$$
\boldsymbol{\alpha}_{ij} \approx
\hat{\boldsymbol{\alpha}}_{ij} +
\mathbf{J}^{\alpha}_{b_a}\delta\mathbf{b}_a +
\mathbf{J}^{\alpha}_{b_g}\delta\mathbf{b}_g
$$

$$
\boldsymbol{\beta}_{ij} \approx
\hat{\boldsymbol{\beta}}_{ij} +
\mathbf{J}^{\beta}_{b_a}\delta\mathbf{b}_a +
\mathbf{J}^{\beta}_{b_g}\delta\mathbf{b}_g
$$

$$
\boldsymbol{\gamma}_{ij} \approx
\hat{\boldsymbol{\gamma}}_{ij}
\exp
\left(
\mathbf{J}^{\gamma}_{b_g}\delta\mathbf{b}_g
\right)
$$

其中：

- 带帽子的量表示在 bias 线性化点处积分得到的预积分量；
- $\mathbf{J}^{\alpha}_{b_a}$ 表示 $\boldsymbol{\alpha}$ 对加速度计 bias 的 Jacobian；
- $\mathbf{J}^{\gamma}_{b_g}$ 表示旋转预积分对陀螺仪 bias 的 Jacobian。

这就是 bias 一阶修正。

## 6. 误差状态传播

预积分误差状态写成：

$$
\delta\mathbf{z} =
\begin{bmatrix}
\delta\boldsymbol{\alpha}\\
\delta\boldsymbol{\theta}\\
\delta\boldsymbol{\beta}\\
\delta\mathbf{b}_a\\
\delta\mathbf{b}_g
\end{bmatrix}
$$

其中：

- $\delta\boldsymbol{\alpha}$ 表示位移预积分误差；
- $\delta\boldsymbol{\theta}$ 表示旋转预积分误差；
- $\delta\boldsymbol{\beta}$ 表示速度预积分误差；
- $\delta\mathbf{b}_a$ 和 $\delta\mathbf{b}_g$ 表示 bias 误差。

它的离散传播可以写成：

$$
\delta\mathbf{z}_{k+1} =
\mathbf{F}_k\delta\mathbf{z}_k +
\mathbf{V}_k\mathbf{n}_k
$$

其中：

- $\mathbf{F}_k$ 是误差状态转移矩阵；
- $\mathbf{V}_k$ 是噪声输入矩阵；
- $\mathbf{n}_k$ 是 IMU 噪声向量。

为了看清楚 $\mathbf{F}_k$ 的结构，可以写成块矩阵：

$$
\mathbf{F}_k =
\begin{bmatrix}
\mathbf{I} & \mathbf{F}_{\alpha\theta} & \mathbf{I}\delta t & \mathbf{F}_{\alpha b_a} & \mathbf{F}_{\alpha b_g}\\
\mathbf{0} & \mathbf{F}_{\theta\theta} & \mathbf{0} & \mathbf{0} & \mathbf{F}_{\theta b_g}\\
\mathbf{0} & \mathbf{F}_{\beta\theta} & \mathbf{I} & \mathbf{F}_{\beta b_a} & \mathbf{F}_{\beta b_g}\\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{I} & \mathbf{0}\\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{I}
\end{bmatrix}
$$

这些块的含义比具体表达式更重要：

- $\mathbf{F}_{\alpha\theta}$ 表示姿态误差如何影响位移预积分；
- $\mathbf{F}_{\alpha b_a}$ 表示加速度计 bias 误差如何影响位移预积分；
- $\mathbf{F}_{\theta b_g}$ 表示陀螺仪 bias 误差如何影响旋转预积分；
- $\mathbf{F}_{\beta\theta}$ 表示姿态误差如何影响速度预积分。

例如，加速度项里有：

$$
\boldsymbol{\gamma}_k
\left(
\hat{\mathbf{a}}_k-\mathbf{b}_a
\right)
$$

当旋转有小扰动 $\delta\boldsymbol{\theta}$ 时，有近似：

$$
\boldsymbol{\gamma}_k\exp(\delta\boldsymbol{\theta}^{\wedge})
\left(
\hat{\mathbf{a}}_k-\mathbf{b}_a
\right) \approx
\boldsymbol{\gamma}_k
\left(
\hat{\mathbf{a}}_k-\mathbf{b}_a
\right) -
\boldsymbol{\gamma}_k
\left(
\hat{\mathbf{a}}_k-\mathbf{b}_a
\right)^{\wedge}
\delta\boldsymbol{\theta}
$$

这就是 $\mathbf{F}_{\alpha\theta}$ 和 $\mathbf{F}_{\beta\theta}$ 中出现反对称矩阵的原因。

## 7. Jacobian 和协方差传播

误差传播有两个用途。

第一，传播预积分量对 bias 的 Jacobian：

$$
\mathbf{J}_{k+1} =
\mathbf{F}_k\mathbf{J}_k
$$

初值为：

$$
\mathbf{J}_i=\mathbf{I}
$$

最终从 $\mathbf{J}_{ij}$ 的不同块中取出：

$$
\mathbf{J}^{\alpha}_{b_a},\quad
\mathbf{J}^{\alpha}_{b_g},\quad
\mathbf{J}^{\beta}_{b_a},\quad
\mathbf{J}^{\beta}_{b_g},\quad
\mathbf{J}^{\gamma}_{b_g}
$$

第二，传播预积分协方差：

$$
\mathbf{P}_{k+1} =
\mathbf{F}_k\mathbf{P}_k\mathbf{F}_k^\top +
\mathbf{V}_k\mathbf{Q}_k\mathbf{V}_k^\top
$$

初值为：

$$
\mathbf{P}_i=\mathbf{0}
$$

其中：

- $\mathbf{P}_k$ 表示预积分误差协方差；
- $\mathbf{Q}_k$ 表示 IMU 噪声协方差；
- $\mathbf{V}_k\mathbf{Q}_k\mathbf{V}_k^\top$ 表示本次 IMU 噪声注入的协方差。

协方差越大，说明这段预积分越不可信。后端优化时会使用信息矩阵：

$$
\mathbf{P}_{ij}^{-1}
$$

来给 IMU 残差加权。

## 8. IMU 残差

预积分最终在后端形成 15 维残差：

$$
\mathbf{r}_{imu} =
\begin{bmatrix}
\mathbf{r}_p\\
\mathbf{r}_q\\
\mathbf{r}_v\\
\mathbf{r}_{ba}\\
\mathbf{r}_{bg}
\end{bmatrix}
$$

其中：

$$
\mathbf{r}_p =
\mathbf{R}_i^\top
\left(
\mathbf{p}_j-\mathbf{p}_i-\mathbf{v}_i\Delta t_{ij}
-\frac{1}{2}\mathbf{g}\Delta t_{ij}^2
\right) -
\boldsymbol{\alpha}_{ij}
$$

$$
\mathbf{r}_q =
2\,\text{vec}
\left(
\boldsymbol{\gamma}_{ij}^{-1}
\otimes
\mathbf{q}_i^{-1}
\otimes
\mathbf{q}_j
\right)
$$

$$
\mathbf{r}_v =
\mathbf{R}_i^\top
\left(
\mathbf{v}_j-\mathbf{v}_i-\mathbf{g}\Delta t_{ij}
\right) -
\boldsymbol{\beta}_{ij}
$$

$$
\mathbf{r}_{ba} =
\mathbf{b}_{a,j}-\mathbf{b}_{a,i}
$$

$$
\mathbf{r}_{bg} =
\mathbf{b}_{g,j}-\mathbf{b}_{g,i}
$$

如果考虑 bias 一阶修正，则这里的 $\boldsymbol{\alpha}_{ij}$、$\boldsymbol{\beta}_{ij}$、$\boldsymbol{\gamma}_{ij}$ 应使用修正后的预积分量。

## 9. 为什么 IMU 残差要乘 sqrt information

IMU 预积分残差的理论代价是马氏距离：

$$
\mathbf{r}_{imu}^{\top}
\mathbf{P}_{ij}^{-1}
\mathbf{r}_{imu}
$$

设：

$$
\mathbf{P}_{ij}^{-1} =
\mathbf{L}\mathbf{L}^{\top}
$$

则：

$$
\mathbf{r}_{imu}^{\top}
\mathbf{P}_{ij}^{-1}
\mathbf{r}_{imu} =
\left(
\mathbf{L}^{\top}\mathbf{r}_{imu}
\right)^{\top}
\left(
\mathbf{L}^{\top}\mathbf{r}_{imu}
\right)
$$

所以优化器中实际使用的残差可以写成：

$$
\tilde{\mathbf{r}}_{imu} =
\mathbf{L}^{\top}\mathbf{r}_{imu}
$$

这就是信息矩阵开方加权。

