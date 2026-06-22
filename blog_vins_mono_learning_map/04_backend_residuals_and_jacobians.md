# 04 后端残差与雅克比：VINS 优化器到底在最小化什么

前面已经讲过因子图和 IMU 预积分。现在可以进入 VINS-Mono 后端优化的核心问题：

$$
\text{给定 IMU 测量和图像观测，如何同时估计滑窗内的位姿、速度、bias、外参和特征点深度？}
$$

后端优化并不是“凭空调参数”，它做的是一个明确的最大后验估计，也就是：

$$
\text{寻找一组状态，使所有测量残差在统计意义下最小。}
$$

这一篇只讲数学，不讲代码。读完这一篇，你应该能看懂：

- VINS 后端状态向量包含什么；
- 为什么目标函数是多个残差的马氏距离之和；
- 高斯牛顿法如何把非线性优化变成线性增量方程；
- IMU 残差每一项从哪里来；
- 视觉重投影残差每一项从哪里来；
- 雅克比到底是对什么变量求导；
- 为什么残差和雅克比都要乘以 sqrt information。

## 1. 滑窗后端在估计什么

设滑窗中共有 $N+1$ 个关键帧，帧编号为：

$$
0,1,\dots,N
$$

VINS-Mono 后端的状态可以抽象写成：

$$
\mathcal{X}
=
\left[
\mathbf{x}_0,\mathbf{x}_1,\dots,\mathbf{x}_N,\,
\mathbf{x}_{bc},\,
\lambda_1,\lambda_2,\dots,\lambda_M,\,
t_d
\right]
$$

其中：

- $\mathcal{X}$ 表示滑窗优化的全部未知量；
- $\mathbf{x}_k$ 表示第 $k$ 帧 IMU 状态；
- $\mathbf{x}_{bc}$ 表示相机和 IMU 之间的外参；
- $\lambda_l$ 表示第 $l$ 个特征点的逆深度；
- $t_d$ 表示相机时间戳和 IMU 时间戳之间的时间偏移，如果不估计时间偏移，可以从状态中去掉。

单帧 IMU 状态为：

$$
\mathbf{x}_k
=
\left[
\mathbf{p}_k,\,
\mathbf{q}_k,\,
\mathbf{v}_k,\,
\mathbf{b}_{a,k},\,
\mathbf{b}_{g,k}
\right]
$$

其中：

- $\mathbf{p}_k\in\mathbb{R}^3$ 表示第 $k$ 帧 IMU 在世界系中的位置；
- $\mathbf{q}_k$ 表示第 $k$ 帧 IMU 到世界系的姿态四元数；
- $\mathbf{R}_k\in SO(3)$ 表示 $\mathbf{q}_k$ 对应的旋转矩阵；
- $\mathbf{v}_k\in\mathbb{R}^3$ 表示第 $k$ 帧 IMU 在世界系中的速度；
- $\mathbf{b}_{a,k}\in\mathbb{R}^3$ 表示第 $k$ 帧加速度计 bias；
- $\mathbf{b}_{g,k}\in\mathbb{R}^3$ 表示第 $k$ 帧陀螺仪 bias。

外参写成：

$$
\mathbf{x}_{bc}
=
\left[
\mathbf{p}_{bc},\,
\mathbf{q}_{bc}
\right]
$$

其中：

- $\mathbf{p}_{bc}$ 表示相机原点在 IMU 机体系 $b$ 下的位置；
- $\mathbf{q}_{bc}$ 表示从相机坐标系 $c$ 到 IMU 机体系 $b$ 的旋转；
- $\mathbf{R}_{bc}$ 表示 $\mathbf{q}_{bc}$ 对应的旋转矩阵。

因此，一个相机系点 $\mathbf{P}_c$ 可以变换到 IMU 机体系：

$$
\mathbf{P}_b
=
\mathbf{R}_{bc}\mathbf{P}_c
+
\mathbf{p}_{bc}
$$

注意：位姿虽然常用 7 个数存储，也就是三维位置加四元数，但优化时姿态不能用普通加法更新。姿态的局部扰动只有 3 维，所以一个位姿块的局部自由度是：

$$
3 + 3 = 6
$$

这就是为什么 VINS 里常见的 IMU 因子连接的参数块维度是：

$$
7,\quad 9,\quad 7,\quad 9
$$

而对应的局部扰动维度是：

$$
6,\quad 9,\quad 6,\quad 9
$$

## 2. 从最大后验到后端目标函数

后端有三类主要信息：

1. 边缘化留下的历史先验；
2. 相邻关键帧之间的 IMU 预积分约束；
3. 多帧观测同一个特征点产生的视觉重投影约束。

如果把所有测量记为 $\mathcal{Z}$，后端要求的是最大后验估计：

$$
\mathcal{X}^{*}
=
\arg\max_{\mathcal{X}}
p(\mathcal{X}\mid\mathcal{Z})
$$

由贝叶斯公式：

$$
p(\mathcal{X}\mid\mathcal{Z})
\propto
p(\mathcal{Z}\mid\mathcal{X})p(\mathcal{X})
$$

其中：

- $p(\mathcal{Z}\mid\mathcal{X})$ 是给定状态后观测出现的概率；
- $p(\mathcal{X})$ 是先验概率；
- $\propto$ 表示忽略与 $\mathcal{X}$ 无关的归一化常数。

如果每个测量误差都服从零均值高斯分布：

$$
\mathbf{r}_i(\mathcal{X})
\sim
\mathcal{N}(\mathbf{0},\mathbf{\Sigma}_i)
$$

其中：

- $\mathbf{r}_i(\mathcal{X})$ 表示第 $i$ 个残差；
- $\mathbf{\Sigma}_i$ 表示这个残差的协方差；
- $\mathbf{\Omega}_i=\mathbf{\Sigma}_i^{-1}$ 表示信息矩阵。

那么负对数似然可以写成：

$$
-\log p(\mathcal{Z}\mid\mathcal{X})
=
\frac{1}{2}
\sum_i
\mathbf{r}_i(\mathcal{X})^{\top}
\mathbf{\Omega}_i
\mathbf{r}_i(\mathcal{X})
+
\text{const}
$$

去掉常数项和不影响最优解的系数 $\frac{1}{2}$，得到最小二乘问题：

$$
\mathcal{X}^{*}
=
\arg\min_{\mathcal{X}}
\sum_i
\left\|
\mathbf{r}_i(\mathcal{X})
\right\|_{\mathbf{\Omega}_i}^{2}
$$

其中马氏距离定义为：

$$
\left\|
\mathbf{r}
\right\|_{\mathbf{\Omega}}^{2}
=
\mathbf{r}^{\top}\mathbf{\Omega}\mathbf{r}
$$

放到 VINS-Mono 里，整体目标函数可以写成：

$$
\min_{\mathcal{X}}
\left(
\left\|
\mathbf{r}_{p}(\mathcal{X})
\right\|^{2}
+
\sum_{k}
\left\|
\mathbf{r}_{imu,k}(\hat{\mathbf{z}}_{k,k+1},\mathcal{X})
\right\|^{2}_{\mathbf{P}_{imu,k}^{-1}}
+
\sum_{(l,j)}
\rho
\left(
\left\|
\mathbf{r}_{proj,l,j}(\hat{\mathbf{z}}_{l,j},\mathcal{X})
\right\|^{2}_{\mathbf{P}_{img}^{-1}}
\right)
\right)
$$

其中：

- $\mathbf{r}_{p}$ 表示边缘化先验残差；
- $\mathbf{r}_{imu,k}$ 表示第 $k$ 帧到第 $k+1$ 帧的 IMU 残差；
- $\hat{\mathbf{z}}_{k,k+1}$ 表示第 $k$ 帧到第 $k+1$ 帧之间的 IMU 预积分测量；
- $\mathbf{P}_{imu,k}$ 表示 IMU 预积分误差协方差；
- $\mathbf{r}_{proj,l,j}$ 表示第 $l$ 个特征点在第 $j$ 帧上的视觉重投影残差；
- $\hat{\mathbf{z}}_{l,j}$ 表示第 $j$ 帧对第 $l$ 个特征点的图像观测；
- $\mathbf{P}_{img}$ 表示视觉观测噪声协方差；
- $\rho(\cdot)$ 表示鲁棒核函数，用于降低外点对优化的影响。

这个目标函数就是因子图的代数形式：

$$
\text{一个残差项}
\quad\Longleftrightarrow\quad
\text{一个因子}
$$

## 3. 高斯牛顿：从非线性残差到线性增量方程

VINS 的残差通常是非线性的，因为里面有旋转、投影、逆深度、时间偏移等变量。优化器不能直接一次求出 $\mathcal{X}^{*}$，只能迭代求局部增量。

设当前状态估计为 $\bar{\mathcal{X}}$，对状态施加一个小扰动 $\delta\boldsymbol{\chi}$：

$$
\mathcal{X}
=
\bar{\mathcal{X}}\boxplus\delta\boldsymbol{\chi}
$$

其中：

- $\bar{\mathcal{X}}$ 表示当前线性化点；
- $\delta\boldsymbol{\chi}$ 表示当前要求解的局部增量；
- $\boxplus$ 表示流形上的加法。

对第 $i$ 个残差做一阶泰勒展开：

$$
\mathbf{r}_i(\bar{\mathcal{X}}\boxplus\delta\boldsymbol{\chi})
\approx
\mathbf{r}_i(\bar{\mathcal{X}})
+
\mathbf{J}_i\delta\boldsymbol{\chi}
$$

其中：

$$
\mathbf{J}_i
=
\frac{\partial
\mathbf{r}_i(\bar{\mathcal{X}}\boxplus\delta\boldsymbol{\chi})
}
{\partial\delta\boldsymbol{\chi}}
\bigg|_{\delta\boldsymbol{\chi}=\mathbf{0}}
$$

$\mathbf{J}_i$ 是残差对局部扰动的 Jacobian。这里特别重要：Jacobian 不是对四元数四个分量直接求普通偏导，而是对姿态的三维扰动求偏导。

把线性化残差代入加权最小二乘：

$$
\min_{\delta\boldsymbol{\chi}}
\frac{1}{2}
\sum_i
\left(
\mathbf{r}_i+\mathbf{J}_i\delta\boldsymbol{\chi}
\right)^{\top}
\mathbf{\Omega}_i
\left(
\mathbf{r}_i+\mathbf{J}_i\delta\boldsymbol{\chi}
\right)
$$

对 $\delta\boldsymbol{\chi}$ 求导并令导数为零：

$$
\sum_i
\mathbf{J}_i^{\top}\mathbf{\Omega}_i
\left(
\mathbf{r}_i+\mathbf{J}_i\delta\boldsymbol{\chi}
\right)
=
\mathbf{0}
$$

展开得到：

$$
\left(
\sum_i
\mathbf{J}_i^{\top}\mathbf{\Omega}_i\mathbf{J}_i
\right)
\delta\boldsymbol{\chi}
=
-
\sum_i
\mathbf{J}_i^{\top}\mathbf{\Omega}_i\mathbf{r}_i
$$

令：

$$
\mathbf{H}
=
\sum_i
\mathbf{J}_i^{\top}\mathbf{\Omega}_i\mathbf{J}_i
$$

$$
\mathbf{b}
=
-
\sum_i
\mathbf{J}_i^{\top}\mathbf{\Omega}_i\mathbf{r}_i
$$

则得到增量方程：

$$
\mathbf{H}\delta\boldsymbol{\chi}
=
\mathbf{b}
$$

其中：

- $\mathbf{H}$ 是近似 Hessian 矩阵；
- $\mathbf{b}$ 是梯度方向的负号形式；
- $\delta\boldsymbol{\chi}$ 是本轮迭代要求解的增量。

求出 $\delta\boldsymbol{\chi}$ 后，更新状态：

$$
\bar{\mathcal{X}}
\leftarrow
\bar{\mathcal{X}}\boxplus\delta\boldsymbol{\chi}
$$

然后重新计算残差和 Jacobian，进入下一轮迭代。

## 4. IMU 残差：预积分测量和当前状态预测是否一致

IMU 因子连接相邻两帧：

$$
f_{imu,k}
\left(
\mathbf{x}_k,\mathbf{x}_{k+1}
\right)
$$

更具体地说，它连接四个参数块：

$$
\left[
\mathbf{p}_k,\mathbf{q}_k
\right],
\quad
\left[
\mathbf{v}_k,\mathbf{b}_{a,k},\mathbf{b}_{g,k}
\right],
\quad
\left[
\mathbf{p}_{k+1},\mathbf{q}_{k+1}
\right],
\quad
\left[
\mathbf{v}_{k+1},\mathbf{b}_{a,k+1},\mathbf{b}_{g,k+1}
\right]
$$

为了让符号更清楚，下面把相邻两帧记为第 $i$ 帧和第 $j$ 帧。两帧时间间隔为：

$$
\Delta t_{ij}=t_j-t_i
$$

第 03 篇已经推导过预积分关系：

$$
\mathbf{p}_j
=
\mathbf{p}_i
+
\mathbf{v}_i\Delta t_{ij}
+
\frac{1}{2}\mathbf{g}\Delta t_{ij}^{2}
+
\mathbf{R}_i\boldsymbol{\alpha}_{ij}
$$

$$
\mathbf{v}_j
=
\mathbf{v}_i
+
\mathbf{g}\Delta t_{ij}
+
\mathbf{R}_i\boldsymbol{\beta}_{ij}
$$

$$
\mathbf{q}_j
=
\mathbf{q}_i
\otimes
\boldsymbol{\gamma}_{ij}
$$

其中：

- $\mathbf{g}$ 表示世界系重力向量；
- $\boldsymbol{\alpha}_{ij}$ 表示第 $i$ 帧局部系下的位移预积分；
- $\boldsymbol{\beta}_{ij}$ 表示第 $i$ 帧局部系下的速度预积分；
- $\boldsymbol{\gamma}_{ij}$ 表示第 $i$ 帧到第 $j$ 帧的旋转预积分；
- $\otimes$ 表示四元数乘法。

如果当前估计的状态完全正确，那么由状态预测出来的相对运动应该等于 IMU 预积分测量。因此把上面三式移项，就得到残差。

### 4.1 bias 一阶修正后的预积分量

预积分是在某个 bias 线性化点上完成的。设预积分时使用的 bias 为：

$$
\bar{\mathbf{b}}_{a,i},\quad
\bar{\mathbf{b}}_{g,i}
$$

当前优化变量中的 bias 为：

$$
\mathbf{b}_{a,i},\quad
\mathbf{b}_{g,i}
$$

定义 bias 变化量：

$$
\delta\mathbf{b}_{a,i}
=
\mathbf{b}_{a,i}-\bar{\mathbf{b}}_{a,i}
$$

$$
\delta\mathbf{b}_{g,i}
=
\mathbf{b}_{g,i}-\bar{\mathbf{b}}_{g,i}
$$

为了避免每次 bias 改变都重新积分 IMU，使用一阶修正：

$$
\boldsymbol{\alpha}_{ij}^{c}
=
\hat{\boldsymbol{\alpha}}_{ij}
+
\mathbf{J}^{\alpha}_{b_a}\delta\mathbf{b}_{a,i}
+
\mathbf{J}^{\alpha}_{b_g}\delta\mathbf{b}_{g,i}
$$

$$
\boldsymbol{\beta}_{ij}^{c}
=
\hat{\boldsymbol{\beta}}_{ij}
+
\mathbf{J}^{\beta}_{b_a}\delta\mathbf{b}_{a,i}
+
\mathbf{J}^{\beta}_{b_g}\delta\mathbf{b}_{g,i}
$$

$$
\boldsymbol{\gamma}_{ij}^{c}
=
\hat{\boldsymbol{\gamma}}_{ij}
\otimes
\delta\mathbf{q}
\left(
\mathbf{J}^{\gamma}_{b_g}\delta\mathbf{b}_{g,i}
\right)
$$

其中：

- 带帽子的 $\hat{\boldsymbol{\alpha}}_{ij}$、$\hat{\boldsymbol{\beta}}_{ij}$、$\hat{\boldsymbol{\gamma}}_{ij}$ 表示原始预积分结果；
- 上标 $c$ 表示 corrected，也就是经过 bias 一阶修正后的预积分量；
- $\mathbf{J}^{\alpha}_{b_a}$ 表示 $\boldsymbol{\alpha}$ 对加速度计 bias 的 Jacobian；
- $\mathbf{J}^{\alpha}_{b_g}$ 表示 $\boldsymbol{\alpha}$ 对陀螺仪 bias 的 Jacobian；
- $\mathbf{J}^{\beta}_{b_a}$ 表示 $\boldsymbol{\beta}$ 对加速度计 bias 的 Jacobian；
- $\mathbf{J}^{\beta}_{b_g}$ 表示 $\boldsymbol{\beta}$ 对陀螺仪 bias 的 Jacobian；
- $\mathbf{J}^{\gamma}_{b_g}$ 表示 $\boldsymbol{\gamma}$ 对陀螺仪 bias 的 Jacobian；
- $\delta\mathbf{q}(\cdot)$ 表示由小角度向量构造的小旋转四元数。

### 4.2 IMU 15 维残差

IMU 残差写成：

$$
\mathbf{r}_{imu}
=
\begin{bmatrix}
\mathbf{r}_p\\
\mathbf{r}_q\\
\mathbf{r}_v\\
\mathbf{r}_{ba}\\
\mathbf{r}_{bg}
\end{bmatrix}
\in
\mathbb{R}^{15}
$$

其中位置残差为：

$$
\mathbf{r}_p
=
\mathbf{R}_i^{\top}
\left(
\mathbf{p}_j-\mathbf{p}_i-\mathbf{v}_i\Delta t_{ij}
-
\frac{1}{2}\mathbf{g}\Delta t_{ij}^{2}
\right)
-
\boldsymbol{\alpha}_{ij}^{c}
$$

姿态残差为：

$$
\mathbf{r}_q
=
2\,\operatorname{vec}
\left(
\left(
\boldsymbol{\gamma}_{ij}^{c}
\right)^{-1}
\otimes
\mathbf{q}_i^{-1}
\otimes
\mathbf{q}_j
\right)
$$

速度残差为：

$$
\mathbf{r}_v
=
\mathbf{R}_i^{\top}
\left(
\mathbf{v}_j-\mathbf{v}_i-\mathbf{g}\Delta t_{ij}
\right)
-
\boldsymbol{\beta}_{ij}^{c}
$$

加速度计 bias 残差为：

$$
\mathbf{r}_{ba}
=
\mathbf{b}_{a,j}-\mathbf{b}_{a,i}
$$

陀螺仪 bias 残差为：

$$
\mathbf{r}_{bg}
=
\mathbf{b}_{g,j}-\mathbf{b}_{g,i}
$$

这里 $\operatorname{vec}(\mathbf{q})$ 表示取四元数的虚部，也就是三维向量部分。

为什么姿态残差前面有 $2$？当误差四元数很小时，可以写成：

$$
\delta\mathbf{q}
\approx
\begin{bmatrix}
1\\
\frac{1}{2}\delta\boldsymbol{\theta}
\end{bmatrix}
$$

因此：

$$
\delta\boldsymbol{\theta}
\approx
2\,\operatorname{vec}(\delta\mathbf{q})
$$

所以姿态残差用 $2\,\operatorname{vec}(\cdot)$ 表示小角度误差。

如果 IMU 残差全部为零，含义是：

$$
\text{当前两帧状态预测出的相对运动}
=
\text{IMU 预积分测到的相对运动}
$$

## 5. IMU Jacobian：每个参数块如何影响 IMU 残差

IMU 残差对四个参数块求导：

$$
\mathbf{J}_{imu}
=
\left[
\mathbf{J}_{\pi_i},\,
\mathbf{J}_{s_i},\,
\mathbf{J}_{\pi_j},\,
\mathbf{J}_{s_j}
\right]
$$

其中：

$$
\boldsymbol{\pi}_i
=
\left[
\mathbf{p}_i,\mathbf{q}_i
\right],
\quad
\mathbf{s}_i
=
\left[
\mathbf{v}_i,\mathbf{b}_{a,i},\mathbf{b}_{g,i}
\right]
$$

$$
\boldsymbol{\pi}_j
=
\left[
\mathbf{p}_j,\mathbf{q}_j
\right],
\quad
\mathbf{s}_j
=
\left[
\mathbf{v}_j,\mathbf{b}_{a,j},\mathbf{b}_{g,j}
\right]
$$

局部扰动为：

$$
\delta\boldsymbol{\pi}_i
=
\left[
\delta\mathbf{p}_i,\delta\boldsymbol{\theta}_i
\right],
\quad
\delta\mathbf{s}_i
=
\left[
\delta\mathbf{v}_i,\delta\mathbf{b}_{a,i},\delta\mathbf{b}_{g,i}
\right]
$$

下面只推导最关键的块。完整实现会包含四元数左右乘矩阵、不同扰动定义下的符号差异，但核心结构相同。

定义：

$$
\mathbf{a}_p
=
\mathbf{p}_j-\mathbf{p}_i-\mathbf{v}_i\Delta t_{ij}
-
\frac{1}{2}\mathbf{g}\Delta t_{ij}^{2}
$$

则：

$$
\mathbf{r}_p
=
\mathbf{R}_i^\top\mathbf{a}_p
-
\boldsymbol{\alpha}_{ij}^{c}
$$

位置相关 Jacobian 很直观：

$$
\frac{\partial\mathbf{r}_p}{\partial\delta\mathbf{p}_i}
=
-
\mathbf{R}_i^\top
$$

$$
\frac{\partial\mathbf{r}_p}{\partial\delta\mathbf{p}_j}
=
\mathbf{R}_i^\top
$$

$$
\frac{\partial\mathbf{r}_p}{\partial\delta\mathbf{v}_i}
=
-
\mathbf{R}_i^\top\Delta t_{ij}
$$

如果采用右扰动：

$$
\mathbf{R}_i
\leftarrow
\mathbf{R}_i\exp(\delta\boldsymbol{\theta}_i^{\wedge})
$$

则：

$$
\frac{\partial\mathbf{r}_p}{\partial\delta\boldsymbol{\theta}_i}
\approx
\left(
\mathbf{R}_i^\top\mathbf{a}_p
\right)^{\wedge}
$$

这个式子来自小角度近似：

$$
\exp(-\delta\boldsymbol{\theta}^{\wedge})\mathbf{y}
\approx
\mathbf{y}
-
\delta\boldsymbol{\theta}^{\wedge}\mathbf{y}
=
\mathbf{y}
+
\mathbf{y}^{\wedge}\delta\boldsymbol{\theta}
$$

其中 $\mathbf{y}=\mathbf{R}_i^\top\mathbf{a}_p$。

再看速度残差。定义：

$$
\mathbf{a}_v
=
\mathbf{v}_j-\mathbf{v}_i-\mathbf{g}\Delta t_{ij}
$$

则：

$$
\mathbf{r}_v
=
\mathbf{R}_i^\top\mathbf{a}_v
-
\boldsymbol{\beta}_{ij}^{c}
$$

所以：

$$
\frac{\partial\mathbf{r}_v}{\partial\delta\mathbf{v}_i}
=
-
\mathbf{R}_i^\top
$$

$$
\frac{\partial\mathbf{r}_v}{\partial\delta\mathbf{v}_j}
=
\mathbf{R}_i^\top
$$

$$
\frac{\partial\mathbf{r}_v}{\partial\delta\boldsymbol{\theta}_i}
\approx
\left(
\mathbf{R}_i^\top\mathbf{a}_v
\right)^{\wedge}
$$

bias 相关 Jacobian 来自 bias 一阶修正。因为残差中减去了修正后的预积分量，所以：

$$
\frac{\partial\mathbf{r}_p}{\partial\delta\mathbf{b}_{a,i}}
=
-
\mathbf{J}^{\alpha}_{b_a}
$$

$$
\frac{\partial\mathbf{r}_p}{\partial\delta\mathbf{b}_{g,i}}
=
-
\mathbf{J}^{\alpha}_{b_g}
$$

$$
\frac{\partial\mathbf{r}_v}{\partial\delta\mathbf{b}_{a,i}}
=
-
\mathbf{J}^{\beta}_{b_a}
$$

$$
\frac{\partial\mathbf{r}_v}{\partial\delta\mathbf{b}_{g,i}}
=
-
\mathbf{J}^{\beta}_{b_g}
$$

姿态残差对陀螺仪 bias 的一阶影响可理解为：

$$
\frac{\partial\mathbf{r}_q}{\partial\delta\mathbf{b}_{g,i}}
\approx
-
\mathbf{J}^{\gamma}_{b_g}
$$

最后，bias 随机游走残差非常简单：

$$
\frac{\partial\mathbf{r}_{ba}}{\partial\delta\mathbf{b}_{a,i}}
=
-
\mathbf{I},
\quad
\frac{\partial\mathbf{r}_{ba}}{\partial\delta\mathbf{b}_{a,j}}
=
\mathbf{I}
$$

$$
\frac{\partial\mathbf{r}_{bg}}{\partial\delta\mathbf{b}_{g,i}}
=
-
\mathbf{I},
\quad
\frac{\partial\mathbf{r}_{bg}}{\partial\delta\mathbf{b}_{g,j}}
=
\mathbf{I}
$$

读 IMU Jacobian 时，不要先陷进四元数左乘右乘的细节。先抓住三件事：

1. 位置、速度残差由运动学方程移项得到；
2. 姿态残差由相对旋转误差的小角度表示得到；
3. bias Jacobian 来自预积分的一阶 bias 修正。

## 6. 视觉残差：同一个空间点在另一帧应该投影到哪里

视觉因子通常连接：

$$
f_{proj}
\left(
\mathbf{x}_i,\mathbf{x}_j,\mathbf{x}_{bc},\lambda_l
\right)
$$

其中：

- $\mathbf{x}_i$ 表示第 $l$ 个特征点第一次被观测到的宿主帧；
- $\mathbf{x}_j$ 表示再次观测到该特征点的目标帧；
- $\mathbf{x}_{bc}$ 表示相机与 IMU 外参；
- $\lambda_l$ 表示该特征点在宿主帧相机坐标系下的逆深度。

### 6.1 用逆深度恢复宿主帧三维点

设第 $l$ 个特征点在宿主帧 $i$ 的归一化平面观测为：

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

- $u_{i,l}$ 表示归一化图像平面的横坐标；
- $v_{i,l}$ 表示归一化图像平面的纵坐标；
- 第三个分量 $1$ 表示归一化深度。

逆深度定义为：

$$
\lambda_l
=
\frac{1}{d_l}
$$

其中 $d_l$ 是该点在宿主帧相机坐标系下的深度。因此三维点在宿主帧相机坐标系中的坐标为：

$$
\mathbf{P}_{c_i,l}
=
\frac{1}{\lambda_l}
\bar{\mathbf{u}}_{i,l}
$$

使用逆深度的好处是：远处点的深度 $d_l$ 很大，但逆深度 $\lambda_l$ 接近 0，更适合优化。

### 6.2 从宿主帧相机系变换到目标帧相机系

先从宿主帧相机系变到宿主帧 IMU 系：

$$
\mathbf{P}_{b_i,l}
=
\mathbf{R}_{bc}\mathbf{P}_{c_i,l}
+
\mathbf{p}_{bc}
$$

再变到世界系：

$$
\mathbf{P}_{w,l}
=
\mathbf{R}_i\mathbf{P}_{b_i,l}
+
\mathbf{p}_i
$$

再变到目标帧 IMU 系：

$$
\mathbf{P}_{b_j,l}
=
\mathbf{R}_j^\top
\left(
\mathbf{P}_{w,l}-\mathbf{p}_j
\right)
$$

最后变到目标帧相机系：

$$
\mathbf{P}_{c_j,l}
=
\mathbf{R}_{bc}^{\top}
\left(
\mathbf{P}_{b_j,l}-\mathbf{p}_{bc}
\right)
$$

把上面几步合起来：

$$
\mathbf{P}_{c_j,l}
=
\mathbf{R}_{bc}^{\top}
\left[
\mathbf{R}_j^\top
\left(
\mathbf{R}_i
\left(
\mathbf{R}_{bc}
\frac{1}{\lambda_l}
\bar{\mathbf{u}}_{i,l}
+
\mathbf{p}_{bc}
\right)
+
\mathbf{p}_i
-
\mathbf{p}_j
\right)
-
\mathbf{p}_{bc}
\right]
$$

这个式子是视觉残差的核心。它表达的是：

$$
\text{用宿主帧的逆深度和两帧位姿，预测该点在目标帧相机坐标系的位置。}
$$

### 6.3 归一化平面重投影残差

设：

$$
\mathbf{P}_{c_j,l}
=
\begin{bmatrix}
X\\Y\\Z
\end{bmatrix}
$$

归一化投影函数为：

$$
\pi
\left(
\begin{bmatrix}
X\\Y\\Z
\end{bmatrix}
\right)
=
\begin{bmatrix}
X/Z\\
Y/Z
\end{bmatrix}
$$

第 $j$ 帧实际观测为：

$$
\mathbf{z}_{j,l}
=
\begin{bmatrix}
u_{j,l}\\
v_{j,l}
\end{bmatrix}
$$

则最常见的重投影残差为：

$$
\mathbf{r}_{proj,l,j}
=
\pi(\mathbf{P}_{c_j,l})
-
\mathbf{z}_{j,l}
$$

如果残差为零，表示该空间点通过当前估计的位姿、外参和逆深度投影到第 $j$ 帧后，正好落在观测位置。

投影函数的 Jacobian 是：

$$
\frac{\partial\pi}{\partial\mathbf{P}_{c_j,l}}
=
\begin{bmatrix}
\frac{1}{Z} & 0 & -\frac{X}{Z^2}\\
0 & \frac{1}{Z} & -\frac{Y}{Z^2}
\end{bmatrix}
$$

这个 $2\times3$ 矩阵是视觉 Jacobian 链式求导的第一环。

### 6.4 单位球面切平面残差

VINS-Mono 还常用单位球面上的视觉残差。它不是直接比较归一化平面上的 $(u,v)$，而是比较两个单位视线方向。

预测方向为：

$$
\mathbf{d}_{pred}
=
\frac{\mathbf{P}_{c_j,l}}
{\left\|\mathbf{P}_{c_j,l}\right\|}
$$

观测方向为：

$$
\mathbf{d}_{obs}
=
\frac{
\begin{bmatrix}
u_{j,l}\\
v_{j,l}\\
1
\end{bmatrix}
}
{
\left\|
\begin{bmatrix}
u_{j,l}\\
v_{j,l}\\
1
\end{bmatrix}
\right\|
}
$$

因为单位球面上的方向误差本质上只有 2 个自由度，所以需要把三维方向差投影到切平面。设 $\mathbf{b}_1,\mathbf{b}_2$ 是观测方向 $\mathbf{d}_{obs}$ 对应切平面上的两个正交基：

$$
\mathbf{b}_1^\top\mathbf{d}_{obs}=0,
\quad
\mathbf{b}_2^\top\mathbf{d}_{obs}=0,
\quad
\mathbf{b}_1^\top\mathbf{b}_2=0
$$

定义：

$$
\mathbf{B}
=
\begin{bmatrix}
\mathbf{b}_1 & \mathbf{b}_2
\end{bmatrix}
\in
\mathbb{R}^{3\times2}
$$

则球面切平面残差为：

$$
\mathbf{r}_{sphere,l,j}
=
\mathbf{B}^{\top}
\left(
\mathbf{d}_{pred}
-
\mathbf{d}_{obs}
\right)
\in
\mathbb{R}^{2}
$$

方向归一化的 Jacobian 为：

$$
\frac{\partial\mathbf{d}_{pred}}
{\partial\mathbf{P}_{c_j,l}}
=
\frac{1}{\left\|\mathbf{P}_{c_j,l}\right\|}
\left(
\mathbf{I}
-
\mathbf{d}_{pred}\mathbf{d}_{pred}^{\top}
\right)
$$

所以球面残差对三维点的 Jacobian 为：

$$
\frac{\partial\mathbf{r}_{sphere,l,j}}
{\partial\mathbf{P}_{c_j,l}}
=
\mathbf{B}^{\top}
\frac{1}{\left\|\mathbf{P}_{c_j,l}\right\|}
\left(
\mathbf{I}
-
\mathbf{d}_{pred}\mathbf{d}_{pred}^{\top}
\right)
$$

归一化平面残差和单位球面残差表达形式不同，但本质都是：

$$
\text{预测视线方向}
-
\text{观测视线方向}
$$

## 7. 视觉 Jacobian：链式法则才是主线

视觉残差的 Jacobian 可以写成链式结构：

$$
\mathbf{J}_{proj}
=
\frac{\partial\mathbf{r}_{proj}}
{\partial\mathbf{P}_{c_j}}
\frac{\partial\mathbf{P}_{c_j}}
{\partial\delta\boldsymbol{\chi}}
$$

其中：

- 第一项由投影模型决定；
- 第二项由坐标变换链决定；
- $\delta\boldsymbol{\chi}$ 可以是宿主帧位姿、目标帧位姿、外参、逆深度或时间偏移。

为了简化符号，下面省略特征点编号 $l$，写作 $\mathbf{P}_{c_i}$ 和 $\mathbf{P}_{c_j}$。

### 7.1 对宿主帧和目标帧位置的导数

由：

$$
\mathbf{P}_{c_j}
=
\mathbf{R}_{bc}^{\top}
\left(
\mathbf{R}_j^\top
\left(
\mathbf{P}_{w}-\mathbf{p}_j
\right)
-
\mathbf{p}_{bc}
\right)
$$

以及：

$$
\mathbf{P}_{w}
=
\mathbf{R}_i\mathbf{P}_{b_i}+\mathbf{p}_i
$$

可得：

$$
\frac{\partial\mathbf{P}_{c_j}}
{\partial\delta\mathbf{p}_i}
=
\mathbf{R}_{bc}^{\top}
\mathbf{R}_j^\top
$$

$$
\frac{\partial\mathbf{P}_{c_j}}
{\partial\delta\mathbf{p}_j}
=
-
\mathbf{R}_{bc}^{\top}
\mathbf{R}_j^\top
$$

这两个式子很好理解：移动宿主帧位置会同向移动世界点，移动目标帧位置会反向改变该点在目标相机中的坐标。

### 7.2 对逆深度的导数

宿主帧相机系三维点为：

$$
\mathbf{P}_{c_i}
=
\frac{1}{\lambda}
\bar{\mathbf{u}}_i
$$

所以：

$$
\frac{\partial\mathbf{P}_{c_i}}
{\partial\lambda}
=
-
\frac{1}{\lambda^2}
\bar{\mathbf{u}}_i
$$

经过坐标变换链：

$$
\frac{\partial\mathbf{P}_{c_j}}
{\partial\lambda}
=
\mathbf{R}_{bc}^{\top}
\mathbf{R}_j^\top
\mathbf{R}_i
\mathbf{R}_{bc}
\left(
-
\frac{1}{\lambda^2}
\bar{\mathbf{u}}_i
\right)
$$

最后乘上投影 Jacobian：

$$
\frac{\partial\mathbf{r}_{proj}}
{\partial\lambda}
=
\frac{\partial\mathbf{r}_{proj}}
{\partial\mathbf{P}_{c_j}}
\frac{\partial\mathbf{P}_{c_j}}
{\partial\lambda}
$$

这就是逆深度 Jacobian 的来源。

### 7.3 对宿主帧姿态的导数

采用右扰动：

$$
\mathbf{R}_i
\leftarrow
\mathbf{R}_i\exp(\delta\boldsymbol{\theta}_i^{\wedge})
$$

有：

$$
\mathbf{R}_i\exp(\delta\boldsymbol{\theta}_i^{\wedge})\mathbf{P}_{b_i}
\approx
\mathbf{R}_i\mathbf{P}_{b_i}
-
\mathbf{R}_i\mathbf{P}_{b_i}^{\wedge}
\delta\boldsymbol{\theta}_i
$$

因此：

$$
\frac{\partial\mathbf{P}_{c_j}}
{\partial\delta\boldsymbol{\theta}_i}
=
-
\mathbf{R}_{bc}^{\top}
\mathbf{R}_j^\top
\mathbf{R}_i
\mathbf{P}_{b_i}^{\wedge}
$$

再乘投影 Jacobian：

$$
\frac{\partial\mathbf{r}_{proj}}
{\partial\delta\boldsymbol{\theta}_i}
=
\frac{\partial\mathbf{r}_{proj}}
{\partial\mathbf{P}_{c_j}}
\left(
-
\mathbf{R}_{bc}^{\top}
\mathbf{R}_j^\top
\mathbf{R}_i
\mathbf{P}_{b_i}^{\wedge}
\right)
$$

这说明宿主帧旋转会通过改变世界点的位置来影响目标帧重投影。

### 7.4 对目标帧姿态的导数

令：

$$
\mathbf{P}_{b_j}
=
\mathbf{R}_j^\top
\left(
\mathbf{P}_{w}-\mathbf{p}_j
\right)
$$

采用右扰动时：

$$
\left(
\mathbf{R}_j\exp(\delta\boldsymbol{\theta}_j^{\wedge})
\right)^\top
\left(
\mathbf{P}_{w}-\mathbf{p}_j
\right)
\approx
\mathbf{P}_{b_j}
+
\mathbf{P}_{b_j}^{\wedge}
\delta\boldsymbol{\theta}_j
$$

因此：

$$
\frac{\partial\mathbf{P}_{c_j}}
{\partial\delta\boldsymbol{\theta}_j}
=
\mathbf{R}_{bc}^{\top}
\mathbf{P}_{b_j}^{\wedge}
$$

再乘投影 Jacobian即可得到：

$$
\frac{\partial\mathbf{r}_{proj}}
{\partial\delta\boldsymbol{\theta}_j}
=
\frac{\partial\mathbf{r}_{proj}}
{\partial\mathbf{P}_{c_j}}
\mathbf{R}_{bc}^{\top}
\mathbf{P}_{b_j}^{\wedge}
$$

不同文献或代码可能因为左扰动、右扰动、世界到机体或机体到世界的定义不同出现符号差异。判断是否正确时，不要只背符号，而要回到坐标变换链和扰动定义。

### 7.5 对外参平移的导数

外参平移 $\mathbf{p}_{bc}$ 在两处出现：

1. 宿主帧相机点变到 IMU 系时加了一次 $\mathbf{p}_{bc}$；
2. 目标帧 IMU 点变到相机系时减了一次 $\mathbf{p}_{bc}$。

因此：

$$
\frac{\partial\mathbf{P}_{c_j}}
{\partial\mathbf{p}_{bc}}
=
\mathbf{R}_{bc}^{\top}
\left(
\mathbf{R}_j^\top\mathbf{R}_i
-
\mathbf{I}
\right)
$$

再乘投影 Jacobian：

$$
\frac{\partial\mathbf{r}_{proj}}
{\partial\mathbf{p}_{bc}}
=
\frac{\partial\mathbf{r}_{proj}}
{\partial\mathbf{P}_{c_j}}
\mathbf{R}_{bc}^{\top}
\left(
\mathbf{R}_j^\top\mathbf{R}_i
-
\mathbf{I}
\right)
$$

这个式子很有物理意义：外参平移影响了相机相对 IMU 的杆臂，杆臂在两帧姿态不同时会改变重投影。

### 7.6 对外参旋转的导数

外参旋转 $\mathbf{R}_{bc}$ 也在两处出现：

1. 宿主帧中，把 $\mathbf{P}_{c_i}$ 旋到 IMU 系；
2. 目标帧中，把 $\mathbf{P}_{b_j}$ 旋回相机系。

采用右扰动：

$$
\mathbf{R}_{bc}
\leftarrow
\mathbf{R}_{bc}
\exp(\delta\boldsymbol{\theta}_{bc}^{\wedge})
$$

外参旋转的导数可以写成：

$$
\frac{\partial\mathbf{P}_{c_j}}
{\partial\delta\boldsymbol{\theta}_{bc}}
\approx
\mathbf{P}_{c_j}^{\wedge}
-
\mathbf{R}_{bc}^{\top}
\mathbf{R}_j^\top
\mathbf{R}_i
\mathbf{R}_{bc}
\mathbf{P}_{c_i}^{\wedge}
$$

因此：

$$
\frac{\partial\mathbf{r}_{proj}}
{\partial\delta\boldsymbol{\theta}_{bc}}
=
\frac{\partial\mathbf{r}_{proj}}
{\partial\mathbf{P}_{c_j}}
\left(
\mathbf{P}_{c_j}^{\wedge}
-
\mathbf{R}_{bc}^{\top}
\mathbf{R}_j^\top
\mathbf{R}_i
\mathbf{R}_{bc}
\mathbf{P}_{c_i}^{\wedge}
\right)
$$

外参旋转 Jacobian 看起来复杂，是因为它同时影响“从宿主相机出发”和“回到目标相机”两段变换。

## 8. 时间偏移对视觉残差的影响

如果相机时间戳和 IMU 时间戳存在偏移 $t_d$，那么图像观测点需要按图像速度做一阶时间修正。

设某个特征点在图像上的观测为：

$$
\mathbf{u}
=
\begin{bmatrix}
u\\v
\end{bmatrix}
$$

该点在图像上的速度为：

$$
\dot{\mathbf{u}}
=
\begin{bmatrix}
\dot{u}\\\dot{v}
\end{bmatrix}
$$

如果相机观测时间需要补偿 $t_d$，一阶近似为：

$$
\mathbf{u}(t+t_d)
\approx
\mathbf{u}(t)
+
t_d\dot{\mathbf{u}}(t)
$$

因此视觉残差会变成：

$$
\mathbf{r}_{proj}
=
\pi(\mathbf{P}_{c_j}(t_d))
-
\left(
\mathbf{u}_{j}
+
t_d\dot{\mathbf{u}}_{j}
\right)
$$

如果宿主帧观测也参与时间补偿，那么 $\mathbf{P}_{c_i}$ 也会受到 $t_d$ 影响，因为：

$$
\bar{\mathbf{u}}_{i}(t_i+t_d)
\approx
\bar{\mathbf{u}}_{i}(t_i)
+
t_d\dot{\bar{\mathbf{u}}}_{i}(t_i)
$$

所以时间偏移 Jacobian 本质上来自两个部分：

$$
\frac{\partial\mathbf{r}_{proj}}{\partial t_d}
=
\frac{\partial\mathbf{r}_{proj}}
{\partial\mathbf{P}_{c_j}}
\frac{\partial\mathbf{P}_{c_j}}
{\partial t_d}
-
\dot{\mathbf{u}}_j
$$

其中：

- 第一项表示宿主帧归一化观测被时间修正后，三维点重建发生变化；
- 第二项 $-\dot{\mathbf{u}}_j$ 表示目标帧观测点自身被时间修正。

这就是时间同步和视觉残差之间的联系。时间偏移不是单独“校时”，而是作为一个变量进入重投影误差。

## 9. sqrt information：把马氏距离变成普通最小二乘

理论上的残差代价是马氏距离：

$$
\left\|
\mathbf{r}
\right\|_{\mathbf{\Omega}}^{2}
=
\mathbf{r}^{\top}\mathbf{\Omega}\mathbf{r}
$$

其中：

$$
\mathbf{\Omega}
=
\mathbf{\Sigma}^{-1}
$$

但很多非线性最小二乘求解器希望收到的是普通残差：

$$
\left\|
\mathbf{e}
\right\|^{2}
=
\mathbf{e}^{\top}\mathbf{e}
$$

因此对信息矩阵做分解：

$$
\mathbf{\Omega}
=
\mathbf{L}\mathbf{L}^{\top}
$$

令：

$$
\mathbf{e}
=
\mathbf{L}^{\top}\mathbf{r}
$$

则：

$$
\mathbf{r}^{\top}\mathbf{\Omega}\mathbf{r}
=
\mathbf{r}^{\top}\mathbf{L}\mathbf{L}^{\top}\mathbf{r}
=
\left(
\mathbf{L}^{\top}\mathbf{r}
\right)^{\top}
\left(
\mathbf{L}^{\top}\mathbf{r}
\right)
=
\mathbf{e}^{\top}\mathbf{e}
$$

这就是 sqrt information 的数学含义：

$$
\text{把带协方差的马氏距离，转换成求解器可接受的普通最小二乘。}
$$

对应地，Jacobian 也要一起左乘：

$$
\tilde{\mathbf{r}}
=
\mathbf{L}^{\top}\mathbf{r}
$$

$$
\tilde{\mathbf{J}}
=
\mathbf{L}^{\top}\mathbf{J}
$$

否则残差被加权了，Jacobian 却没有被加权，线性化方程就不一致。

对于视觉观测，如果像素噪声标准差为 $\sigma_{pix}$，相机焦距为 $f$，那么归一化平面上的噪声标准差近似为：

$$
\sigma_{norm}
=
\frac{\sigma_{pix}}{f}
$$

协方差为：

$$
\mathbf{\Sigma}_{img}
=
\left(
\frac{\sigma_{pix}}{f}
\right)^2
\mathbf{I}_{2}
$$

信息矩阵为：

$$
\mathbf{\Omega}_{img}
=
\mathbf{\Sigma}_{img}^{-1}
=
\left(
\frac{f}{\sigma_{pix}}
\right)^2
\mathbf{I}_{2}
$$

sqrt information 为：

$$
\mathbf{L}^{\top}
=
\frac{f}{\sigma_{pix}}
\mathbf{I}_{2}
$$

这解释了为什么视觉残差的权重和像素噪声、焦距有关。

## 10. 鲁棒核：为什么要降低外点影响

视觉跟踪中可能出现误匹配。如果直接用平方误差：

$$
\left\|\mathbf{r}\right\|^2
$$

大残差会被平方放大，严重拉偏优化。鲁棒核把代价写成：

$$
\rho
\left(
\left\|\mathbf{r}\right\|^2
\right)
$$

其中 $\rho(\cdot)$ 是增长速度比平方函数更慢的函数。这样小残差仍然接近普通最小二乘，大残差的影响会被压低。

直观理解：

$$
\text{内点}
\Rightarrow
\text{正常参与优化}
$$

$$
\text{外点}
\Rightarrow
\text{降低权重，避免污染状态}
$$

## 11. 本篇小结

VINS 后端优化可以沿着一条主线理解：

$$
\text{测量模型}
\rightarrow
\text{残差}
\rightarrow
\text{一阶线性化}
\rightarrow
\text{Jacobian}
\rightarrow
\text{加权正规方程}
\rightarrow
\text{状态更新}
$$

IMU 因子回答的问题是：

$$
\text{两帧状态之间的相对运动，是否符合 IMU 预积分？}
$$

视觉因子回答的问题是：

$$
\text{同一个空间点从宿主帧重建后，投影到目标帧是否落在观测位置？}
$$

sqrt information 回答的问题是：

$$
\text{不同残差可信度不同，如何把协方差正确放进最小二乘？}
$$

把这三件事连起来，就能理解 VINS-Mono 后端优化的数学骨架。
