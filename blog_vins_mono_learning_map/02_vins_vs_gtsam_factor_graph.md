# 02 VINS 滑窗优化和 GTSAM 因子图：数学上一样，工程上不同

## 1. 为什么要比较 VINS 和 GTSAM

学习 VINS-Mono 时，很容易产生一个疑问：

> VINS 里看起来是在手动构造 Ceres 优化问题，GTSAM 里看起来是在构造 factor graph。它们到底是不是同一种东西？

答案是：在数学层面，它们本质上都是因子图上的非线性最小二乘优化；不同之处主要在工程抽象、变量管理、线性化组织、增量更新方式和边缘化接口。

换句话说，VINS 的滑窗优化并不是“不是因子图”，它只是没有把因子图抽象成 GTSAM 那样的显式图对象。VINS 中的参数块和残差块组合起来，数学上就是一个因子图。

这一篇的目标是说明三件事：

1. 一般形式的因子图是什么；
2. 为什么 VINS 滑窗优化和 GTSAM factor graph 在数学上等价；
3. 它们在工程实现和使用方式上有什么不同。

## 2. 一般形式的因子图

设系统中有一组未知变量：

$$
\mathcal{X}
=
\left\{
\mathbf{x}_1,\mathbf{x}_2,\dots,\mathbf{x}_n
\right\}
$$

其中：

- $\mathcal{X}$ 表示所有待估计变量的集合；
- $\mathbf{x}_i$ 表示第 $i$ 个变量；
- $n$ 表示变量数量。

在 VIO 中，变量可以是位姿、速度、IMU 零偏、特征点逆深度、外参、时间延迟等。在 SLAM 中，变量也可以是机器人位姿、路标位置、传感器标定参数等。

测量约束通常不是作用在所有变量上，而是只作用在少数几个变量上。第 $a$ 个因子可以写成：

$$
f_a(\mathcal{X}_a)
$$

其中：

- $f_a$ 表示第 $a$ 个因子；
- $\mathcal{X}_a \subseteq \mathcal{X}$ 表示这个因子依赖的变量子集。

例如，一个里程计因子可能只依赖两个连续位姿：

$$
f_{\text{odom},k}(\mathbf{x}_k,\mathbf{x}_{k+1})
$$

一个视觉重投影因子可能依赖两个相机位姿、一个特征点深度和外参：

$$
f_{\text{proj}}(\mathbf{x}_i,\mathbf{x}_j,\lambda_l,\mathbf{T}_{ic})
$$

其中：

- $\lambda_l$ 表示第 $l$ 个特征点的逆深度；
- $\mathbf{T}_{ic}$ 表示相机和 IMU 之间的外参。

因子图可以定义为：

$$
\mathcal{G}
=
\left(
\mathcal{V},\mathcal{F},\mathcal{E}
\right)
$$

其中：

- $\mathcal{G}$ 表示因子图；
- $\mathcal{V}$ 表示变量节点集合；
- $\mathcal{F}$ 表示因子节点集合；
- $\mathcal{E}$ 表示边集合，描述某个因子连接哪些变量。

如果因子 $f_a$ 依赖变量 $\mathbf{x}_i$，则图中有一条边：

$$
(f_a,\mathbf{x}_i)\in\mathcal{E}
$$

这就是因子图的基本结构。

## 3. 从概率因子到最小二乘因子

从概率角度看，因子图表示联合概率分布的分解：

$$
p(\mathcal{X}\mid\mathcal{Z})
\propto
\prod_a
f_a(\mathcal{X}_a)
$$

其中：

- $p(\mathcal{X}\mid\mathcal{Z})$ 表示给定观测 $\mathcal{Z}$ 后状态 $\mathcal{X}$ 的后验概率；
- $\propto$ 表示正比于；
- $\prod_a$ 表示对所有因子相乘；
- $f_a(\mathcal{X}_a)$ 表示第 $a$ 个概率因子。

如果第 $a$ 个测量模型为：

$$
\mathbf{z}_a
=
h_a(\mathcal{X}_a)
+
\mathbf{n}_a
$$

其中：

- $\mathbf{z}_a$ 表示第 $a$ 个测量；
- $h_a(\mathcal{X}_a)$ 表示由变量预测测量的函数；
- $\mathbf{n}_a$ 表示测量噪声。

定义残差：

$$
\mathbf{r}_a(\mathcal{X}_a)
=
h_a(\mathcal{X}_a)-\mathbf{z}_a
$$

如果噪声服从高斯分布：

$$
\mathbf{n}_a
\sim
\mathcal{N}(\mathbf{0},\mathbf{\Sigma}_a)
$$

其中 $\mathbf{\Sigma}_a$ 表示测量噪声协方差，则最大后验估计等价于：

$$
\mathcal{X}^{*}
=
\arg\min_{\mathcal{X}}
\sum_a
\left\|
\mathbf{r}_a(\mathcal{X}_a)
\right\|^2_{\mathbf{\Sigma}_a^{-1}}
$$

其中：

- $\mathcal{X}^{*}$ 表示最优估计；
- $\mathbf{\Sigma}_a^{-1}$ 表示信息矩阵；
- $\|\mathbf{r}\|^2_{\mathbf{\Sigma}^{-1}}=\mathbf{r}^{\top}\mathbf{\Sigma}^{-1}\mathbf{r}$。

这就是一般因子图优化的数学形式。

## 4. GTSAM 的因子图在数学上是什么

GTSAM 中的 factor graph，本质上就是上面的数学对象：

$$
\mathcal{G}
=
\left(
\mathcal{V},\mathcal{F},\mathcal{E}
\right)
$$

每个变量有一个 key，例如：

$$
X_0,\quad X_1,\quad V_0,\quad B_0,\quad L_3
$$

这些 key 只是变量身份标识。数学上它们对应：

$$
\mathbf{x}_0,\quad \mathbf{x}_1,\quad \mathbf{v}_0,\quad \mathbf{b}_0,\quad \mathbf{l}_3
$$

其中：

- $\mathbf{x}_k$ 可以表示第 $k$ 个位姿；
- $\mathbf{v}_k$ 可以表示第 $k$ 个速度；
- $\mathbf{b}_k$ 可以表示第 $k$ 个 IMU 零偏；
- $\mathbf{l}_3$ 可以表示第 $3$ 个路标。

每个 GTSAM factor 定义一个误差函数：

$$
\mathbf{r}_a(\mathcal{X}_a)
$$

再配一个噪声模型：

$$
\mathbf{\Sigma}_a
$$

所以一个 GTSAM factor 的数学含义就是：

$$
\left\|
\mathbf{r}_a(\mathcal{X}_a)
\right\|^2_{\mathbf{\Sigma}_a^{-1}}
$$

所有 factor 加起来，就是一个非线性最小二乘问题。

## 5. VINS 滑窗优化在数学上是什么

VINS 滑窗优化中，也有一组窗口变量：

$$
\mathcal{X}_{\text{win}}
=
\left\{
\mathbf{x}_0,\dots,\mathbf{x}_W,\,
\lambda_1,\dots,\lambda_M,\,
\mathbf{T}_{ic}
\right\}
$$

其中：

- $\mathcal{X}_{\text{win}}$ 表示滑动窗口内的变量；
- $W$ 表示窗口长度；
- $M$ 表示窗口内参与优化的特征点数量。

它也有一组残差：

$$
\mathbf{r}_{\text{imu}},\quad
\mathbf{r}_{\text{proj}},\quad
\mathbf{r}_{\text{prior}}
$$

其中：

- $\mathbf{r}_{\text{imu}}$ 表示 IMU 预积分残差；
- $\mathbf{r}_{\text{proj}}$ 表示视觉重投影残差；
- $\mathbf{r}_{\text{prior}}$ 表示边缘化留下的先验残差。

目标函数可以写成：

$$
\min_{\mathcal{X}_{\text{win}}}
\left(
\sum_k
\left\|
\mathbf{r}_{\text{imu},k}
\right\|^2_{\mathbf{\Sigma}_{\text{imu},k}^{-1}}
+
\sum_l\sum_{(i,j)\in\mathcal{O}_l}
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

- $\mathcal{O}_l$ 表示观测到第 $l$ 个特征点的帧对集合；
- $\rho(\cdot)$ 表示鲁棒核函数；
- $\mathbf{\Sigma}_{\text{img}}$ 表示图像观测协方差。

这和一般因子图形式完全一致。区别只是 VINS 更偏向手动管理参数块和残差块，而 GTSAM 更显式地维护变量 key 和 factor graph。

## 6. 数学层面为什么它们是一样的

无论是 VINS 还是 GTSAM，核心流程都是：

$$
\text{定义变量}
\rightarrow
\text{定义残差}
\rightarrow
\text{线性化残差}
\rightarrow
\text{求解增量}
\rightarrow
\text{更新状态}
$$

对于任意因子：

$$
\mathbf{r}_a(\mathcal{X}_a)
$$

在当前估计 $\bar{\mathcal{X}}_a$ 附近线性化：

$$
\mathbf{r}_a(\mathcal{X}_a)
\approx
\mathbf{r}_a(\bar{\mathcal{X}}_a)
+
\mathbf{J}_a
\delta\mathcal{X}_a
$$

其中：

- $\bar{\mathcal{X}}_a$ 表示当前线性化点；
- $\mathbf{J}_a$ 表示第 $a$ 个因子的雅克比；
- $\delta\mathcal{X}_a$ 表示该因子相关变量的局部增量。

把所有因子堆叠起来：

$$
\mathbf{r}
\approx
\mathbf{r}_0+\mathbf{J}\delta\mathcal{X}
$$

得到正规方程：

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

不管你把因子叫作 Ceres residual block、GTSAM nonlinear factor，还是手写 least-squares term，它们最后都会进入这组线性化方程。

所以数学等价性可以概括为：

$$
\text{VINS residual block}
\equiv
\text{GTSAM nonlinear factor}
\equiv
\text{因子图中的一个因子}
$$

$$
\text{VINS parameter block}
\equiv
\text{GTSAM variable key}
\equiv
\text{因子图中的变量节点}
$$

## 7. 第一类差异：图是否是显式对象

GTSAM 显式维护一个 factor graph。变量通过 key 管理，因子通过连接的 key 表达图结构。

可以抽象为：

$$
\text{Factor}
\left(
X_i,X_j,L_k
\right)
$$

其中 $X_i$、$X_j$、$L_k$ 是变量 key。

VINS 的滑窗优化通常没有显式维护一个“图对象”。它直接在优化器中添加参数块和残差块。某个残差依赖哪些参数块，就在构造残差时传入哪些参数。

数学上这仍然定义了边：

$$
f_a
\leftrightarrow
\left\{
\mathbf{x}_i,\mathbf{x}_j,\lambda_l
\right\}
$$

只是这个边关系没有被包装成 GTSAM 那样的图数据结构。

因此：

$$
\text{GTSAM 更像显式图建模框架}
$$

$$
\text{VINS 更像面向固定滑窗问题的手工优化程序}
$$

## 8. 第二类差异：变量组织方式

GTSAM 的变量通常是 key-value 形式：

$$
\text{Values}
=
\left\{
X_0\mapsto\mathbf{x}_0,\,
X_1\mapsto\mathbf{x}_1,\,
B_0\mapsto\mathbf{b}_0
\right\}
$$

其中 $\mapsto$ 表示 key 到变量值的映射。

这种方式适合通用 SLAM 问题，因为变量数量、类型和连接关系可以很灵活。

VINS 滑窗优化通常使用固定窗口数组或固定结构管理变量：

$$
\mathbf{x}_0,\mathbf{x}_1,\dots,\mathbf{x}_W
$$

窗口长度固定，状态顺序固定，IMU 因子主要连接相邻状态，视觉因子按特征观测关系连接多个状态。

因此：

$$
\text{GTSAM 变量组织更通用}
$$

$$
\text{VINS 变量组织更贴合实时 VIO 固定窗口}
$$

## 9. 第三类差异：求解方式和线性代数策略

GTSAM 的强项之一是图结构上的稀疏线性代数。它可以使用变量消元、Bayes tree、iSAM2 等机制进行批量或增量求解。

一般线性化后都得到：

$$
\mathbf{H}\delta\mathcal{X}
=
\mathbf{g}
$$

GTSAM 会特别关注变量消元顺序。不同消元顺序会影响填充项，也就是原本稀疏的矩阵在分解过程中变稠密的程度。

VINS 滑窗优化通常面对的是一个规模相对固定的问题。它可以选择适合该问题结构的求解器，例如 Schur 类型方法或密集 Schur 方法。因为窗口长度固定，系统更容易针对 VIO 结构做工程优化。

因此：

$$
\text{GTSAM 强调通用稀疏图优化和增量更新}
$$

$$
\text{VINS 强调固定规模滑窗内的高效批量优化}
$$

## 10. GTSAM 如何批量优化，又如何淘汰历史帧

GTSAM 的批量优化，指的是把当前给定的全部变量和全部因子一次性组成一个非线性最小二乘问题，然后整体线性化、整体求解、整体更新。

设当前图中的变量为：

$$
\mathcal{X}
=
\left\{
\mathbf{x}_1,\mathbf{x}_2,\dots,\mathbf{x}_n
\right\}
$$

当前图中的因子为：

$$
\mathcal{F}
=
\left\{
f_1,f_2,\dots,f_m
\right\}
$$

其中：

- $\mathcal{X}$ 表示当前优化图里的变量集合；
- $\mathbf{x}_i$ 表示第 $i$ 个变量；
- $\mathcal{F}$ 表示当前优化图里的因子集合；
- $f_j$ 表示第 $j$ 个因子；
- $n$ 表示变量数量；
- $m$ 表示因子数量。

批量优化对应的问题是：

$$
\mathcal{X}^{*}
=
\arg\min_{\mathcal{X}}
\sum_{j=1}^{m}
\left\|
\mathbf{r}_j(\mathcal{X}_j)
\right\|^2_{\mathbf{\Sigma}_j^{-1}}
$$

其中：

- $\mathbf{r}_j(\mathcal{X}_j)$ 表示第 $j$ 个因子的残差；
- $\mathcal{X}_j$ 表示第 $j$ 个因子连接的变量子集；
- $\mathbf{\Sigma}_j$ 表示该因子的噪声协方差。

它的流程可以抽象为：

$$
\text{NonlinearFactorGraph + Values}
\rightarrow
\text{线性化}
\rightarrow
\text{GaussianFactorGraph}
\rightarrow
\text{变量消元}
\rightarrow
\text{求解增量}
\rightarrow
\text{retract 更新变量}
$$

更具体地说，每个非线性因子在当前估计 $\bar{\mathcal{X}}_j$ 处线性化：

$$
\mathbf{r}_j(\mathcal{X}_j)
\approx
\mathbf{r}_j(\bar{\mathcal{X}}_j)
+
\mathbf{J}_j
\delta\mathcal{X}_j
$$

其中：

- $\bar{\mathcal{X}}_j$ 表示当前线性化点；
- $\mathbf{J}_j$ 表示第 $j$ 个因子的雅克比；
- $\delta\mathcal{X}_j$ 表示该因子相关变量的局部增量。

所有因子线性化后，得到线性系统：

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
-\mathbf{J}^{\top}\mathbf{r}
$$

这里：

- $\mathbf{J}$ 表示所有因子雅克比堆叠后的总雅克比；
- $\mathbf{r}$ 表示所有因子残差堆叠后的总残差；
- $\delta\mathcal{X}$ 表示全部变量的局部增量。

GTSAM 的重点不是把这个系统当作完全稠密矩阵硬解，而是利用因子图稀疏结构，通过变量排序、消元和稀疏分解来求解。变量消元顺序会影响填充项，也会影响求解效率。

这里容易误解的一点是：普通 batch optimizer 本身不会自动淘汰历史帧。它只优化你交给它的那张图。

也就是说：

$$
\text{给 GTSAM 全历史图}
\Rightarrow
\text{优化全历史变量}
$$

$$
\text{给 GTSAM 当前滑窗图}
\Rightarrow
\text{优化当前滑窗变量}
$$

是否淘汰历史帧，是图管理或 smoother 的职责，不是普通批量优化器的默认行为。

如果要正确淘汰历史帧，不能简单删除历史变量和相关因子。因为这样会丢失历史信息。正确做法仍然是边缘化。

把变量分成：

$$
\mathcal{X}
=
\left[
\mathcal{X}_m,\mathcal{X}_r
\right]
$$

其中：

- $\mathcal{X}_m$ 表示要淘汰的历史变量；
- $\mathcal{X}_r$ 表示要保留的当前变量。

线性化后的正规方程写成分块形式：

$$
\begin{bmatrix}
\mathbf{H}_{mm} & \mathbf{H}_{mr} \\
\mathbf{H}_{rm} & \mathbf{H}_{rr}
\end{bmatrix}
\begin{bmatrix}
\delta\mathcal{X}_m \\
\delta\mathcal{X}_r
\end{bmatrix}
=
\begin{bmatrix}
\mathbf{g}_m \\
\mathbf{g}_r
\end{bmatrix}
$$

通过 Schur 补消去 $\delta\mathcal{X}_m$：

$$
\mathbf{H}^{\text{prior}}_r
=
\mathbf{H}_{rr}
-
\mathbf{H}_{rm}
\mathbf{H}_{mm}^{-1}
\mathbf{H}_{mr}
$$

$$
\mathbf{g}^{\text{prior}}_r
=
\mathbf{g}_{r}
-
\mathbf{H}_{rm}
\mathbf{H}_{mm}^{-1}
\mathbf{g}_{m}
$$

其中：

- $\mathbf{H}^{\text{prior}}_r$ 表示边缘化后作用在保留变量上的先验 Hessian；
- $\mathbf{g}^{\text{prior}}_r$ 表示边缘化后作用在保留变量上的先验梯度。

这样，历史变量 $\mathcal{X}_m$ 被移出图，但它们对 $\mathcal{X}_r$ 的信息被压缩成新的 prior factor 加回图中。

GTSAM 中常见的滑窗思路是 fixed-lag smoothing。它保留最近一段时间长度内的变量。设当前时间为 $t$，固定滞后窗口长度为 $L$，某个变量时间戳为 $t_k$。如果：

$$
t-t_k>L
$$

那么这个变量就超过窗口范围，需要被边缘化。

因此：

$$
\text{VINS 滑窗}
\approx
\text{固定帧数窗口}
$$

$$
\text{GTSAM fixed-lag smoothing}
\approx
\text{固定时间长度窗口}
$$

两者淘汰历史帧的数学本质一样：

$$
\text{淘汰历史帧}
\neq
\text{直接删除历史帧}
$$

而是：

$$
\text{淘汰历史帧}
=
\text{边缘化历史变量}
+
\text{生成剩余变量上的先验因子}
$$

区别在于，VINS 往往手动决定边缘化最老帧还是次新帧，手动收集相关残差，手动构造 Hessian 和 Schur 补，并显式维护 FEJ 线性化点；GTSAM 更倾向于把这些操作放在通用的图优化、smoother 或 fixed-lag smoothing 抽象中处理。

## 11. 第四类差异：边缘化的表达方式

VINS 滑窗优化非常依赖边缘化。因为窗口大小固定，每次滑窗都要把旧状态或非关键帧移出窗口。

边缘化后的先验可以写成：

$$
\mathbf{r}_{\text{prior}}
=
\mathbf{r}^{\text{prior}}_0
+
\mathbf{J}^{\text{prior}}
\left(
\mathcal{X}_r\boxminus\bar{\mathcal{X}}_r
\right)
$$

其中：

- $\mathcal{X}_r$ 表示保留变量；
- $\bar{\mathcal{X}}_r$ 表示边缘化时的线性化点；
- $\mathbf{J}^{\text{prior}}$ 表示边缘化先验雅克比。

这个先验是一个多元因子，可能连接窗口中多个保留状态。

GTSAM 也可以表达先验因子，也可以做 marginalization 或 fixed-lag smoothing。不同在于，GTSAM 通常会从图优化框架角度处理这些操作，而 VINS 通常会为了 FEJ、一致性和实时性手动维护这个先验的线性化点、雅克比和残差。

因此：

$$
\text{二者数学上的边缘化都是 Schur 补}
$$

但工程关注点不同：

$$
\text{VINS 更强调边缘化先验的手动构造和 FEJ 使用}
$$

$$
\text{GTSAM 更强调图层面的通用 marginalization / fixed-lag smoothing 抽象}
$$

## 12. 第五类差异：流形和局部参数化

VIO 中位姿包含旋转，而旋转属于 $SO(3)$，不是普通欧式空间。无论 VINS 还是 GTSAM，都不能直接对姿态做普通加法。

一般需要局部扰动：

$$
\mathbf{R}
\leftarrow
\mathbf{R}\exp(\delta\boldsymbol{\theta}^{\wedge})
$$

或者四元数形式：

$$
\mathbf{q}
\leftarrow
\mathbf{q}\otimes\delta\mathbf{q}
$$

其中：

- $\mathbf{R}$ 表示旋转矩阵；
- $\exp(\cdot)$ 表示李群指数映射；
- $\delta\boldsymbol{\theta}\in\mathbb{R}^3$ 表示小角度扰动；
- $(\cdot)^{\wedge}$ 表示把向量变成反对称矩阵；
- $\mathbf{q}$ 表示单位四元数；
- $\delta\mathbf{q}$ 表示由小角度扰动构造的增量四元数。

GTSAM 通常通过 manifold 类型和 retract/localCoordinates 表达这个过程。VINS 则通常通过优化器的 local parameterization 或手写局部扰动表达。

数学上它们都在做：

$$
\mathcal{X}
\leftarrow
\mathcal{X}\boxplus\delta\mathcal{X}
$$

其中 $\boxplus$ 表示流形上的加法。

## 13. 第六类差异：通用性和专用性

GTSAM 是通用因子图优化库。它适合表达许多问题：

- pose graph optimization；
- visual SLAM；
- fixed-lag smoothing；
- IMU preintegration；
- multi-sensor calibration；
- bundle adjustment；
- incremental smoothing。

VINS 滑窗优化是面向单目视觉惯性里程计的专用系统。它的变量、残差、滑窗策略、边缘化策略都围绕实时 VIO 设计。

所以：

$$
\text{GTSAM 是通用建模和求解框架}
$$

$$
\text{VINS 是专用 VIO 系统中的优化模块}
$$

这就是为什么二者看起来风格差异很大，但底层数学高度一致。

## 14. 用一张表总结

| 对比项 | VINS 滑窗优化 | GTSAM 因子图 |
|---|---|---|
| 数学本质 | 非线性因子图最小二乘 | 非线性因子图最小二乘 |
| 变量表示 | 固定窗口状态、逆深度、外参等 | key-value 变量 |
| 因子表示 | 残差块 | nonlinear factor |
| 图结构 | 隐式存在于参数块连接关系中 | 显式 factor graph |
| 求解方式 | 固定窗口批量优化 | 批量或增量图优化 |
| 边缘化 | 手动构造先验，常配合 FEJ | 通用 marginalization 或 fixed-lag smoothing |
| 适用场景 | 专用实时 VIO | 通用 SLAM / Smoothing |
| 流形处理 | local parameterization | manifold retract/local coordinates |

## 15. 如何把 VINS 问题翻译成 GTSAM 语言

如果用 GTSAM 风格描述 VINS 滑窗，可以想象如下变量：

$$
X_k = \mathbf{T}_{wi,k}
$$

$$
V_k = \mathbf{v}_k
$$

$$
B_k = \left[\mathbf{b}_{a,k},\mathbf{b}_{g,k}\right]
$$

$$
L_l = \lambda_l
$$

$$
E = \mathbf{T}_{ic}
$$

其中：

- $X_k$ 表示第 $k$ 帧 IMU 位姿；
- $V_k$ 表示第 $k$ 帧速度；
- $B_k$ 表示第 $k$ 帧 IMU 零偏；
- $L_l$ 表示第 $l$ 个特征点逆深度；
- $E$ 表示外参。

IMU 因子可以写成：

$$
f_{\text{imu},k}(X_k,V_k,B_k,X_{k+1},V_{k+1},B_{k+1})
$$

视觉因子可以写成：

$$
f_{\text{proj},l}^{i,j}(X_i,X_j,L_l,E)
$$

边缘化先验可以写成：

$$
f_{\text{prior}}(\mathcal{X}_r)
$$

于是整个 VINS 滑窗就是一个 fixed-lag nonlinear factor graph。

## 16. 最后一句话

VINS 滑窗优化和 GTSAM 因子图的关系可以这样理解：

> GTSAM 把因子图作为显式的一等公民来建模和求解；VINS 把同一个数学问题嵌入到专用 VIO 滑动窗口中手动管理。二者在线性化、残差、雅克比、Hessian、Schur 补这些数学层面是同一套东西，差异主要来自工程抽象和系统目标。
