# 07 滑动窗口与边缘化：如何丢掉旧变量但保留历史信息

## 1. 因子图会越长越大

上一篇我们把 VIO 写成了一个因子图优化问题。假设系统一直运行，每来一帧图像，就会增加一个新的状态：

$$
\mathbf{x}_{N+1}
=
\left[
\mathbf{p}_{N+1},\,
\mathbf{q}_{N+1},\,
\mathbf{v}_{N+1},\,
\mathbf{b}_{a,N+1},\,
\mathbf{b}_{g,N+1}
\right]
$$

其中 $\mathbf{x}_{N+1}$ 表示新到来的第 $N+1$ 个时刻的状态。

同时，新图像还会带来新的视觉观测，IMU 也会在相邻帧之间形成新的预积分约束。如果我们永远保留所有历史状态和所有历史特征点，那么优化变量会越来越多：

$$
\mathcal{X}_{0:N}
=
\left\{
\mathbf{x}_0,\mathbf{x}_1,\dots,\mathbf{x}_N
\right\}
$$

当 $N$ 不断增长时，求解非线性最小二乘问题的代价也会持续增长。对于实时机器人系统，这是不可接受的。

因此，滑动窗口 VIO 只维护最近一段时间内的变量，例如：

$$
\mathcal{X}_{k:k+W}
=
\left\{
\mathbf{x}_k,\mathbf{x}_{k+1},\dots,\mathbf{x}_{k+W}
\right\}
$$

其中：

- $k$ 表示当前窗口中最老的状态索引；
- $W$ 表示窗口长度；
- $\mathcal{X}_{k:k+W}$ 表示从第 $k$ 个状态到第 $k+W$ 个状态组成的滑动窗口变量集合。

滑动窗口的目的，是让每次优化的变量规模近似固定，从而保证实时性。

## 2. 直接丢掉旧状态为什么不行

假设窗口中最老的状态是 $\mathbf{x}_0$。新帧进来后，我们想把 $\mathbf{x}_0$ 移出窗口。最简单的做法是直接删除它，以及所有连接它的因子。

但这样会造成信息丢失。

举个例子。假设有三个连续状态：

$$
\mathbf{x}_0,\quad \mathbf{x}_1,\quad \mathbf{x}_2
$$

IMU 约束连接 $\mathbf{x}_0$ 和 $\mathbf{x}_1$，另一个 IMU 约束连接 $\mathbf{x}_1$ 和 $\mathbf{x}_2$。如果直接删除 $\mathbf{x}_0$ 以及它和 $\mathbf{x}_1$ 之间的约束，那么 $\mathbf{x}_1$ 的估计会失去一部分历史约束。系统会变得更漂，甚至在某些方向上变得过于自由。

所以我们真正想做的是：

> 删除旧变量，但把旧变量对剩余变量提供的信息压缩成一个新的先验。

这就是边缘化。

## 3. 从概率角度理解边缘化

设当前窗口中的变量被分成两类：

$$
\mathcal{X}
=
\left[
\mathcal{X}_m,\,
\mathcal{X}_r
\right]
$$

其中：

- $\mathcal{X}$ 表示当前优化问题中的全部变量；
- $\mathcal{X}_m$ 表示即将被移除的变量，$m$ 可以理解为 marginalized；
- $\mathcal{X}_r$ 表示将继续保留在窗口中的变量，$r$ 可以理解为 remaining。

如果我们已有联合后验：

$$
p(\mathcal{X}_m,\mathcal{X}_r\mid\mathcal{Z})
$$

其中 $\mathcal{Z}$ 表示已有测量，那么删除 $\mathcal{X}_m$ 后，对保留变量 $\mathcal{X}_r$ 的正确概率应该是：

$$
p(\mathcal{X}_r\mid\mathcal{Z})
=
\int
p(\mathcal{X}_m,\mathcal{X}_r\mid\mathcal{Z})
d\mathcal{X}_m
$$

这个积分就是概率意义上的边缘化。它的含义是：我们不再关心 $\mathcal{X}_m$ 具体取值，但要把所有可能的 $\mathcal{X}_m$ 对 $\mathcal{X}_r$ 的影响加总起来。

对于高斯分布，这个积分可以用线性代数里的 Schur 补高效完成。

为了直观理解这个积分，先看一个二维高斯例子。设有两个标量变量：

$$
x_m,\quad x_r
$$

其中 $x_m$ 是要删除的变量，$x_r$ 是要保留的变量。假设它们的联合负对数概率是一个二次型：

$$
E(x_m,x_r)
=
\frac{1}{2}
\begin{bmatrix}
x_m\\x_r
\end{bmatrix}^{\top}
\begin{bmatrix}
H_{mm} & H_{mr}\\
H_{rm} & H_{rr}
\end{bmatrix}
\begin{bmatrix}
x_m\\x_r
\end{bmatrix}
-
\begin{bmatrix}
g_m\\g_r
\end{bmatrix}^{\top}
\begin{bmatrix}
x_m\\x_r
\end{bmatrix}
$$

其中：

- $E(x_m,x_r)$ 表示联合概率对应的能量函数；
- $H_{mm}$、$H_{mr}$、$H_{rm}$、$H_{rr}$ 是 Hessian 的分块；
- $g_m$ 和 $g_r$ 是梯度向量的分块。

边缘化 $x_m$ 后，我们希望得到只关于 $x_r$ 的能量：

$$
E_{\text{marg}}(x_r)
=
-
\log
\int
\exp
\left(
-
E(x_m,x_r)
\right)
dx_m
$$

对高斯分布来说，这个积分的结果仍然是关于 $x_r$ 的二次函数：

$$
E_{\text{marg}}(x_r)
=
\frac{1}{2}
x_r^{\top}
H^{prior}_r
x_r
-
\left(
g^{prior}_r
\right)^{\top}
x_r
+
\text{const}
$$

其中 $H^{prior}_r$ 和 $g^{prior}_r$ 正是后面 Schur 补得到的结果。也就是说：

$$
\text{概率积分掉变量}
\quad\Longleftrightarrow\quad
\text{在线性高斯二次型中做 Schur 补}
$$

## 4. 在线性化后的二次问题中做边缘化

非线性优化每次迭代时，都会在当前线性化点附近建立一个二次近似。把所有残差堆叠起来：

$$
\mathbf{r}(\mathcal{X})
\approx
\mathbf{r}_0
+
\mathbf{J}\delta\mathcal{X}
$$

其中：

- $\mathbf{r}(\mathcal{X})$ 表示总残差；
- $\mathbf{r}_0$ 表示当前线性化点处的残差；
- $\mathbf{J}$ 表示总雅克比矩阵；
- $\delta\mathcal{X}$ 表示状态增量。

在边缘化语境下，这个式子需要更精确地理解。通常边缘化发生在当前滑窗优化完成之后。此时优化器已经得到一个当前窗口的估计值，记为：

$$
\bar{\mathcal{X}}
$$

其中：

- $\bar{\mathcal{X}}$ 表示边缘化发生时的线性化点；
- 它通常来自当前滑窗非线性优化结束后的状态估计；
- 上方的横线表示“固定参考值”或“线性化参考点”。

于是残差的一阶近似更明确地写成：

$$
\mathbf{r}(\mathcal{X})
\approx
\mathbf{r}(\bar{\mathcal{X}})
+
\mathbf{J}(\bar{\mathcal{X}})
\left(
\mathcal{X}\boxminus\bar{\mathcal{X}}
\right)
$$

其中：

- $\mathcal{X}$ 表示后续可能变化的状态变量；
- $\mathbf{r}(\bar{\mathcal{X}})$ 表示在边缘化线性化点处计算得到的残差，也就是上面简写的 $\mathbf{r}_0$；
- $\mathbf{J}(\bar{\mathcal{X}})$ 表示在 $\bar{\mathcal{X}}$ 处计算得到的雅克比，也就是上面简写的 $\mathbf{J}$；
- $\boxminus$ 表示流形上的差分操作；
- $\mathcal{X}\boxminus\bar{\mathcal{X}}$ 表示当前状态相对于边缘化线性化点的局部扰动，也就是上面简写的 $\delta\mathcal{X}$。

因此：

$$
\mathbf{r}_0
=
\mathbf{r}(\bar{\mathcal{X}})
$$

$$
\mathbf{J}
=
\mathbf{J}(\bar{\mathcal{X}})
$$

$$
\delta\mathcal{X}
=
\mathcal{X}\boxminus\bar{\mathcal{X}}
$$

这里最容易混淆的是 $\delta\mathcal{X}$。它不是“最后一次迭代已经求出来并应用掉的那个增量”。在一次普通高斯牛顿迭代中，确实会求解某个迭代增量：

$$
\delta\mathcal{X}^{(t)}
$$

其中 $t$ 表示第 $t$ 次迭代。该增量会被用于更新：

$$
\mathcal{X}^{(t+1)}
=
\mathcal{X}^{(t)}
\boxplus
\delta\mathcal{X}^{(t)}
$$

其中 $\boxplus$ 表示流形上的加法操作。当前滑窗优化结束后，这些迭代增量已经被应用掉，最终得到 $\bar{\mathcal{X}}$。

边缘化时重新使用的 $\delta\mathcal{X}$，指的是“任意后续状态相对于 $\bar{\mathcal{X}}$ 的局部差值”。尤其在边缘化先验被后续窗口继续使用时，会固定：

$$
\mathbf{J}(\bar{\mathcal{X}})
$$

而每次根据当前状态重新计算：

$$
\mathcal{X}_{\text{current}}\boxminus\bar{\mathcal{X}}
$$

其中 $\mathcal{X}_{\text{current}}$ 表示后续优化中当前的状态估计。也就是说，边缘化先验固定的是线性化点和雅克比，变化的是当前状态相对这个线性化点的局部差值。

线性化后的最小二乘问题是：

$$
\min_{\delta\mathcal{X}}
\left\|
\mathbf{r}_0
+
\mathbf{J}\delta\mathcal{X}
\right\|^2
$$

如果每个残差都有协方差，那么更一般的形式是：

$$
\min_{\delta\mathcal{X}}
\left\|
\mathbf{r}_0
+
\mathbf{J}\delta\mathcal{X}
\right\|_{\mathbf{\Omega}}^2
$$

其中：

- $\mathbf{\Omega}$ 表示所有残差的信息矩阵；
- 如果已经把残差和 Jacobian 乘过 sqrt information，那么可以把 $\mathbf{\Omega}$ 看成单位矩阵。

它对应正规方程：

$$
\mathbf{H}\delta\mathcal{X}
=
\mathbf{g}
$$

其中：

$$
\mathbf{H}
=
\mathbf{J}^{\top}\mathbf{\Omega}\mathbf{J}
$$

$$
\mathbf{g}
=
-
\mathbf{J}^{\top}\mathbf{\Omega}\mathbf{r}_0
$$

如果使用白化后的残差：

$$
\tilde{\mathbf{r}}_0
=
\mathbf{L}^{\top}\mathbf{r}_0,
\quad
\tilde{\mathbf{J}}
=
\mathbf{L}^{\top}\mathbf{J},
\quad
\mathbf{\Omega}
=
\mathbf{L}\mathbf{L}^{\top}
$$

那么上式就退化成：

$$
\mathbf{H}
=
\tilde{\mathbf{J}}^{\top}\tilde{\mathbf{J}}
$$

$$
\mathbf{g}
=
-
\tilde{\mathbf{J}}^{\top}\tilde{\mathbf{r}}_0
$$

现在把增量分成两部分：

$$
\delta\mathcal{X}
=
\begin{bmatrix}
\delta\mathcal{X}_m \\
\delta\mathcal{X}_r
\end{bmatrix}
$$

其中：

- $\delta\mathcal{X}_m$ 表示待边缘化变量的增量；
- $\delta\mathcal{X}_r$ 表示保留变量的增量。

于是正规方程可以分块写成：

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

其中：

- $\mathbf{H}_{mm}$ 表示待边缘化变量与自身相关的 Hessian 块；
- $\mathbf{H}_{mr}$ 表示待边缘化变量与保留变量之间的 Hessian 块；
- $\mathbf{H}_{rm}$ 表示保留变量与待边缘化变量之间的 Hessian 块；
- $\mathbf{H}_{rr}$ 表示保留变量与自身相关的 Hessian 块；
- $\mathbf{g}_m$ 表示待边缘化变量对应的梯度块；
- $\mathbf{g}_r$ 表示保留变量对应的梯度块。

第一行是：

$$
\mathbf{H}_{mm}\delta\mathcal{X}_m
+
\mathbf{H}_{mr}\delta\mathcal{X}_r
=
\mathbf{g}_m
$$

如果 $\mathbf{H}_{mm}$ 可逆，则：

$$
\delta\mathcal{X}_m
=
\mathbf{H}_{mm}^{-1}
\left(
\mathbf{g}_m
-
\mathbf{H}_{mr}\delta\mathcal{X}_r
\right)
$$

把它代入第二行：

$$
\mathbf{H}_{rm}\delta\mathcal{X}_m
+
\mathbf{H}_{rr}\delta\mathcal{X}_r
=
\mathbf{g}_r
$$

得到：

$$
\left(
\mathbf{H}_{rr}
-
\mathbf{H}_{rm}\mathbf{H}_{mm}^{-1}\mathbf{H}_{mr}
\right)
\delta\mathcal{X}_r
=
\mathbf{g}_r
-
\mathbf{H}_{rm}\mathbf{H}_{mm}^{-1}\mathbf{g}_m
$$

定义：

$$
\mathbf{H}^{\text{prior}}_r
=
\mathbf{H}_{rr}
-
\mathbf{H}_{rm}\mathbf{H}_{mm}^{-1}\mathbf{H}_{mr}
$$

$$
\mathbf{g}^{\text{prior}}_r
=
\mathbf{g}_r
-
\mathbf{H}_{rm}\mathbf{H}_{mm}^{-1}\mathbf{g}_m
$$

其中：

- $\mathbf{H}^{\text{prior}}_r$ 表示边缘化后作用在保留变量上的先验 Hessian；
- $\mathbf{g}^{\text{prior}}_r$ 表示边缘化后作用在保留变量上的先验梯度。

这就是 Schur 补边缘化。

从二次能量角度看，边缘化前的局部能量是：

$$
E(\delta\mathcal{X}_m,\delta\mathcal{X}_r)
=
\frac{1}{2}
\begin{bmatrix}
\delta\mathcal{X}_m\\
\delta\mathcal{X}_r
\end{bmatrix}^{\top}
\begin{bmatrix}
\mathbf{H}_{mm} & \mathbf{H}_{mr}\\
\mathbf{H}_{rm} & \mathbf{H}_{rr}
\end{bmatrix}
\begin{bmatrix}
\delta\mathcal{X}_m\\
\delta\mathcal{X}_r
\end{bmatrix}
-
\begin{bmatrix}
\mathbf{g}_m\\
\mathbf{g}_r
\end{bmatrix}^{\top}
\begin{bmatrix}
\delta\mathcal{X}_m\\
\delta\mathcal{X}_r
\end{bmatrix}
$$

边缘化后变成：

$$
E^{prior}(\delta\mathcal{X}_r)
=
\frac{1}{2}
\delta\mathcal{X}_r^{\top}
\mathbf{H}^{prior}_r
\delta\mathcal{X}_r
-
\left(
\mathbf{g}^{prior}_r
\right)^{\top}
\delta\mathcal{X}_r
$$

因此，边缘化先验不是一个“固定值约束”，而是一个关于保留变量的二次能量。

## 5. Schur 补到底保留了什么

Schur 补的效果可以用一个简单例子说明。

设有两个变量 $x_0$ 和 $x_1$，它们之间有一个相对约束：

$$
z_{01}
=
x_1-x_0+n
$$

其中：

- $z_{01}$ 表示从 $x_0$ 到 $x_1$ 的相对测量；
- $n$ 表示测量噪声。

如果直接删除 $x_0$，那么 $x_1$ 会丢掉与历史有关的信息。但如果我们对 $x_0$ 做边缘化，就会得到一个关于 $x_1$ 的先验。这个先验不一定是简单地固定 $x_1$，而是表达“在历史测量约束下，$x_1$ 应该落在什么概率分布里”。

在高维 VIO 中也是一样。边缘化不是简单删除，而是把被删除变量和保留变量之间的相关性压缩到保留变量的先验中。

## 6. 从 Hessian 先验回到残差先验

优化器通常接受的是残差形式，而不是直接接受一个 Hessian 和梯度。因此边缘化之后，还需要把：

$$
\mathbf{H}^{\text{prior}}_r,\quad
\mathbf{g}^{\text{prior}}_r
$$

重新表示成一个残差：

$$
\mathbf{r}_{\text{prior}}
=
\mathbf{r}^{\text{prior}}_0
+
\mathbf{J}^{\text{prior}}\delta\mathcal{X}_r
$$

其中：

- $\mathbf{r}_{\text{prior}}$ 表示先验残差；
- $\mathbf{r}^{\text{prior}}_0$ 表示先验在边缘化线性化点处的残差；
- $\mathbf{J}^{\text{prior}}$ 表示先验残差对保留变量增量的雅克比。

我们希望：

$$
\left(\mathbf{J}^{\text{prior}}\right)^{\top}
\mathbf{J}^{\text{prior}}
=
\mathbf{H}^{\text{prior}}_r
$$

以及：

$$
\left(\mathbf{J}^{\text{prior}}\right)^{\top}
\mathbf{r}^{\text{prior}}_0
=
-\mathbf{g}^{\text{prior}}_r
$$

一种常见做法是对 $\mathbf{H}^{\text{prior}}_r$ 做特征值分解：

$$
\mathbf{H}^{\text{prior}}_r
=
\mathbf{V}\mathbf{S}\mathbf{V}^{\top}
$$

其中：

- $\mathbf{V}$ 表示特征向量组成的正交矩阵；
- $\mathbf{S}$ 表示特征值组成的对角矩阵。

可以取：

$$
\mathbf{J}^{\text{prior}}
=
\mathbf{S}^{\frac{1}{2}}\mathbf{V}^{\top}
$$

因为：

$$
\left(\mathbf{J}^{\text{prior}}\right)^{\top}
\mathbf{J}^{\text{prior}}
=
\mathbf{V}\mathbf{S}^{\frac{1}{2}}
\mathbf{S}^{\frac{1}{2}}\mathbf{V}^{\top}
=
\mathbf{V}\mathbf{S}\mathbf{V}^{\top}
=
\mathbf{H}^{\text{prior}}_r
$$

现在还需要构造 $\mathbf{r}^{\text{prior}}_0$，使得：

$$
\left(\mathbf{J}^{\text{prior}}\right)^{\top}
\mathbf{r}^{\text{prior}}_0
=
-
\mathbf{g}^{\text{prior}}_r
$$

由于：

$$
\mathbf{J}^{\text{prior}}
=
\mathbf{S}^{\frac{1}{2}}\mathbf{V}^{\top}
$$

所以：

$$
\left(\mathbf{J}^{\text{prior}}\right)^{\top}
=
\mathbf{V}\mathbf{S}^{\frac{1}{2}}
$$

我们可以取：

$$
\mathbf{r}^{\text{prior}}_0
=
-
\mathbf{S}^{-\frac{1}{2}}
\mathbf{V}^{\top}
\mathbf{g}^{\text{prior}}_r
$$

验证一下：

$$
\left(\mathbf{J}^{\text{prior}}\right)^{\top}
\mathbf{r}^{\text{prior}}_0
=
\mathbf{V}\mathbf{S}^{\frac{1}{2}}
\left(
-
\mathbf{S}^{-\frac{1}{2}}
\mathbf{V}^{\top}
\mathbf{g}^{\text{prior}}_r
\right)
=
-
\mathbf{V}\mathbf{V}^{\top}
\mathbf{g}^{\text{prior}}_r
=
-
\mathbf{g}^{\text{prior}}_r
$$

这样构造出来的先验残差满足：

$$
\left\|
\mathbf{r}^{\text{prior}}_0
+
\mathbf{J}^{\text{prior}}\delta\mathcal{X}_r
\right\|^2
$$

它展开后的二次项和一次项，正好等价于 Schur 补得到的：

$$
\mathbf{H}^{prior}_r,\quad
\mathbf{g}^{prior}_r
$$

实际系统中还会处理很小的特征值。若某个特征值接近 $0$，说明这个方向信息不足，直接求倒数会放大数值噪声。常见做法是只保留大于阈值的特征值，对接近零的方向置零。这是数值稳定性处理，不改变边缘化的数学主线。

## 7. 滑动窗口中通常边缘化谁

滑动窗口 VIO 常见有两种策略。

第一种是边缘化最老帧。当当前帧被认为是关键帧时，保留新帧，移除窗口中最老的帧。这样窗口向前滑动。

第二种是边缘化次新帧。当当前帧不是关键帧时，通常不希望它占据窗口名额，于是保留最新帧，移除倒数第二帧，并把相关 IMU 信息合并。

这两种策略服务于同一个目的：窗口中尽量保留信息量大的关键帧，同时保持固定规模。

## 8. 哪些变量会一起被边缘化

在 VIO 中，边缘化一个旧位姿时，通常还要处理与它强相关的变量。例如：

- 旧时刻的位姿；
- 旧时刻的速度；
- 旧时刻的 IMU 零偏；
- 只被旧帧或即将失效帧支撑的特征点逆深度。

为什么特征点也要考虑？因为很多特征点是以首次观测帧为锚点，用逆深度参数化的。如果这个锚点帧被移除，那么该特征点的参数化参考系也会消失。系统要么把它边缘化，要么把它转移到新的参考帧下重新参数化。

## 9. 边缘化会带来稠密先验

Schur 补有一个重要副作用：它会让保留变量之间出现新的相关性。

原来的因子图可能很稀疏。例如 IMU 因子只连接相邻帧，视觉因子只连接少数帧。但边缘化某个变量后，它曾经连接过的所有保留变量之间都会通过先验产生耦合。

这可以理解为：旧变量虽然消失了，但它像一个中介，曾经把很多变量联系在一起。删除这个中介后，为了保持等价信息，剩余变量之间必须直接建立新的相关约束。

这就是为什么边缘化先验通常是一个多元因子，而不是简单的一元因子。

## 10. 边缘化不是免费的

边缘化解决了实时性问题，但也引入几个代价。

第一，它依赖线性化。被边缘化的信息是在线性化点附近形成的二次近似，而不是原始非线性问题的精确积分。

第二，先验因子会变稠密。虽然窗口大小固定了，但先验会连接多个保留状态。

第三，一旦变量被边缘化，历史非线性因子就不能再被重新线性化。我们只保留了它们在某个点附近的线性近似。

第三点最关键。它直接引出下一篇的主题：如果边缘化先验来自某个固定线性化点，那么后续优化中应该如何使用它？能不能每次都根据当前状态重新计算它的雅克比？答案是不能随便这么做。

这就是 FEJ 的出场位置。
