# 07 学习地图：从因子图到 FEJ 再到可观性分析

## 1. 总问题

VINS-Mono 这类系统要解决的问题可以概括为：

$$
\text{如何用相机和 IMU 实时估计运动轨迹，并在丢弃旧状态时保持估计一致性？}
$$

这个问题可以拆成四层：

$$
\text{建模}
\rightarrow
\text{实时化}
\rightarrow
\text{线性化一致性}
\rightarrow
\text{可观性保持}
$$

对应到技术关键词就是：

$$
\text{Factor Graph}
\rightarrow
\text{Marginalization}
\rightarrow
\text{FEJ}
\rightarrow
\text{Observability Preservation}
$$

## 2. 第一层：因子图解决“如何融合约束”

VIO 中的未知量包括：

$$
\mathcal{X}
=
\left\{
\mathbf{x}_k,\lambda_l,\mathbf{T}_{ic}
\right\}
$$

其中：

- $\mathbf{x}_k$ 表示第 $k$ 个时刻的 IMU 状态；
- $\lambda_l$ 表示第 $l$ 个特征点的逆深度；
- $\mathbf{T}_{ic}$ 表示相机和 IMU 之间的外参。

传感器提供的约束包括：

$$
f_{\text{imu}},\quad
f_{\text{proj}},\quad
f_{\text{prior}}
$$

其中：

- $f_{\text{imu}}$ 表示 IMU 预积分因子；
- $f_{\text{proj}}$ 表示视觉重投影因子；
- $f_{\text{prior}}$ 表示先验因子。

整体优化问题是：

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

- $\mathcal{X}^{*}$ 表示最优状态；
- $\mathbf{r}_i$ 表示第 $i$ 个残差；
- $\mathbf{\Sigma}_i$ 表示第 $i$ 个残差的噪声协方差。

这一层的目的：

$$
\text{把视觉、IMU、先验等异构信息统一成一个最小二乘问题。}
$$

这一层解决的问题：

- IMU 提供高频运动约束；
- 相机提供几何约束；
- 逆深度处理单目特征点深度；
- 鲁棒核降低视觉外点影响；
- 图结构暴露稀疏性，便于高效求解。

这一层要建立的直觉：

> 因子图不是一种代码组织方式，而是最大后验估计在高斯噪声假设下的自然表达。

## 3. 第二层：边缘化解决“如何实时运行”

如果系统一直添加新状态：

$$
\mathbf{x}_0,\mathbf{x}_1,\dots,\mathbf{x}_N
$$

那么优化问题会无限增长。滑动窗口只保留：

$$
\mathcal{X}_{k:k+W}
=
\left\{
\mathbf{x}_k,\dots,\mathbf{x}_{k+W}
\right\}
$$

其中 $W$ 表示窗口长度。

但直接删除旧变量会丢失历史信息。因此把变量分成：

$$
\mathcal{X}
=
\left[
\mathcal{X}_m,\mathcal{X}_r
\right]
$$

其中：

- $\mathcal{X}_m$ 表示待边缘化变量；
- $\mathcal{X}_r$ 表示保留变量。

在线性化后的正规方程中：

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

用 Schur 补消去 $\delta\mathcal{X}_m$：

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

这一层的目的：

$$
\text{删除旧变量，同时把历史信息压缩成作用在保留变量上的先验。}
$$

这一层解决的问题：

- 控制优化变量规模；
- 保持实时性；
- 避免直接丢弃历史约束；
- 让滑动窗口可以长期运行。

这一层要建立的直觉：

> 边缘化不是删除信息，而是把旧变量携带的信息转移到剩余变量之间。

## 4. 第三层：FEJ 解决“边缘化先验如何继续使用”

边缘化得到的先验不是原始非线性因子，而是在某个线性化点附近形成的一阶近似：

$$
\mathbf{r}_{\text{prior}}(\mathcal{X}_r)
=
\mathbf{r}^{\text{prior}}_0
+
\mathbf{J}^{\text{prior}}
\left(
\mathcal{X}_r\boxminus\bar{\mathcal{X}}_r
\right)
$$

其中：

- $\mathbf{r}_{\text{prior}}$ 表示边缘化先验残差；
- $\mathbf{r}^{\text{prior}}_0$ 表示边缘化时的先验残差；
- $\mathbf{J}^{\text{prior}}$ 表示边缘化时得到的先验雅克比；
- $\bar{\mathcal{X}}_r$ 表示 first estimate；
- $\boxminus$ 表示流形差分。

FEJ 的核心要求是：

$$
\mathbf{J}^{\text{prior}}
\text{ 固定为 first estimate 处的雅克比}
$$

后续只更新：

$$
\mathcal{X}_r\boxminus\bar{\mathcal{X}}_r
$$

这一层的目的：

$$
\text{让已经边缘化的历史先验保持其原始线性化结构。}
$$

这一层解决的问题：

- 边缘化后原始非线性因子已经不存在；
- 不能严格重新边缘化已经删除的变量；
- 随意更新先验雅克比会改变历史信息含义；
- 固定 first estimate 雅克比可以减少先验结构漂移。

这一层要建立的直觉：

> 对仍在窗口内的原始非线性因子，可以重线性化；对已经压缩的边缘化先验，应固定它形成时的雅克比。

## 5. 第四层：可观性分析解决“系统不应该知道什么”

VIO 并不是所有状态都能被传感器唯一确定。对于没有全局传感器的单目 VIO，典型不可观方向是：

$$
3\text{ DOF global translation}
+
1\text{ DOF global yaw}
$$

也就是：

$$
\dim(\mathcal{N})=4
$$

其中 $\mathcal{N}$ 表示不可观子空间。

在线性化系统中，如果 $\mathbf{n}$ 是不可观方向，理想情况下应满足：

$$
\mathbf{J}\mathbf{n}
=
\mathbf{0}
$$

以及：

$$
\mathbf{H}\mathbf{n}
=
\mathbf{0}
$$

其中：

- $\mathbf{n}$ 表示不可观方向；
- $\mathbf{J}$ 表示雅克比；
- $\mathbf{H}=\mathbf{J}^{\top}\mathbf{J}$ 表示 Hessian。

如果错误线性化或错误边缘化导致：

$$
\mathbf{H}\mathbf{n}
\neq
\mathbf{0}
$$

系统就在不可观方向上产生了虚假信息。

这一层的目的：

$$
\text{保证估计器不会在传感器根本测不到的方向上过度自信。}
$$

这一层解决的问题：

- 全局平移没有绝对参考；
- 全局 yaw 没有绝对参考；
- 边缘化先验可能破坏零空间；
- FEJ 有助于保持先验零空间结构。

这一层要建立的直觉：

> 好的 VIO 系统不仅要估计轨迹，还要知道哪些东西自己无法知道。

## 6. 四个概念之间的因果关系

不要把 Factor Graph、Marginalization、FEJ、Observability 当成并列知识点。它们之间有明确因果关系：

$$
\text{传感器融合需要统一概率建模}
\Rightarrow
\text{因子图}
$$

$$
\text{因子图会无限增长}
\Rightarrow
\text{滑动窗口}
$$

$$
\text{滑动窗口不能直接丢历史}
\Rightarrow
\text{边缘化}
$$

$$
\text{边缘化只保留线性化先验}
\Rightarrow
\text{不能随便重线性化}
$$

$$
\text{固定边缘化雅克比}
\Rightarrow
\text{FEJ}
$$

$$
\text{固定线性化结构有助于保持零空间}
\Rightarrow
\text{可观性保持}
$$

## 7. 常见误区

误区一：边缘化就是删除旧帧。

更准确地说，边缘化是删除旧变量，同时把它们对保留变量的信息转化成先验。

误区二：FEJ 是为了提高数值精度。

FEJ 的主要目标不是提高非线性近似精度，而是保持边缘化先验的一致性和零空间结构。

误区三：不可观性是算法没做好。

不可观性来自传感器信息结构。没有全局观测时，全局平移和全局 yaw 本来就不可观。

误区四：只要优化残差更小，系统就更好。

残差更小不一定代表估计更一致。如果系统在不可观方向上制造虚假约束，短期残差可能很好，但长期会过度自信甚至漂移异常。

## 8. 复习时可以问自己的问题

读完这组文章后，可以用以下问题检查是否真正理解：

1. 为什么 VIO 的最大后验估计可以写成最小二乘？
2. IMU 因子和视觉因子分别约束了哪些变量？
3. 为什么滑动窗口不能直接删除最老帧？
4. Schur 补边缘化消去了什么，又保留了什么？
5. 边缘化先验为什么是线性的？
6. FEJ 中的 first estimate 指什么？
7. 为什么边缘化先验的雅克比不应随当前状态随意更新？
8. 单目 VIO 为什么全局平移不可观？
9. 单目 VIO 为什么全局 yaw 不可观？
10. FEJ 如何帮助减少不可观方向上的虚假信息？

如果这些问题都能讲清楚，就基本建立了从因子图到 FEJ 再到可观性分析的完整认知框架。

## 9. 最后一条主线

可以用一句话总结整组内容：

> 因子图负责把视觉和 IMU 约束组织成优化问题；边缘化负责让这个优化问题保持实时规模；FEJ 负责让边缘化后的历史先验不随意改变线性化结构；可观性分析负责检查系统是否在本来测不到的方向上制造了虚假信息。

