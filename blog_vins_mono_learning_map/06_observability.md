# 06 可观性分析：VIO 不应该凭空知道什么

## 1. 可观性问题问的是什么

可观性，英文是 observability。它关心的问题不是“估计结果准不准”，而是：

> 给定系统模型和传感器测量，某些状态量是否原则上可以被测量信息唯一确定？

如果一个方向是可观的，说明测量中包含足够信息来约束它。如果一个方向是不可观的，说明无论算法多强、优化多充分，仅靠这些测量都无法唯一确定它。

在 VIO 中，可观性分析尤其重要。因为 VIO 没有 GPS、磁力计或已知地图等全局观测，它只能依赖：

- 相机看到的相对几何关系；
- IMU 测到的角速度和比力；
- 重力方向提供的物理参考。

这些信息足以恢复局部运动和尺度，但并不提供完整的全局位姿。

## 2. 什么叫不可观方向

考虑一个状态向量：

$$
\mathcal{X}
$$

以及由这个状态预测出来的所有测量：

$$
\hat{\mathcal{Z}}(\mathcal{X})
$$

其中：

- $\mathcal{X}$ 表示系统状态；
- $\hat{\mathcal{Z}}(\mathcal{X})$ 表示根据状态预测出的测量。

如果存在一个非零扰动方向 $\mathbf{n}$，使得沿着这个方向改变状态后，预测测量不变：

$$
\hat{\mathcal{Z}}(\mathcal{X})
=
\hat{\mathcal{Z}}(\mathcal{X}\boxplus\epsilon\mathbf{n})
$$

对于足够小的 $\epsilon$ 成立，其中：

- $\mathbf{n}$ 表示状态空间中的一个扰动方向；
- $\epsilon$ 表示一个很小的标量；
- $\boxplus$ 表示状态流形上的扰动加法。

那么 $\mathbf{n}$ 就是一个不可观方向。

在线性化系统中，如果残差为：

$$
\mathbf{r}(\mathcal{X})
$$

在当前点附近线性化：

$$
\mathbf{r}(\mathcal{X}\boxplus\delta\mathcal{X})
\approx
\mathbf{r}(\mathcal{X})
+
\mathbf{J}\delta\mathcal{X}
$$

其中 $\mathbf{J}$ 表示残差雅克比。如果某个方向 $\mathbf{n}$ 满足：

$$
\mathbf{J}\mathbf{n}
=
\mathbf{0}
$$

则 $\mathbf{n}$ 位于雅克比的零空间中。沿着这个方向移动状态，不会改变一阶预测残差。

等价地，Hessian 为：

$$
\mathbf{H}
=
\mathbf{J}^{\top}\mathbf{J}
$$

则：

$$
\mathbf{H}\mathbf{n}
=
\mathbf{0}
$$

这说明该方向没有二次代价约束。

## 3. 单目 VIO 的全局平移不可观

假设所有位姿位置和所有地图点位置同时平移一个相同向量：

$$
\mathbf{t}_0\in\mathbb{R}^3
$$

其中 $\mathbf{t}_0$ 表示任意全局平移。新的状态为：

$$
\mathbf{p}'_k
=
\mathbf{p}_k+\mathbf{t}_0
$$

$$
\mathbf{P}'_l
=
\mathbf{P}_l+\mathbf{t}_0
$$

其中：

- $\mathbf{p}_k$ 表示第 $k$ 个 IMU 位姿的位置；
- $\mathbf{P}_l$ 表示第 $l$ 个空间点在世界坐标系下的位置；
- 上标 $'$ 表示变换后的量。

视觉观测依赖相机和空间点之间的相对位置。以第 $k$ 帧观测第 $l$ 个点为例，相机坐标系下的点为：

$$
\mathbf{P}_{c_k,l}
=
\mathbf{R}_{ck}^{\top}
\left(
\mathbf{P}_l-\mathbf{p}_{c_k}
\right)
$$

其中：

- $\mathbf{P}_{c_k,l}$ 表示点在第 $k$ 帧相机坐标系下的坐标；
- $\mathbf{R}_{ck}$ 表示第 $k$ 帧相机坐标系到世界坐标系的旋转；
- $\mathbf{p}_{c_k}$ 表示第 $k$ 帧相机在世界坐标系下的位置。

平移后：

$$
\mathbf{P}'_{c_k,l}
=
\mathbf{R}_{ck}^{\top}
\left(
\mathbf{P}_l+\mathbf{t}_0
-
\mathbf{p}_{c_k}
-
\mathbf{t}_0
\right)
$$

消去 $\mathbf{t}_0$：

$$
\mathbf{P}'_{c_k,l}
=
\mathbf{R}_{ck}^{\top}
\left(
\mathbf{P}_l-\mathbf{p}_{c_k}
\right)
=
\mathbf{P}_{c_k,l}
$$

因此投影不变：

$$
\pi(\mathbf{P}'_{c_k,l})
=
\pi(\mathbf{P}_{c_k,l})
$$

IMU 预积分约束也主要依赖相邻状态之间的相对运动：

$$
\mathbf{p}_{j}-\mathbf{p}_{i}
$$

平移后：

$$
\mathbf{p}'_{j}-\mathbf{p}'_{i}
=
\left(\mathbf{p}_{j}+\mathbf{t}_0\right)
-
\left(\mathbf{p}_{i}+\mathbf{t}_0\right)
=
\mathbf{p}_{j}-\mathbf{p}_{i}
$$

所以全局平移不会改变视觉和 IMU 相对测量。

结论：

$$
\text{VIO 的全局三维平移是不可观的}
$$

这意味着系统不应该凭空知道世界原点在哪里。

## 4. 绕重力方向的全局 yaw 不可观

IMU 可以感知重力方向。静止或低动态时，加速度计能提供关于 roll 和 pitch 的信息，因为重力给了一个竖直方向参考。但绕重力方向的旋转，也就是 yaw，通常没有绝对参考。

设世界坐标系中的重力方向为：

$$
\mathbf{g}
$$

考虑一个绕重力方向的全局旋转：

$$
\mathbf{R}_{\psi}
$$

其中：

- $\mathbf{R}_{\psi}$ 表示绕重力方向旋转 yaw 角 $\psi$ 的旋转矩阵；
- $\psi$ 表示任意 yaw 角。

因为旋转轴就是重力方向，所以：

$$
\mathbf{R}_{\psi}\mathbf{g}
=
\mathbf{g}
$$

现在对所有位姿和地图点同时做这个全局 yaw 旋转：

$$
\mathbf{R}'_k
=
\mathbf{R}_{\psi}\mathbf{R}_k
$$

$$
\mathbf{p}'_k
=
\mathbf{R}_{\psi}\mathbf{p}_k
$$

$$
\mathbf{P}'_l
=
\mathbf{R}_{\psi}\mathbf{P}_l
$$

视觉观测中，相机看到的是点相对于相机的方向。变换后：

$$
(\mathbf{R}'_k)^{\top}
\left(
\mathbf{P}'_l-\mathbf{p}'_k
\right)
=
(\mathbf{R}_{\psi}\mathbf{R}_k)^{\top}
\left(
\mathbf{R}_{\psi}\mathbf{P}_l
-
\mathbf{R}_{\psi}\mathbf{p}_k
\right)
$$

展开：

$$
=
\mathbf{R}_k^{\top}\mathbf{R}_{\psi}^{\top}
\mathbf{R}_{\psi}
\left(
\mathbf{P}_l-\mathbf{p}_k
\right)
$$

因为：

$$
\mathbf{R}_{\psi}^{\top}\mathbf{R}_{\psi}
=
\mathbf{I}
$$

所以：

$$
(\mathbf{R}'_k)^{\top}
\left(
\mathbf{P}'_l-\mathbf{p}'_k
\right)
=
\mathbf{R}_k^{\top}
\left(
\mathbf{P}_l-\mathbf{p}_k
\right)
$$

视觉投影不变。

对 IMU 运动模型而言，重力项也不变，因为：

$$
\mathbf{R}_{\psi}\mathbf{g}
=
\mathbf{g}
$$

因此绕重力方向整体旋转所有状态，不会改变相机和 IMU 的相对测量。

结论：

$$
\text{VIO 的全局 yaw 是不可观的}
$$

## 5. 单目尺度为什么在 VIO 中通常可观

纯单目视觉 SLAM 中，如果所有位置和地图点同时乘以一个尺度 $s$：

$$
\mathbf{p}'_k=s\mathbf{p}_k
$$

$$
\mathbf{P}'_l=s\mathbf{P}_l
$$

视觉投影仍然不变，因为投影会除以深度：

$$
\pi(s\mathbf{P})
=
\pi(\mathbf{P})
$$

所以纯单目视觉的尺度不可观。

但 VIO 引入了 IMU。加速度计测量有物理单位，时间间隔也已知，重力大小也提供了度量尺度。因此，经过足够激励后，单目 VIO 的尺度通常可以被估计出来。

这也是 VIO 相比纯单目的重要优势：IMU 让尺度从不可观变成可观，前提是运动激励和初始化足够好。

## 6. VIO 的典型不可观维度

在没有 GPS、磁力计、已知地图或其他全局观测的情况下，初始化后的单目 VIO 通常有四个不可观自由度：

$$
3\text{ DOF global translation}
+
1\text{ DOF global yaw}
$$

其中 DOF 表示 degree of freedom，即自由度。

可以写成：

$$
\dim(\mathcal{N})
=
4
$$

其中：

- $\mathcal{N}$ 表示系统不可观子空间；
- $\dim(\mathcal{N})$ 表示不可观子空间的维度。

这并不是算法缺陷，而是传感器信息结构决定的。

## 7. VINS 优化是不是基于相对位置

VINS 的约束大多确实是相对约束。例如：

- IMU 预积分约束相邻两帧之间的相对运动；
- 视觉重投影约束相机位姿和空间点之间的相对几何关系；
- 边缘化先验约束历史信息对当前窗口变量的影响。

但优化变量本身通常仍然写成某个世界坐标系下的状态，例如：

$$
\mathbf{p}_k,\quad \mathbf{R}_k
$$

其中：

- $\mathbf{p}_k$ 表示第 $k$ 帧 IMU 在世界坐标系下的位置；
- $\mathbf{R}_k$ 表示第 $k$ 帧 IMU 到世界坐标系的旋转。

这里的“世界坐标系”不是由相机和 IMU 绝对测出来的，而是在初始化时人为选定的参考坐标系。由于全局平移和全局 yaw 不可观，整个轨迹整体平移，或者绕重力方向整体旋转，很多测量残差并不会变化。

因此，VINS 并不是靠传感器真的测到了世界原点来固定轨迹，而是需要在优化问题中选择一个 gauge，也就是选择一种坐标系表达方式。常见做法包括：

- 初始化时把第一帧附近定义为世界坐标系参考；
- 通过边缘化先验保留历史窗口对当前窗口的约束；
- 在边缘化先验中固定线性化点和雅克比结构，避免历史约束语义漂移；
- 在求解器中用阻尼、先验或参数化处理数值上的退化方向。

所以，最老帧并不是一直作为变量留在窗口里动来动去。滑动窗口向前推进时，最老帧会被边缘化掉：

$$
\mathbf{x}_0,\mathbf{x}_1,\mathbf{x}_2,\cdots
\quad
\Longrightarrow
\quad
\mathbf{x}_0 \text{ 被删除，历史信息变成 prior}
$$

边缘化之后，$\mathbf{x}_0$ 不再参与后续优化。它对保留变量的影响被压缩成边缘化先验：

$$
\mathbf{r}_{\text{prior}}(\mathcal{X}_r)
\approx
\mathbf{r}^{\text{prior}}(\bar{\mathcal{X}}_r)
+
\mathbf{J}^{\text{prior}}(\bar{\mathcal{X}}_r)
\left(
\mathcal{X}_r\boxminus\bar{\mathcal{X}}_r
\right)
$$

其中：

- $\mathcal{X}_r$ 表示边缘化后保留下来的窗口变量；
- $\bar{\mathcal{X}}_r$ 表示边缘化发生时保存的 first estimate；
- $\mathbf{J}^{\text{prior}}(\bar{\mathcal{X}}_r)$ 表示在 first estimate 处计算并固定下来的先验雅克比。

如果边缘化先验的参考点或雅克比随着后续优化随意改变，那么历史约束的含义确实会“飘”。它可能把原本不可观的全局位置或全局 yaw 错误地约束起来，让系统产生虚假的确定性。

FEJ 的意义就在这里：它不是简单地把某一帧物理锁死，而是让边缘化先验始终相对于同一个 first estimate 来解释。当前状态 $\mathcal{X}_r$ 可以继续被优化，但历史先验的线性化结构保持一致。

可以粗略理解为：

$$
\text{不是固定最老帧不动}
$$

$$
\text{而是固定历史先验的参考线性化结构}
$$

这样既允许窗口内状态继续优化，又避免已经压缩的历史约束在不可观方向上反复改变语义。

## 8. 估计一致性是什么

估计一致性，英文是 consistency，指估计器对自身不确定性的判断是否合理。

如果一个方向本来不可观，估计器就不应该在这个方向上变得非常自信。数学上，如果 $\mathbf{n}$ 是不可观方向，那么理想情况下：

$$
\mathbf{H}\mathbf{n}
=
\mathbf{0}
$$

也就是说 Hessian 不应在这个方向上产生信息。

如果由于线性化或边缘化错误，得到：

$$
\mathbf{H}\mathbf{n}
\neq
\mathbf{0}
$$

那么优化问题就在不可观方向上产生了虚假约束。这种虚假约束会让系统误以为自己知道全局位置或全局 yaw，表现为协方差过小、估计过度自信、轨迹可能出现不合理偏置。

## 9. 为什么重线性化会破坏不可观性

非线性系统的可观性在连续模型或真实非线性模型中有其固有结构。但实际优化使用的是线性化模型。

假设某个残差在真实非线性意义下对不可观变换不敏感：

$$
\mathbf{r}(\mathcal{X})
=
\mathbf{r}(\mathcal{T}_{\alpha}(\mathcal{X}))
$$

其中：

- $\mathcal{T}_{\alpha}$ 表示某个不可观变换，例如全局平移或全局 yaw；
- $\alpha$ 表示该变换的参数。

在理想线性化点下，不可观方向 $\mathbf{n}$ 应满足：

$$
\mathbf{J}\mathbf{n}
=
\mathbf{0}
$$

但如果不同因子在不同、不一致的线性化点上被线性化，或者边缘化先验的雅克比被不恰当地更新，那么各个因子对应的零空间可能不再对齐。

可以直观理解为：每个因子都认为“不可观方向”略有不同。把它们加在一起后，公共零空间变小，甚至消失。于是系统开始在本应不可观的方向上产生约束。

## 10. 边缘化为什么特别容易引入虚假信息

普通窗口内的非线性因子还保留着原始函数。即使线性化有误，下一次迭代还可以重新线性化。

边缘化先验不同。它已经把历史非线性信息压缩成：

$$
\mathbf{r}_{\text{prior}}
=
\mathbf{r}^{\text{prior}}_0
+
\mathbf{J}^{\text{prior}}
\delta\mathcal{X}_r
$$

这个先验的零空间由 $\mathbf{J}^{\text{prior}}$ 决定。如果后续错误地改变 $\mathbf{J}^{\text{prior}}$，就等于改变了历史信息对当前状态的约束方向。

一旦先验在不可观方向上产生了非零响应：

$$
\mathbf{J}^{\text{prior}}\mathbf{n}
\neq
\mathbf{0}
$$

那么边缘化先验就会给不可观方向添加虚假信息。

因为先验会一直存在于后续优化中，这种虚假信息还会长期影响系统。

## 11. FEJ 如何帮助可观性保持

FEJ 的核心做法是：

$$
\mathbf{J}^{\text{prior}}
=
\mathbf{J}^{\text{prior}}(\bar{\mathcal{X}})
$$

其中 $\bar{\mathcal{X}}$ 表示 first estimate，也就是边缘化发生时的线性化点。

后续先验残差更新为：

$$
\mathbf{r}_{\text{prior}}(\mathcal{X})
=
\mathbf{r}^{\text{prior}}_0
+
\mathbf{J}^{\text{prior}}(\bar{\mathcal{X}})
\left(
\mathcal{X}\boxminus\bar{\mathcal{X}}
\right)
$$

这样做的意义是：边缘化先验的零空间结构不会随着后续状态估计变化而改变。也就是说，如果边缘化时该先验没有约束某个不可观方向，那么后续它不会因为雅克比更新而突然开始约束这个方向。

FEJ 并不是显式地构造不可观子空间并强制投影，而是通过固定 first estimate 雅克比，减少边缘化先验对不可观方向的虚假信息注入。

## 12. FEJ 和 OC 方法的区别

有一类方法称为 observability-constrained 方法，简称 OC 方法。它们会显式分析系统的不可观子空间，并修改或投影雅克比，使线性化模型满足预期的不可观性质。

可以抽象为：

$$
\mathbf{J}_{\text{modified}}\mathbf{N}
=
\mathbf{0}
$$

其中：

- $\mathbf{J}_{\text{modified}}$ 表示修正后的雅克比；
- $\mathbf{N}$ 表示不可观子空间基矩阵。

FEJ 更简单。它不一定显式构造 $\mathbf{N}$，而是固定某些关键雅克比在 first estimate 处，从经验和理论上缓解由重线性化造成的不可观性破坏。

可以粗略理解为：

$$
\text{OC}
\Rightarrow
\text{显式约束零空间}
$$

$$
\text{FEJ}
\Rightarrow
\text{固定线性化点以保持零空间结构}
$$

## 13. 一个直觉例子：系统不应该知道世界原点

假设一台机器人在房间里运行，只使用单目相机和 IMU。你把世界坐标系原点设在房间门口，或者设在桌子角上，对相机图像和 IMU 读数没有任何影响。

如果优化器最后声称：“我非常确定世界原点应该在桌子角上，而不是门口”，这就是不合理的。因为传感器从未测量过世界原点。

边缘化和线性化如果处理不当，就可能让系统产生类似的虚假自信。FEJ 的作用，是尽量避免历史先验在这些本来测不到的方向上越来越强。

## 14. 从因子图到可观性的一条完整逻辑

现在可以把整条链串起来：

$$
\text{VIO 测量形成因子图}
$$

$$
\Downarrow
$$

$$
\text{实时性要求固定窗口大小}
$$

$$
\Downarrow
$$

$$
\text{旧变量需要被边缘化}
$$

$$
\Downarrow
$$

$$
\text{边缘化把非线性历史压缩成线性先验}
$$

$$
\Downarrow
$$

$$
\text{线性先验必须固定其线性化结构}
$$

$$
\Downarrow
$$

$$
\text{FEJ 固定 first estimate 雅克比}
$$

$$
\Downarrow
$$

$$
\text{减少不可观方向上的虚假信息}
$$

这就是从因子图到 FEJ 再到可观性分析的核心逻辑。
