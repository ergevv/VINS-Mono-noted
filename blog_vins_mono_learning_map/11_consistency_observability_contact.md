# 11 一致性、弱可观与接触约束：从 FEJ 到 Contact Constraint 的秩分析

这一篇回答四个常见但很容易混在一起的问题：

1. 为什么 FEJ 会改善 consistency？
2. 为什么边缘化会破坏可观性？
3. 为什么某个状态只是弱可观，而不是完全可观或完全不可观？
4. 额外拓展：腿式里程计中，为什么 Contact Constraint 能提高秩？

这四个问题的共同核心是：

$$
\text{测量到底给状态空间的哪些方向提供了信息？}
$$

如果某个方向没有信息，它应该留在零空间里；如果某个方向有一点信息但很弱，它会对应很小的奇异值或特征值；如果新约束提供了独立信息，它就可能提高观测矩阵或信息矩阵的秩。

## 1. 先统一几个概念

设系统状态为：

$$
\mathbf{x}
\in
\mathbb{R}^{n}
$$

其中：

- $\mathbf{x}$ 表示需要估计的全部状态；
- $n$ 表示状态维度。

一个测量残差写成：

$$
\mathbf{r}(\mathbf{x}) =
\mathbf{z} -
h(\mathbf{x})
$$

其中：

- $\mathbf{r}(\mathbf{x})$ 表示残差；
- $\mathbf{z}$ 表示传感器测量；
- $h(\mathbf{x})$ 表示由状态预测测量的函数。

在当前线性化点 $\bar{\mathbf{x}}$ 处做一阶近似：

$$
\mathbf{r}(\bar{\mathbf{x}}\boxplus\delta\mathbf{x}) \approx
\mathbf{r}(\bar{\mathbf{x}}) +
\mathbf{J}\delta\mathbf{x}
$$

其中：

- $\bar{\mathbf{x}}$ 表示线性化点；
- $\delta\mathbf{x}$ 表示局部状态扰动；
- $\boxplus$ 表示流形上的加法；
- $\mathbf{J}$ 表示残差对局部扰动的 Jacobian。

加权最小二乘的二次项为：

$$
\left\|
\mathbf{r}
\right\|_{\mathbf{\Omega}}^2 =
\mathbf{r}^{\top}\mathbf{\Omega}\mathbf{r}
$$

其中：

- $\mathbf{\Omega}$ 表示信息矩阵；
- $\mathbf{\Omega}=\mathbf{\Sigma}^{-1}$；
- $\mathbf{\Sigma}$ 表示测量噪声协方差。

线性化后，对状态增量的近似 Hessian 为：

$$
\mathbf{H} =
\mathbf{J}^{\top}
\mathbf{\Omega}
\mathbf{J}
$$

如果有多个残差，则：

$$
\mathbf{H} =
\sum_i
\mathbf{J}_i^{\top}
\mathbf{\Omega}_i
\mathbf{J}_i
$$

其中：

- $\mathbf{J}_i$ 表示第 $i$ 个残差的 Jacobian；
- $\mathbf{\Omega}_i$ 表示第 $i$ 个残差的信息矩阵。

这个 $\mathbf{H}$ 可以理解为局部信息矩阵。某个方向 $\mathbf{n}$ 上的信息量可以看：

$$
\mathbf{n}^{\top}
\mathbf{H}
\mathbf{n}
$$

其中 $\mathbf{n}$ 表示状态空间中的一个扰动方向。

如果：

$$
\mathbf{n}^{\top}
\mathbf{H}
\mathbf{n} =
0
$$

则该方向没有被当前线性化测量约束。由于 $\mathbf{H}$ 是半正定矩阵，上式等价于：

$$
\mathbf{H}\mathbf{n} =
\mathbf{0}
$$

也等价于：

$$
\mathbf{J}_i\mathbf{n} =
\mathbf{0}
\quad
\text{对所有残差 } i
$$

这就是零空间与不可观方向的联系。

## 2. 可观、不可观、弱可观

### 2.1 不可观

如果存在非零方向 $\mathbf{n}$，使得所有测量对这个方向都没有响应：

$$
\mathbf{J}\mathbf{n} =
\mathbf{0}
$$

那么 $\mathbf{n}$ 是不可观方向。

例如在没有 GPS、磁力计、已知地图的 VIO 中，全局平移和全局 yaw 通常不可观：

$$
\dim(\mathcal{N})=4
$$

其中：

- $\mathcal{N}$ 表示不可观子空间；
- $4$ 表示三维全局平移加一维全局 yaw。

### 2.2 可观

如果某个方向 $\mathbf{n}$ 满足：

$$
\mathbf{J}\mathbf{n} \neq
\mathbf{0}
$$

说明沿着这个方向改变状态会改变测量预测，因此测量对这个方向有信息。

从信息矩阵看，就是：

$$
\mathbf{n}^{\top}\mathbf{H}\mathbf{n} >
0
$$

### 2.3 弱可观

弱可观不是“完全不可观”。它表示这个方向确实有信息，但信息很弱。

对 Jacobian 做奇异值分解：

$$
\mathbf{J} =
\mathbf{U}
\mathbf{S}
\mathbf{V}^{\top}
$$

其中：

- $\mathbf{U}$ 表示左奇异向量矩阵；
- $\mathbf{S}$ 表示奇异值对角矩阵；
- $\mathbf{V}$ 表示右奇异向量矩阵。

如果某个方向对应的奇异值为零：

$$
\sigma_i=0
$$

该方向不可观。

如果某个方向对应的奇异值很小但非零：

$$
0<\sigma_i\ll 1
$$

该方向弱可观。

对应到 Hessian：

$$
\mathbf{H} =
\mathbf{J}^{\top}\mathbf{J} =
\mathbf{V}
\mathbf{S}^{\top}\mathbf{S}
\mathbf{V}^{\top}
$$

所以 Hessian 的特征值近似为：

$$
\lambda_i =
\sigma_i^2
$$

弱可观方向就是：

$$
0<\lambda_i\ll 1
$$

直观解释是：

$$
\text{这个方向不是完全测不到，但测得很软、很慢、很容易受噪声影响。}
$$

## 3. Consistency 到底是什么意思

Consistency 通常翻译为“估计一致性”。它关心的是：

$$
\text{估计器对自身不确定性的判断是否合理？}
$$

如果一个方向本来不可观，估计器就不应该在这个方向上变得非常自信。

设估计误差为：

$$
\tilde{\mathbf{x}} =
\mathbf{x} -
\hat{\mathbf{x}}
$$

其中：

- $\mathbf{x}$ 表示真实状态；
- $\hat{\mathbf{x}}$ 表示估计状态；
- $\tilde{\mathbf{x}}$ 表示估计误差。

设估计器给出的协方差为：

$$
\mathbf{P} =
\mathbb{E}
\left[
\tilde{\mathbf{x}}\tilde{\mathbf{x}}^{\top}
\right]
$$

现在取状态空间中的一个方向：

$$
\mathbf{n}
$$

其中 $\mathbf{n}$ 可以理解为“我们关心的某个状态扰动方向”。例如在 VIO 中，$\mathbf{n}$ 可以表示全局平移方向，也可以表示全局 yaw 方向。

如果估计误差为 $\tilde{\mathbf{x}}$，那么误差在方向 $\mathbf{n}$ 上的投影是一个标量：

$$
e_n =
\mathbf{n}^{\top}\tilde{\mathbf{x}}
$$

其中：

- $e_n$ 表示估计误差沿方向 $\mathbf{n}$ 的分量；
- 如果 $e_n$ 很大，说明状态在这个方向上误差很大；
- 如果 $e_n$ 很小，说明状态在这个方向上估计得比较准。

这个标量误差的方差为：

$$
\operatorname{Var}(e_n) =
\mathbb{E}
\left[
e_n^2
\right]
$$

代入 $e_n=\mathbf{n}^{\top}\tilde{\mathbf{x}}$：

$$
\operatorname{Var}(e_n) =
\mathbb{E}
\left[
\left(
\mathbf{n}^{\top}\tilde{\mathbf{x}}
\right)
\left(
\mathbf{n}^{\top}\tilde{\mathbf{x}}
\right)
\right]
$$

把第二项改写成：

$$
\mathbf{n}^{\top}\tilde{\mathbf{x}} =
\tilde{\mathbf{x}}^{\top}\mathbf{n}
$$

于是：

$$
\operatorname{Var}(e_n) =
\mathbb{E}
\left[
\mathbf{n}^{\top}
\tilde{\mathbf{x}}
\tilde{\mathbf{x}}^{\top}
\mathbf{n}
\right]
$$

因为 $\mathbf{n}$ 不是随机变量，可以移到期望外面：

$$
\operatorname{Var}(e_n) =
\mathbf{n}^{\top}
\mathbb{E}
\left[
\tilde{\mathbf{x}}
\tilde{\mathbf{x}}^{\top}
\right]
\mathbf{n}
$$

再利用协方差定义：

$$
\mathbf{P} =
\mathbb{E}
\left[
\tilde{\mathbf{x}}
\tilde{\mathbf{x}}^{\top}
\right]
$$

得到：

$$
\mathbf{n}^{\top}
\mathbf{P}
\mathbf{n}
$$

也就是：

$$
\operatorname{Var}(e_n) =
\mathbf{n}^{\top}
\mathbf{P}
\mathbf{n}
$$

这就是 $\mathbf{n}^{\top}\mathbf{P}\mathbf{n}$ 的含义：它表示估计误差在方向 $\mathbf{n}$ 上的方差。

如果方向 $\mathbf{n}$ 是不可观方向，系统没有真实测量信息来约束它。因此估计器不应该让：

$$
\mathbf{n}^{\top}
\mathbf{P}
\mathbf{n}
$$

变得过小。因为过小意味着估计器认为“我在这个方向上很确定”，而这和不可观性矛盾。

从信息矩阵角度看：

$$
\mathbf{P} \approx
\mathbf{H}^{-1}
$$

这个式子来自高斯近似。线性化后的最小二乘代价可以写成：

$$
E(\delta\mathbf{x}) =
\frac{1}{2}
\delta\mathbf{x}^{\top}
\mathbf{H}
\delta\mathbf{x}
$$

其中：

- $E(\delta\mathbf{x})$ 表示局部二次能量；
- $\delta\mathbf{x}$ 表示状态扰动；
- $\mathbf{H}$ 表示 Hessian 或信息矩阵。

对应的高斯分布为：

$$
p(\delta\mathbf{x})
\propto
\exp
\left( -
\frac{1}{2}
\delta\mathbf{x}^{\top}
\mathbf{H}
\delta\mathbf{x}
\right)
$$

而一个高斯分布也可以写成：

$$
p(\delta\mathbf{x})
\propto
\exp
\left( -
\frac{1}{2}
\delta\mathbf{x}^{\top}
\mathbf{P}^{-1}
\delta\mathbf{x}
\right)
$$

对比这两个式子，就得到：

$$
\mathbf{H} \approx
\mathbf{P}^{-1}
$$

也就是：

$$
\mathbf{P} \approx
\mathbf{H}^{-1}
$$

严格来说，如果系统存在不可观方向，$\mathbf{H}$ 是奇异的，不能直接求普通逆。这时可以理解为在可观子空间上求逆，或者使用伪逆、阻尼、gauge fixing 后的近似协方差。这里写 $\mathbf{P}\approx\mathbf{H}^{-1}$，表达的是“信息越大，协方差越小”的局部高斯近似关系。

为了看清楚方向 $\mathbf{n}$ 上的信息和方差的关系，假设 $\mathbf{n}$ 是单位方向：

$$
\|\mathbf{n}\|=1
$$

并且它是 $\mathbf{H}$ 的一个特征方向：

$$
\mathbf{H}\mathbf{n} =
\lambda\mathbf{n}
$$

其中 $\lambda$ 表示该方向上的信息强度。沿着这个方向取扰动：

$$
\delta\mathbf{x} =
s\mathbf{n}
$$

其中 $s$ 表示沿方向 $\mathbf{n}$ 移动的标量幅度。代入二次能量：

$$
E(s\mathbf{n}) =
\frac{1}{2}
s^2
\mathbf{n}^{\top}
\mathbf{H}
\mathbf{n}
$$

由于 $\mathbf{H}\mathbf{n}=\lambda\mathbf{n}$ 且 $\mathbf{n}^{\top}\mathbf{n}=1$：

$$
E(s\mathbf{n}) =
\frac{1}{2}
\lambda s^2
$$

这说明 $\lambda$ 越大，沿方向 $\mathbf{n}$ 偏离一点点的代价越大，优化器越“确信”这个方向不能乱动。

如果 $\lambda>0$，对应协方差在这个方向上的大小近似为：

$$
\mathbf{n}^{\top}
\mathbf{P}
\mathbf{n} \approx
\frac{1}{\lambda}
$$

如果 $\lambda$ 很大：

$$
\lambda\uparrow \Rightarrow
\mathbf{n}^{\top}\mathbf{P}\mathbf{n}
\downarrow
$$

如果 $\lambda=0$：

$$
E(s\mathbf{n})=0
$$

表示沿这个方向怎么移动都没有二次代价。这个方向就是不可观方向，理论上它对应无限大不确定性，或者在数值系统中表现为非常大的协方差。

因此，如果某个本来不可观的方向 $\mathbf{n}$ 被错误地加入了信息：

$$
\mathbf{n}^{\top}\mathbf{H}\mathbf{n} >
0
$$

那么对应协方差会变小：

$$
\mathbf{n}^{\top}\mathbf{P}\mathbf{n}
\downarrow
$$

这就叫不一致：系统误以为自己知道了其实传感器没有告诉它的东西。

在 VIO 中，这种不一致常表现为：

- 全局位置方向协方差过小；
- 全局 yaw 方向被错误约束；
- bias 或尺度在激励不足时被估计得过度自信；
- 轨迹短期看起来平滑，但长期误差或漂移模式异常。

## 4. 为什么 FEJ 会改善 Consistency

FEJ 的全称是 First Estimate Jacobian。核心做法是：

$$
\text{在 first estimate 处计算 Jacobian，并固定使用。}
$$

设某个残差为：

$$
\mathbf{r}(\mathbf{x})
$$

普通重线性化会在每次迭代的当前估计 $\mathbf{x}^{(t)}$ 处计算：

$$
\mathbf{J}^{(t)} =
\frac{\partial\mathbf{r}}
{\partial\delta\mathbf{x}}
\bigg|_{\mathbf{x}^{(t)}}
$$

FEJ 则使用第一次估计 $\bar{\mathbf{x}}$：

$$
\mathbf{J}_{FEJ} =
\frac{\partial\mathbf{r}}
{\partial\delta\mathbf{x}}
\bigg|_{\bar{\mathbf{x}}}
$$

后续即使当前状态变成 $\mathbf{x}^{(t)}$，仍然使用：

$$
\mathbf{J}_{FEJ}
$$

但残差中的状态差分仍然可以变化：

$$
\delta\mathbf{x} =
\mathbf{x}^{(t)}\boxminus\bar{\mathbf{x}}
$$

### 4.1 真正的问题不是“Jacobian 变了”，而是“零空间变了”

假设真实非线性系统有一个不可观变换：

$$
\mathcal{T}_{\alpha}(\mathbf{x})
$$

其中：

- $\mathcal{T}_{\alpha}$ 表示对状态施加某个不可观变换；
- $\alpha$ 表示该变换的参数。

不可观意味着：

$$
\mathbf{r}(\mathbf{x}) =
\mathbf{r}(\mathcal{T}_{\alpha}(\mathbf{x}))
$$

对 $\alpha$ 求导，可以得到对应的不可观方向：

$$
\mathbf{N}(\mathbf{x}) =
\frac{\partial \mathcal{T}_{\alpha}(\mathbf{x})}
{\partial\alpha}
\bigg|_{\alpha=0}
$$

其中 $\mathbf{N}(\mathbf{x})$ 是不可观子空间基矩阵。

理想线性化应该满足：

$$
\mathbf{J}(\mathbf{x})\mathbf{N}(\mathbf{x}) =
\mathbf{0}
$$

注意这里的零空间 $\mathbf{N}(\mathbf{x})$ 依赖当前线性化点 $\mathbf{x}$。

如果不同因子在不同点线性化：

$$
\mathbf{J}_1(\mathbf{x}_1),\quad
\mathbf{J}_2(\mathbf{x}_2)
$$

那么它们各自满足的零空间可能是：

$$
\mathbf{N}(\mathbf{x}_1),\quad
\mathbf{N}(\mathbf{x}_2)
$$

这两个零空间不一定完全相同。

组合后的信息矩阵为：

$$
\mathbf{H} =
\mathbf{J}_1^{\top}\mathbf{\Omega}_1\mathbf{J}_1 +
\mathbf{J}_2^{\top}\mathbf{\Omega}_2\mathbf{J}_2
$$

如果不存在一个共同方向 $\mathbf{n}$ 同时满足：

$$
\mathbf{J}_1\mathbf{n} =
\mathbf{0}
$$

$$
\mathbf{J}_2\mathbf{n} =
\mathbf{0}
$$

那么原本应该不可观的方向就可能被约束住。

FEJ 的作用，就是让相关 Jacobian 尽量在同一个 first estimate 上定义，从而共享同一个零空间：

$$
\mathbf{J}_1(\bar{\mathbf{x}})\mathbf{N}(\bar{\mathbf{x}}) \approx
\mathbf{0}
$$

$$
\mathbf{J}_2(\bar{\mathbf{x}})\mathbf{N}(\bar{\mathbf{x}}) \approx
\mathbf{0}
$$

这样组合后仍然有：

$$
\mathbf{H}\mathbf{N}(\bar{\mathbf{x}}) \approx
\mathbf{0}
$$

### 4.2 FEJ 改善 Consistency 的本质

FEJ 改善 consistency，不是因为它让线性化更精确。恰恰相反，固定 Jacobian 有时会牺牲非线性精度。

它改善 consistency 的原因是：

$$
\text{FEJ 更好地保持了不可观方向的零空间结构。}
$$

换句话说：

$$
\text{系统本来不知道的东西，FEJ 不让线性化过程假装知道。}
$$

这会减少：

$$
\mathbf{n}^{\top}\mathbf{H}\mathbf{n}>0
$$

在不可观方向 $\mathbf{n}$ 上错误出现的概率。

因此 FEJ 的收益主要是：

- 避免不可观方向被错误约束；
- 避免协方差在不可观方向上过度收缩；
- 降低估计器过度自信；
- 提升长期一致性。

FEJ 的代价是：

- 如果 first estimate 很差，固定 Jacobian 会降低局部精度；
- 如果状态远离 first estimate，线性模型误差会增大；
- 它主要解决一致性，不保证每个数据集上误差最小。

### 4.3 FEJ 是缓解，不是完美消除影响

这里要避免一个误解：FEJ 不是让 Jacobian 更“实时”、更“精确”。它恰恰是不实时更新某些 Jacobian。

普通重线性化使用：

$$
\mathbf{J} =
\mathbf{J}(\mathbf{x}^{current})
$$

其中 $\mathbf{x}^{current}$ 表示当前估计状态。它的优点是更贴近当前非线性函数；缺点是历史 prior、视觉因子、IMU 因子可能在不同线性化点上拥有不同零空间。

FEJ 使用：

$$
\mathbf{J} =
\mathbf{J}(\bar{\mathbf{x}})
$$

其中 $\bar{\mathbf{x}}$ 表示 first estimate。它的优点是零空间结构更稳定；缺点是当：

$$
\mathbf{x}^{current} \neq
\bar{\mathbf{x}}
$$

并且二者差距变大时，线性化误差也会变大。

所以 FEJ 的核心取舍是：

$$
\text{牺牲一部分当前线性化精度} \Longleftrightarrow
\quad
\text{换取更好的零空间一致性}
$$

更具体地说，FEJ 不是通过“人为增大协方差”来改善一致性，而是通过让不可观方向尽量继续满足：

$$
\mathbf{J}(\bar{\mathbf{x}})
\mathbf{N}(\bar{\mathbf{x}}) \approx
\mathbf{0}
$$

从而避免总信息矩阵在不可观方向上错误出现：

$$
\mathbf{N}^{\top}
\mathbf{H}
\mathbf{N} >
\mathbf{0}
$$

如果 first estimate 很差，或者后续状态离 first estimate 太远，那么 FEJ 固定下来的 Jacobian 也会带来误差。也就是说：

$$
\text{FEJ 能缓解 inconsistency} \neq
\text{FEJ 能完全消除 inconsistency}
$$

它只是把主要风险从：

$$
\text{不可观方向被错误约束}
$$

转移成：

$$
\text{固定 Jacobian 带来的线性化误差}
$$

在 VIO 中，这个取舍常常是值得的，因为不可观方向上的虚假信息会让系统长期过度自信，而 FEJ 的线性化误差可以通过较好的初始化、较短滑窗、频繁更新窗口来控制。

### 4.4 严格 FEJ 和 FEJ-like 的具体区别

前面一直在说 FEJ，但工程里经常会看到两种不同做法：

$$
\text{strict FEJ}
\quad
\text{和}
\quad
\text{FEJ-like}
$$

二者的区别不在于“有没有固定某个 Jacobian”，而在于：

$$
\text{是否让所有影响可观性的相关 Jacobian 都使用 first estimate。}
$$

#### 严格 FEJ 要维护两份状态

对每一个进入估计器的状态块 $\mathbf{x}_i$，严格 FEJ 会维护两份值：

$$
\mathbf{x}_i^{cur}
\quad
\text{和}
\quad
\bar{\mathbf{x}}_i^{FEJ}
$$

其中：

- $\mathbf{x}_i^{cur}$ 表示当前估计值，会随着优化不断更新；
- $\bar{\mathbf{x}}_i^{FEJ}$ 表示 first estimate，也就是该状态第一次进入系统时保存下来的估计值；
- 下标 $i$ 表示第 $i$ 个状态块，例如第 $i$ 帧位姿、速度、bias，或者第 $i$ 个特征点参数；
- 上标 $cur$ 表示 current estimate；
- 上标 $FEJ$ 表示用于计算 FEJ Jacobian 的固定参考值。

当一个新状态第一次进入系统时，保存：

$$
\bar{\mathbf{x}}_i^{FEJ} \leftarrow
\mathbf{x}_i^{init}
$$

其中 $\mathbf{x}_i^{init}$ 表示该状态的初始估计。

之后优化过程中，只更新当前值：

$$
\mathbf{x}_i^{cur} \leftarrow
\mathbf{x}_i^{cur}\boxplus\delta\mathbf{x}_i
$$

但 first estimate 不再改变：

$$
\bar{\mathbf{x}}_i^{FEJ} \leftarrow
\bar{\mathbf{x}}_i^{FEJ}
$$

其中：

- $\delta\mathbf{x}_i$ 表示优化求出来的局部状态增量；
- $\boxplus$ 表示流形上的加法，例如位置可以直接相加，旋转则需要通过指数映射更新。

对 VIO 状态来说，一帧 IMU 状态通常包含：

$$
\mathbf{x}_i =
\left[
\mathbf{R}_i,\,
\mathbf{p}_i,\,
\mathbf{v}_i,\,
\mathbf{b}_{a_i},\,
\mathbf{b}_{g_i}
\right]
$$

其中：

- $\mathbf{R}_i$ 表示第 $i$ 帧机体系到世界系的旋转；
- $\mathbf{p}_i$ 表示第 $i$ 帧机体系在世界系下的位置；
- $\mathbf{v}_i$ 表示第 $i$ 帧速度；
- $\mathbf{b}_{a_i}$ 表示第 $i$ 帧加速度计 bias；
- $\mathbf{b}_{g_i}$ 表示第 $i$ 帧陀螺仪 bias。

严格 FEJ 会同时保存：

$$
\mathbf{x}_i^{cur} =
\left[
\mathbf{R}_i^{cur},\,
\mathbf{p}_i^{cur},\,
\mathbf{v}_i^{cur},\,
\mathbf{b}_{a_i}^{cur},\,
\mathbf{b}_{g_i}^{cur}
\right]
$$

以及：

$$
\bar{\mathbf{x}}_i^{FEJ} =
\left[
\bar{\mathbf{R}}_i^{FEJ},\,
\bar{\mathbf{p}}_i^{FEJ},\,
\bar{\mathbf{v}}_i^{FEJ},\,
\bar{\mathbf{b}}_{a_i}^{FEJ},\,
\bar{\mathbf{b}}_{g_i}^{FEJ}
\right]
$$

如果特征点用逆深度 $\lambda_j$ 表示，也会保存：

$$
\lambda_j^{cur}
\quad
\text{和}
\quad
\bar{\lambda}_j^{FEJ}
$$

其中 $\lambda_j$ 表示第 $j$ 个特征点的逆深度。

#### 普通重线性化怎么做

设第 $k$ 个因子的残差为：

$$
\mathbf{r}_k(\mathbf{x}_{\mathcal{S}_k})
$$

其中：

- $\mathbf{r}_k$ 表示第 $k$ 个残差；
- $\mathcal{S}_k$ 表示这个因子连接的状态块集合；
- $\mathbf{x}_{\mathcal{S}_k}$ 表示该因子依赖的所有状态。

普通 Gauss-Newton 或 Levenberg-Marquardt 会在当前状态处线性化：

$$
\mathbf{r}_k
\left(
\mathbf{x}_{\mathcal{S}_k}^{cur}\boxplus\delta\mathbf{x}_{\mathcal{S}_k}
\right) \approx
\mathbf{r}_k
\left(
\mathbf{x}_{\mathcal{S}_k}^{cur}
\right) +
\mathbf{J}_k
\left(
\mathbf{x}_{\mathcal{S}_k}^{cur}
\right)
\delta\mathbf{x}_{\mathcal{S}_k}
$$

也就是残差值和 Jacobian 都围绕当前状态：

$$
\mathbf{r}_k^{cur} =
\mathbf{r}_k(\mathbf{x}^{cur})
$$

$$
\mathbf{J}_k^{cur} =
\left.
\frac{\partial\mathbf{r}_k}{\partial\delta\mathbf{x}}
\right|_{\mathbf{x}=\mathbf{x}^{cur}}
$$

这种做法局部精度高，但不同因子会随着状态更新在不同位置反复重线性化，零空间结构可能不断变化。

#### 严格 FEJ 怎么做

严格 FEJ 的关键做法是：

$$
\text{残差值可以用当前状态计算，Jacobian 用 first estimate 计算。}
$$

也就是：

$$
\mathbf{r}_k^{cur} =
\mathbf{r}_k
\left(
\mathbf{x}_{\mathcal{S}_k}^{cur}
\right)
$$

但 Jacobian 使用：

$$
\mathbf{J}_k^{FEJ} =
\left.
\frac{\partial\mathbf{r}_k}{\partial\delta\mathbf{x}_{\mathcal{S}_k}}
\right|_{\mathbf{x}_{\mathcal{S}_k} =
\bar{\mathbf{x}}_{\mathcal{S}_k}^{FEJ}}
$$

于是严格 FEJ 的线性化模型是：

$$
\mathbf{r}_k
\left(
\mathbf{x}^{cur}\boxplus\delta\mathbf{x}
\right) \approx
\mathbf{r}_k
\left(
\mathbf{x}^{cur}
\right) +
\mathbf{J}_k^{FEJ}
\delta\mathbf{x}
$$

注意，这个式子不是严格意义上的 Taylor 展开，因为残差值和 Jacobian 不是在同一个点计算的。它是一个 consistency-preserving approximation，也就是为了保持一致性而采用的近似。

整体优化问题写成：

$$
\min_{\delta\mathbf{x}}
\sum_k
\left\|
\mathbf{r}_k
\left(
\mathbf{x}^{cur}
\right) +
\mathbf{J}_k^{FEJ}
\delta\mathbf{x}
\right\|_{\mathbf{\Omega}_k}^{2}
$$

其中 $\mathbf{\Omega}_k$ 表示第 $k$ 个因子的信息矩阵。

求解出 $\delta\mathbf{x}$ 后，只更新当前状态：

$$
\mathbf{x}^{cur} \leftarrow
\mathbf{x}^{cur}\boxplus\delta\mathbf{x}
$$

但仍然保持：

$$
\bar{\mathbf{x}}^{FEJ}
\text{ 不变}
$$

下一轮优化时，重复同样的规则：

$$
\mathbf{r}_k
\text{ 用 }
\mathbf{x}^{cur}
\text{ 算}
$$

$$
\mathbf{J}_k
\text{ 用 }
\bar{\mathbf{x}}^{FEJ}
\text{ 算}
$$

#### 新进来的因子如何计算 Jacobian

严格 FEJ 中，“新因子”的 Jacobian 不是随便固定，也不是说新因子自己有一个 first estimate。更准确地说：

$$
\text{新因子的 Jacobian 由它连接的变量的 first estimate 决定。}
$$

设新加入的因子为：

$$
\mathbf{r}_k =
\mathbf{r}_k
\left(
\mathbf{x}_{\mathcal{S}_k}
\right)
$$

其中：

- $\mathbf{r}_k$ 表示新加入的第 $k$ 个残差；
- $\mathcal{S}_k$ 表示这个因子连接的变量索引集合；
- $\mathbf{x}_{\mathcal{S}_k}$ 表示这个因子依赖的所有变量。

严格 FEJ 计算该因子的 Jacobian 时，先检查它连接的每个变量是否已经有 first estimate：

$$
\bar{\mathbf{x}}_s^{FEJ},
\quad
s\in\mathcal{S}_k
$$

其中 $s$ 表示该因子连接的某一个变量索引。

变量的 first estimate 来源分几种情况：

- 如果 $\mathbf{x}_s$ 是旧状态，它在更早进入滑窗时已经保存了 $\bar{\mathbf{x}}_s^{FEJ}$；
- 如果 $\mathbf{x}_s$ 是刚加入的新帧，就在新帧初始化并进入估计器时立刻保存 $\bar{\mathbf{x}}_s^{FEJ}$；
- 如果 $\mathbf{x}_s$ 是刚初始化的新特征点，就在特征点第一次被参数化时保存 $\bar{\lambda}_s^{FEJ}$ 或 $\bar{\mathbf{P}}_s^{FEJ}$；
- 如果某个变量还没有初始化，就不能稳定地构造这个变量对应的 FEJ Jacobian，通常要延迟加入该因子，或者等变量完成初始化。

因此，新因子的 FEJ Jacobian 是：

$$
\mathbf{J}_k^{FEJ} =
\left.
\frac{\partial\mathbf{r}_k}
{\partial\delta\mathbf{x}_{\mathcal{S}_k}}
\right|_{
\mathbf{x}_s=\bar{\mathbf{x}}_s^{FEJ},
\ s\in\mathcal{S}_k
}
$$

而它的残差值仍然可以用当前状态计算：

$$
\mathbf{r}_k^{cur} =
\mathbf{r}_k
\left(
\mathbf{x}_{\mathcal{S}_k}^{cur}
\right)
$$

最终进入线性系统的是：

$$
\mathbf{r}_k \approx
\mathbf{r}_k^{cur} +
\mathbf{J}_k^{FEJ}
\delta\mathbf{x}_{\mathcal{S}_k}
$$

这句话非常重要：

$$
\text{新因子新不新不关键，关键是它连接的变量各自有没有 first estimate。}
$$

##### 新视觉因子的例子

假设一个新视觉重投影因子连接：

$$
\mathbf{x}_i,\quad
\mathbf{x}_j,\quad
\lambda_l
$$

其中：

- $\mathbf{x}_i$ 表示特征点的锚定帧状态；
- $\mathbf{x}_j$ 表示当前观测帧状态；
- $\lambda_l$ 表示第 $l$ 个特征点的逆深度。

如果第 $i$ 帧是旧帧，那么它早已有：

$$
\bar{\mathbf{x}}_i^{FEJ}
$$

如果第 $j$ 帧是刚进来的新帧，则在它进入滑窗时保存：

$$
\bar{\mathbf{x}}_j^{FEJ} \leftarrow
\mathbf{x}_j^{init}
$$

如果特征点 $\lambda_l$ 是刚初始化的，则保存：

$$
\bar{\lambda}_l^{FEJ} \leftarrow
\lambda_l^{init}
$$

于是这个新视觉因子的 Jacobian 在严格 FEJ 下为：

$$
\mathbf{J}_{ij,l}^{FEJ} =
\left.
\frac{\partial\mathbf{r}_{ij,l}}
{\partial
\left[
\delta\mathbf{x}_i,\,
\delta\mathbf{x}_j,\,
\delta\lambda_l
\right]}
\right|_{\substack{
\mathbf{x}_i=\bar{\mathbf{x}}_i^{FEJ},\\
\mathbf{x}_j=\bar{\mathbf{x}}_j^{FEJ},\\
\lambda_l=\bar{\lambda}_l^{FEJ}
}}
$$

但残差值可以用当前状态：

$$
\mathbf{r}_{ij,l}^{cur} =
\mathbf{r}_{ij,l}
\left(
\mathbf{x}_i^{cur},
\mathbf{x}_j^{cur},
\lambda_l^{cur}
\right)
$$

所以这个新因子加入系统时，线性化形式是：

$$
\mathbf{r}_{ij,l} \approx
\mathbf{r}_{ij,l}^{cur} +
\mathbf{J}_{ij,l}^{FEJ}
\begin{bmatrix}
\delta\mathbf{x}_i\\
\delta\mathbf{x}_j\\
\delta\lambda_l
\end{bmatrix}
$$

##### 新 IMU 因子的例子

IMU 预积分因子通常连接相邻两帧：

$$
\mathbf{x}_i,\quad
\mathbf{x}_{i+1}
$$

若第 $i+1$ 帧刚加入系统，则先保存：

$$
\bar{\mathbf{x}}_{i+1}^{FEJ} \leftarrow
\mathbf{x}_{i+1}^{init}
$$

严格 FEJ 下，新 IMU 因子的 Jacobian 为：

$$
\mathbf{J}_{imu,i}^{FEJ} =
\left.
\frac{\partial\mathbf{r}_{imu,i}}
{\partial
\left[
\delta\mathbf{x}_i,\,
\delta\mathbf{x}_{i+1}
\right]}
\right|_{\substack{
\mathbf{x}_i=\bar{\mathbf{x}}_i^{FEJ},\\
\mathbf{x}_{i+1}=\bar{\mathbf{x}}_{i+1}^{FEJ}
}}
$$

而 IMU 残差值可以用：

$$
\mathbf{r}_{imu,i}^{cur} =
\mathbf{r}_{imu,i}
\left(
\mathbf{x}_i^{cur},
\mathbf{x}_{i+1}^{cur}
\right)
$$

所以新 IMU 因子同样遵循：

$$
\text{残差看当前状态，Jacobian 看 first estimate。}
$$

用流程表示就是：

1. 新变量进入系统，保存 first estimate；
2. 新因子进入系统，找到它连接的变量集合 $\mathcal{S}_k$；
3. 用 $\bar{\mathbf{x}}_{\mathcal{S}_k}^{FEJ}$ 计算 Jacobian；
4. 用 $\mathbf{x}_{\mathcal{S}_k}^{cur}$ 计算残差；
5. 求解增量后只更新 current estimate，不更新 first estimate。

#### 多个变量的因子怎么处理

假设一个视觉重投影因子连接：

$$
\mathbf{x}_i,\quad
\mathbf{x}_j,\quad
\lambda_l
$$

其中：

- $\mathbf{x}_i$ 表示特征点第一次观测所在帧的状态；
- $\mathbf{x}_j$ 表示当前观测帧的状态；
- $\lambda_l$ 表示第 $l$ 个特征点的逆深度。

普通重线性化会用：

$$
\mathbf{J}_{k,i}^{cur} =
\left.
\frac{\partial\mathbf{r}_k}{\partial\delta\mathbf{x}_i}
\right|_{\substack{
\mathbf{x}_i=\mathbf{x}_i^{cur},\\
\mathbf{x}_j=\mathbf{x}_j^{cur},\\
\lambda_l=\lambda_l^{cur}
}}
$$

$$
\mathbf{J}_{k,j}^{cur} =
\left.
\frac{\partial\mathbf{r}_k}{\partial\delta\mathbf{x}_j}
\right|_{\substack{
\mathbf{x}_i=\mathbf{x}_i^{cur},\\
\mathbf{x}_j=\mathbf{x}_j^{cur},\\
\lambda_l=\lambda_l^{cur}
}}
$$

严格 FEJ 则使用：

$$
\mathbf{J}_{k,i}^{FEJ} =
\left.
\frac{\partial\mathbf{r}_k}{\partial\delta\mathbf{x}_i}
\right|_{\substack{
\mathbf{x}_i=\bar{\mathbf{x}}_i^{FEJ},\\
\mathbf{x}_j=\bar{\mathbf{x}}_j^{FEJ},\\
\lambda_l=\bar{\lambda}_l^{FEJ}
}}
$$

$$
\mathbf{J}_{k,j}^{FEJ} =
\left.
\frac{\partial\mathbf{r}_k}{\partial\delta\mathbf{x}_j}
\right|_{\substack{
\mathbf{x}_i=\bar{\mathbf{x}}_i^{FEJ},\\
\mathbf{x}_j=\bar{\mathbf{x}}_j^{FEJ},\\
\lambda_l=\bar{\lambda}_l^{FEJ}
}}
$$

也就是说，只要这个因子的 Jacobian 依赖某些状态，严格 FEJ 就用这些状态的 first estimate 来计算。

IMU 预积分因子也是同理。若 IMU 因子连接：

$$
\mathbf{x}_i,\quad
\mathbf{x}_{i+1}
$$

严格 FEJ 会用：

$$
\bar{\mathbf{x}}_i^{FEJ},
\quad
\bar{\mathbf{x}}_{i+1}^{FEJ}
$$

来计算该 IMU 因子的相关 Jacobian，而不是用当前迭代中的：

$$
\mathbf{x}_i^{cur},
\quad
\mathbf{x}_{i+1}^{cur}
$$

#### 边缘化时严格 FEJ 怎么做

边缘化会把历史因子压缩成 prior。严格 FEJ 下，形成边缘化 prior 时使用的 Jacobian 也应来自 first estimate：

$$
\mathbf{J}_{m}^{FEJ} =
\mathbf{J}_{m}
\left(
\bar{\mathbf{x}}^{FEJ}
\right)
$$

然后构造 Hessian：

$$
\mathbf{H}^{FEJ} =
\left(
\mathbf{J}^{FEJ}
\right)^{\top}
\mathbf{\Omega}
\mathbf{J}^{FEJ}
$$

再通过 Schur 补消去要边缘化的变量，得到 prior：

$$
\mathbf{H}_{prior}^{FEJ} =
\mathbf{H}_{rr}^{FEJ} -
\mathbf{H}_{rm}^{FEJ}
\left(
\mathbf{H}_{mm}^{FEJ}
\right)^{-1}
\mathbf{H}_{mr}^{FEJ}
$$

其中：

- 下标 $m$ 表示 marginalized variables，也就是要删除的变量；
- 下标 $r$ 表示 remaining variables，也就是保留下来的变量；
- $\mathbf{H}_{mm}^{FEJ}$、$\mathbf{H}_{mr}^{FEJ}$、$\mathbf{H}_{rm}^{FEJ}$、$\mathbf{H}_{rr}^{FEJ}$ 是按变量分块后的 FEJ Hessian。

边缘化之后，prior 被固定下来。保留下来的变量继续维护：

$$
\mathbf{x}_r^{cur}
\quad
\text{和}
\quad
\bar{\mathbf{x}}_r^{FEJ}
$$

后续使用 prior 时，仍然不要把 $\bar{\mathbf{x}}_r^{FEJ}$ 更新成当前值，否则就失去了 strict FEJ 的意义。

#### 为什么严格 FEJ 更能保持零空间

如果每个相关因子都使用同一组 first estimate 计算 Jacobian，那么对同一个不可观子空间：

$$
\mathbf{N}
\left(
\bar{\mathbf{x}}^{FEJ}
\right)
$$

理想情况下每个因子都满足：

$$
\mathbf{J}_k^{FEJ}
\mathbf{N}
\left(
\bar{\mathbf{x}}^{FEJ}
\right) \approx
\mathbf{0}
$$

总信息矩阵为：

$$
\mathbf{H}^{FEJ} =
\sum_k
\left(
\mathbf{J}_k^{FEJ}
\right)^{\top}
\mathbf{\Omega}_k
\mathbf{J}_k^{FEJ}
$$

于是：

$$
\mathbf{H}^{FEJ}
\mathbf{N}
\left(
\bar{\mathbf{x}}^{FEJ}
\right) \approx
\mathbf{0}
$$

这表示不可观方向仍然尽量留在零空间里。

而如果只有 marginalization prior 使用旧线性化点，但普通视觉因子和 IMU 因子继续使用当前状态重线性化，就会变成：

$$
\mathbf{J}_{prior} =
\mathbf{J}_{prior}
\left(
\bar{\mathbf{x}}
\right)
$$

$$
\mathbf{J}_{vision} =
\mathbf{J}_{vision}
\left(
\mathbf{x}^{cur}
\right)
$$

$$
\mathbf{J}_{imu} =
\mathbf{J}_{imu}
\left(
\mathbf{x}^{cur}
\right)
$$

这些 Jacobian 对应的零空间可能不是同一个，组合起来就可能出现：

$$
\mathbf{N}^{\top}
\mathbf{H}
\mathbf{N} >
\mathbf{0}
$$

这就是 FEJ-like 和 strict FEJ 在一致性上的差别。

#### VINS-Mono 为什么更像 FEJ-like

VINS-Mono 的边缘化 prior 会固定在边缘化发生时的线性化点上。这个 prior 后续不会重新根据当前状态恢复成原始历史因子再重新 Schur 补，所以它具有 FEJ 的影子：

$$
\mathbf{J}_{prior}
\text{ 被固定}
$$

但是，VINS-Mono 的普通视觉重投影因子、IMU 预积分因子在滑窗优化过程中仍然会根据当前状态重新计算残差和 Jacobian：

$$
\mathbf{J}_{vision} =
\mathbf{J}_{vision}
\left(
\mathbf{x}^{cur}
\right)
$$

$$
\mathbf{J}_{imu} =
\mathbf{J}_{imu}
\left(
\mathbf{x}^{cur}
\right)
$$

所以它不是严格意义上的：

$$
\forall k,\quad
\mathbf{J}_k =
\mathbf{J}_k
\left(
\bar{\mathbf{x}}^{FEJ}
\right)
$$

更准确地说：

$$
\text{VINS-Mono} \approx
\text{marginalization-prior FEJ-like}
$$

它的工程取舍是：

$$
\text{固定 prior 保留一部分一致性} +
\text{普通因子重线性化保持局部精度}
$$

这个取舍很实用，但不等价于严格 FEJ。严格 FEJ 的 consistency 更强，代价是 Jacobian 更不贴近当前状态，可能牺牲更多局部精度，并且需要为所有状态和特征维护 first estimate。

#### 严格 FEJ 对 first estimate 的依赖

严格 FEJ 还有一个很现实的风险：

$$
\text{如果 first estimate 很差，严格 FEJ 的 Jacobian 会长期在一个较差的点上计算。}
$$

设真实非线性残差为：

$$
\mathbf{r}(\mathbf{x})
$$

普通重线性化在当前状态 $\mathbf{x}^{cur}$ 处使用：

$$
\mathbf{r}
\left(
\mathbf{x}^{cur}\boxplus\delta\mathbf{x}
\right) \approx
\mathbf{r}
\left(
\mathbf{x}^{cur}
\right) +
\mathbf{J}
\left(
\mathbf{x}^{cur}
\right)
\delta\mathbf{x}
$$

如果 $\mathbf{x}^{cur}$ 已经接近真实状态，那么这个线性模型通常比较精确。

严格 FEJ 使用：

$$
\mathbf{r}
\left(
\mathbf{x}^{cur}\boxplus\delta\mathbf{x}
\right) \approx
\mathbf{r}
\left(
\mathbf{x}^{cur}
\right) +
\mathbf{J}
\left(
\bar{\mathbf{x}}^{FEJ}
\right)
\delta\mathbf{x}
$$

如果 first estimate 和当前状态差很多：

$$
\left\|
\mathbf{x}^{cur}
\boxminus
\bar{\mathbf{x}}^{FEJ}
\right\|
\text{ 很大}
$$

那么：

$$
\mathbf{J}
\left(
\bar{\mathbf{x}}^{FEJ}
\right)
\not\approx
\mathbf{J}
\left(
\mathbf{x}^{cur}
\right)
$$

此时线性模型可能明显不准。

这会带来几个后果：

- 优化方向可能变差；
- 收敛速度可能变慢；
- 局部精度可能下降；
- 强非线性因子更容易受影响，例如视觉重投影、逆深度、外参、时间偏移；
- 如果 first estimate 偏差很大，严格 FEJ 可能保持了零空间一致性，却牺牲了较多 accuracy。

所以严格 FEJ 不是“更准”的方法。它更准确的定位是：

$$
\text{FEJ 主要改善 consistency，不保证最小化每一轮的 nonlinear residual error。}
$$

也可以说：

$$
\text{FEJ 防的是估计器过度自信，不是防所有估计误差。}
$$

这也是 VINS-Mono 更偏向 FEJ-like 的重要原因。它选择：

$$
\text{marginalization prior 固定} +
\text{普通视觉 / IMU 因子继续重线性化}
$$

也就是用一部分 consistency，换取更好的局部优化精度和工程稳定性。

把三种做法放在一起看：

| 方法 | 主要优点 | 主要风险 |
|---|---|---|
| 严格 FEJ | 零空间更一致，consistency 更好 | 强依赖 first estimate，初值差时精度和收敛可能变差 |
| FEJ-like | 工程折中，prior 不反复漂移，普通因子还能重线性化 | consistency 不如严格 FEJ |
| 普通重线性化 | 当前线性化精度高，非线性优化更自然 | 更容易让不可观方向被错误约束 |

因此，严格 FEJ 的优缺点可以压缩成一句话：

$$
\text{严格 FEJ 用 accuracy 的一部分代价，换 consistency 的提升。}
$$

### 4.5 prior 只约束旧变量，为什么会影响最新变量

这里还有一个很容易困惑的点：

$$
\text{边缘化 prior 的 Jacobian 往往只作用在一部分旧变量上。}
$$

那么它为什么会影响最新帧？最新帧明明不在这个 prior 里面。

答案是：

$$
\text{prior 直接约束旧变量；相对因子把旧变量和新变量连起来；所以新变量会被间接约束。}
$$

#### 一个最小例子

先看一维位置问题。假设窗口里有两个位置：

$$
p_0,\quad p_1
$$

其中：

- $p_0$ 表示旧位置；
- $p_1$ 表示新位置。

假设唯一真实测量是二者之间的相对位移：

$$
z =
p_1-p_0
+ n
$$

其中：

- $z$ 表示测得的相对位移；
- $n$ 表示测量噪声。

对应残差可以写成：

$$
r_{01} =
(p_1-p_0)-z
$$

对状态：

$$
\mathbf{x} =
\begin{bmatrix}
p_0\\
p_1
\end{bmatrix}
$$

线性化后，Jacobian 是：

$$
\mathbf{J}_{01} =
\begin{bmatrix}
-1 & 1
\end{bmatrix}
$$

因为：

$$
\frac{\partial r_{01}}{\partial p_0}=-1,
\quad
\frac{\partial r_{01}}{\partial p_1}=1
$$

这个系统没有绝对位置测量，所以全局平移不可观。全局平移方向是：

$$
\mathbf{n} =
\begin{bmatrix}
1\\
1
\end{bmatrix}
$$

它的含义是：

$$
p_0 \leftarrow p_0+s,
\quad
p_1 \leftarrow p_1+s
$$

其中 $s$ 表示任意平移量。

沿这个方向移动时，相对位移不变：

$$
(p_1+s)-(p_0+s) =
p_1-p_0
$$

用 Jacobian 表示就是：

$$
\mathbf{J}_{01}\mathbf{n} =
\begin{bmatrix}
-1 & 1
\end{bmatrix}
\begin{bmatrix}
1\\
1
\end{bmatrix} =
0
$$

所以相对因子不会约束全局平移方向。

#### 如果 prior 错误锚定旧变量

现在假设边缘化之后，系统产生了一个作用在旧变量 $p_0$ 上的 prior：

$$
r_p =
p_0-\bar{p}_0
$$

其中 $\bar{p}_0$ 表示 prior 形成时 $p_0$ 的线性化参考值。

这个 prior 的 Jacobian 是：

$$
\mathbf{J}_p =
\begin{bmatrix}
1 & 0
\end{bmatrix}
$$

注意，它确实没有直接约束 $p_1$。

但是沿全局平移方向：

$$
\mathbf{J}_p\mathbf{n} =
\begin{bmatrix}
1 & 0
\end{bmatrix}
\begin{bmatrix}
1\\
1
\end{bmatrix} =
1 \neq
0
$$

这说明 prior 对全局平移方向产生了响应。

如果 prior 的权重是 $\omega_p$，那么 prior 对应的信息矩阵是：

$$
\mathbf{H}_p =
\mathbf{J}_p^\top
\omega_p
\mathbf{J}_p =
\begin{bmatrix}
\omega_p & 0\\
0 & 0
\end{bmatrix}
$$

沿全局平移方向的二次代价是：

$$
\frac{1}{2}
s^2
\mathbf{n}^{\top}
\mathbf{H}_p
\mathbf{n} =
\frac{1}{2}
s^2
\omega_p
$$

这个值大于零，表示系统认为整体平移会增加代价。于是本来不可观的全局平移方向被错误约束了。

重点是：

$$
\text{prior 没有直接约束 }p_1,
\quad
\text{但它约束了整体平移方向中的 }p_0\text{ 分量。}
$$

而真正的全局平移不是“只移动 $p_1$”，而是：

$$
\begin{bmatrix}
p_0\\
p_1
\end{bmatrix} \leftarrow
\begin{bmatrix}
p_0\\
p_1
\end{bmatrix} +
s
\begin{bmatrix}
1\\
1
\end{bmatrix}
$$

只要 $p_0$ 这一部分被错误锚定，整个全局平移方向就不再自由。

#### 为什么新变量也被间接锚住

从另一个角度看，如果 $p_0$ 被 prior 锚住，而 $p_1$ 想单独平移：

$$
p_1 \leftarrow p_1+s,
\quad
p_0 \text{ 不动}
$$

相对残差会变成：

$$
r_{01}^{new} =
\left(
p_1+s-p_0
\right)
-z =
r_{01}+s
$$

这会被相对因子惩罚。

因此：

$$
\text{prior 锚住 }p_0 +
\text{相对因子约束 }p_1-p_0 \Rightarrow
\quad
p_1\text{ 也被间接锚住}
$$

这就是“最新变量没有直接出现在 prior 中，却仍然被 prior 影响”的原因。

#### 写成一般矩阵形式

把状态分成两部分：

$$
\mathbf{x} =
\begin{bmatrix}
\mathbf{x}_a\\
\mathbf{x}_b
\end{bmatrix}
$$

其中：

- $\mathbf{x}_a$ 表示被边缘化 prior 直接连接的旧变量；
- $\mathbf{x}_b$ 表示没有被 prior 直接连接的新变量。

假设 prior 只作用在 $\mathbf{x}_a$ 上：

$$
\mathbf{r}_p \approx
\mathbf{r}_{p0} +
\mathbf{J}_p
\delta\mathbf{x}_a
$$

嵌入完整状态后，它的信息矩阵具有块结构：

$$
\mathbf{H}_p =
\begin{bmatrix}
\mathbf{H}_{aa}^{p} & \mathbf{0}\\
\mathbf{0} & \mathbf{0}
\end{bmatrix}
$$

其中：

- $\mathbf{H}_{aa}^{p}=\mathbf{J}_p^\top\mathbf{\Omega}_p\mathbf{J}_p$；
- $\mathbf{\Omega}_p$ 表示 prior 的信息矩阵；
- 右下角的 $\mathbf{0}$ 表示 prior 不直接约束 $\mathbf{x}_b$。

如果旧变量和新变量之间还有相对因子：

$$
\mathbf{r}_{ab} =
\mathbf{h}(\mathbf{x}_a,\mathbf{x}_b)-\mathbf{z}_{ab}
$$

线性化后：

$$
\mathbf{r}_{ab} \approx
\mathbf{r}_{ab0} +
\mathbf{J}_a\delta\mathbf{x}_a +
\mathbf{J}_b\delta\mathbf{x}_b
$$

对应的信息矩阵会产生交叉块：

$$
\mathbf{H}_{ab} =
\mathbf{J}_a^\top
\mathbf{\Omega}_{ab}
\mathbf{J}_b
$$

所以总信息矩阵近似为：

$$
\mathbf{H} =
\begin{bmatrix}
\mathbf{H}_{aa}^{p} & \mathbf{0}\\
\mathbf{0} & \mathbf{0}
\end{bmatrix} +
\begin{bmatrix}
\mathbf{J}_a^\top\mathbf{\Omega}_{ab}\mathbf{J}_a
&
\mathbf{J}_a^\top\mathbf{\Omega}_{ab}\mathbf{J}_b
\\
\mathbf{J}_b^\top\mathbf{\Omega}_{ab}\mathbf{J}_a
&
\mathbf{J}_b^\top\mathbf{\Omega}_{ab}\mathbf{J}_b
\end{bmatrix}
$$

设完整系统的某个不可观方向为：

$$
\mathbf{N} =
\begin{bmatrix}
\mathbf{N}_a\\
\mathbf{N}_b
\end{bmatrix}
$$

其中：

- $\mathbf{N}_a$ 表示不可观方向在旧变量 $\mathbf{x}_a$ 上的分量；
- $\mathbf{N}_b$ 表示不可观方向在新变量 $\mathbf{x}_b$ 上的分量。

如果相对因子正确保持这个不可观方向，那么应该有：

$$
\mathbf{J}_a\mathbf{N}_a +
\mathbf{J}_b\mathbf{N}_b =
\mathbf{0}
$$

也就是说，相对因子不会惩罚整体 gauge 运动。

但是如果 prior 错误约束了旧变量上的不可观分量：

$$
\mathbf{H}_{aa}^{p}\mathbf{N}_a \neq
\mathbf{0}
$$

那么完整方向就不再是总信息矩阵的零空间。令扰动为：

$$
\delta\mathbf{x} =
s\mathbf{N}
$$

其中 $s$ 表示沿不可观方向移动的幅度。沿这个方向的二次代价为：

$$
\frac{1}{2}
\delta\mathbf{x}^{\top}
\mathbf{H}
\delta\mathbf{x} =
\frac{1}{2}
s^2
\mathbf{N}^{\top}
\mathbf{H}
\mathbf{N} =
\frac{1}{2}
s^2
\mathbf{N}_a^{\top}
\mathbf{H}_{aa}^{p}
\mathbf{N}_a >
0
$$

上式右边只来自 prior 对旧变量的约束。相对因子本身可以仍然满足零响应，但因为 prior 已经把 $\mathbf{N}_a$ 锚住，完整的不可观方向 $\mathbf{N}$ 也被破坏了。

#### 回到 VIO 滑窗

在 VINS 这类滑窗 VIO 中，最新帧通常通过以下因子和历史状态连接：

- IMU 预积分因子连接相邻帧状态；
- 视觉重投影因子连接观测帧、特征点和相机外参；
- 边缘化 prior 连接窗口中保留下来的旧状态；
- 多帧特征跟踪让不同时间的位姿共享几何约束。

所以边缘化 prior 不需要直接连接最新帧，也可以影响最新帧。影响路径通常是：

$$
\text{marginalization prior} \rightarrow
\text{旧保留状态} \rightarrow
\text{IMU / 视觉相对因子} \rightarrow
\text{最新状态}
$$

如果最新状态完全不和旧状态相连，那么它当然不会被这个 prior 约束。但那样它也不属于同一个滑窗估计问题；在正常 VIO 后端里，最新帧必然通过 IMU 和视觉约束接入历史图。

因此，更准确的说法是：

$$
\text{边缘化 prior 直接约束的是旧变量，但它约束的是整个连通图的 gauge 自由度。}
$$

FEJ 要避免的正是这种情况：不要让 prior 在旧变量分量 $\mathbf{N}_a$ 上错误产生信息，否则最新变量会通过相对因子被一起间接锚住。

## 5. 为什么边缘化会破坏可观性

边缘化的目的是删除旧变量，同时保留旧变量对剩余变量的信息。

把状态分成：

$$
\mathbf{x} =
\begin{bmatrix}
\mathbf{x}_m\\
\mathbf{x}_r
\end{bmatrix}
$$

其中：

- $\mathbf{x}_m$ 表示要边缘化的变量；
- $\mathbf{x}_r$ 表示保留变量。

非线性残差为：

$$
\mathbf{r}(\mathbf{x}_m,\mathbf{x}_r)
$$

边缘化前，需要先在某个点线性化：

$$
\bar{\mathbf{x}}_m,\quad
\bar{\mathbf{x}}_r
$$

得到：

$$
\mathbf{r} \approx
\bar{\mathbf{r}} +
\mathbf{J}_m\delta\mathbf{x}_m +
\mathbf{J}_r\delta\mathbf{x}_r
$$

其中：

- $\bar{\mathbf{r}}$ 表示线性化点处残差；
- $\mathbf{J}_m$ 表示残差对被边缘化变量的 Jacobian；
- $\mathbf{J}_r$ 表示残差对保留变量的 Jacobian；
- $\delta\mathbf{x}_m=\mathbf{x}_m\boxminus\bar{\mathbf{x}}_m$；
- $\delta\mathbf{x}_r=\mathbf{x}_r\boxminus\bar{\mathbf{x}}_r$。

正规方程分块为：

$$
\begin{bmatrix}
\mathbf{H}_{mm} & \mathbf{H}_{mr}\\
\mathbf{H}_{rm} & \mathbf{H}_{rr}
\end{bmatrix}
\begin{bmatrix}
\delta\mathbf{x}_m\\
\delta\mathbf{x}_r
\end{bmatrix} =
\begin{bmatrix}
\mathbf{g}_m\\
\mathbf{g}_r
\end{bmatrix}
$$

Schur 补得到保留变量上的先验：

$$
\mathbf{H}^{prior}_r =
\mathbf{H}_{rr} -
\mathbf{H}_{rm}
\mathbf{H}_{mm}^{-1}
\mathbf{H}_{mr}
$$

$$
\mathbf{g}^{prior}_r =
\mathbf{g}_r -
\mathbf{H}_{rm}
\mathbf{H}_{mm}^{-1}
\mathbf{g}_m
$$

边缘化后的先验是一个线性化先验：

$$
\mathbf{r}_{prior}(\mathbf{x}_r) \approx
\mathbf{r}^{prior}_0 +
\mathbf{J}^{prior}
\left(
\mathbf{x}_r\boxminus\bar{\mathbf{x}}_r
\right)
$$

### 5.1 边缘化时没有问题，问题发生在后续

这一节慢一点推。先说结论：

$$
\text{Schur 补本身不一定破坏可观性；真正危险的是“旧 prior 的零空间”和“后续当前状态的零空间”不再一致。}
$$

先把状态分成两部分：

$$
\delta\mathbf{x} =
\begin{bmatrix}
\delta\mathbf{x}_m\\
\delta\mathbf{x}_r
\end{bmatrix}
$$

其中：

- $\delta\mathbf{x}_m$ 表示即将被边缘化的变量扰动；
- $\delta\mathbf{x}_r$ 表示会保留下来的变量扰动。

现在假设完整系统存在一个不可观方向。这个不可观方向也要按同样的变量顺序分块：

$$
\mathbf{N} =
\begin{bmatrix}
\mathbf{N}_m\\
\mathbf{N}_r
\end{bmatrix}
$$

其中：

- $\mathbf{N}$ 表示完整系统的不可观方向或不可观子空间基；
- $\mathbf{N}_m$ 表示这个不可观方向在“将被边缘化变量”上的分量；
- $\mathbf{N}_r$ 表示这个不可观方向在“保留变量”上的分量。

举个直观例子：全局平移不可观时，所有 pose 和 landmark 都一起平移。如果最老帧要被边缘化，那么这个“大家一起平移”的方向里，最老帧的平移部分属于 $\mathbf{N}_m$，窗口里保留帧的平移部分属于 $\mathbf{N}_r$。

边缘化前的 Hessian 分块为：

$$
\mathbf{H} =
\begin{bmatrix}
\mathbf{H}_{mm} & \mathbf{H}_{mr}\\
\mathbf{H}_{rm} & \mathbf{H}_{rr}
\end{bmatrix}
$$

如果 $\mathbf{N}$ 是不可观方向，那么：

$$
\mathbf{H}\mathbf{N} =
\mathbf{0}
$$

代入分块形式：

$$
\begin{bmatrix}
\mathbf{H}_{mm} & \mathbf{H}_{mr}\\
\mathbf{H}_{rm} & \mathbf{H}_{rr}
\end{bmatrix}
\begin{bmatrix}
\mathbf{N}_m\\
\mathbf{N}_r
\end{bmatrix} =
\begin{bmatrix}
\mathbf{0}\\
\mathbf{0}
\end{bmatrix}
$$

展开成两行：

$$
\mathbf{H}_{mm}\mathbf{N}_m +
\mathbf{H}_{mr}\mathbf{N}_r =
\mathbf{0}
$$

$$
\mathbf{H}_{rm}\mathbf{N}_m +
\mathbf{H}_{rr}\mathbf{N}_r =
\mathbf{0}
$$

第一行可以解出 $\mathbf{N}_m$ 和 $\mathbf{N}_r$ 的关系。假设 $\mathbf{H}_{mm}$ 在要消元的子空间里可逆，则：

$$
\mathbf{N}_m = -
\mathbf{H}_{mm}^{-1}
\mathbf{H}_{mr}
\mathbf{N}_r
$$

现在把它代入第二行：

$$
\mathbf{H}_{rm}
\left( -
\mathbf{H}_{mm}^{-1}
\mathbf{H}_{mr}
\mathbf{N}_r
\right) +
\mathbf{H}_{rr}\mathbf{N}_r =
\mathbf{0}
$$

整理：

$$
\left(
\mathbf{H}_{rr} -
\mathbf{H}_{rm}
\mathbf{H}_{mm}^{-1}
\mathbf{H}_{mr}
\right)
\mathbf{N}_r =
\mathbf{0}
$$

括号里的矩阵正是边缘化后的 Schur 补先验 Hessian：

$$
\mathbf{H}^{prior}_r =
\mathbf{H}_{rr} -
\mathbf{H}_{rm}
\mathbf{H}_{mm}^{-1}
\mathbf{H}_{mr}
$$

因此：

$$
\mathbf{H}^{prior}_r
\mathbf{N}_r =
\mathbf{0}
$$

这一步说明了一个重要事实：

$$
\text{如果边缘化前完整系统的不可观方向是正确的，那么理想 Schur 补后的 prior 对保留变量那部分方向也仍然保持零响应。}
$$

也就是说，在边缘化发生的那个线性化点，Schur 补本身并不必然破坏可观性。

为了强调“线性化点”，把上面的方向写成：

$$
\mathbf{N}_r(\bar{\mathbf{x}}_r)
$$

其中：

- $\bar{\mathbf{x}}_r$ 表示边缘化发生时保留变量的线性化点；
- $\mathbf{N}_r(\bar{\mathbf{x}}_r)$ 表示在这个线性化点下，不可观方向落在保留变量上的分量。

于是边缘化先验满足：

$$
\mathbf{H}^{prior}_r
\mathbf{N}_r(\bar{\mathbf{x}}_r) =
\mathbf{0}
$$

到这里都没有问题。

真正的问题发生在后续。边缘化之后，$\mathbf{x}_m$ 被删除，系统只保留 $\mathbf{x}_r$。随后滑窗继续优化，保留变量会变化：

$$
\mathbf{x}_r^{new} \neq
\bar{\mathbf{x}}_r
$$

很多不可观方向不是一个固定常向量，而是和当前状态有关。纯全局平移方向通常比较简单，常常可以近似看成固定方向；但全局 yaw、姿态相关方向、以及包含位姿和地图点耦合的不可观方向，往往依赖当前状态。比如全局 yaw 不可观方向包含：

$$
- \mathbf{p}_k^{\wedge}\mathbf{u}_g
$$

其中：

- $\mathbf{p}_k$ 是第 $k$ 个位姿的位置；
- $\mathbf{u}_g$ 是重力方向单位向量。

当位置 $\mathbf{p}_k$ 改变时，这个 yaw 不可观方向的坐标表达也会改变。因此一般有：

$$
\mathbf{N}_r(\mathbf{x}_r^{new}) \neq
\mathbf{N}_r(\bar{\mathbf{x}}_r)
$$

但是边缘化先验已经被冻结在边缘化时刻：

$$
\mathbf{H}^{prior}_r
\quad
\text{固定不变}
$$

它保证的是：

$$
\mathbf{H}^{prior}_r
\mathbf{N}_r(\bar{\mathbf{x}}_r) =
\mathbf{0}
$$

而不是保证：

$$
\mathbf{H}^{prior}_r
\mathbf{N}_r(\mathbf{x}_r^{new}) =
\mathbf{0}
$$

所以后续可能出现：

$$
\mathbf{H}^{prior}_r
\mathbf{N}_r(\mathbf{x}_r^{new}) \neq
\mathbf{0}
$$

这句话的意思是：旧 prior 对“当前状态下真正应该不可观的方向”产生了响应。

换成能量语言，边缘化先验的二次代价是：

$$
E_{prior} =
\frac{1}{2}
\delta\mathbf{x}_r^{\top}
\mathbf{H}^{prior}_r
\delta\mathbf{x}_r
$$

如果沿当前不可观方向扰动：

$$
\delta\mathbf{x}_r =
s\mathbf{N}_r(\mathbf{x}_r^{new})
$$

则：

$$
E_{prior} =
\frac{1}{2}
s^2
\mathbf{N}_r(\mathbf{x}_r^{new})^{\top}
\mathbf{H}^{prior}_r
\mathbf{N}_r(\mathbf{x}_r^{new})
$$

如果：

$$
\mathbf{N}_r(\mathbf{x}_r^{new})^{\top}
\mathbf{H}^{prior}_r
\mathbf{N}_r(\mathbf{x}_r^{new}) >
0
$$

说明先验认为这个方向移动会增加代价。也就是说，它把当前不可观方向错误地当成了可观方向。

这就是边缘化破坏可观性的核心。

因此，“边缘化”和“FEJ”的关系可以这样理解：

$$
\text{边缘化} \Rightarrow
\text{历史非线性因子被压缩成固定线性 prior}
$$

$$
\text{固定线性 prior} \Rightarrow
\text{后续容易和当前因子的零空间不一致}
$$

$$
\text{零空间不一致} \Rightarrow
\text{可能破坏可观性和 consistency}
$$

$$
\text{FEJ} \Rightarrow
\text{让相关 Jacobian 尽量使用同一个 first estimate，缓解零空间不一致}
$$

所以不是说：

$$
\text{只要边缘化，就一定立刻破坏可观性}
$$

更准确地说是：

$$
\text{边缘化制造了“历史 prior 被冻结”的条件；后续重线性化如果处理不当，就可能破坏可观性。}
$$

FEJ 正是为这个问题服务的。

但即使使用 FEJ，也不能说影响完全消失。因为：

- first estimate 可能有误差；
- 固定 Jacobian 会带来线性化精度损失；
- 如果只固定 marginalization prior，而普通视觉或 IMU 因子仍按当前状态重线性化，那么这只是 FEJ-like，不是严格 FEJ；
- 阻尼、gauge fixing、错误数据关联、错误 contact constraint 等也可能引入额外虚假信息。

所以最终应理解为：

$$
\text{边缘化是可观性风险来源之一；FEJ 是缓解这个风险的 consistency-preserving 技巧。}
$$

### 5.2 与新因子组合后，公共零空间可能缩小

后续新加入的视觉或 IMU 因子通常会在当前状态处线性化：

$$
\mathbf{J}_{new} =
\mathbf{J}_{new}(\mathbf{x}_r^{new})
$$

它们可能满足：

$$
\mathbf{J}_{new}
\mathbf{N}_r(\mathbf{x}_r^{new}) =
\mathbf{0}
$$

但旧 prior 满足的是：

$$
\mathbf{J}_{prior}
\mathbf{N}_r(\bar{\mathbf{x}}_r) =
\mathbf{0}
$$

两个零空间基不同：

$$
\mathbf{N}_r(\mathbf{x}_r^{new}) \neq
\mathbf{N}_r(\bar{\mathbf{x}}_r)
$$

组合信息矩阵为：

$$
\mathbf{H}_{total} =
\mathbf{H}_{prior} +
\mathbf{J}_{new}^{\top}
\mathbf{\Omega}_{new}
\mathbf{J}_{new}
$$

如果旧 prior 和新因子的零空间不一致，公共零空间会变小：

$$
\mathcal{N}(\mathbf{H}_{total}) =
\mathcal{N}(\mathbf{H}_{prior})
\cap
\mathcal{N}(\mathbf{J}_{new})
$$

其中 $\mathcal{N}(\cdot)$ 表示零空间。

一旦这个交集维度变小：

$$
\dim
\left(
\mathcal{N}(\mathbf{H}_{total})
\right) <
\dim
\left(
\mathcal{N}_{true}
\right)
$$

系统就错误地把一些不可观方向变成了可观方向。

这就是边缘化破坏可观性的核心机制。

### 5.3 为什么边缘化比普通重线性化更危险

普通非线性因子还保留原始函数：

$$
\mathbf{r}_{new}(\mathbf{x})
$$

如果当前线性化不好，下一次迭代还可以重新线性化。

边缘化 prior 不同。它已经把历史非线性因子压缩成线性先验：

$$
\mathbf{r}_{prior} =
\mathbf{r}^{prior}_0 +
\mathbf{J}^{prior}
\delta\mathbf{x}_r
$$

原来的 $\mathbf{x}_m$ 已经被删掉。你无法在新状态处严格重新边缘化，因为重新 Schur 补需要已经删除的变量：

$$
\mathbf{x}_m
$$

所以边缘化 prior 一旦形成，就会长期携带当时线性化点的零空间结构。

如果这个结构与后续状态越来越不一致，它就可能长期注入虚假信息。

### 5.4 目前有哪些解决方案

这个问题有没有彻底解决方案？更准确的回答是：

$$
\text{没有一个同时满足完全一致性、最高线性化精度、实时高效和工程简单的完美方案。}
$$

现有方法本质上都在三个目标之间做取舍：

$$
\text{Consistency}
\quad
\text{Accuracy}
\quad
\text{Efficiency}
$$

其中：

- consistency 表示估计器不要在不可观方向上过度自信；
- accuracy 表示线性化模型要尽量贴近当前真实非线性函数；
- efficiency 表示系统要能实时运行，不能无限保留历史变量。

下面把常见解决路线放在同一张体系里。

#### 5.4.1 FEJ / FEJ-like prior

FEJ 这类方法最容易混淆的地方是：它不是简单地说“某个 Jacobian 固定了”，而是要问：

$$
\text{哪些因子的 Jacobian 被固定？固定在谁的 first estimate 上？}
$$

为了说清楚，先把一个非线性因子写成：

$$
\mathbf{r}_k =
\mathbf{r}_k(\mathbf{x}_{\mathcal{S}_k})
$$

其中：

- $\mathbf{r}_k$ 表示第 $k$ 个残差；
- $\mathbf{x}_{\mathcal{S}_k}$ 表示这个因子连接的所有状态块；
- $\mathcal{S}_k$ 表示这些状态块的索引集合。

普通重线性化会在当前状态处计算 Jacobian：

$$
\mathbf{J}_k^{cur} =
\left.
\frac{\partial\mathbf{r}_k}{\partial\delta\mathbf{x}_{\mathcal{S}_k}}
\right|_{\mathbf{x}_{\mathcal{S}_k}=\mathbf{x}_{\mathcal{S}_k}^{cur}}
$$

其中：

- $\mathbf{x}^{cur}$ 表示当前估计状态；
- $\delta\mathbf{x}$ 表示局部扰动量；
- $\mathbf{J}_k^{cur}$ 表示当前线性化点处的 Jacobian。

FEJ 的核心是固定 first estimate 处的 Jacobian：

$$
\mathbf{J} =
\mathbf{J}(\bar{\mathbf{x}})
$$

其中 $\bar{\mathbf{x}}$ 表示 first estimate。它不是每次使用当前状态处的 Jacobian：

$$
\mathbf{J} =
\mathbf{J}(\mathbf{x}^{current})
$$

这样做的目标是让相关因子共享同一个不可观零空间：

$$
\mathbf{J}(\bar{\mathbf{x}})
\mathbf{N}(\bar{\mathbf{x}}) \approx
\mathbf{0}
$$

其中 $\mathbf{N}(\bar{\mathbf{x}})$ 表示在 first estimate 处表达的不可观子空间。

##### 严格 FEJ：所有相关因子都用 first estimate

严格 FEJ 的规则是：

$$
\forall k,\quad
\mathbf{J}_k =
\mathbf{J}_k
\left(
\bar{\mathbf{x}}^{FEJ}
\right)
$$

也就是说，视觉因子、IMU 因子、边缘化 prior 等所有影响可观性的 Jacobian，都尽量使用对应状态的 first estimate。

但是残差值通常仍然可以用当前状态计算：

$$
\mathbf{r}_k^{cur} =
\mathbf{r}_k
\left(
\mathbf{x}_{\mathcal{S}_k}^{cur}
\right)
$$

于是严格 FEJ 的线性化模型是：

$$
\mathbf{r}_k
\left(
\mathbf{x}^{cur}\boxplus\delta\mathbf{x}
\right) \approx
\mathbf{r}_k
\left(
\mathbf{x}^{cur}
\right) +
\mathbf{J}_k
\left(
\bar{\mathbf{x}}^{FEJ}
\right)
\delta\mathbf{x}
$$

注意这个式子不是标准 Taylor 展开。标准 Taylor 展开要求残差和 Jacobian 在同一个点计算；严格 FEJ 则有意让 Jacobian 固定在 first estimate，以换取零空间一致性。

如果所有相关因子都满足：

$$
\mathbf{J}_k
\left(
\bar{\mathbf{x}}^{FEJ}
\right)
\mathbf{N}
\left(
\bar{\mathbf{x}}^{FEJ}
\right) \approx
\mathbf{0}
$$

那么总信息矩阵：

$$
\mathbf{H}^{FEJ} =
\sum_k
\mathbf{J}_k^\top
\mathbf{\Omega}_k
\mathbf{J}_k
$$

也会满足：

$$
\mathbf{H}^{FEJ}
\mathbf{N}
\left(
\bar{\mathbf{x}}^{FEJ}
\right) \approx
\mathbf{0}
$$

这就是严格 FEJ 的重点：不是某一个因子单独一致，而是所有相关因子在同一套零空间下共同一致。

对新加入的因子也是同样规则。新因子连接的变量集合为 $\mathcal{S}_k$ 时，严格 FEJ 使用：

$$
\mathbf{J}_k^{FEJ} =
\left.
\frac{\partial\mathbf{r}_k}
{\partial\delta\mathbf{x}_{\mathcal{S}_k}}
\right|_{
\mathbf{x}_s=\bar{\mathbf{x}}_s^{FEJ},
\ s\in\mathcal{S}_k
}
$$

也就是说，新因子并不是用当前状态算 Jacobian，而是用它连接的每个变量各自的 first estimate。若某个变量刚加入系统，就先保存它的 first estimate，再用这个值参与新因子的 Jacobian 计算。

这也解释了严格 FEJ 的主要风险：如果这些 first estimate 本身偏差较大，那么新因子的 Jacobian 也会长期受这个偏差影响。

##### FEJ-like：只固定一部分因子

FEJ-like 则通常只是部分 Jacobian 被固定。

例如 VINS-Mono 更接近：

$$
\mathbf{J}_{prior} =
\mathbf{J}_{prior}
\left(
\bar{\mathbf{x}}_{marg}
\right)
$$

也就是边缘化 prior 固定在形成 prior 时的线性化点 $\bar{\mathbf{x}}_{marg}$。

但普通滑窗内因子仍然使用当前状态：

$$
\mathbf{J}_{vision} =
\mathbf{J}_{vision}
\left(
\mathbf{x}^{cur}
\right)
$$

$$
\mathbf{J}_{imu} =
\mathbf{J}_{imu}
\left(
\mathbf{x}^{cur}
\right)
$$

所以 FEJ-like 的结构是：

$$
\text{一部分 Jacobian 使用旧线性化点} +
\text{一部分 Jacobian 使用当前线性化点}
$$

问题在于：不同线性化点下的不可观子空间可能不是同一个。

边缘化 prior 可能满足：

$$
\mathbf{J}_{prior}
\left(
\bar{\mathbf{x}}_{marg}
\right)
\mathbf{N}
\left(
\bar{\mathbf{x}}_{marg}
\right) =
\mathbf{0}
$$

而新视觉或 IMU 因子可能满足：

$$
\mathbf{J}_{new}
\left(
\mathbf{x}^{cur}
\right)
\mathbf{N}
\left(
\mathbf{x}^{cur}
\right) =
\mathbf{0}
$$

但是这两个零空间未必相同：

$$
\mathbf{N}
\left(
\bar{\mathbf{x}}_{marg}
\right) \neq
\mathbf{N}
\left(
\mathbf{x}^{cur}
\right)
$$

于是组合起来可能没有公共零空间：

$$
\mathcal{N}
\left(
\mathbf{H}_{prior} +
\mathbf{H}_{new}
\right) \neq
\mathcal{N}_{true}
$$

这就是 FEJ-like 比严格 FEJ 弱的地方。

##### 一个具体数值例子：严格 FEJ 保留零空间，FEJ-like 可能错误满秩

用一个二维状态做最小例子：

$$
\mathbf{x} =
\begin{bmatrix}
x_1\\
x_2
\end{bmatrix}
$$

假设系统真实存在一个不可观方向。在线性化点 $\bar{\mathbf{x}}$ 处，这个方向为：

$$
\mathbf{n}_0 =
\begin{bmatrix}
1\\
1
\end{bmatrix}
$$

它表示两个变量一起同向移动。例如在相对位移问题里，这对应全局平移。

现在有两个因子：

- 一个旧 prior 因子；
- 一个新加入的普通因子。

先看严格 FEJ。因为两个因子的 Jacobian 都使用同一个 first estimate，所以它们可以共享同一个零空间。假设两个 Jacobian 都是：

$$
\mathbf{J}_{prior}^{FEJ} =
\begin{bmatrix}
1 & -1
\end{bmatrix}
$$

$$
\mathbf{J}_{new}^{FEJ} =
\begin{bmatrix}
1 & -1
\end{bmatrix}
$$

检查不可观方向：

$$
\mathbf{J}_{prior}^{FEJ}\mathbf{n}_0 =
\begin{bmatrix}
1 & -1
\end{bmatrix}
\begin{bmatrix}
1\\
1
\end{bmatrix} =
0
$$

$$
\mathbf{J}_{new}^{FEJ}\mathbf{n}_0 =
\begin{bmatrix}
1 & -1
\end{bmatrix}
\begin{bmatrix}
1\\
1
\end{bmatrix} =
0
$$

所以两个因子都不约束 $\mathbf{n}_0$。

令两个因子的权重都为 $1$，总 Hessian 为：

$$
\mathbf{H}^{FEJ} =
\left(
\mathbf{J}_{prior}^{FEJ}
\right)^\top
\mathbf{J}_{prior}^{FEJ} +
\left(
\mathbf{J}_{new}^{FEJ}
\right)^\top
\mathbf{J}_{new}^{FEJ}
$$

也就是：

$$
\mathbf{H}^{FEJ} =
2
\begin{bmatrix}
1\\
-1
\end{bmatrix}
\begin{bmatrix}
1 & -1
\end{bmatrix} =
\begin{bmatrix}
2 & -2\\
-2 & 2
\end{bmatrix}
$$

它的行列式为：

$$
\det
\left(
\mathbf{H}^{FEJ}
\right) =
2\cdot2-(-2)^2 =
0
$$

因此 $\mathbf{H}^{FEJ}$ 仍然是奇异的，仍然保留一个零空间。并且：

$$
\mathbf{H}^{FEJ}\mathbf{n}_0 =
\mathbf{0}
$$

这表示不可观方向没有被错误约束。

再看 FEJ-like。旧 prior 仍然固定在旧线性化点：

$$
\mathbf{J}_{prior} =
\begin{bmatrix}
1 & -1
\end{bmatrix}
$$

但新因子按当前状态重线性化。假设由于当前线性化点变化，新因子的零空间方向变成：

$$
\mathbf{n}_{cur} =
\begin{bmatrix}
1\\
1+\epsilon
\end{bmatrix}
$$

其中 $\epsilon$ 是一个很小但非零的数，表示当前零空间表达和旧零空间表达之间出现了偏差。

为了让新因子在当前线性化点下仍然“不约束自己的不可观方向”，可以设：

$$
\mathbf{J}_{new}^{cur} =
\begin{bmatrix}
1+\epsilon & -1
\end{bmatrix}
$$

因为：

$$
\mathbf{J}_{new}^{cur}
\mathbf{n}_{cur} =
\begin{bmatrix}
1+\epsilon & -1
\end{bmatrix}
\begin{bmatrix}
1\\
1+\epsilon
\end{bmatrix} =
0
$$

也就是说，新因子单独看没有错：它在自己的当前零空间下是正确的。

但是旧 prior 对当前零空间不再为零：

$$
\mathbf{J}_{prior}
\mathbf{n}_{cur} =
\begin{bmatrix}
1 & -1
\end{bmatrix}
\begin{bmatrix}
1\\
1+\epsilon
\end{bmatrix} =
-\epsilon \neq
0
$$

同样，新因子对旧零空间也不再为零：

$$
\mathbf{J}_{new}^{cur}
\mathbf{n}_0 =
\begin{bmatrix}
1+\epsilon & -1
\end{bmatrix}
\begin{bmatrix}
1\\
1
\end{bmatrix} =
\epsilon \neq
0
$$

这就是零空间不一致。

此时总 Hessian 为：

$$
\mathbf{H}^{like} =
\mathbf{J}_{prior}^{\top}\mathbf{J}_{prior} +
\left(
\mathbf{J}_{new}^{cur}
\right)^{\top}
\mathbf{J}_{new}^{cur}
$$

代入：

$$
\mathbf{H}^{like} =
\begin{bmatrix}
1\\
-1
\end{bmatrix}
\begin{bmatrix}
1 & -1
\end{bmatrix} +
\begin{bmatrix}
1+\epsilon\\
-1
\end{bmatrix}
\begin{bmatrix}
1+\epsilon & -1
\end{bmatrix}
$$

得到：

$$
\mathbf{H}^{like} =
\begin{bmatrix}
2+2\epsilon+\epsilon^2 & -2-\epsilon\\
-2-\epsilon & 2
\end{bmatrix}
$$

它的行列式是：

$$
\det
\left(
\mathbf{H}^{like}
\right) =
\epsilon^2
$$

只要：

$$
\epsilon\neq0
$$

就有：

$$
\det
\left(
\mathbf{H}^{like}
\right) >
0
$$

这说明 $\mathbf{H}^{like}$ 从奇异矩阵变成了满秩矩阵。可是这个系统理论上应该还有一个不可观方向。满秩意味着系统错误地认为两个方向都可观，于是产生了虚假信息。

这个例子说明：

$$
\text{每个因子单独看都可能“合理”，但它们的零空间不一致，组合后就会错误增加秩。}
$$

严格 FEJ 的作用，就是尽量避免这种不一致：

$$
\mathbf{J}_{prior}^{FEJ}\mathbf{n}_0=0,
\quad
\mathbf{J}_{new}^{FEJ}\mathbf{n}_0=0
$$

FEJ-like 只固定 prior，因此更像：

$$
\mathbf{J}_{prior}\mathbf{n}_0=0,
\quad
\mathbf{J}_{new}^{cur}\mathbf{n}_{cur}=0
$$

它们各自有零空间，但未必是同一个零空间。

这个例子不是说 VINS-Mono 的真实视觉或 IMU Jacobian 就长成上面的二维矩阵。它只是抽象出一个核心现象：

$$
\text{不可观方向的坐标表达会随线性化点变化。}
$$

在 VIO 中，全局 yaw、姿态-位置耦合方向、特征点和相机位姿共同变化的 gauge 方向，都可能随当前状态改变。于是旧 prior 记住的是旧线性化点下的零空间，新视觉或 IMU 因子看到的是当前线性化点下的零空间。二者只要差一点点，就可能像上面的 $\epsilon$ 一样，给本来不可观的方向注入小但非零的信息。

##### 回到 VINS-Mono

VINS-Mono 的做法更接近 FEJ-like，因为：

$$
\text{marginalization prior 固定}
$$

但：

$$
\text{视觉重投影因子和 IMU 因子仍会按当前状态重线性化}
$$

所以它能缓解一部分边缘化 prior 带来的 consistency 问题，但不是严格 FEJ。

严格 FEJ 更一致，但代价也更明显：

- 要给每个状态和特征维护 first estimate；
- 普通视觉和 IMU Jacobian 不再是当前状态处最精确的 Jacobian；
- 如果 first estimate 本身较差，固定 Jacobian 会引入较大的线性化误差；
- 工程侵入更大，尤其对复杂滑窗后端、外参、时间偏移、多传感器因子更麻烦。

FEJ-like 的好处是工程折中：

- prior 固定，避免历史信息反复漂移；
- 普通因子仍重线性化，优化精度和收敛性更好；
- 实现成本较低。

它的代价是 consistency 不如 strict FEJ：

$$
\text{FEJ-like} \Rightarrow
\text{只缓解零空间不一致，不保证完全消除零空间不一致}
$$

最后把二者放在一起看：

| 方法 | 固定哪些 Jacobian | 零空间一致性 | 当前线性化精度 | 工程代价 |
|---|---|---|---|---|
| 严格 FEJ | 视觉、IMU、prior 等相关因子都尽量用 first estimate | 更强 | 较弱 | 更高 |
| FEJ-like | 通常只固定边缘化 prior 或部分因子 | 中等，只能缓解 | 较强 | 较低 |

所以可以理解为：

$$
\text{FEJ} \Rightarrow
\text{用线性化精度换零空间一致性}
$$

#### 5.4.2 OC 方法：Observability-Constrained

OC 方法更直接。它显式构造不可观子空间：

$$
\mathbf{N}
$$

然后要求修正后的 Jacobian 满足：

$$
\mathbf{J}_{oc}\mathbf{N} =
\mathbf{0}
$$

如果原始 Jacobian 不满足：

$$
\mathbf{J}\mathbf{N} \neq
\mathbf{0}
$$

就对 Jacobian 做投影或修正，让它不要在不可观方向上产生响应。

一种抽象写法是，把 Jacobian 中落在不可观方向上的分量去掉：

$$
\mathbf{J}_{oc} =
\mathbf{J}
\left(
\mathbf{I} -
\mathbf{N}
\left(
\mathbf{N}^{\top}\mathbf{N}
\right)^{-1}
\mathbf{N}^{\top}
\right)
$$

这个式子中的括号可以理解为“投影到不可观子空间的正交补”。实际系统会根据具体状态参数化和权重做更细致的构造。

优点是：

- 理论目标清晰；
- 直接约束零空间；
- 比单纯固定 first estimate 更主动。

缺点是：

- 必须准确知道系统的不可观子空间；
- 多传感器、外参、时间偏移、接触约束会让 $\mathbf{N}$ 变复杂；
- 工程实现比 FEJ 更难。

#### 5.4.3 Invariant EKF / 群不变误差

Invariant EKF 的思路不是事后修 Jacobian，而是从误差定义上保持系统对称性。

普通误差可能写成：

$$
\tilde{\mathbf{x}} =
\mathbf{x} -
\hat{\mathbf{x}}
$$

但对于位姿、速度、IMU bias 这类状态，简单相减不一定符合系统几何结构。Invariant 方法会在 Lie group 上定义左不变或右不变误差，例如：

$$
\tilde{\mathbf{X}} =
\hat{\mathbf{X}}^{-1}\mathbf{X}
$$

或：

$$
\tilde{\mathbf{X}} =
\mathbf{X}\hat{\mathbf{X}}^{-1}
$$

其中：

- $\mathbf{X}$ 表示真实群状态；
- $\hat{\mathbf{X}}$ 表示估计群状态；
- $\tilde{\mathbf{X}}$ 表示群意义下的误差。

它的优势是：

$$
\text{误差动力学更符合系统对称性} \Rightarrow
\text{不可观方向更容易自然保持}
$$

优点是：

- consistency 通常比普通 EKF 更好；
- 从状态误差定义层面减少零空间破坏；
- 对惯性导航、腿式状态估计很有吸引力。

缺点是：

- 建模要求较高；
- 需要把系统状态组织成合适的群结构；
- 对复杂 VIO/SLAM 后端不一定容易直接迁移。

#### 5.4.4 Robocentric / 相对坐标表达

Robocentric 方法不把所有东西都放在固定世界系里，而是用机器人当前局部坐标系表达状态。

例如不是长期维护：

$$
\mathbf{p}_{w b_k}
$$

而是维护相对于当前机器人或局部参考帧的位姿和特征点：

$$
\mathbf{p}_{b_i b_j},\quad
\mathbf{P}_{b_i l}
$$

这样可以减少全局 gauge 自由度对优化的影响。全局平移和全局 yaw 不再那么容易被历史 prior 错误锁死。

优点是：

- 更贴近传感器实际测到的相对量；
- 有助于降低全局不可观方向污染；
- 对局部 VIO 或滤波系统很有用。

缺点是：

- 坐标变换和状态管理更复杂；
- 闭环、全局地图、长时间一致性仍需要额外处理；
- 边缘化和重参数化依然要小心。

#### 5.4.5 少边缘化，或者保留原始因子

如果不把历史非线性因子压缩成固定 prior，而是保留原始因子：

$$
\mathbf{r}_{old}(\mathbf{x})
$$

那么后续可以继续重新线性化：

$$
\mathbf{J}_{old} =
\mathbf{J}_{old}(\mathbf{x}^{current})
$$

这会减少“历史 prior 冻结”的问题。

这类方法包括：

- full batch optimization；
- full smoothing；
- 更大窗口的 fixed-lag smoothing；
- keyframe graph 中保留更多历史非线性约束。

优点是：

- 更接近原始非线性问题；
- 历史因子可以重新线性化；
- 边缘化 prior 的冻结问题减少。

缺点是：

- 计算量增长；
- 内存增长；
- 实时性变差。

这也是滑窗 VIO 必须边缘化的根本原因：

$$
\text{实时性} \Rightarrow
\text{不能无限保留原始历史因子}
$$

#### 5.4.6 Nullspace Projection / Prior 修正

还有一类方法是在边缘化之后检查 prior 是否破坏不可观方向。

如果期望 prior 满足：

$$
\mathbf{H}_{prior}\mathbf{N} =
\mathbf{0}
$$

但实际得到：

$$
\mathbf{H}_{prior}\mathbf{N} \neq
\mathbf{0}
$$

就对 prior 做修正，把不可观方向上的信息投影掉。

例如可以构造投影矩阵：

$$
\mathbf{P}_{\perp} =
\mathbf{I} -
\mathbf{N}
\left(
\mathbf{N}^{\top}\mathbf{N}
\right)^{-1}
\mathbf{N}^{\top}
$$

然后修正 Hessian：

$$
\mathbf{H}_{prior}^{new} =
\mathbf{P}_{\perp}^{\top}
\mathbf{H}_{prior}
\mathbf{P}_{\perp}
$$

这样可以减少 prior 在不可观方向上的虚假信息。

优点是：

- 直接作用在边缘化 prior 上；
- 可以和滑窗优化结合；
- 思路清晰。

缺点是：

- 需要准确构造 $\mathbf{N}$；
- 投影会丢掉一些信息；
- 若 $\mathbf{N}$ 本身估计不准，可能误删有效信息或保留虚假信息。

#### 5.4.7 引入真实额外约束

前面几类方法主要是在“不要错误制造信息”。另一类方法是引入真实的新测量，让原本不可观或弱可观方向真的变得更可观。

例如：

- GPS 提供全局位置；
- 磁力计提供绝对 heading；
- 已知地图提供全局约束；
- AprilTag 或 motion capture 提供绝对位姿；
- Contact Constraint 提供足端不滑动约束；
- 轮速计提供地面运动约束。

这类约束会真正改变信息矩阵：

$$
\mathbf{H}' =
\mathbf{H} +
\mathbf{J}_{extra}^{\top}
\mathbf{\Omega}_{extra}
\mathbf{J}_{extra}
$$

如果新增约束对原来的零空间方向有响应：

$$
\mathbf{J}_{extra}\mathbf{n} \neq
\mathbf{0}
$$

那么这个方向就真的被观测到了。

这和 FEJ / OC 不一样。FEJ / OC 的目标是：

$$
\text{不要把不可观方向错误变成可观}
$$

真实额外约束的目标是：

$$
\text{用新的传感器信息让某些方向真的变可观}
$$

后面的 Contact Constraint 就属于这一类。

#### 5.4.8 小结：没有银弹，只有取舍

可以把所有路线放在一条谱系上：

| 方法 | 核心思想 | 优点 | 代价 |
|---|---|---|---|
| FEJ / FEJ-like | 固定 first estimate Jacobian | 简单、实用、改善一致性 | 牺牲部分线性化精度 |
| OC | 显式约束不可观子空间 | 理论更直接 | 需要准确构造零空间 |
| Invariant EKF | 用群不变误差保持对称性 | 从误差定义上改善一致性 | 建模和实现复杂 |
| Robocentric | 用局部相对坐标表达 | 减少全局 gauge 污染 | 状态管理复杂 |
| 少边缘化 / 保留原始因子 | 允许历史因子重线性化 | 更接近原始非线性问题 | 计算和内存代价高 |
| Nullspace Projection | 修正 prior 的虚假信息 | 直接处理 prior | 可能丢信息 |
| 真实额外约束 | 引入新测量提高秩 | 真的增加信息 | 依赖约束可靠性 |

所以对这个问题最稳妥的认知是：

$$
\text{FEJ 是工程上常用的 consistency 保护手段；OC 和 invariant formulation 更系统；真实额外约束能从传感器信息层面改变可观性。}
$$

## 6. 为什么某个状态弱可观

一个状态弱可观，通常不是因为它完全没有测量，而是因为测量对它的敏感度很低。

设状态分成两部分：

$$
\mathbf{x} =
\begin{bmatrix}
\mathbf{x}_a\\
\mathbf{x}_b
\end{bmatrix}
$$

其中：

- $\mathbf{x}_a$ 表示比较容易观测的状态；
- $\mathbf{x}_b$ 表示可能弱可观的状态。

线性化残差为：

$$
\mathbf{r} \approx
\mathbf{r}_0 +
\mathbf{J}_a\delta\mathbf{x}_a +
\mathbf{J}_b\delta\mathbf{x}_b
$$

如果 $\mathbf{J}_b$ 的列很小：

$$
\|\mathbf{J}_b\|
\ll
\|\mathbf{J}_a\|
$$

说明残差对 $\mathbf{x}_b$ 不敏感，$\mathbf{x}_b$ 弱可观。

如果 $\mathbf{J}_b$ 的列几乎可以由 $\mathbf{J}_a$ 的列线性组合得到：

$$
\mathbf{J}_b \approx
\mathbf{J}_a\mathbf{C}
$$

其中 $\mathbf{C}$ 是某个系数矩阵，那么 $\mathbf{x}_b$ 与 $\mathbf{x}_a$ 强耦合，很难分开估计。这也是弱可观。

### 6.1 用奇异值看弱可观

设观测矩阵为：

$$
\mathbf{O}
$$

在固定窗口或滤波系统中，$\mathbf{O}$ 可以由多时刻线性化测量堆叠得到。线性系统中常见形式是：

$$
\delta\mathbf{x}_{k+1} =
\mathbf{F}_k\delta\mathbf{x}_k
$$

$$
\delta\mathbf{z}_k =
\mathbf{H}_k\delta\mathbf{x}_k
$$

其中：

- $\mathbf{F}_k$ 表示第 $k$ 时刻误差状态转移矩阵；
- $\mathbf{H}_k$ 表示第 $k$ 时刻测量 Jacobian；
- $\delta\mathbf{z}_k$ 表示测量残差的一阶扰动。

对初始状态 $\delta\mathbf{x}_0$ 的观测矩阵可以写成：

$$
\mathbf{O} =
\begin{bmatrix}
\mathbf{H}_0\\
\mathbf{H}_1\mathbf{F}_0\\
\mathbf{H}_2\mathbf{F}_1\mathbf{F}_0\\
\vdots
\end{bmatrix}
$$

其中 $\mathbf{O}$ 表示 observability matrix。

如果：

$$
\operatorname{rank}(\mathbf{O})<n
$$

则系统存在不可观方向。

对 $\mathbf{O}$ 做奇异值分解：

$$
\mathbf{O} =
\mathbf{U}\mathbf{S}\mathbf{V}^{\top}
$$

如果某个奇异值满足：

$$
\sigma_i=0
$$

对应方向不可观。

如果：

$$
0<\sigma_i\ll\sigma_{max}
$$

对应方向弱可观。

### 6.2 弱可观的常见原因

第一，运动激励不足。

例如单目 VIO 的尺度需要平移和惯性激励。如果设备几乎纯旋转，视觉能估计旋转，但尺度很弱：

$$
\text{纯旋转} \Rightarrow
\text{尺度弱可观或不可观}
$$

第二，状态之间强耦合。

例如加速度计 bias 和重力方向都能影响速度积分：

$$
\dot{\mathbf{v}} =
\mathbf{R}
\left(
\hat{\mathbf{a}}-\mathbf{b}_a
\right) +
\mathbf{g}
$$

其中：

- $\mathbf{b}_a$ 表示加速度计 bias；
- $\mathbf{g}$ 表示重力向量。

如果运动不丰富，$\mathbf{b}_a$ 和 $\mathbf{g}$ 的影响可能很难区分。

第三，观测几何退化。

例如所有特征点都很远，视差很小，深度方向就弱可观。

第四，窗口太短。

某些状态需要时间累积才显现。例如 bias 通过积分影响速度和位置，如果窗口太短，信息还没有充分积累。

第五，噪声太大。

即使 $\mathbf{J}\mathbf{n}\neq 0$，如果测量噪声协方差很大，信息矩阵：

$$
\mathbf{\Omega} =
\mathbf{\Sigma}^{-1}
$$

会很小。该方向的信息量：

$$
\mathbf{n}^{\top}
\mathbf{J}^{\top}
\mathbf{\Omega}
\mathbf{J}
\mathbf{n}
$$

也会很小。

### 6.3 弱可观的后果

弱可观状态通常有几个表现：

- 估计会慢慢收敛，而不是快速收敛；
- 对初值敏感；
- 对噪声和外点敏感；
- 对先验和边缘化更敏感；
- Hessian 条件数变差；
- 协方差较大，或者若处理不当会过度自信。

所以“弱可观”不是一个二值标签，而是一个强度问题：

$$
\text{不可观} \rightarrow
\text{弱可观} \rightarrow
\text{强可观}
$$

对应：

$$
\sigma=0 \rightarrow
0<\sigma\ll 1 \rightarrow
\sigma \text{ 足够大}
$$

## 7. Contact Constraint 是什么

Contact Constraint 通常出现在腿式机器人、足式惯导、视觉-惯性-腿式里程计等系统中。

它的核心假设是：

$$
\text{当脚或接触点稳定接触地面时，该接触点在世界系中不滑动。}
$$

设机器人机体系为 $b$，世界系为 $w$。机体状态包括：

$$
\mathbf{p}_{wb},\quad
\mathbf{R}_{wb},\quad
\mathbf{v}_{wb}
$$

其中：

- $\mathbf{p}_{wb}$ 表示机体在世界系中的位置；
- $\mathbf{R}_{wb}$ 表示从机体系到世界系的旋转；
- $\mathbf{v}_{wb}$ 表示机体在世界系中的速度。

设接触点在机体系中的位置为：

$$
\mathbf{p}_{bf}
$$

其中 $\mathbf{p}_{bf}$ 表示足端或接触点相对于机体的向量。

接触点在世界系中的位置为：

$$
\mathbf{p}_{wf} =
\mathbf{p}_{wb} +
\mathbf{R}_{wb}\mathbf{p}_{bf}
$$

如果接触点不滑动，那么在接触期间：

$$
\mathbf{p}_{wf,k+1} -
\mathbf{p}_{wf,k} =
\mathbf{0}
$$

这可以构成位置接触约束：

$$
\mathbf{r}_{contact,p} =
\left(
\mathbf{p}_{wb,k+1} +
\mathbf{R}_{wb,k+1}\mathbf{p}_{bf,k+1}
\right) -
\left(
\mathbf{p}_{wb,k} +
\mathbf{R}_{wb,k}\mathbf{p}_{bf,k}
\right)
$$

如果约束成立，理想情况下：

$$
\mathbf{r}_{contact,p} =
\mathbf{0}
$$

也可以写速度接触约束。接触点世界系速度为：

$$
\mathbf{v}_{wf} =
\mathbf{v}_{wb} +
\mathbf{R}_{wb}
\left(
\boldsymbol{\omega}_{b}^{\wedge}\mathbf{p}_{bf} +
\dot{\mathbf{p}}_{bf}
\right)
$$

其中：

- $\mathbf{v}_{wf}$ 表示接触点在世界系中的速度；
- $\boldsymbol{\omega}_{b}$ 表示机体系角速度；
- $\dot{\mathbf{p}}_{bf}$ 表示接触点在机体系中的相对速度；
- $(\cdot)^{\wedge}$ 表示反对称矩阵。

稳定接触时：

$$
\mathbf{v}_{wf} =
\mathbf{0}
$$

所以速度接触残差为：

$$
\mathbf{r}_{contact,v} =
\mathbf{v}_{wb} +
\mathbf{R}_{wb}
\left(
\boldsymbol{\omega}_{b}^{\wedge}\mathbf{p}_{bf} +
\dot{\mathbf{p}}_{bf}
\right)
$$

这类残差就是 Contact Constraint。

## 8. 为什么 Contact Constraint 能提高秩

假设原来的信息矩阵为：

$$
\mathbf{H} =
\sum_i
\mathbf{J}_i^{\top}
\mathbf{\Omega}_i
\mathbf{J}_i
$$

加入接触约束后，多了一个残差：

$$
\mathbf{r}_c(\mathbf{x})
$$

它的 Jacobian 为：

$$
\mathbf{J}_c =
\frac{\partial\mathbf{r}_c}
{\partial\delta\mathbf{x}}
$$

信息矩阵变成：

$$
\mathbf{H}' =
\mathbf{H} +
\mathbf{J}_c^{\top}
\mathbf{\Omega}_c
\mathbf{J}_c
$$

其中：

- $\mathbf{H}'$ 表示加入 contact constraint 后的信息矩阵；
- $\mathbf{\Omega}_c$ 表示接触约束的信息矩阵；
- $\mathbf{J}_c^{\top}\mathbf{\Omega}_c\mathbf{J}_c$ 表示接触约束新增的信息。

由于：

$$
\mathbf{J}_c^{\top}
\mathbf{\Omega}_c
\mathbf{J}_c
$$

是半正定矩阵，所以：

$$
\operatorname{rank}(\mathbf{H}') \ge
\operatorname{rank}(\mathbf{H})
$$

更准确地看零空间。对任意方向 $\mathbf{n}$：

$$
\mathbf{n}^{\top}\mathbf{H}'\mathbf{n} =
\mathbf{n}^{\top}\mathbf{H}\mathbf{n} +
\mathbf{n}^{\top}
\mathbf{J}_c^{\top}
\mathbf{\Omega}_c
\mathbf{J}_c
\mathbf{n}
$$

第二项可以写成：

$$
\mathbf{n}^{\top}
\mathbf{J}_c^{\top}
\mathbf{\Omega}_c
\mathbf{J}_c
\mathbf{n} =
\left\|
\mathbf{\Omega}_c^{1/2}
\mathbf{J}_c
\mathbf{n}
\right\|^2
$$

所以：

$$
\mathbf{n}^{\top}\mathbf{H}'\mathbf{n} =
\mathbf{n}^{\top}\mathbf{H}\mathbf{n} +
\left\|
\mathbf{\Omega}_c^{1/2}
\mathbf{J}_c
\mathbf{n}
\right\|^2
$$

如果 $\mathbf{n}$ 属于新信息矩阵的零空间，则必须同时满足：

$$
\mathbf{n}^{\top}\mathbf{H}\mathbf{n} =
0
$$

以及：

$$
\mathbf{J}_c\mathbf{n} =
\mathbf{0}
$$

因此：

$$
\mathcal{N}(\mathbf{H}') =
\mathcal{N}(\mathbf{H})
\cap
\mathcal{N}(\mathbf{J}_c)
$$

其中：

- $\mathcal{N}(\mathbf{H})$ 表示原信息矩阵的零空间；
- $\mathcal{N}(\mathbf{J}_c)$ 表示接触约束 Jacobian 的零空间。

这条公式非常重要。它说明：

$$
\text{加入 Contact Constraint 后，新的零空间是旧零空间和接触约束零空间的交集。}
$$

如果接触约束对旧的不可观方向有响应：

$$
\mathbf{J}_c\mathbf{n} \neq
\mathbf{0}
$$

那么这个方向就不再属于新零空间。零空间维度下降，信息矩阵秩上升。

也就是：

$$
\dim\mathcal{N}(\mathbf{H}') <
\dim\mathcal{N}(\mathbf{H})
$$

等价于：

$$
\operatorname{rank}(\mathbf{H}') >
\operatorname{rank}(\mathbf{H})
$$

这就是 Contact Constraint 能提高秩的数学原因。

## 9. Contact Constraint 提高哪些状态的可观性

Contact Constraint 不会神奇地让所有状态都强可观。它提高哪些方向的秩，取决于它的 Jacobian 对哪些方向敏感。

### 9.1 速度更可观

速度接触约束为：

$$
\mathbf{r}_{contact,v} =
\mathbf{v}_{wb} +
\mathbf{R}_{wb}
\left(
\boldsymbol{\omega}_{b}^{\wedge}\mathbf{p}_{bf} +
\dot{\mathbf{p}}_{bf}
\right)
$$

它对机体速度的 Jacobian 近似包含：

$$
\frac{\partial\mathbf{r}_{contact,v}}
{\partial\mathbf{v}_{wb}} =
\mathbf{I}
$$

这意味着速度被直接约束。

如果没有接触约束，速度主要通过 IMU 积分和视觉间接约束。加入接触后，某些速度方向会从弱可观变得更强可观。

### 9.2 加速度计 bias 更可观

IMU 速度传播中有：

$$
\mathbf{v}_{k+1} =
\mathbf{v}_k +
\left[
\mathbf{R}_k
\left(
\hat{\mathbf{a}}_k-\mathbf{b}_{a,k}
\right) +
\mathbf{g}
\right]\Delta t
$$

其中 $\mathbf{b}_{a,k}$ 是加速度计 bias。

如果 contact constraint 约束了速度：

$$
\mathbf{v}_{wf} \approx
\mathbf{0}
$$

那么速度积分误差不能任意漂移。加速度计 bias 的错误会通过速度误差显现出来，因此 $\mathbf{b}_a$ 的可观性增强。

直观地说：

$$
\text{脚站住了，身体速度不能随便漂。}
$$

所以 IMU bias 更容易被识别。

### 9.3 相对位置和姿态更可观

位置接触约束为：

$$
\mathbf{p}_{wf,k+1} -
\mathbf{p}_{wf,k} =
\mathbf{0}
$$

展开：

$$
\left(
\mathbf{p}_{wb,k+1} +
\mathbf{R}_{wb,k+1}\mathbf{p}_{bf,k+1}
\right) -
\left(
\mathbf{p}_{wb,k} +
\mathbf{R}_{wb,k}\mathbf{p}_{bf,k}
\right) =
\mathbf{0}
$$

这个约束连接两个时刻的机体位置和姿态。

它对平移有 Jacobian：

$$
\frac{\partial\mathbf{r}_{contact,p}}
{\partial\mathbf{p}_{wb,k+1}} =
\mathbf{I}
$$

$$
\frac{\partial\mathbf{r}_{contact,p}}
{\partial\mathbf{p}_{wb,k}} = -
\mathbf{I}
$$

它对姿态也有 Jacobian：

$$
\frac{\partial
\left(
\mathbf{R}_{wb}\mathbf{p}_{bf}
\right)}
{\partial\delta\boldsymbol{\theta}} \approx -
\mathbf{R}_{wb}
\mathbf{p}_{bf}^{\wedge}
$$

这意味着 contact constraint 不仅约束平移，也通过足端杆臂约束姿态。

如果接触点离机体中心较远，$\mathbf{p}_{bf}$ 较大，那么姿态变化会造成明显足端位置变化，姿态约束更强。

### 9.4 yaw 是否会变得可观，要看接触模型

这是很关键的细节。

单个未知接触点、未知世界位置、且地面没有方向信息时，全局 yaw 不一定完全可观。因为整个系统绕重力方向旋转，接触点世界位置也一起旋转，很多相对约束仍然不变。

但以下情况可能增强 yaw 可观性：

- 接触点世界位置已知；
- 接触发生在已知地图或已知地形上；
- 有多个非共线接触点；
- 足端姿态或接触面法向提供方向信息；
- 接触约束与视觉、IMU、腿部运动学形成足够丰富的组合激励。

因此更准确地说：

$$
\text{Contact Constraint 能提高秩，但提高哪些方向的秩取决于接触提供了哪些独立信息。}
$$

## 10. 用观测矩阵解释 Contact Constraint 提高秩

在线性时变系统中：

$$
\delta\mathbf{x}_{k+1} =
\mathbf{F}_k\delta\mathbf{x}_k
$$

$$
\delta\mathbf{z}_k =
\mathbf{H}_k\delta\mathbf{x}_k
$$

原来的观测矩阵为：

$$
\mathbf{O} =
\begin{bmatrix}
\mathbf{H}_0\\
\mathbf{H}_1\mathbf{F}_0\\
\mathbf{H}_2\mathbf{F}_1\mathbf{F}_0\\
\vdots
\end{bmatrix}
$$

加入接触测量后，接触测量 Jacobian 为：

$$
\mathbf{H}_{c,k}
$$

观测矩阵变为：

$$
\mathbf{O}' =
\begin{bmatrix}
\mathbf{H}_0\\
\mathbf{H}_{c,0}\\
\mathbf{H}_1\mathbf{F}_0\\
\mathbf{H}_{c,1}\mathbf{F}_0\\
\mathbf{H}_2\mathbf{F}_1\mathbf{F}_0\\
\mathbf{H}_{c,2}\mathbf{F}_1\mathbf{F}_0\\
\vdots
\end{bmatrix}
$$

也就是说，Contact Constraint 给观测矩阵增加了新行。

如果新增行是原来行空间的线性组合：

$$
\operatorname{row}(\mathbf{H}_{c,k})
\subseteq
\operatorname{row}(\mathbf{O})
$$

那么秩不变。

如果新增行提供了原来没有的独立方向：

$$
\operatorname{row}(\mathbf{H}_{c,k})
\not\subseteq
\operatorname{row}(\mathbf{O})
$$

则：

$$
\operatorname{rank}(\mathbf{O}') >
\operatorname{rank}(\mathbf{O})
$$

由于信息矩阵可以写成：

$$
\mathbf{H} =
\mathbf{O}^{\top}
\mathbf{R}^{-1}
\mathbf{O}
$$

在噪声协方差 $\mathbf{R}$ 正定时：

$$
\operatorname{rank}(\mathbf{H}) =
\operatorname{rank}(\mathbf{O})
$$

所以观测矩阵秩提高，也对应信息矩阵秩提高。

## 11. Contact Constraint 也可能只是改善条件数

有时 contact constraint 并不会真正增加秩，但会让弱可观方向变强。

例如原来某方向已经有很小的信息：

$$
0<\lambda_i\ll1
$$

加入接触后：

$$
\lambda_i' =
\lambda_i +
\Delta\lambda_i
$$

如果：

$$
\Delta\lambda_i>0
$$

则该方向更容易估计。

此时秩可能没有变化：

$$
\operatorname{rank}(\mathbf{H}') =
\operatorname{rank}(\mathbf{H})
$$

但条件数改善：

$$
\kappa(\mathbf{H}') <
\kappa(\mathbf{H})
$$

其中 $\kappa(\cdot)$ 表示条件数。

这也很有意义。因为实际系统不只关心 rank，还关心数值稳定性和估计方差。

## 12. Contact Constraint 失效时会发生什么

Contact Constraint 的前提是接触可靠。如果足端打滑，真实约束不再成立：

$$
\mathbf{v}_{wf} \neq
\mathbf{0}
$$

但系统仍然强行使用：

$$
\mathbf{v}_{wf} =
\mathbf{0}
$$

那么接触约束会注入错误信息。

从信息矩阵看，它仍然会提高某些方向的信息量，但这些信息是错误的：

$$
\mathbf{J}_c^{\top}
\mathbf{\Omega}_c
\mathbf{J}_c
$$

会把状态拉向错误方向。

因此 contact constraint 通常需要：

- 接触检测；
- 滑动检测；
- 根据接触置信度调整 $\mathbf{\Omega}_c$；
- 对接触残差使用鲁棒核；
- 在可能打滑时降低约束权重。

这也说明：

$$
\text{提高秩不一定总是好事，前提是新增约束必须真实可靠。}
$$

## 13. 几个问题的直接回答

### 13.1 为什么 FEJ 会改善 Consistency？

因为 FEJ 固定 first estimate 处的 Jacobian，让相关因子的零空间尽量保持一致。

理想情况下，不可观方向 $\mathbf{N}$ 应满足：

$$
\mathbf{J}\mathbf{N} =
\mathbf{0}
$$

普通重线性化可能让不同因子在不同线性化点拥有不同零空间，组合后产生：

$$
\mathbf{N}^{\top}\mathbf{H}\mathbf{N} >
0
$$

也就是不可观方向被错误注入信息。

FEJ 通过固定线性化参考点，使这些方向更容易保持在零空间中，因此减少过度自信，改善 consistency。

严格 FEJ 的要求更强：

$$
\forall k,\quad
\mathbf{J}_k =
\mathbf{J}_k
\left(
\bar{\mathbf{x}}^{FEJ}
\right)
$$

也就是所有相关因子的 Jacobian 都尽量用 first estimate 计算。VINS-Mono 更接近 FEJ-like：边缘化 prior 固定旧线性化点，但普通视觉和 IMU 因子仍然按当前状态重线性化。

新因子加入时也遵循同样规则：它的 Jacobian 不是由“新因子自己的 first estimate”决定，而是由它连接的变量的 first estimate 决定：

$$
\mathbf{J}_k^{FEJ} =
\left.
\frac{\partial\mathbf{r}_k}
{\partial\delta\mathbf{x}_{\mathcal{S}_k}}
\right|_{
\mathbf{x}_s=\bar{\mathbf{x}}_s^{FEJ},
\ s\in\mathcal{S}_k
}
$$

因此严格 FEJ 的主要代价也很明确：如果 first estimate 很差，后续新因子的 Jacobian 也会长期受这个错误参考点影响。FEJ 改善的是 consistency，不保证每次线性化都最准确。

### 13.2 为什么边缘化会破坏可观性？

因为边缘化把历史非线性因子压缩成固定线性先验。

这个 prior 在边缘化点 $\bar{\mathbf{x}}$ 附近有正确零空间：

$$
\mathbf{H}^{prior}
\mathbf{N}(\bar{\mathbf{x}}) =
\mathbf{0}
$$

但后续状态变为 $\mathbf{x}^{new}$ 后，真实不可观方向可能变为：

$$
\mathbf{N}(\mathbf{x}^{new})
$$

通常：

$$
\mathbf{N}(\mathbf{x}^{new}) \neq
\mathbf{N}(\bar{\mathbf{x}})
$$

于是旧 prior 可能满足不了：

$$
\mathbf{H}^{prior}
\mathbf{N}(\mathbf{x}^{new}) =
\mathbf{0}
$$

它就会对本该不可观的方向产生约束。由于被边缘化的变量已经删除，无法严格重新边缘化，所以这个问题会长期存在。

注意，边缘化 prior 不一定要直接连接最新变量，才能影响最新变量。只要最新变量通过 IMU 或视觉相对因子和旧变量在同一个连通图里，那么：

$$
\text{旧变量被 prior 错误锚住} +
\text{相对因子连接旧变量和新变量} \Rightarrow
\quad
\text{新变量也会被间接锚住}
$$

所以问题不在于 prior 是否直接作用到最新帧，而在于它是否错误约束了整个滑窗共同拥有的 gauge 自由度。

### 13.3 为什么某个状态弱可观？

因为测量对这个状态有信息，但信息量很小。

数学上表现为观测矩阵或信息矩阵存在很小但非零的奇异值或特征值：

$$
0<\sigma_i\ll1
$$

或：

$$
0<\lambda_i\ll1
$$

常见原因包括：

- 运动激励不足；
- 状态之间强耦合；
- 几何退化；
- 窗口太短；
- 测量噪声太大；
- 该状态只通过积分链条间接影响观测。

弱可观方向不是完全不可估，但估计慢、方差大、对初值和先验敏感。

### 13.4 为什么 Contact Constraint 能提高秩？

因为 Contact Constraint 添加了新的独立测量行。

加入接触约束后：

$$
\mathbf{H}' =
\mathbf{H} +
\mathbf{J}_c^{\top}
\mathbf{\Omega}_c
\mathbf{J}_c
$$

新零空间为：

$$
\mathcal{N}(\mathbf{H}') =
\mathcal{N}(\mathbf{H})
\cap
\mathcal{N}(\mathbf{J}_c)
$$

如果某个旧不可观方向 $\mathbf{n}$ 被 contact constraint 观测到：

$$
\mathbf{J}_c\mathbf{n} \neq
\mathbf{0}
$$

那么它不再属于新零空间。零空间维度下降，秩上升：

$$
\operatorname{rank}(\mathbf{H}') >
\operatorname{rank}(\mathbf{H})
$$

如果 contact constraint 没有增加独立方向，它也可能不提高秩，但仍可能增大某些小特征值，从而改善弱可观方向的条件数。

### 13.5 目前有没有解决方案？

有，但没有一个方案能同时做到：

$$
\text{完全一致} +
\text{最高精度} +
\text{实时高效} +
\text{工程简单}
$$

所以实际系统通常是在不同目标之间组合取舍。

如果目标是“不要把不可观方向错误变成可观”，核心约束是：

$$
\mathbf{J}\mathbf{N} =
\mathbf{0}
$$

也就是因子 Jacobian $\mathbf{J}$ 不应该对不可观方向 $\mathbf{N}$ 有响应。FEJ、OC 方法、nullspace projection 和 invariant formulation 都是在围绕这个目标做文章。

如果目标是“减少边缘化 prior 冻结带来的历史误差”，可以保留更多原始非线性因子、增大滑窗，或者做 full smoothing。这样历史因子仍然可以在当前状态处重新线性化，但代价是计算量和内存增长。

如果目标是“让某些方向真的变得可观”，则需要引入真实额外测量：

$$
\mathbf{H}' =
\mathbf{H} +
\mathbf{J}_{extra}^{\top}
\mathbf{\Omega}_{extra}
\mathbf{J}_{extra}
$$

当新增约束满足：

$$
\mathbf{J}_{extra}\mathbf{n} \neq
\mathbf{0}
$$

它才是真的给原来的弱可观或不可观方向 $\mathbf{n}$ 增加了信息。GPS、磁力计、已知地图、AprilTag、轮速计、Contact Constraint 都属于这类思路。

因此可以把解决方案分成三层：

| 层次 | 解决什么 | 典型方法 |
|---|---|---|
| 保持零空间 | 不要制造虚假可观性 | FEJ、OC、nullspace projection、Invariant EKF |
| 减少冻结误差 | 降低边缘化 prior 的历史线性化影响 | 大窗口、fixed-lag smoothing、full smoothing、保留更多原始因子 |
| 增加真实信息 | 让状态真的更可观 | GPS、磁力计、地图约束、Contact Constraint、轮速计 |

一句话总结：

$$
\text{FEJ 等方法是在防止“假信息”；额外传感器或 contact constraint 是在引入“真信息”。}
$$

## 14. 最后一条主线

把整篇压缩成一句话：

$$
\text{Consistency 关心不要在不可观方向上过度自信；FEJ 通过固定线性化零空间来改善它；边缘化因为冻结历史线性先验而容易破坏零空间；弱可观对应小奇异值；Contact Constraint 通过增加独立观测行收缩零空间，从而提高秩或改善条件数；工程解决方案本质是在防假信息、减少冻结误差和引入真信息之间取舍。}
$$
