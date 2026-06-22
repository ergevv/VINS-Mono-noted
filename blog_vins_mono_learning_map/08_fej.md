# 08 FEJ：为什么边缘化先验要固定第一次线性化点

## 1. FEJ 的定义

FEJ 是 First Estimate Jacobian 的缩写，可以直译为“首次估计雅克比”。它的核心定义是：

$$
\text{FEJ}
\;=\;
\text{在 first estimate 处计算并固定使用的雅克比}
$$

也就是说，对于某些已经线性化并压缩过的信息，后续优化时不再随着当前状态估计变化而反复重算雅克比，而是继续使用第一次估计点处的雅克比。

在滑动窗口优化中，FEJ 最容易和边缘化先验联系起来。边缘化会把旧变量和旧因子压缩成一个线性先验，这个先验一旦形成，就应该保持它的线性化结构一致。FEJ 要固定的，正是历史信息对应的参考线性化点和雅克比结构。

不过，对 VINS-Mono 要特别区分两个层次：

$$
\text{固定边缘化先验的线性化点}
\neq
\text{严格意义上的 FEJ 策略}
$$

VINS-Mono 的边缘化先验在生成之后，确实会固定当时的线性化点和先验雅克比。也就是说，后续使用这个 prior factor 时，不会重新对已经边缘化掉的历史非线性因子做线性化。

但崔华坤的《VINS 论文推导及代码解析》6.4 节提到“VINS 中并未使用 FEJ 的策略”，这里说的是更严格的 FEJ：对仍留在滑动窗口内、后续还会继续参与普通视觉/IMU 因子的变量，也要使用它们的 first estimate 来计算相关雅克比，从而让不同因子的零空间保持一致。

VINS-Mono 并没有对所有仍在窗口内的普通视觉因子、IMU 因子都采用 first-estimate Jacobian。普通非线性因子仍会随着当前估计重新线性化。因此，更准确的说法是：

$$
\text{VINS-Mono 固定了边缘化 prior 的线性化结构}
$$

$$
\text{但没有实现严格的全局 FEJ 策略}
$$

可以先粗略记住：

$$
\text{当前状态可以继续优化}
$$

$$
\text{但历史先验的雅克比固定在 first estimate 上}
$$

严格 FEJ 的目的，是避免已经边缘化的历史信息和后续仍在重线性化的因子之间产生不一致的线性化结构，尤其是避免给全局平移、全局 yaw 这类不可观方向注入虚假信息。

可以把这三种说法放在一起看：

| 说法 | 固定什么 | 重新线性化什么 | 含义 |
|---|---|---|---|
| 普通滑窗优化 | 不固定普通因子的雅克比 | 当前窗口内视觉/IMU 因子 | 更贴近当前非线性状态 |
| VINS-Mono 的边缘化 prior | 固定 prior 的线性化点和 prior 雅克比 | 新加入或仍存在的普通因子 | 历史信息被压缩后不再重算 |
| 严格 FEJ | 相关变量的普通因子雅克比也使用 first estimate | 只更新残差中的状态差分 | 更强调一致的不可观子空间 |

所以，“VINS 没有使用 FEJ”和“VINS 固定 marginalization prior”并不矛盾。前者说的是没有采用严格全局 FEJ 策略，后者说的是边缘化先验作为线性化信息必须固定。

## 2. 边缘化留下的是线性化后的历史

边缘化的目标是删除旧变量，同时保留旧变量对当前窗口的约束。上一篇中，我们把边缘化后的先验写成：

$$
\mathbf{r}_{\text{prior}}
=
\mathbf{r}^{\text{prior}}_0
+
\mathbf{J}^{\text{prior}}\delta\mathcal{X}_r
$$

其中：

- $\mathbf{r}_{\text{prior}}$ 表示边缘化后形成的先验残差；
- $\mathbf{r}^{\text{prior}}_0$ 表示在边缘化线性化点处的先验残差；
- $\mathbf{J}^{\text{prior}}$ 表示先验残差对保留变量增量的雅克比；
- $\delta\mathcal{X}_r$ 表示保留变量相对于边缘化线性化点的增量；
- $\mathcal{X}_r$ 表示边缘化后仍保留在窗口中的变量。

注意一个关键事实：边缘化发生时，旧的非线性因子已经被压缩成了一个线性先验。原始非线性残差已经不在了。也就是说，边缘化之后我们不再拥有完整的函数：

$$
\mathbf{r}_{\text{old}}(\mathcal{X}_m,\mathcal{X}_r)
$$

其中：

- $\mathbf{r}_{\text{old}}$ 表示边缘化前涉及旧变量的原始非线性残差；
- $\mathcal{X}_m$ 表示已经被边缘化的变量；
- $\mathcal{X}_r$ 表示保留下来的变量。

我们只拥有它在某个点附近的一阶近似。这个点可以记作：

$$
\bar{\mathcal{X}}_r
$$

其中 $\bar{\mathcal{X}}_r$ 表示边缘化发生时保留变量的线性化点。

因此，边缘化先验更准确的写法是：

$$
\mathbf{r}_{\text{prior}}(\mathcal{X}_r)
\approx
\mathbf{r}^{\text{prior}}(\bar{\mathcal{X}}_r)
+
\mathbf{J}^{\text{prior}}(\bar{\mathcal{X}}_r)
\left(
\mathcal{X}_r \boxminus \bar{\mathcal{X}}_r
\right)
$$

其中：

- $\boxminus$ 表示流形上的差分操作；
- $\mathbf{J}^{\text{prior}}(\bar{\mathcal{X}}_r)$ 表示在 $\bar{\mathcal{X}}_r$ 处计算得到的先验雅克比。

这就是边缘化 prior 中会固定的对象：已经压缩好的先验雅克比，以及它对应的线性化参考点。它和 FEJ 思想很接近，但是否称为严格 FEJ，还取决于窗口内其他仍然保留的因子是否也统一使用 first estimate 雅克比。

## 3. 什么是 First Estimate

First Estimate 可以翻译成“第一次估计”或“首次线性化估计”。在滑动窗口边缘化语境下，它指的是某个变量第一次参与边缘化先验构造时的取值。

记这个值为：

$$
\bar{\mathbf{x}}_k
$$

其中：

- $\bar{\mathbf{x}}_k$ 表示状态 $\mathbf{x}_k$ 的 first estimate；
- $\mathbf{x}_k$ 表示后续优化过程中的当前状态估计。

更直观地说，$\mathbf{x}_k$ 是优化过程中正在被更新的状态量，它会随着迭代不断变化；而 $\bar{\mathbf{x}}_k$ 是这个状态第一次被拿来做线性化或构造边缘化先验时记录下来的参考值。后续即使 $\mathbf{x}_k$ 继续被优化，$\bar{\mathbf{x}}_k$ 通常也保持不变，用来保证先验雅克比和残差增量始终相对于同一个参考点来解释。

对于普通非线性因子，例如当前窗口中的视觉重投影因子，我们仍然可以在每次迭代中重新线性化：

$$
\mathbf{r}(\mathbf{x})
\approx
\mathbf{r}(\mathbf{x}^{(t)})
+
\mathbf{J}(\mathbf{x}^{(t)})
\delta\mathbf{x}
$$

其中：

- $\mathbf{x}^{(t)}$ 表示第 $t$ 次迭代时的当前估计；
- $\mathbf{J}(\mathbf{x}^{(t)})$ 表示在当前估计处计算的雅克比；
- $\delta\mathbf{x}$ 表示本次迭代要求解的增量。

但对于已经边缘化得到的先验，FEJ 或 FEJ-like 的边缘化 prior 会使用：

$$
\mathbf{r}_{\text{prior}}(\mathbf{x})
\approx
\mathbf{r}_{\text{prior}}(\bar{\mathbf{x}})
+
\mathbf{J}_{\text{prior}}(\bar{\mathbf{x}})
\left(
\mathbf{x}\boxminus\bar{\mathbf{x}}
\right)
$$

其中：

- $\bar{\mathbf{x}}$ 是 first estimate；
- $\mathbf{J}_{\text{prior}}(\bar{\mathbf{x}})$ 是在 first estimate 处计算并固定不变的先验雅克比；
- $\mathbf{x}\boxminus\bar{\mathbf{x}}$ 是当前状态相对 first estimate 的增量。

## 4. 为什么不能随便重算边缘化先验的雅克比

设一个非线性残差为：

$$
\mathbf{r}(\mathbf{x}_m,\mathbf{x}_r)
$$

其中：

- $\mathbf{x}_m$ 是将被边缘化的变量；
- $\mathbf{x}_r$ 是将被保留的变量。

边缘化发生在某个线性化点：

$$
\bar{\mathbf{x}}_m,\quad \bar{\mathbf{x}}_r
$$

在这个点附近线性化：

$$
\mathbf{r}(\mathbf{x}_m,\mathbf{x}_r)
\approx
\bar{\mathbf{r}}
+
\mathbf{J}_m\delta\mathbf{x}_m
+
\mathbf{J}_r\delta\mathbf{x}_r
$$

其中：

- $\bar{\mathbf{r}}=\mathbf{r}(\bar{\mathbf{x}}_m,\bar{\mathbf{x}}_r)$ 表示线性化点处残差；
- $\mathbf{J}_m$ 表示残差对 $\mathbf{x}_m$ 增量的雅克比；
- $\mathbf{J}_r$ 表示残差对 $\mathbf{x}_r$ 增量的雅克比；
- $\delta\mathbf{x}_m=\mathbf{x}_m\boxminus\bar{\mathbf{x}}_m$；
- $\delta\mathbf{x}_r=\mathbf{x}_r\boxminus\bar{\mathbf{x}}_r$。

然后我们通过 Schur 补消去 $\delta\mathbf{x}_m$，得到关于 $\delta\mathbf{x}_r$ 的先验：

$$
\mathbf{r}_{\text{prior}}
\approx
\mathbf{r}^{\text{prior}}_0
+
\mathbf{J}^{\text{prior}}_r\delta\mathbf{x}_r
$$

这里的 $\mathbf{J}^{\text{prior}}_r$ 是由 $\bar{\mathbf{r}}$、$\mathbf{J}_m$、$\mathbf{J}_r$ 共同决定的。它不是原始非线性函数的一个简单雅克比，而是“边缘化之后的线性信息压缩结果”。

边缘化之后，$\mathbf{x}_m$ 已经不存在。假如后续 $\mathbf{x}_r$ 变化了，我们没有办法回到原始非线性函数中重新计算严格等价的新 Schur 补，因为重新边缘化需要 $\mathbf{x}_m$，而它已经被丢掉了。

因此，如果后续对这个先验的雅克比做某种“看起来合理”的更新，很可能会得到一个并不对应任何真实边缘概率的先验。

FEJ 的做法非常克制：

$$
\mathbf{J}^{\text{prior}}_r
\quad
\text{固定为边缘化时得到的值}
$$

后续只更新：

$$
\delta\mathbf{x}_r
=
\mathbf{x}_r\boxminus\bar{\mathbf{x}}_r
$$

也就是：

$$
\mathbf{r}_{\text{prior}}(\mathbf{x}_r)
=
\mathbf{r}^{\text{prior}}_0
+
\mathbf{J}^{\text{prior}}_r
\left(
\mathbf{x}_r\boxminus\bar{\mathbf{x}}_r
\right)
$$

这就是“残差随当前状态变化，雅克比固定在 first estimate”的含义。

## 5. 一个一维例子：线性化点改变会改变约束含义

考虑一个简单非线性残差：

$$
r(x)
=
x^2-z
$$

其中：

- $x$ 是未知变量；
- $z$ 是测量值；
- $r(x)$ 是预测值 $x^2$ 与测量 $z$ 的差。

在 first estimate $\bar{x}$ 处线性化：

$$
r(x)
\approx
r(\bar{x})
+
J(\bar{x})(x-\bar{x})
$$

其中：

$$
J(\bar{x})
=
\frac{dr}{dx}\bigg|_{\bar{x}}
=
2\bar{x}
$$

如果 $\bar{x}=1$，则：

$$
J(\bar{x})=2
$$

线性化模型是：

$$
r(x)
\approx
1-z+2(x-1)
$$

如果后续当前估计变成 $x=2$，普通非线性优化可以重新计算雅克比：

$$
J(2)=4
$$

但如果这个因子已经被边缘化压缩成先验，原始函数 $r(x)=x^2-z$ 已经不再存在。此时重新把雅克比改成 $4$，等价于假装我们仍然拥有原始非线性因子，并且还能重新线性化它。这在边缘化先验中通常是不成立的。

所以 FEJ 固定：

$$
J=J(\bar{x})=2
$$

后续只改变：

$$
x-\bar{x}
$$

这个例子虽然简单，但揭示了核心问题：边缘化先验不是原始非线性函数本身，而是原始函数在某个点附近压缩出来的一阶信息。

## 6. 流形状态下的 FEJ

VIO 中的姿态不是普通欧式变量，而在旋转群 $SO(3)$ 上。位姿通常写成：

$$
\mathbf{T}
=
\left[
\mathbf{p},\mathbf{q}
\right]
$$

其中：

- $\mathbf{T}$ 表示位姿；
- $\mathbf{p}\in\mathbb{R}^3$ 表示位置；
- $\mathbf{q}$ 表示单位四元数形式的姿态。

对于位姿，不能简单做：

$$
\mathbf{q}-\bar{\mathbf{q}}
$$

因为四元数不是普通向量。通常使用局部扰动：

$$
\delta\boldsymbol{\theta}
=
2\,\text{vec}
\left(
\bar{\mathbf{q}}^{-1}\otimes\mathbf{q}
\right)
$$

其中：

- $\delta\boldsymbol{\theta}\in\mathbb{R}^3$ 表示姿态误差的小角度近似；
- $\bar{\mathbf{q}}$ 表示 first estimate 姿态；
- $\mathbf{q}$ 表示当前姿态；
- $\bar{\mathbf{q}}^{-1}$ 表示 first estimate 姿态的逆；
- $\text{vec}(\cdot)$ 表示取四元数虚部。

因此位姿相对 first estimate 的局部增量可以写成：

$$
\delta\mathbf{T}
=
\begin{bmatrix}
\mathbf{p}-\bar{\mathbf{p}} \\
2\,\text{vec}
\left(
\bar{\mathbf{q}}^{-1}\otimes\mathbf{q}
\right)
\end{bmatrix}
$$

其中：

- $\bar{\mathbf{p}}$ 表示 first estimate 位置；
- $\mathbf{p}$ 表示当前位置。

边缘化先验在位姿流形上的形式是：

$$
\mathbf{r}_{\text{prior}}
=
\mathbf{r}^{\text{prior}}_0
+
\mathbf{J}^{\text{prior}}
\delta\mathbf{T}
$$

这也是为什么 FEJ 不只是“缓存一个雅克比”那么简单。它还要求后续残差更新时，状态差分必须始终相对于 first estimate 来计算。

## 7. FEJ、边缘化 prior 和普通重线性化的分工

在滑动窗口优化中，要区分三类情况。

第一类是仍然留在窗口中的原始非线性因子，例如当前视觉重投影因子和当前 IMU 因子。在 VINS-Mono 中，这些因子通常会在每次迭代中按当前估计重线性化：

$$
\mathbf{J}^{(t)}
=
\frac{\partial\mathbf{r}}{\partial\delta\mathbf{x}}
\bigg|_{\mathbf{x}^{(t)}}
$$

其中 $\mathbf{J}^{(t)}$ 表示第 $t$ 次迭代的雅克比。

第二类是已经边缘化形成的历史先验。这个先验来自某次边缘化时的线性化结果，因此后续应固定：

$$
\mathbf{J}_{\text{prior}}
=
\mathbf{J}_{\text{prior}}(\bar{\mathbf{x}})
$$

并保持固定。

第三类是严格 FEJ 策略。它会进一步要求某些仍保留在窗口内的变量，在相关雅克比计算中也使用 first estimate，而不是总在当前估计处重线性化。这样做的目标是让所有相关线性化模型共享一致的不可观子空间。

因此，更准确的分工可以总结为：

$$
\text{原始非线性因子}
\Rightarrow
\text{VINS-Mono 中通常按当前估计重线性化}
$$

$$
\text{边缘化压缩后的先验因子}
\Rightarrow
\text{固定边缘化时的 prior 雅克比和参考点}
$$

$$
\text{严格 FEJ}
\Rightarrow
\text{窗口内相关雅克比也使用 first estimate}
$$

## 8. FEJ 解决的核心问题

FEJ 主要解决两个问题。

第一，数学一致性问题。边缘化先验来自某个线性化点附近的二次近似。固定该点处的 prior 雅克比，是对这个近似本身的尊重。

第二，估计一致性问题。VIO 中存在不可观方向，例如全局平移和绕重力方向的 yaw。如果不同因子使用不一致的线性化点，或者边缘化先验在后续过程中不恰当地改变雅克比，可能会给这些不可观方向引入虚假的约束，使系统变得过度自信。

第二点会在下一篇详细展开。这里先给出一个直觉：如果系统本来无法从相机和 IMU 测量中知道“世界原点到底在哪里”，那么优化问题就不应该通过边缘化先验悄悄制造出“世界原点应该在这里”的信息。

FEJ 的目的不是让系统更准地利用每一个非线性因子，而是让已经边缘化的历史信息不要在后续优化中改变其原本的约束结构。

这个问题也可以用零空间语言表达。设某个线性化残差为：

$$
\mathbf{r}
\approx
\mathbf{r}_0
+
\mathbf{J}\delta\mathbf{x}
$$

如果某个方向 $\mathbf{n}$ 是不可观方向，那么沿着这个方向改变状态不应该改变残差的一阶近似：

$$
\mathbf{J}\mathbf{n}
=
\mathbf{0}
$$

其中：

- $\mathbf{n}$ 表示状态空间中的某个扰动方向；
- $\mathbf{J}\mathbf{n}=\mathbf{0}$ 表示这个方向位于 Jacobian 的零空间；
- 零空间方向对应测量无法约束的自由度。

例如 VIO 中的全局平移方向就是不可观的。把所有位姿和所有特征点同时平移同一个向量，视觉重投影和 IMU 相对运动都不会改变。因此理想线性化下，全局平移方向应该落在所有相关 Jacobian 的零空间里。

问题在于，非线性系统每次线性化都会在不同点计算 Jacobian：

$$
\mathbf{J}(\mathbf{x}^{(1)}),\quad
\mathbf{J}(\mathbf{x}^{(2)}),\quad
\mathbf{J}(\mathbf{x}^{(3)})
$$

这些 Jacobian 的零空间不一定完全一致。尤其边缘化先验固定在旧线性化点，而普通因子又在新线性化点不断变化时，组合起来的总 Hessian 可能会对本该不可观的方向产生非零曲率：

$$
\mathbf{n}^{\top}\mathbf{H}\mathbf{n}
>
0
$$

其中：

- $\mathbf{H}=\mathbf{J}^{\top}\mathbf{J}$ 表示线性化后的 Hessian；
- $\mathbf{n}^{\top}\mathbf{H}\mathbf{n}>0$ 表示优化器认为方向 $\mathbf{n}$ 是有信息的。

这就是“不一致”的本质：系统从数学模型上本不该知道某个方向，但线性化和边缘化过程让 Hessian 看起来知道了。

严格 FEJ 希望通过统一 first estimate，让相关 Jacobian 使用同一个参考点，从而让不可观方向在所有线性化因子中尽量保持同一个零空间：

$$
\mathbf{J}_{prior}(\bar{\mathbf{x}})\mathbf{n}
\approx
\mathbf{0},
\quad
\mathbf{J}_{vision}(\bar{\mathbf{x}})\mathbf{n}
\approx
\mathbf{0},
\quad
\mathbf{J}_{imu}(\bar{\mathbf{x}})\mathbf{n}
\approx
\mathbf{0}
$$

这样组合后的系统就不容易凭空制造出虚假的可观信息。

## 9. FEJ 的代价

FEJ 也有代价。

因为雅克比固定在 first estimate，如果当前状态离 first estimate 很远，那么这个线性近似会变差。这意味着 FEJ 在一致性和非线性精度之间做了取舍。

因此，滑动窗口 VIO 通常依赖两个条件：

- 窗口足够短，使边缘化先验不会长期在很差的线性化点附近使用；
- 初始估计足够好，使 first estimate 不至于偏离真实值太远。

换句话说，FEJ 不是万能修复器。它是为了在边缘化不可避免的情况下，尽量保持先验信息的结构一致。

下一篇将回答：什么叫“不可观方向”？为什么 VIO 中天然存在不可观方向？为什么错误的边缘化先验会破坏这些不可观方向？
