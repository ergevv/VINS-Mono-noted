# VINS-Mono 从因子图到 FEJ 再到可观性分析

这是一组面向视觉惯性里程计学习者的递进式教程。它不依赖具体代码实现，而是从状态估计问题本身出发，逐步解释 VINS-Mono 这类滑动窗口优化系统为什么需要因子图、为什么需要边缘化、为什么边缘化之后会引出 FEJ，以及 FEJ 和可观性保持之间有什么关系。

适合读者：

- 熟悉 EKF、VINS、LIO-SAM 或图优化的机器人算法工程师；
- 想从数学结构理解 VINS-Mono，而不是只停留在“代码在哪里加残差”的读者；
- 想把 marginalization、FEJ、observability preservation 串成一条完整逻辑链的读者。

推荐阅读顺序：

1. [01_factor_graph.md](01_factor_graph.md)：从状态估计问题出发，理解 VIO 为什么自然形成因子图。
2. [02_vins_vs_gtsam_factor_graph.md](02_vins_vs_gtsam_factor_graph.md)：理解一般因子图形式，以及 VINS 滑窗优化和 GTSAM 因子图为什么数学等价。
3. [03_initialization_and_time_sync.md](03_initialization_and_time_sync.md)：理解单目 VIO 初始化、视觉惯性对齐、外参初值和时间偏移建模。
4. [04_marginalization.md](04_marginalization.md)：理解滑动窗口为什么必须丢状态，以及如何用 Schur 补保留历史信息。
5. [05_fej.md](05_fej.md)：理解边缘化先验为什么不能随意重线性化，以及 FEJ 的数学本质。
6. [06_observability.md](06_observability.md)：理解单目 VIO 的不可观方向，以及 FEJ 如何帮助保持一致性。
7. [07_learning_route.md](07_learning_route.md)：把核心内容压缩成一张学习地图，用于复习和建立全局认知。

整组文章的主线是：

$$
\text{多传感器约束}
\rightarrow
\text{因子图优化}
\rightarrow
\text{通用图模型对照}
\rightarrow
\text{初始化与时间同步}
\rightarrow
\text{滑动窗口实时性}
\rightarrow
\text{边缘化先验}
\rightarrow
\text{固定线性化点 FEJ}
\rightarrow
\text{可观性保持与估计一致性}
$$

阅读时建议带着一个核心问题：

> 一个 VIO 系统如何在实时运行时不断丢弃旧状态，却又不错误地制造出本来不存在的绝对位姿信息？

这个问题的答案，就是从因子图到 FEJ 再到可观性分析的完整学习路径。
