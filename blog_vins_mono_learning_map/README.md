# VINS-Mono 从零到因子图、预积分、边缘化与可观性

这是一组面向视觉惯性里程计学习者的递进式教程。它参考了《VINS 论文推导及代码解析 崔华坤.pdf》的推导脉络，但不直接照搬原文，而是按“从零建立概念、再推公式、最后串系统”的方式重新组织。

整套教程从 VINS-Mono 总体框架开始，逐步讲清楚因子图、IMU 预积分、后端残差、初始化、前端闭环、边缘化、FEJ-like prior 和可观性分析。

适合读者：

- 熟悉 EKF、VINS、LIO-SAM 或图优化的机器人算法工程师；
- 想从数学结构理解 VINS-Mono，而不是只停留在“代码在哪里加残差”的读者；
- 想把预积分、初始化、滑窗优化、marginalization、FEJ 和 observability preservation 串成一条完整逻辑链的读者。

推荐阅读顺序：

0. [00_overview.md](00_overview.md)：从零建立 VINS-Mono 模块全景。
1. [01_factor_graph.md](01_factor_graph.md)：从状态估计问题出发，理解 VIO 为什么自然形成因子图。
2. [02_vins_vs_gtsam_factor_graph.md](02_vins_vs_gtsam_factor_graph.md)：理解一般因子图形式，以及 VINS 滑窗优化和 GTSAM 因子图为什么数学等价。
3. [03_imu_preintegration.md](03_imu_preintegration.md)：详细理解 IMU 预积分、bias 修正、协方差和 IMU 残差。
4. [04_backend_residuals_and_jacobians.md](04_backend_residuals_and_jacobians.md)：理解后端状态向量、目标函数、IMU/视觉残差和雅克比。
5. [05_initialization_and_time_sync.md](05_initialization_and_time_sync.md)：理解单目 VIO 初始化、视觉惯性对齐、外参初值和时间偏移建模。
6. [06_frontend_loop_and_system_details.md](06_frontend_loop_and_system_details.md)：理解前端特征、关键帧、闭环检测、重定位和 4DoF pose graph。
7. [07_marginalization.md](07_marginalization.md)：理解滑动窗口为什么必须丢状态，以及如何用 Schur 补保留历史信息。
8. [08_fej.md](08_fej.md)：理解边缘化 prior、严格 FEJ 和 VINS-Mono 实际做法的区别。
9. [09_observability.md](09_observability.md)：理解单目 VIO 的不可观方向，以及 FEJ-like prior 和严格 FEJ 如何帮助一致性。
10. [10_learning_route.md](10_learning_route.md)：把核心内容压缩成一张学习地图，用于复习和建立全局认知。

整组文章的主线是：

$$
\text{多传感器约束}
\rightarrow
\text{因子图优化}
\rightarrow
\text{通用图模型对照}
\rightarrow
\text{IMU 预积分}
\rightarrow
\text{后端残差与雅克比}
\rightarrow
\text{初始化与时间同步}
\rightarrow
\text{前端与闭环}
\rightarrow
\text{滑动窗口实时性}
\rightarrow
\text{边缘化先验}
\rightarrow
\text{FEJ-like prior 与严格 FEJ}
\rightarrow
\text{可观性保持与估计一致性}
$$

阅读时建议带着一个核心问题：

> 一个 VIO 系统如何在实时运行时不断丢弃旧状态，却又不错误地制造出本来不存在的绝对位姿信息？

这个问题的答案，就是从因子图到 FEJ 再到可观性分析的完整学习路径。

PDF 内容对应关系：

- 总体框架：对应 [00_overview.md](00_overview.md)
- IMU 预积分：对应 [03_imu_preintegration.md](03_imu_preintegration.md)
- 后端非线性优化、IMU 约束、视觉约束：对应 [04_backend_residuals_and_jacobians.md](04_backend_residuals_and_jacobians.md)
- 初始化：对应 [05_initialization_and_time_sync.md](05_initialization_and_time_sync.md)
- 边缘化和 FEJ：对应 [07_marginalization.md](07_marginalization.md)、[08_fej.md](08_fej.md)
- 闭环检测和优化、关键帧策略等系统细节：对应 [06_frontend_loop_and_system_details.md](06_frontend_loop_and_system_details.md)
- 不可观性、一致性和 yaw 问题：对应 [09_observability.md](09_observability.md)
