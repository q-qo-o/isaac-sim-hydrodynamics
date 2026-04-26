# 气动力学/水动力学仿真方案与动力学模型

## 1. 概述
本项目针对Isaac Sim扩展开发了基于物理的流体动力学（Hydrodynamics/Aerodynamics）仿真模块。主要负责在刚体（RigidBody）六自由度运动中计算并施加由于周围流体产生的受力与力矩。

当前的方案主要包括两大核心模块：
1. **浮力模型（Buoyancy Model）**：通过动态计算网格在水面以下的浸没体积及其形心来施加浮力。
2. **水动力/气动力阻力模型（Hydrodynamic Resistance Model）**：在刚体局部坐标系（Body Frame）下，基于经典船舶与水下航行器操纵性方程，利用经验系数（附加质量、线性阻尼、二次阻尼）计算流体对刚体的反作用力与力矩。

---

## 2. 浮力模型 (Buoyancy Model)

### 2.1 浸没体积计算
浮力计算基于网格模型的三维几何信息。仿真在每帧物理更新（`on_update`）中：
1. 提取所有关联Mesh的局部顶点，并转换至世界坐标系。
2. 与设定的水面高度（Water Level, $z = h$）进行求交裁剪（Triangle Clipping）。
3. 对所有完全或部分浸入水下的三角形，通过计算水下多边形对应的体积积分（四面体体积累加），求得总浸没体积 $V_{sub}$，以及加权体积形心位置 $C_{sub}$。

### 2.2 浮力方程
依据阿基米德原理，浮力大小等于排开流体的重力：
$$ F_B = \begin{bmatrix} 0 \\ 0 \\ \rho \cdot g \cdot V_{sub} \end{bmatrix} $$
其中：
- $F_B$：浮力向量，方向竖直向上（世界坐标系 $+Z$ 方向）。
- $\rho$：流体密度（Fluid Density, 默认 $1000\ kg/m^3$）。
- $g$：重力加速度（Gravity, 默认 $9.81\ m/s^2$）。
- $V_{sub}$：当前水面以下的实际浸没体积。

该力通过 `apply_force_at_pos` 接口直接施加于所计算出的形心 $C_{sub}$ 处（Center of Buoyancy）。

---

## 3. 水动力学/气动力学阻力模型 (Hydrodynamic Resistance Model)

此部分作用力模型基于6自由度操纵性方程进行了解耦简化，重点包含了线性阻尼、二次阻尼以及附加质量力。所有动力学参数均沿刚体局部坐标系（Body Frame）进行解算，最终转换到世界坐标系并作用在刚体质心处。

设刚体局部坐标系下：
- 线速度向量：$v = [u, v, w]^T$ (分别对应 Forward, Lateral, Vertical)
- 角速度向量：$\omega = [p, q, r]^T$ (分别对应 Roll, Pitch, Yaw)
- 线加速度向量：$\dot{v} = [\dot{u}, \dot{v}, \dot{w}]^T$ (采用一阶有限差分求得)
- 角加速度向量：$\dot{\omega} = [\dot{p}, \dot{q}, \dot{r}]^T$ (采用一阶有限差分求得)

### 3.1 线性阻尼力 (Linear Damping)
描述低速运动下的粘性阻力，与速度成正比：
$$ F_{lin} = - \begin{bmatrix} X_u \\ Y_v \\ Z_w \end{bmatrix} \circ \begin{bmatrix} u \\ v \\ w \end{bmatrix} $$
$$ T_{lin} = - \begin{bmatrix} K_p \\ M_q \\ N_r \end{bmatrix} \circ \begin{bmatrix} p \\ q \\ r \end{bmatrix} $$
*注：$\circ$ 表示哈达玛积（Hadamard product），即逐元素相乘。*

### 3.2 二次阻尼力 (Quadratic Damping)
描述由涡流分离和压差造成的形状阻力，与速度的平方成正比（保留速度符号以确定受力方向）：
$$ F_{quad} = - \begin{bmatrix} X_{u|u|} \\ Y_{v|v|} \\ Z_{w|w|} \end{bmatrix} \circ \begin{bmatrix} |u|u \\ |v|v \\ |w|w \end{bmatrix} $$
$$ T_{quad} = - \begin{bmatrix} K_{p|p|} \\ M_{q|q|} \\ N_{r|r|} \end{bmatrix} \circ \begin{bmatrix} |p|p \\ |q|q \\ |r|r \end{bmatrix} $$

### 3.3 附加质量力 (Added Mass Force)
描述刚体加速运动时，带动周围流体一起加速所产生的反作用力。模型采用解耦的对角附加质量矩阵：
$$ F_{am} = - \begin{bmatrix} X_{\dot{u}} \\ Y_{\dot{v}} \\ Z_{\dot{w}} \end{bmatrix} \circ \begin{bmatrix} \dot{u} \\ \dot{v} \\ \dot{w} \end{bmatrix} $$
$$ T_{am} = - \begin{bmatrix} K_{\dot{p}} \\ M_{\dot{q}} \\ N_{\dot{r}} \end{bmatrix} \circ \begin{bmatrix} \dot{p} \\ \dot{q} \\ \dot{r} \end{bmatrix} $$

### 3.4 总体受力计算与施加
在局部坐标系（Body Frame）下的总流体受力与力矩分别为各项之和：
$$ F_{body} = F_{lin} + F_{quad} + F_{am} $$
$$ T_{body} = T_{lin} + T_{quad} + T_{am} $$

计算完毕后，利用世界旋转矩阵 $R_{world}$ 将其转换至世界坐标系：
$$ F_{world} = R_{world} \cdot F_{body} $$
$$ T_{world} = R_{world} \cdot T_{body} $$
最后，将 $F_{world}$ 与 $T_{world}$ 分别作为力与力矩施加在刚体原点（通常为质心/COM）上。

---

## 4. 运动学与状态更新策略 (Kinematics & State Tracking)
为支持上述计算，仿真引擎每帧执行以下运动学更新：
1. **坐标系变换**：每一物理步中，通过 PhysX 接口读取刚体在世界系下的线速度与角速度，并乘以 $R_{world}^T$ (即由World至Body的旋转矩阵) 变换至刚体局部系。
2. **加速度数值差分**：局部坐标系下的线加速度与角加速度无法直接从引擎获取，故采用前向有限差分法，由前后两帧局部系速度之差除以时间步长 $\Delta t$ 得到，即 $a = (v_t - v_{t-\Delta t}) / \Delta t$。
3. **特征尺寸自适应 (Characteristic Length)**：当用户未手动指定特征长度（$\leq 0$）时，系统支持通过当前包围盒（BBoxLocal）或排开体积等效边长动态估算，这为进一步引入雷诺数等无量纲流体力学扩展预留了空间。