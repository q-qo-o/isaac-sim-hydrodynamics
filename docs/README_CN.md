# Isaac Sim Hydrodynamics 扩展

本扩展为 NVIDIA Isaac Sim 中的刚体提供高保真的水动力学模拟功能。它实现了基于阿基米德原理的浮力系统、线性和二次阻尼，以及影响加速度的附加质量效应。

## 特性 (Features)

- **浮力模拟 (Buoyancy)**：基于碰撞网格浸没体积的实时浮力计算。
- **6自由度阻尼 (6-DOF Damping)**：支持所有6个自由度（前向、横向、垂向、滚转、俯仰、偏航）的线性和二次阻尼系数。
- **附加质量 (Added Mass)**：模拟由于流体跟随刚体运动而产生的附加惯性效应。
- **动态几何尺寸 (Dynamic Sizing)**：如果不指定参考长度，系统将根据物体的 AABB 包围盒或体积自动计算特征长度。
- **自定义 UI (Custom UI)**：集成了属性面板小部件，便于调整水动力学系数。

## 安装指南 (Installation)

1. 将此扩展克隆或复制到您的 Isaac Sim 扩展目录（`exts` 或用户扩展路径）中。
2. 启动 Isaac Sim，在 **Extension Manager** 中搜索并启用 "hydrodynamics"。

## 使用说明 (Usage)

1. **选择目标刚体**：在场景（Stage）中选中任意刚体（Primitive）。确保该刚体具有 `PhysicsRigidBodyAPI`，并在属性面板中可见。
2. **应用水动力学**：
   - 在视口（Viewport）或场景树（Stage）中右键点击该刚体。
   - 从上下文菜单中选择 **Hydrodynamics**。
   - *或者*：使用菜单栏 `Window -> Hydrodynamics`（如果已配置），或在属性面板的 "Add > Hydrodynamics" 按钮（如果有）。
3. **配置参数**：
   - 在 **Property** 面板中找到新增的 **Hydrodynamics** 分组。
   - 调整流体属性（Fluid Properties）和水动力系数。

## 参数说明 (Parameters)

### 流体属性 (Fluid Properties)
- **Water Level**：水面的 Z 轴高度（世界坐标系）。
- **Fluid Density**：流体密度（默认：1000 kg/m³）。
- **Gravity**：重力加速度（默认：9.81 m/s²）。
- **Characteristic Length**：无量纲系数计算使用的参考长度。设为 0 时根据几何体动态计算。

### 水动力系数 (Hydrodynamic Coefficients)
本扩展使用以下标准的运动方程更新力：
$$ F_{hydro} = -( D_{linear} V + D_{quadratic} |V|V + M_{added} \dot{V} ) $$

其中 $V$ 是速度向量 $[u, v, w, p, q, r]^T$。

#### 线性阻尼 (Linear Damping)
- **Forward**：前向 ($X$方向) 的线性阻尼系数 ($X_u$)。
- **Lateral**：横向 ($Y$方向) 的线性阻尼系数 ($Y_v$)。
- **Vertical**：垂向 ($Z$方向) 的线性阻尼系数 ($Z_w$)。
- **Roll/Pitch/Yaw**：旋转自由度的线性阻尼系数 ($K_p, M_q, N_r$)。

#### 二次阻尼 (Quadratic Damping)
- **Forward**：前向 ($X$方向) 的二次阻尼系数 ($X_{uu}$).
- **Lateral**：横向 ($Y$方向) 的二次阻尼系数 ($Y_{vv}$).
- **Vertical**：垂向 ($Z$方向) 的二次阻尼系数 ($Z_{ww}$).
- **Roll/Pitch/Yaw**：旋转自由度的二次阻尼系数 ($K_{pp}, M_{qq}, N_{rr}$).

#### 附加质量 (Added Mass)
- **Forward**：前向 ($X$方向) 的附加质量 ($X_{\dot{u}}$).
- **Lateral**：横向 ($Y$方向) 的附加质量 ($Y_{\dot{v}}$).
- **Vertical**：垂向 ($Z$方向) 的附加质量 ($Z_{\dot{w}}$).
- **Roll/Pitch/Yaw**：附加转动惯量 ($K_{\dot{p}}, M_{\dot{q}}, N_{\dot{r}}$).

## 理论说明 (Theory)

该模拟在**体坐标系 (Body Frame)** 下计算水动力，并通过 PhysX 接口将其施加到**世界坐标系 (World Frame)**。

1. **速度变换**：将世界坐标系的刚体速度变换至体坐标系。
2. **力计算**：在体坐标系下计算阻尼力和附加质量力。
3. **施加力**：将计算得到的力变换回世界坐标系，并使用 `physx.apply_force_at_pos` 施加。
4. **浮力**：通过在世界坐标系下裁剪网格三角形与水平面的相交部分来计算。

## 许可证 (License)

[MIT License](LICENSE)
