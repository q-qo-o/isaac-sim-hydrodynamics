# Hydrodynamics Extension for Isaac Sim

This extension provides high-fidelity hydrodynamic simulation capabilities for Rigid Bodies within NVIDIA Isaac Sim. It implements buoyancy, linear and quadratic damping, and added mass effects based on standard marine craft equations of motion.

## Features

- **Buoyancy Simulation**: real-time buoyancy calculation based on the submerged volume of the collision mesh (Archimedes' principle).
- **6-DOF Damping**: Supports both Linear and Quadratic damping coefficients for all 6 degrees of freedom (Forward, Lateral, Vertical, Roll, Pitch, Yaw).
- **Added Mass**: Simulation of added mass effects affecting acceleration.
- **Dynamic Sizing**: Automatic calculation of characteristic length based on the object's AABB or volume if not specified.
- **Custom UI**: Integrated property widget for easy tuning of hydrodynamic coefficients.

## Installation

1. Clone or copy this extension into your Isaac Sim extensions directory (`exts` or user extension path).
2. Enable the extension in the **Extension Manager** by searching for "hydrodynamics".

## Usage

1. **Select a Target Prim**: Click on any Rigid Body prim in the Stage (must have `PhysicsRigidBodyAPI` and `CollisionAPI` is recommended for accurate buoyancy).
2. **Apply Hydrodynamics**:
   - Right-click on the prim in the Viewport or Stage tree.
   - Select **Hydrodynamics** from the context menu.
   - *Alternatively*: Use the menu `Window -> Hydrodynamics` (if available) or the "Add > Hydrodynamics" button in the Property window if configured.
3. **Configure Parameters**:
   - Navigate to the **Hydrodynamics** section in the **Property** window.
   - Adjust fluid properties and hydrodynamic coefficients.

## Parameters

### Fluid Properties
- **Water Level**: Z-height of the water surface (in world coordinates).
- **Fluid Density**: Density of the fluid (default: 1000 kg/m³).
- **Gravity**: Gravitational acceleration (default: 9.81 m/s²).
- **Characteristic Length**: Reference length used for dimensionless coefficient calculations (optional). If set to 0, it is computed dynamically from the object's geometry.

### Hydrodynamic Coefficients
The extension uses the standard update equation: $F_{hydro} = -( D_{linear} V + D_{quadratic} |V|V + M_{added} \dot{V} )$

Where $V$ is the velocity vector $[u, v, w, p, q, r]^T$.

#### Linear Damping
- **Forward**: Linear damping in the Forward ($X$) direction ($X_u$).
- **Lateral**: Linear damping in the Lateral ($Y$) direction ($Y_v$).
- **Vertical**: Linear damping in the Vertical ($Z$) direction ($Z_w$).
- **Roll/Pitch/Yaw**: Linear damping for rotational axes ($K_p, M_q, N_r$).

#### Quadratic Damping
- **Forward**: Quadratic damping in the Forward direction ($X_{uu}$).
- **Lateral**: Quadratic damping in the Lateral direction ($Y_{vv}$).
- **Vertical**: Quadratic damping in the Vertical direction ($Z_{ww}$).
- **Roll/Pitch/Yaw**: Quadratic damping for rotational axes ($K_{pp}, M_{qq}, N_{rr}$).

#### Added Mass
- **Forward**: Added mass in the Forward direction ($X_{\dot{u}}$).
- **Lateral**: Added mass in the Lateral direction ($Y_{\dot{v}}$).
- **Vertical**: Added mass in the Vertical direction ($Z_{\dot{w}}$).
- **Roll/Pitch/Yaw**: Added rotational inertia ($K_{\dot{p}}, M_{\dot{q}}, N_{\dot{r}}$).

## Theory

The simulation operates in the Body Frame for hydrodynamic forces and transforms them to the World Frame for application via the PhysX interface.

1. **Velocity Transformation**: World velocity is transformed to Body frame.
2. **Force Calculation**: Damping and Added Mass forces are computed in the Body frame.
3. **Application**: Forces are transformed back to World frame and applied using `physx.apply_force_at_pos`.
4. **Buoyancy**: Computed in World frame by clipping the mesh triangles against the water plane.

## License

[MIT License](LICENSE)
