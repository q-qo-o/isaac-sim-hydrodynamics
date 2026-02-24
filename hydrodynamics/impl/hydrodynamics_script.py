import carb
import numpy as np
from pxr import UsdGeom, Gf, Usd, PhysxSchema, Sdf, PhysicsSchemaTools, UsdPhysics
from omni.kit.scripting import BehaviorScript
import omni.physx
import omni.usd
from omni.physx import get_physx_simulation_interface
import omni.kit.window.property

class HydrodynamicsScript(BehaviorScript):
    BEHAVIOR_NS = "hydrodynamics"
    EXPOSED_ATTR_NS = "exposedVar"

    VARIABLES_TO_EXPOSE = [
        # Environmental Parameters
        {
            "attr_name": "waterLevel",
            "attr_type": Sdf.ValueTypeNames.Float,
            "default_value": 0.0,
            "doc": "Water Level (m)",
            "group": "Fluid Properties"
        },
        {
            "attr_name": "fluidDensity",
            "attr_type": Sdf.ValueTypeNames.Float,
            "default_value": 1000.0,
            "doc": "Fluid Density (kg/m^3)",
            "group": "Fluid Properties"
        },
        {
            "attr_name": "gravity",
            "attr_type": Sdf.ValueTypeNames.Float,
            "default_value": 9.81,
            "doc": "Gravity (m/s^2)",
            "group": "Fluid Properties"
        },
        # Characteristic Dimensions
        {
            "attr_name": "characteristicLength",
            "attr_type": Sdf.ValueTypeNames.Float,
            "default_value": 0.0,
            "doc": "Reference length (m). If <= 0, computed dynamically from AABB.",
            "group": "Fluid Properties"
        },
        # Linear Damping Coefficients
        {"attr_name": "linearDampingForward", "attr_type": Sdf.ValueTypeNames.Float, "default_value": 0.0, "doc": "Xu (N/(m/s)) - Forward", "group": "Linear Damping Derivatives"},
        {"attr_name": "linearDampingLateral", "attr_type": Sdf.ValueTypeNames.Float, "default_value": 0.0, "doc": "Yv (N/(m/s)) - Lateral", "group": "Linear Damping Derivatives"},
        {"attr_name": "linearDampingVertical", "attr_type": Sdf.ValueTypeNames.Float, "default_value": 0.0, "doc": "Zw (N/(m/s)) - Vertical", "group": "Linear Damping Derivatives"},
        {"attr_name": "linearDampingRoll", "attr_type": Sdf.ValueTypeNames.Float, "default_value": 0.0, "doc": "Kp (Nm/(rad/s))", "group": "Linear Damping Derivatives"},
        {"attr_name": "linearDampingPitch", "attr_type": Sdf.ValueTypeNames.Float, "default_value": 0.0, "doc": "Mq (Nm/(rad/s))", "group": "Linear Damping Derivatives"},
        {"attr_name": "linearDampingYaw", "attr_type": Sdf.ValueTypeNames.Float, "default_value": 0.0, "doc": "Nr (Nm/(rad/s))", "group": "Linear Damping Derivatives"},
        # Quadratic Damping Coefficients
        {"attr_name": "quadraticDampingForward", "attr_type": Sdf.ValueTypeNames.Float, "default_value": 0.0, "doc": "Xuu (N/(m/s)^2) - Forward", "group": "Quadratic Damping Derivatives"},
        {"attr_name": "quadraticDampingLateral", "attr_type": Sdf.ValueTypeNames.Float, "default_value": 0.0, "doc": "Yvv (N/(m/s)^2) - Lateral", "group": "Quadratic Damping Derivatives"},
        {"attr_name": "quadraticDampingVertical", "attr_type": Sdf.ValueTypeNames.Float, "default_value": 0.0, "doc": "Zww (N/(m/s)^2) - Vertical", "group": "Quadratic Damping Derivatives"},
        {"attr_name": "quadraticDampingRoll", "attr_type": Sdf.ValueTypeNames.Float, "default_value": 0.0, "doc": "Kpp (Nm/(rad/s)^2)", "group": "Quadratic Damping Derivatives"},
        {"attr_name": "quadraticDampingPitch", "attr_type": Sdf.ValueTypeNames.Float, "default_value": 0.0, "doc": "Mqq (Nm/(rad/s)^2)", "group": "Quadratic Damping Derivatives"},
        {"attr_name": "quadraticDampingYaw", "attr_type": Sdf.ValueTypeNames.Float, "default_value": 0.0, "doc": "Nrr (Nm/(rad/s)^2)", "group": "Quadratic Damping Derivatives"},
        # Added Mass Coefficients
        {"attr_name": "addedMassForward", "attr_type": Sdf.ValueTypeNames.Float, "default_value": 0.0, "doc": "Xudot (kg) - Forward", "group": "Added Mass Derivatives"},
        {"attr_name": "addedMassLateral", "attr_type": Sdf.ValueTypeNames.Float, "default_value": 0.0, "doc": "Yvdot (kg) - Lateral", "group": "Added Mass Derivatives"},
        {"attr_name": "addedMassVertical", "attr_type": Sdf.ValueTypeNames.Float, "default_value": 0.0, "doc": "Zwdot (kg) - Vertical", "group": "Added Mass Derivatives"},
        {"attr_name": "addedMassRoll", "attr_type": Sdf.ValueTypeNames.Float, "default_value": 0.0, "doc": "Kpdot (kg*m^2/rad)", "group": "Added Mass Derivatives"},
        {"attr_name": "addedMassPitch", "attr_type": Sdf.ValueTypeNames.Float, "default_value": 0.0, "doc": "Mqdot (kg*m^2/rad)", "group": "Added Mass Derivatives"},
        {"attr_name": "addedMassYaw", "attr_type": Sdf.ValueTypeNames.Float, "default_value": 0.0, "doc": "Nrdot (kg*m^2/rad)", "group": "Added Mass Derivatives"},
    ]

    def on_init(self):
        # 1. Default settings
        self._water_level = 0.0
        self._fluid_density = 1000.0
        self._gravity = 9.81
        self._characteristic_length = 0.0
        
        # Coefficients
        self._lin_damping = np.zeros(6)
        self._quad_damping = np.zeros(6)
        self._added_mass = np.zeros(6)
        
        self._meshes = []
        self._rb_path = None
        
        # State tracking for acceleration
        self._prev_linear_vel = np.zeros(3)
        self._prev_angular_vel = np.zeros(3)
        self._initialized_state = False

        carb.log_info(f"{type(self).__name__}.on_init()->{self.prim_path}")
        
        # Expose variables
        for var in self.VARIABLES_TO_EXPOSE:
            attr_name = var["attr_name"]
            full_attr_name = f"{self.EXPOSED_ATTR_NS}:{self.BEHAVIOR_NS}:{attr_name}"
            attr = self.prim.GetAttribute(full_attr_name)
            if not attr:
                attr = self.prim.CreateAttribute(full_attr_name, var["attr_type"])
                attr.Set(var["default_value"])
                if var.get("doc"):
                    attr.SetDocumentation(var["doc"])
                if var.get("group"):
                    attr.SetDisplayGroup(var["group"])
                    
        omni.kit.window.property.get_window().request_rebuild()

    def on_destroy(self):
        carb.log_info(f"{type(self).__name__}.on_destroy()->{self.prim_path}")
        
        # Remove exposed variables
        for var in self.VARIABLES_TO_EXPOSE:
            attr_name = var["attr_name"]
            full_attr_name = f"{self.EXPOSED_ATTR_NS}:{self.BEHAVIOR_NS}:{attr_name}"
            attr = self.prim.GetAttribute(full_attr_name)
            if attr:
                self.prim.RemoveProperty(attr.GetName())
                
        omni.kit.window.property.get_window().request_rebuild()

    def _get_exposed_variable(self, attr_name):
        full_attr_name = f"{self.EXPOSED_ATTR_NS}:{self.BEHAVIOR_NS}:{attr_name}"
        attr = self.prim.GetAttribute(full_attr_name)
        if attr:
            return attr.Get()
        return None

    def _update_params(self):
        # Update Environment
        wl = self._get_exposed_variable("waterLevel")
        if wl is not None: self._water_level = wl
        
        fd = self._get_exposed_variable("fluidDensity")
        if fd is not None: self._fluid_density = fd
        
        g = self._get_exposed_variable("gravity")
        if g is not None: self._gravity = g
        
        cl = self._get_exposed_variable("characteristicLength")
        if cl is not None: self._characteristic_length = cl

        # Update Coefficients
        
        # Linear Damping
        lin_names = ["linearDampingForward", "linearDampingLateral", "linearDampingVertical", 
                     "linearDampingRoll", "linearDampingPitch", "linearDampingYaw"]
        for i, name in enumerate(lin_names):
            val = self._get_exposed_variable(name)
            if val is not None: self._lin_damping[i] = val

        # Quadratic Damping
        quad_names = ["quadraticDampingForward", "quadraticDampingLateral", "quadraticDampingVertical",
                      "quadraticDampingRoll", "quadraticDampingPitch", "quadraticDampingYaw"]
        for i, name in enumerate(quad_names):
            val = self._get_exposed_variable(name)
            if val is not None: self._quad_damping[i] = val

        # Added Mass
        am_names = ["addedMassForward", "addedMassLateral", "addedMassVertical",
                    "addedMassRoll", "addedMassPitch", "addedMassYaw"]
        for i, name in enumerate(am_names):
            val = self._get_exposed_variable(name)
            if val is not None: self._added_mass[i] = val

    def on_play(self):
        stage = omni.usd.get_context().get_stage()
        self._prim = stage.GetPrimAtPath(self.prim_path)
        self._update_params()
        
        # Initialize state tracking
        self._prev_linear_vel = np.zeros(3)
        self._prev_angular_vel = np.zeros(3)
        self._initialized_state = False
        
        # Find RigidBody API
        curr = self._prim
        self._rb_path = None
        while curr and curr.GetPath() != Sdf.Path("/"):
            if curr.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
                self._rb_path = str(curr.GetPath())
                break
            curr = curr.GetParent()
        
        if not self._rb_path and self._prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
            self._rb_path = str(self.prim_path)
            
        # Collect Meshes
        self._meshes = []
        def collect_meshes(p):
            if p.IsA(UsdGeom.Mesh):
                mesh = UsdGeom.Mesh(p)
                pts = mesh.GetPointsAttr().Get()
                idxs = mesh.GetFaceVertexIndicesAttr().Get()
                cnts = mesh.GetFaceVertexCountsAttr().Get()
                if pts and idxs and cnts:
                    self._meshes.append({
                        "prim": p,
                        "local_points": np.array(pts, dtype=np.float32),
                        "indices": np.array(idxs, dtype=np.int32),
                        "counts": np.array(cnts, dtype=np.int32)
                    })
            for child in p.GetChildren():
                collect_meshes(child)
        
        collect_meshes(self._prim)
        carb.log_info(f"Hydrodynamics: Found {len(self._meshes)} meshes. RigidBody path: {self._rb_path}")

    def on_pause(self):
        pass

    def on_stop(self):
        pass

    def on_update(self, current_time: float, delta_time: float):
        if not self._rb_path or not self._meshes:
            return

        self._update_params()
        
        stage = omni.usd.get_context().get_stage()
        rb_prim = stage.GetPrimAtPath(self._rb_path)
        rb_api = UsdPhysics.RigidBodyAPI(rb_prim)
        
        # World Velocity
        lin_vel_world = np.array(rb_api.GetVelocityAttr().Get() or [0,0,0])
        ang_vel_world = np.array(rb_api.GetAngularVelocityAttr().Get() or [0,0,0])
        
        # Transform World Velocity to Body Local Frame
        world_transform = omni.usd.get_world_transform_matrix(rb_prim)
        m_np = np.array(world_transform).reshape(4, 4)
        rot_mat = m_np[:3, :3] # World to Body rotation (actually Body to World R)
        
        # v_body = R^T * v_world
        lin_vel_body = rot_mat.T @ lin_vel_world
        ang_vel_body = rot_mat.T @ ang_vel_world
        
        # Calculate Acceleration (Finite Difference)
        if not self._initialized_state:
            self._prev_linear_vel = lin_vel_body
            self._prev_angular_vel = ang_vel_body
            self._initialized_state = True
            lin_acc_body = np.zeros(3)
            ang_acc_body = np.zeros(3)
        else:
            if delta_time > 1e-6:
                lin_acc_body = (lin_vel_body - self._prev_linear_vel) / delta_time
                ang_acc_body = (ang_vel_body - self._prev_angular_vel) / delta_time
            else:
                lin_acc_body = np.zeros(3)
                ang_acc_body = np.zeros(3)
            
            self._prev_linear_vel = lin_vel_body
            self._prev_angular_vel = ang_vel_body

        # 2. Buoyancy Calculation (Existing Logic)
        total_volume = 0.0
        weighted_centroid = np.array([0.0, 0.0, 0.0])
        
        for mesh_data in self._meshes:
            mesh_prim = mesh_data["prim"]
            m_mesh_world = np.array(omni.usd.get_world_transform_matrix(mesh_prim)).reshape(4, 4)
            points_world = mesh_data["local_points"] @ m_mesh_world[:3, :3] + m_mesh_world[3, :3]
            
            indices = mesh_data["indices"]
            counts = mesh_data["counts"]
            
            if np.min(points_world[:, 2]) >= self._water_level:
                continue
            
            offset = 0
            for count in counts:
                for j in range(1, count - 1):
                    v0 = points_world[indices[offset]]
                    v1 = points_world[indices[offset + j]]
                    v2 = points_world[indices[offset + j + 1]]
                    
                    clipped_tris = self._clip_triangle_to_water(v0, v1, v2, self._water_level)
                    for t in clipped_tris:
                        r0 = t[0] - np.array([0, 0, self._water_level])
                        r1 = t[1] - np.array([0, 0, self._water_level])
                        r2 = t[2] - np.array([0, 0, self._water_level])
                        dv = np.dot(r0, np.cross(r1, r2)) / 6.0
                        dc = (np.array([0, 0, self._water_level]) + t[0] + t[1] + t[2]) / 4.0
                        total_volume += dv
                        weighted_centroid += dc * dv
                offset += count
        
        # 3. Apply Forces
        stage_id = omni.usd.get_context().get_stage_id()
        body_path_int = int(PhysicsSchemaTools.sdfPathToInt(Sdf.Path(self._rb_path)))
        physx = get_physx_simulation_interface()

        # Buoyancy Force (World Frame)
        if total_volume > 1e-9:
            centroid = weighted_centroid / total_volume
            buoyancy_force = np.array([0.0, 0.0, total_volume * self._fluid_density * self._gravity])
            
            physx.apply_force_at_pos(
                stage_id,
                body_path_int,
                carb.Float3(float(buoyancy_force[0]), float(buoyancy_force[1]), float(buoyancy_force[2])),
                carb.Float3(float(centroid[0]), float(centroid[1]), float(centroid[2])),
                "Force"
            )

        # 4. Hydrodynamic Resistance (Body Frame)
        # Determine Characteristic Length
        L = self._characteristic_length
        if L <= 0:
            # Dynamically compute length based on current geometry state
            # Using UsdGeom.Imageable(rb_prim).ComputeLocalBound ... 
            # This accounts for any time-varying changes if the mesh deforms or if we want a robust live value.
            bbox_cache = UsdGeom.BBoxCache(current_time, [UsdGeom.Tokens.default_])
            # Compute World Bound to account for scale, but we want a "Body Local" size metric for standard coefficients.
            # Usually coefficients are normalized by a fixed length (Lpp).
            # If we want "real-time" length based on submerged volume, that is D = cbrt(Vol).
            # If user wants bounding box extent:
            
            # Using Local Extent scaled by transform scale
            boundable = UsdGeom.Boundable(rb_prim)
            if boundable:
                # Get Local Range
                local_range = bbox_cache.ComputeLocalBound(rb_prim).GetRange()
                size = np.array(local_range.GetSize())
                
                # If we want the max dimension
                L = max(size)
            
            # If still invalid, or if we want volume-based dynamic length for submerged handling:
            if L <= 0:
                 if total_volume > 0:
                     L = np.cbrt(total_volume) # Fallback to submerged volume dimension
                 else:
                     L = 1.0

        # Forces Calculation (F = - (Linear*V + Quad*|V|V + AddedMass*Acc))
        # Note: We apply these forces in BODY frame, then rotate to WORLD.
        
        # Linear Damping
        F_lin = - self._lin_damping[:3] * lin_vel_body
        T_lin = - self._lin_damping[3:] * ang_vel_body
        
        # Quadratic Damping
        F_quad = - self._quad_damping[:3] * np.abs(lin_vel_body) * lin_vel_body
        T_quad = - self._quad_damping[3:] * np.abs(ang_vel_body) * ang_vel_body
        
        # Added Mass (F = - Ma * a)
        F_am = - self._added_mass[:3] * lin_acc_body
        T_am = - self._added_mass[3:] * ang_acc_body
        
        # Total Body Force/Torque
        total_force_body = F_lin + F_quad + F_am
        total_torque_body = T_lin + T_quad + T_am
        
        # Transform Force to World Frame
        total_force_world = rot_mat @ total_force_body
        total_torque_world = rot_mat @ total_torque_body
        
        # Apply Hydrodynamic Forces (at Body Origin/COM)
        body_pos_world = m_np[3, :3]
        
        # Apply Force
        carb_force = carb.Float3(float(total_force_world[0]), float(total_force_world[1]), float(total_force_world[2]))
        carb_pos = carb.Float3(float(body_pos_world[0]), float(body_pos_world[1]), float(body_pos_world[2]))
        
        if np.linalg.norm(total_force_world) > 1e-6:
            physx.apply_force_at_pos(stage_id, body_path_int, carb_force, carb_pos, "Force")
        
        # Apply Torque
        carb_torque = carb.Float3(float(total_torque_world[0]), float(total_torque_world[1]), float(total_torque_world[2]))
        
        if np.linalg.norm(total_torque_world) > 1e-6:
             physx.apply_torque(stage_id, body_path_int, carb_torque)

    def _clip_triangle_to_water(self, v0, v1, v2, h):
        """Clip triangle to z < h, returning a list of clipped triangles."""
        pts = [v0, v1, v2]
        below = [v[2] < h for v in pts]
        count = sum(below)
        
        if count == 3: return [pts]
        if count == 0: return []
            
        def intersect(a, b):
            t = (h - a[2]) / (b[2] - a[2])
            return a + t * (b - a)

        if count == 1:
            i = below.index(True)
            p0, p1, p2 = pts[i], pts[(i+1)%3], pts[(i+2)%3]
            return [[p0, intersect(p0, p1), intersect(p0, p2)]]
        
        if count == 2:
            i = below.index(False)
            p_above, p1, p2 = pts[i], pts[(i+1)%3], pts[(i+2)%3]
            i1a = intersect(p1, p_above)
            i2a = intersect(p2, p_above)
            return [[p1, p2, i2a], [p1, i2a, i1a]]
            
        return []
