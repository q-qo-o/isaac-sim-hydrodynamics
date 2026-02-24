import carb
import numpy as np
from pxr import UsdGeom, Gf, Usd, PhysxSchema, Sdf, PhysicsSchemaTools
from omni.kit.scripting import BehaviorScript
import omni.physx
import omni.usd
from omni.physx import get_physx_simulation_interface
import omni.kit.window.property

class HydrodynamicsScript(BehaviorScript):
    BEHAVIOR_NS = "hydrodynamics"
    EXPOSED_ATTR_NS = "exposedVar"

    VARIABLES_TO_EXPOSE = [
        {
            "attr_name": "waterLevel",
            "attr_type": Sdf.ValueTypeNames.Float,
            "default_value": 0.0,
            "doc": "水平面高度",
        },
        {
            "attr_name": "fluidDensity",
            "attr_type": Sdf.ValueTypeNames.Float,
            "default_value": 1000.0,
            "doc": "流体密度 (kg/m^3)",
        },
        {
            "attr_name": "gravity",
            "attr_type": Sdf.ValueTypeNames.Float,
            "default_value": 9.81,
            "doc": "重力加速度 (m/s^2)",
        },
    ]

    def on_init(self):
        # 1. 默认设置
        self._water_level = 0.0          # 水平面高度
        self._fluid_density = 1000.0     # 流体密度 (kg/m^3)
        self._gravity = 9.81            # 重力加速度 (m/s^2)
        self._meshes = []
        self._rb_path = None
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

    def on_play(self):
        stage = omni.usd.get_context().get_stage()
        self._prim = stage.GetPrimAtPath(self.prim_path)
        
        # Fetch exposed variables
        water_level = self._get_exposed_variable("waterLevel")
        if water_level is not None:
            self._water_level = water_level
            
        fluid_density = self._get_exposed_variable("fluidDensity")
        if fluid_density is not None:
            self._fluid_density = fluid_density
            
        gravity = self._get_exposed_variable("gravity")
        if gravity is not None:
            self._gravity = gravity
        
        # 寻找拥有刚体 API 的路径（自身或父级）以施加力
        curr = self._prim
        self._rb_path = None
        while curr and curr.GetPath() != Sdf.Path("/"):
            if curr.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
                self._rb_path = str(curr.GetPath())
                break
            curr = curr.GetParent()
        
        if not self._rb_path and self._prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
            self._rb_path = str(self.prim_path)
            
        # 2. 收集所有子 Mesh 的几何数据 (Mesh Query)
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
        carb.log_info(f"Buoyancy: Found {len(self._meshes)} meshes. RigidBody path: {self._rb_path}")

    def on_pause(self):
        carb.log_info(f"{type(self).__name__}.on_pause()->{self.prim_path}")

    def on_stop(self):
        carb.log_info(f"{type(self).__name__}.on_stop()->{self.prim_path}")

    def on_update(self, current_time: float, delta_time: float):
        if not self._rb_path or not self._meshes:
            return

        # Fetch exposed variables dynamically
        water_level = self._get_exposed_variable("waterLevel")
        if water_level is not None:
            self._water_level = water_level
            
        fluid_density = self._get_exposed_variable("fluidDensity")
        if fluid_density is not None:
            self._fluid_density = fluid_density
            
        gravity = self._get_exposed_variable("gravity")
        if gravity is not None:
            self._gravity = gravity

        total_volume = 0.0
        weighted_centroid = np.array([0.0, 0.0, 0.0])

        for mesh_data in self._meshes:
            # 获取世界变换
            world_transform = omni.usd.get_world_transform_matrix(mesh_data["prim"])
            m_np = np.array(world_transform).reshape(4, 4)
            
            # 将局部坐标转换为世界坐标
            points_world = mesh_data["local_points"] @ m_np[:3, :3] + m_np[3, :3]
            indices = mesh_data["indices"]
            counts = mesh_data["counts"]
            
            # AABB 预查
            if np.min(points_world[:, 2]) >= self._water_level:
                continue
            
            # 遍历面
            offset = 0
            for count in counts:
                # 三角化处理每个面
                for j in range(1, count - 1):
                    v0 = points_world[indices[offset]]
                    v1 = points_world[indices[offset + j]]
                    v2 = points_world[indices[offset + j + 1]]
                    
                    # 3. 计算体积中心 (Centroid): 动态计算浸没部分
                    clipped_tris = self._clip_triangle_to_water(v0, v1, v2, self._water_level)
                    
                    for t in clipped_tris:
                        # 参考点设在 z=water_level，确保裁剪面的体积贡献为 0
                        r0 = t[0] - np.array([0, 0, self._water_level])
                        r1 = t[1] - np.array([0, 0, self._water_level])
                        r2 = t[2] - np.array([0, 0, self._water_level])
                        
                        dv = np.dot(r0, np.cross(r1, r2)) / 6.0
                        dc = (np.array([0, 0, self._water_level]) + t[0] + t[1] + t[2]) / 4.0
                        
                        total_volume += dv
                        weighted_centroid += dc * dv
                offset += count

        # 4. 施加力: 在计算出的几何中心施加浮力
        # 注意：此处必须在所有 Mesh 计算完成后再调用，避免重复施力
        if total_volume > 1e-9:
            centroid = weighted_centroid / total_volume
            force_mag = total_volume * self._fluid_density * self._gravity
            
            # 打印计算结果 (浮心位置、排水体积、浮力大小)
            # carb.log_info(f"Buoyancy Debug [{self.prim_path}]:")
            # carb.log_info(f"  - Submerged Volume: {total_volume:.6f} m^3")
            # carb.log_info(f"  - Buoyancy Force: {force_mag:.2f} N")
            # carb.log_info(f"  - Centroid (World): ({centroid[0]:.3f}, {centroid[1]:.3f}, {centroid[2]:.3f})")

            # 获取 Stage ID 和 路径哈希 (Isaac Sim 5.1.0 系统要求)
            stage_id = omni.usd.get_context().get_stage_id()
            body_path_int = int(PhysicsSchemaTools.sdfPathToInt(Sdf.Path(self._rb_path)))
            
            # 正确调用 PhysX 仿真接口
            get_physx_simulation_interface().apply_force_at_pos(
                stage_id,
                body_path_int,
                carb.Float3(0.0, 0.0, float(force_mag)),
                carb.Float3(float(centroid[0]), float(centroid[1]), float(centroid[2])),
                "Force"
            )

    def _clip_triangle_to_water(self, v0, v1, v2, h):
        """将三角形裁剪至 z < h，返回裁剪后的三角形列表。"""
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
