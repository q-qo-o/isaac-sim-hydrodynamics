# MIT License
# 
# Copyright (c) 2024 <COPYRIGHT_HOLDERS>
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# 

import carb
import omni.ext
import omni.kit.app
import omni.kit.commands
import omni.usd
import omni.kit.context_menu
from pxr import Sdf, Usd
from omni.kit.menu.utils import MenuItemDescription, add_menu_items, remove_menu_items
import weakref
import omni.kit.window.property

from .ui_builder import UIBuilder
from .hydrodynamics_script import HydrodynamicsScript
from .hydrodynamics_property_widget import HydrodynamicsPropertyWidget

class Extension(omni.ext.IExt):
    """The Extension class"""

    def on_startup(self, ext_id):
        """Method called when the extension is loaded/enabled"""
        carb.log_info(f"on_startup {ext_id}")
        ext_path = omni.kit.app.get_app().get_extension_manager().get_extension_path(ext_id)

        # UI handler
        self.ui_builder = UIBuilder(window_title="Hydrodynamics", menu_path="Window/Hydrodynamics")

        # Add context menu
        self._menu_items = [
            MenuItemDescription(
                name="Hydrodynamics",
                onclick_fn=lambda *_: self._add_hydrodynamics(),
            )
        ]

        from omni.kit.property.usd import PrimPathWidget
        self._add_button_menu = PrimPathWidget.add_button_menu_entry(
            "ExtPhysics/Hydrodynamics",
            show_fn=lambda objects: True,
            onclick_fn=lambda payload: self._add_hydrodynamics()
        )

        # Register property widget
        self._register_widget()

    def _register_widget(self):
        property_window = omni.kit.window.property.get_window()
        if property_window:
            property_window.register_widget(
                "prim",
                "hydrodynamics_property_widget",
                HydrodynamicsPropertyWidget(title="Hydrodynamics", attribute_namespace_filter=["exposedVar"]),
            )
            self._registered = True

    def _unregister_widget(self):
        property_window = omni.kit.window.property.get_window()
        if property_window:
            property_window.unregister_widget("prim", "hydrodynamics_property_widget")
            self._registered = False

    def _add_hydrodynamics(self):
        import os
        import shutil
        import omni.usd
        import omni.kit.app
        import omni.kit.commands
        import carb
        from pxr import Sdf

        stage = omni.usd.get_context().get_stage()
        if not stage:
            return

        selected_paths = omni.usd.get_context().get_selection().get_selected_prim_paths()
        if not selected_paths:
            return

        # Determine the script destination path
        layer = stage.GetRootLayer()
        ext_id = omni.kit.app.get_app().get_extension_manager().get_extension_id_by_module("hydrodynamics")
        ext_path = omni.kit.app.get_app().get_extension_manager().get_extension_path(ext_id)
        src_script_path = os.path.join(ext_path, "hydrodynamics", "impl", "hydrodynamics_script.py")

        if layer.anonymous:
            carb.log_warning("Stage is unsaved. Using absolute path for the script.")
            script_path_to_set = src_script_path.replace("\\", "/")
        else:
            usd_dir = os.path.dirname(layer.realPath)
            target_dir = os.path.join(usd_dir, ".extphysics", "scripts")
            os.makedirs(target_dir, exist_ok=True)
            target_script_path = os.path.join(target_dir, "hydrodynamics_script.py")
            
            if os.path.exists(src_script_path):
                shutil.copy(src_script_path, target_script_path)
            
            script_path_to_set = "./.extphysics/scripts/hydrodynamics_script.py"

        for path in selected_paths:
            prim = stage.GetPrimAtPath(path)
            if not prim.IsValid():
                continue

            # Ensure the scripting API is applied to the prim
            scripts_attr = prim.GetAttribute("omni:scripting:scripts")
            if not scripts_attr:
                omni.kit.commands.execute("ApplyScriptingAPICommand", paths=[prim.GetPath()])
                scripts_attr = prim.GetAttribute("omni:scripting:scripts")
                if not scripts_attr:
                    carb.log_error(f"Failed to create scripting attribute on prim: {prim.GetPath()}")
                    continue

            # Add the script
            current_scripts = [asset.path for asset in scripts_attr.Get() or []]
            if script_path_to_set not in current_scripts:
                current_scripts.append(script_path_to_set)
                scripts_attr.Set(Sdf.AssetPathArray(current_scripts))
                carb.log_info(f"Added Hydrodynamics script to {path}")
                
            # Force script initialization to create attributes immediately
            omni.kit.commands.execute("RefreshScriptingPropertyWindowCommand")
            
            # Manually create attributes if script hasn't initialized yet
            from pxr import Sdf
            for var in [
                {"name": "waterLevel", "type": Sdf.ValueTypeNames.Float, "default": 0.0},
                {"name": "fluidDensity", "type": Sdf.ValueTypeNames.Float, "default": 1000.0},
                {"name": "gravity", "type": Sdf.ValueTypeNames.Float, "default": 9.81},
            ]:
                attr_name = f"exposedVar:hydrodynamics:{var['name']}"
                attr = prim.GetAttribute(attr_name)
                if not attr:
                    attr = prim.CreateAttribute(attr_name, var["type"])
                    attr.Set(var["default"])
            
            import omni.kit.window.property as property_window_module
            window = property_window_module.get_window()
            if window:
                window.request_rebuild()

    def on_shutdown(self):
        """Method called when the extension is disabled"""
        carb.log_info(f"on_shutdown")

        # clean up UI
        self.ui_builder.cleanup()
        
        if hasattr(self, "_add_button_menu") and self._add_button_menu:
            from omni.kit.property.usd import PrimPathWidget
            PrimPathWidget.remove_button_menu_entry(self._add_button_menu)
            self._add_button_menu = None
            
        if hasattr(self, "_registered") and self._registered:
            self._unregister_widget()
