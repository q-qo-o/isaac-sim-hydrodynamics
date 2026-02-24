import re
from omni.kit.property.usd.usd_property_widget import UsdPropertiesWidget, UsdPropertyUiEntry

class HydrodynamicsPropertyWidget(UsdPropertiesWidget):
    def __init__(self, title: str, attribute_namespace_filter: list, collapsed: bool = False):
        super().__init__(title, collapsed, multi_edit=False)
        self._attribute_namespace_filter = attribute_namespace_filter
        self._props_to_build = []
        self._multiple_selection = False

    def on_new_payload(self, payload):
        if not super().on_new_payload(payload):
            return False

        if not self._payload or len(self._payload) == 0:
            return False

        stage = self._payload.get_stage()
        if not stage:
            return False

        self._props_to_build = []
        for prim_path in self._payload:
            prim = self._get_prim(prim_path)
            if not prim:
                continue
            if not prim.HasAttribute("omni:scripting:scripts"):
                continue
            if not prim.GetAttribute("omni:scripting:scripts").Get():
                continue
            props = self._get_prim_properties(prim)
            for prop in props:
                prop_name = prop.GetName()
                if not prop_name.split(":")[0] in self._attribute_namespace_filter:
                    continue
                if len(prop_name.split(":")) > 1 and prop_name.split(":")[1] != "hydrodynamics":
                    continue

                display_group = str(prim_path)
                metadata_with_default = prop.GetAllMetadata()
                metadata_with_default.update({"default": prop.Get()})
                prop_type = prop.GetPropertyType() if hasattr(prop, "GetPropertyType") else type(prop)
                
                ui_entry = UsdPropertyUiEntry(prop_name, display_group, metadata_with_default, prop_type)
                self._props_to_build.append(ui_entry)

        self._multiple_selection = len(self._payload) > 1
        return bool(self._props_to_build)

    def _get_shared_properties_from_selected_prims(self, anchor_prim):
        return self._props_to_build

    def _customize_props_layout(self, props):
        for prop in props:
            prim_path = prop.display_group
            if not self._multiple_selection:
                prim_path = ""

            # Check for explicit displayGroup metadata
            custom_group = prop.metadata.get("displayGroup")
            prop_name = prop.prop_name
            parts = prop_name.split(":")
            display_name = parts[-1]

            if custom_group:
                new_display_group = f"{prim_path}:{custom_group}" if prim_path else custom_group
            elif len(parts) >= 3:
                parts = parts[1:]
                display_name = parts[-1]
                group_titles = [self._make_capitalized_title(part) for part in parts[:-1]]
                new_display_group = f"{prim_path}:{':'.join(group_titles)}" if prim_path else ":".join(group_titles)
            else:
                group_title = "Other"
                display_name = prop_name
                new_display_group = f"{prim_path}:{group_title}" if prim_path else group_title
                
            prop.override_display_group(new_display_group)
            prop.override_display_name(display_name)

        return props

    def _make_capitalized_title(self, namespace_name):
        if "_" in namespace_name:
            return namespace_name.replace("_", " ").title()
        return re.sub(r"(?<!^)(?=[A-Z])", " ", namespace_name).title()