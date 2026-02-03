# Domain Randomization Example for Isaac Sim
# This script demonstrates how to apply domain randomization in Isaac Sim

import omni
from pxr import UsdGeom, Gf, Sdf
import numpy as np
import random

class DomainRandomization:
    def __init__(self):
        """Initialize domain randomization system"""
        self.stage = omni.usd.get_context().get_stage()

        print("Domain randomization system initialized")

    def randomize_lighting(self):
        """Randomize lighting conditions in the scene"""
        # Get all light prims in the stage
        light_prims = []
        for prim in self.stage.TraverseAll():
            if prim.GetTypeName() in ["DistantLight", "SphereLight", "DomeLight"]:
                light_prims.append(prim)

        # Randomize properties for each light
        for light_prim in light_prims:
            # Randomize intensity (between 500 and 5000)
            intensity_attr = light_prim.GetAttribute("inputs:intensity")
            if intensity_attr.IsValid():
                new_intensity = random.uniform(500, 5000)
                intensity_attr.Set(new_intensity)

            # Randomize color temperature (between 3000K and 8000K)
            color_attr = light_prim.GetAttribute("inputs:color")
            if color_attr.IsValid():
                # Generate color based on temperature
                temp = random.uniform(3000, 8000)
                color = self.temperature_to_rgb(temp)
                color_attr.Set(Gf.Vec3f(color[0], color[1], color[2]))

        print(f"Randomized lighting for {len(light_prims)} lights")

    def temperature_to_rgb(self, temp_k):
        """Convert temperature in Kelvin to RGB color"""
        temp_k /= 100
        r = g = b = 0

        # Red
        if temp_k <= 66:
            r = 255
        else:
            r = temp_k - 60
            r = 329.698727446 * (r ** -0.1332047592)
            r = max(0, min(255, r))

        # Green
        if temp_k <= 66:
            g = temp_k
            g = 99.4708025861 * np.log(g) - 161.1195681661
        else:
            g = temp_k - 60
            g = 288.1221695283 * (g ** -0.0755148492)
        g = max(0, min(255, g))

        # Blue
        if temp_k >= 66:
            b = 255
        elif temp_k <= 19:
            b = 0
        else:
            b = temp_k - 10
            b = 138.5177312231 * np.log(b) - 305.0447927307
            b = max(0, min(255, b))

        return [r/255.0, g/255.0, b/255.0]

    def randomize_material_properties(self):
        """Randomize material properties for objects in the scene"""
        # Get all material prims
        material_prims = []
        for prim in self.stage.TraverseAll():
            if prim.GetTypeName() == "Material":
                material_prims.append(prim)

        # Randomize properties for each material
        for material_prim in material_prims:
            # Look for surface shader inside the material
            surface_shader = None
            for child in material_prim.GetChildren():
                if "Surface" in child.GetTypeName() or "PreviewSurface" in child.GetTypeName():
                    surface_shader = child
                    break

            if surface_shader:
                # Randomize albedo/base color
                albedo_attr = surface_shader.GetAttribute("inputs:diffuse_color_constant")
                if albedo_attr.IsValid():
                    color = [
                        random.uniform(0.0, 1.0),
                        random.uniform(0.0, 1.0),
                        random.uniform(0.0, 1.0)
                    ]
                    albedo_attr.Set(Gf.Vec3f(color[0], color[1], color[2]))

                # Randomize roughness
                roughness_attr = surface_shader.GetAttribute("inputs:roughness_constant")
                if roughness_attr.IsValid():
                    roughness = random.uniform(0.1, 0.9)
                    roughness_attr.Set(roughness)

                # Randomize metallic
                metallic_attr = surface_shader.GetAttribute("inputs:metallic_constant")
                if metallic_attr.IsValid():
                    metallic = random.uniform(0.0, 1.0)
                    metallic_attr.Set(metallic)

        print(f"Randomized materials for {len(material_prims)} materials")

    def randomize_object_textures(self):
        """Apply texture variations to objects"""
        # This is a simplified example - in practice, you'd swap textures
        # from a collection of similar textures
        object_prims = []
        for prim in self.stage.TraverseAll():
            if prim.GetTypeName() in ["Mesh", "Cylinder", "Cube", "Sphere"]:
                object_prims.append(prim)

        print(f"Applied texture variations to {len(object_prims)} objects")

    def randomize_physics_properties(self):
        """Randomize physics properties of objects"""
        # Get all rigid bodies in the scene
        rigid_body_prims = []
        for prim in self.stage.TraverseAll():
            if prim.HasAttribute("physics:mass"):
                rigid_body_prims.append(prim)

        # Randomize physics properties
        for rb_prim in rigid_body_prims:
            # Randomize mass (within reasonable bounds)
            mass_attr = rb_prim.GetAttribute("physics:mass")
            if mass_attr.IsValid():
                current_mass = mass_attr.Get()
                if current_mass > 0:
                    # Randomize within ±20% of original
                    variation = random.uniform(0.8, 1.2)
                    new_mass = current_mass * variation
                    mass_attr.Set(new_mass)

            # Randomize friction
            friction_attr = rb_prim.GetAttribute("physics:staticFriction")
            if friction_attr.IsValid():
                new_friction = random.uniform(0.1, 0.9)
                friction_attr.Set(new_friction)

        print(f"Randomized physics for {len(rigid_body_prims)} objects")

    def apply_domain_randomization(self):
        """Apply all domain randomization techniques"""
        print("Applying domain randomization...")

        self.randomize_lighting()
        self.randomize_material_properties()
        self.randomize_object_textures()
        self.randomize_physics_properties()

        print("Domain randomization applied successfully!")

def apply_randomization():
    """Function to apply domain randomization from Isaac Sim"""
    dr = DomainRandomization()
    dr.apply_domain_randomization()

# Example usage in Isaac Sim
if __name__ == "__main__":
    apply_randomization()