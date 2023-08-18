from omni.isaac.kit import SimulationApp

# This needs to come first before loading other isaac extensions
simulation_app = SimulationApp({
    "headless": True,
    "renderer": "RayTracedLighting",
})

from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf, Tf
from omni.isaac.core.utils.prims import define_prim
from omni.isaac.core.materials import PhysicsMaterial
from omni.isaac.core.prims import RigidPrim, GeometryPrim
import numpy as np
import mapbox_earcut as earcut
import constants

USD_FORMAT = constants.USD_FORMAT
OUTPUT_DIR = constants.OUTPUT_DIR


def make_initial_stage(path):
    stage = Usd.Stage.CreateNew(path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    stage.SetStartTimeCode(1)
    stage.SetEndTimeCode(192)
    return stage

def add_material():
    pass

def add_physical_mass():
    define_prim("/rigid_prim")
    define_prim("/rigid_prim/geometry_prim", prim_type="Cube")

    rigid_prim = RigidPrim(prim_path="/rigid_prim",
                        position=np.array([0, 0, 0]),
                        orientation=np.array([1, 0, 0, 0]),
                        name="rigid_prim",
                        scale=np.array([1] * 3))

    geometry_prim = GeometryPrim(prim_path="/rigid_prim/geometry_prim",
                                position=np.array([0, 0, 0]),
                                orientation=np.array([1, 0, 0, 0]),
                                name="geometry_prim",
                                scale=np.array([1] * 3))

    geometry_prim.apply_physics_material(
        PhysicsMaterial(
            prim_path= "/rigid_prim/physics_material",
            static_friction=0.75,
            dynamic_friction=None,
            restitution=None
        )        
    )

    rigid_prim.set_mass(5.0)

def create_usd_polygon(verts, rings, file_name, color):
    """
    This function creates the usd polygon from verts and rings
    
    'verts' is the list of points of x, y coordinates
    
    'rings', from the mapbox_earcut package, defines the which part of the points in verts is outer edge,
    and which points are holes. For example, if verts consists of 8 vertices, and the first four are outer edge and the
    last four define the hole. Then, rings would be [4, 8]. The last element in rings is always the length of the verts variable
    
    This function will export a usda file to the usda_file_path
    """
    
    # Create the destination path to export the usd file 
    usda_file_path = f'{OUTPUT_DIR}{file_name}.{USD_FORMAT}'

    # Use the mapbox_earcut package to calculate the triangulated faces
    triangulated_faces = earcut.triangulate_float32(verts, rings)
    
    # Create the stage for USD
    stage = make_initial_stage(usda_file_path)

    # Physics scene definition
    scene = UsdPhysics.Scene.Define(stage, "/physicsScene")

    # setup gravity
    # note that gravity has to respect the selected units, if we are using cm, the gravity has to respect that
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(981.0)
    
    # Create a new Xform prim
    xformPrim = stage.DefinePrim(f"/{file_name}", "Xform")

    # Create a new Mesh prim as a child of the Xform prim
    meshPrim = stage.DefinePrim(xformPrim.GetPath().AppendChild("Mesh"), "Mesh")

    # Create attributes for points and faceVertexCounts
    pointsAttr = meshPrim.CreateAttribute("points", Sdf.ValueTypeNames.Point3fArray)
    faceVertexCountsAttr = meshPrim.CreateAttribute("faceVertexCounts", Sdf.ValueTypeNames.IntArray)

    # Set points of the mesh
    points = [
        Gf.Vec3f(x, y, 0) for [x, y] in verts
    ]
    pointsAttr.Set(points)
    
    # Set face vertex counts of the mesh
    faceVertexCounts = [3]*(len(triangulated_faces)//3)
    faceVertexCountsAttr.Set(faceVertexCounts)

    # Create attribute for face vertex indices
    faceVertexIndicesAttr = meshPrim.CreateAttribute("faceVertexIndices", Sdf.ValueTypeNames.IntArray)

    # Set face vertex indices of the mesh
    faceVertexIndices = triangulated_faces
    faceVertexIndicesAttr.Set(faceVertexIndices)
    
    # Create a new Material
    material = UsdShade.Material.Define(stage, f"/{file_name}/Material")
    
    # Create a new Shader for the Material
    shader = UsdShade.Shader.Define(stage, f"/{file_name}/Material/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    
    # Set the color attribute of the Shader
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(color)
    
    # # Get the surface output of the Material
    # material_surface_output = material.GetSurfaceOutput()
    
    # # Connect the Shader to the Material's surface output
    # shader.CreateOutput("surface", Sdf.ValueTypeNames.Token).ConnectToSource(material_surface_output)
    
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    
    meshPrim.GetPrim().ApplyAPI(UsdShade.MaterialBindingAPI)
    # Create a new MaterialBinding to apply the Material to the Mesh
    material_binding = UsdShade.MaterialBindingAPI(meshPrim)
    material_binding.Bind(material)
    
    UsdPhysics.CollisionAPI.Apply(meshPrim)
    # UsdPhysics.RigidBodyAPI.Apply(xformPrim)
    # UsdPhysics.RigidBodyAPI.SetGravityEnabled(False)
    # meshPrim.disable_rigid_body_physics()

    # add_physical_mass()
    # define_prim("/rigid_prim")
    # define_prim("/rigid_prim/geometry_prim", prim_type="Cube")

    # rigid_prim = RigidPrim(prim_path="/rigid_prim",
    #                     position=np.array([0, 0, 0]),
    #                     orientation=np.array([1, 0, 0, 0]),
    #                     name="rigid_prim",
    #                     scale=np.array([1] * 3))

    # geometry_prim = GeometryPrim(prim_path="/rigid_prim/geometry_prim",
    #                             position=np.array([0, 0, 0]),
    #                             orientation=np.array([1, 0, 0, 0]),
    #                             name="geometry_prim",
    #                             scale=np.array([1] * 3))

    # geometry_prim.apply_physics_material(
    #     PhysicsMaterial(
    #         prim_path= "/rigid_prim/physics_material",
    #         static_friction=0.75,
    #         dynamic_friction=None,
    #         restitution=None
    #     )        
    # )

    # rigid_prim.set_mass(5.0)
    
    stage.GetRootLayer().Save()
    print("USD file saved:", usda_file_path)
    return stage


if __name__ == "__main__":
    # Example
    verts = [[-2, -2], [2, -2], [2, 2], [-2, 2], [-1, -1], [1, -1], [1, 1], [-1, 1]]
    rings = [4, 8]
    create_usd_polygon(verts, rings, "physics", (1, 0, 0))
simulation_app.close()  # Cleanup application
