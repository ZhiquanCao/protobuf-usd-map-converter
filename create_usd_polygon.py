from pxr import Usd, UsdGeom, UsdShade, UsdPhysics, Gf, Sdf
from semantics.schema.editor import PrimSemanticData
import mapbox_earcut as earcut


def make_initial_stage(path):
    # stage = Usd.Stage.CreateNew(path)
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    return stage


def create_usd_polygon(usd_input):
    """Creates usd file(s) from metadata.

    Creates and exports usd file(s) from list of vertices, rings, and other
    metadata represented by usd_input. The function triangulates the polygon,
    add color material, applies the collision api, and attaches the semantic
    label.

    Args:
        usd_input:  An UsdInput instance containing the list of all vertices,
            the rings to separate the outer vertices and hole vertices, the
            directory to save the usd file, the color of the usd object, the
            semantic label, and the height of the usd object if there is any.

            Attributes of UsdInput:
                verts: A list of points of x, y coordinates.
                rings: A list of separation positions. For example, if verts
                    consists of 8 vertices, and the first four represent the
                    outer edge and the last four define the hole. Then, rings
                    would be [4, 8]. The last element in rings is always the
                    length of the verts.

    Returns:
        The stage of the usd object. The function also has the side effect of
        saving the usd file into the specified directory.
    """
    file_name = usd_input.full_path[usd_input.full_path.rfind('/')+1:usd_input.full_path.rfind('.')]
    triangulated_faces = earcut.triangulate_float32(usd_input.verts, usd_input.rings)
    stage = make_initial_stage(usd_input.full_path)
    xformPrim = stage.DefinePrim('/%(file_name)s' % {"file_name":file_name}, "Xform")
    mesh = UsdGeom.Mesh.Define(stage, '/%(file_name)s/Mesh' % {"file_name":file_name})
    mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.bilinear)  # Subdivision is set to either none or bilinear to display normal light effect
    pointsAttr = mesh.CreatePointsAttr()
    faceVertexCountsAttr = mesh.CreateFaceVertexCountsAttr()
    faceVertexCounts = [3]*(len(triangulated_faces)//3)
    faceVertexIndicesAttr = mesh.CreateFaceVertexIndicesAttr()
    faceVertexIndices = triangulated_faces.tolist()
    if not usd_input.height:
        points = [Gf.Vec3f(x, y, 0) for [x, y] in usd_input.verts]
    else:
        points = [
            Gf.Vec3f(x, y, usd_input.height) for [x, y] in usd_input.verts]+[
            Gf.Vec3f(x, y, 0) for [x, y] in usd_input.verts]
        sideVertexCounts = [4]*(len(usd_input.verts))
        faceVertexCounts += sideVertexCounts
        for i in range(len(usd_input.verts)):
            length = len(usd_input.verts)
            faceVertexIndices.append(i)
            faceVertexIndices.append((i+1) % length)
            faceVertexIndices.append((i+1) % length+length)
            faceVertexIndices.append(i+len(usd_input.verts))
    pointsAttr.Set(points)
    faceVertexCountsAttr.Set(faceVertexCounts)
    faceVertexIndicesAttr.Set(faceVertexIndices)
    
    # Attach material and color

    material = UsdShade.Material.Define(stage, f"/{file_name}/Material")
    shader = UsdShade.Shader.Define(stage, f"/{file_name}/Material/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(usd_input.color)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    mesh.GetPrim().ApplyAPI(UsdShade.MaterialBindingAPI)
    material_binding = UsdShade.MaterialBindingAPI(mesh)
    material_binding.Bind(material)

    meshPrim = stage.GetPrimAtPath('/%(file_name)s/Mesh' % {"file_name":file_name})
    UsdPhysics.CollisionAPI.Apply(meshPrim)
    prim_sd = PrimSemanticData(meshPrim)
    prim_sd.add_entry("class", usd_input.semantic_label)

    # stage.GetRootLayer().Save()
    # usd_data = stage.GetRootLayer().ExportToString()

    print("USD file saved:", usd_input.full_path)
    return stage


if __name__ == "__main__":
    pass
