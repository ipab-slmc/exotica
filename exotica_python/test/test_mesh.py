import unittest

import pyexotica as exo
try:
    import trimesh
except ImportError:
    import warnings
    warnings.warn("trimesh not found, skipping test")
    exit()

def validate_mesh(mesh):
    print(mesh, mesh.vertex_count, mesh.triangle_count)
    assert mesh.vertex_count == 33
    assert mesh.triangle_count == 62

class TestPythonMeshCreation(unittest.TestCase):
    def test_create_mesh_from_resource_package_path(self):
        # Load mesh from resource path
        print(">>> Loading STL directly from package-path")
        mesh = exo.Mesh.createMeshFromResource("package://exotica_examples/resources/cone.stl")
        validate_mesh(mesh)

    def test_create_mesh_from_resource_exotica_resource_path(self):
        # Load STL mesh from Exotica resource path
        print(">>> Loading STL from Exotica-resource-path")
        mesh = exo.Mesh.createMeshFromResource("{exotica_examples}/resources/cone.stl")
        validate_mesh(mesh)

        # Load OBJ from Exotica resource path
        print(">>> Loading OBJ from Exotica-resource-path")
        mesh = exo.Mesh.createMeshFromResource("{exotica_examples}/resources/cone.obj")
        validate_mesh(mesh)

    def test_create_mesh_from_vertices(self):
        # Load mesh from vertices and triangles
        print(">>> Creating mesh from list of vertices")
        # Note: exotica's load_obj is not compatible with all obj files, so don't use it.
        m = trimesh.load(exo.Tools.parse_path("{exotica_examples}/resources/cone.obj"))
        vertices = [m.vertices[i] for i in m.faces.flatten()]
        mesh = exo.Mesh.createMeshFromVertices(vertices)
        validate_mesh(mesh)

    def teset_create_mesh_from_vertices_and_triangles(self):
        # STL
        print(">>> Creating mesh from list of vertices and list of triangles")
        m = trimesh.load(exo.Tools.parse_path("{exotica_examples}/resources/cone.stl"))
        mesh = exo.Mesh.createMeshFromVertices(m.vertices, m.faces.flatten())
        validate_mesh(mesh)

if __name__ == "__main__":
    unittest.main()
