from moveit_msgs.msg import CollisionObject
from geometry_msgs.msg import Pose, Point
from shape_msgs.msg import SolidPrimitive, Plane, Mesh, MeshTriangle
import pyexotica as exo

try:
    from pyassimp import pyassimp
except:
    try:
        import pyassimp
    except:
        raise Exception("Failed to import pyassimp")


def create_pose(position, orientation, frame='/world_frame'):
    pose = Pose()
    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = position[2]
    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]
    pose.orientation.w = orientation[3]
    return pose


def create_sphere(name, pose, radius, frame_id='/world_frame'):
    co = CollisionObject()
    co.operation = CollisionObject.ADD
    co.id = name
    co.header.frame_id = frame_id
    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [radius]
    co.primitives = [sphere]
    co.primitive_poses = [pose]
    return co


def create_box(name, pose, size, frame_id='/world_frame'):
    co = CollisionObject()
    co.operation = CollisionObject.ADD
    co.id = name
    co.header.frame_id = frame_id
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = list(size)
    co.primitives = [box]
    co.primitive_poses = [pose]
    return co


def create_mesh(name, pose, filename, scale=(1, 1, 1),
                frame_id='/world_frame'):
    co = CollisionObject()
    scene = pyassimp.load(str(exo.Tools.parsePath(filename)))
    if not scene.meshes or len(scene.meshes) == 0:
        raise Exception("There are no meshes in the file")
    if len(scene.meshes[0].faces) == 0:
        raise Exception("There are no faces in the mesh")
    co.operation = CollisionObject.ADD
    co.id = name
    co.header.frame_id = frame_id

    mesh = Mesh()
    first_face = scene.meshes[0].faces[0]
    if hasattr(first_face, '__len__'):
        for face in scene.meshes[0].faces:
            if len(face) == 3:
                triangle = MeshTriangle()
                triangle.vertex_indices = [face[0], face[1], face[2]]
                mesh.triangles.append(triangle)
    elif hasattr(first_face, 'indices'):
        for face in scene.meshes[0].faces:
            if len(face.indices) == 3:
                triangle = MeshTriangle()
                triangle.vertex_indices = [face.indices[0],
                                           face.indices[1],
                                           face.indices[2]]
                mesh.triangles.append(triangle)
    else:
        raise Exception(
            "Unable to build triangles from mesh")
    for vertex in scene.meshes[0].vertices:
        point = Point()
        point.x = vertex[0]*scale[0]
        point.y = vertex[1]*scale[1]
        point.z = vertex[2]*scale[2]
        mesh.vertices.append(point)
    co.meshes = [mesh]
    co.mesh_poses = [pose]
    pyassimp.release(scene)
    return co


def create_plane(name, pose, normal=(0, 0, 1), offset=0,
                 frame_id='/world_frame'):
    co = CollisionObject()
    co.operation = CollisionObject.ADD
    co.id = name
    co.header.frame_id = frame_id
    p = Plane()
    p.coef = list(normal)
    p.coef.append(offset)
    co.planes = [p]
    co.plane_poses = [pose]
    return co
