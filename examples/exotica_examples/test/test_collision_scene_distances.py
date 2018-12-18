#!/usr/bin/env python
from __future__ import print_function
import pyexotica as exo
import math
import time
import numpy as np

PKG = 'exotica_examples'
import roslib; roslib.load_manifest(PKG)  # This line is not needed with Catkin.

import unittest

exo.Setup.init_ros()

PENETRATING_DISTANCE_ATOL = 1e-6
AFAR_DISTANCE_ATOL = 1e-3
CLOSE_DISTANCE_ATOL = 1e-6
PUBLISH_PROXIES = False

def get_problem_initializer(collision_scene, URDF):
    return ('exotica/UnconstrainedEndPoseProblem',
            {'Name': 'TestProblem',
             'PlanningScene': [('exotica/Scene',
                                {'CollisionScene': collision_scene,
                                 'JointGroup': 'group1',
                                 'Name': 'TestScene',
                                 'Debug': '0',
                                 'SRDF': '{exotica_examples}/test/resources/A_vs_B.srdf',
                                 'SetRobotDescriptionRosParams': '1',
                                 'URDF': URDF})]})


def debug_publish(p, scene):
    if PUBLISH_PROXIES:
        scene.get_solver().publish_frames()
        scene.publish_proxies(p)
        time.sleep(.5)


def testSphereVsSphereDistance(collisionScene):
    PrimitiveSphere_vs_PrimitiveSphere_Distance = get_problem_initializer(collisionScene, '{exotica_examples}/test/resources/PrimitiveSphere_vs_PrimitiveSphere_Distance.urdf')
    prob = exo.Setup.create_problem(PrimitiveSphere_vs_PrimitiveSphere_Distance)
    prob.update(np.zeros(prob.N,))
    scene = prob.get_scene()

    assert(scene.is_state_valid(True) == True)
    print('PrimitiveSphere_vs_PrimitiveSphere_Distance: is_state_valid(True): PASSED')

    p = scene.get_collision_distance("A", "B")
    debug_publish(p, scene)
    assert(len(p) == 1)
    assert(np.isclose(p[0].distance, 1.))
    expected_contact_1 = np.array([-0.5, 0, 0])
    expected_contact_2 = np.array([0.5, 0, 0])
    expected_normal_1 = np.array([1, 0, 0])
    expected_normal_2 = np.array([-1, 0, 0])
    assert(np.isclose(p[0].contact_1, expected_contact_1).all())
    assert(np.isclose(p[0].contact_2, expected_contact_2).all())
    assert(np.isclose(p[0].normal_1, expected_normal_1).all())
    assert(np.isclose(p[0].normal_2, expected_normal_2).all())
    print('PrimitiveSphere_vs_PrimitiveSphere_Distance: Distance, Contact Points, Normals: PASSED')


def testSphereVsSpherePenetrating(collisionScene):
    PrimitiveSphere_vs_PrimitiveSphere_Penetrating = get_problem_initializer(collisionScene, '{exotica_examples}/test/resources/PrimitiveSphere_vs_PrimitiveSphere_Penetrating.urdf')
    prob = exo.Setup.create_problem(PrimitiveSphere_vs_PrimitiveSphere_Penetrating)
    prob.update(np.zeros(prob.N,))
    scene = prob.get_scene()

    assert(scene.is_state_valid(True) == False)
    print('PrimitiveSphere_vs_PrimitiveSphere_Penetrating: is_state_valid(True): PASSED')

    p = scene.get_collision_distance("A", "B")
    debug_publish(p, scene)
    assert(len(p) == 1)
    assert(np.isclose(p[0].distance, -.4, atol=PENETRATING_DISTANCE_ATOL))
    expected_contact_1 = np.array([0.2, 0, 0])
    expected_contact_2 = np.array([-0.2, 0, 0])
    expected_normal_1 = np.array([-1, 0, 0])
    expected_normal_2 = np.array([1, 0, 0])
    assert(np.isclose(p[0].contact_1, expected_contact_1, atol=AFAR_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].contact_2, expected_contact_2, atol=AFAR_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].normal_1, expected_normal_1, atol=AFAR_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].normal_2, expected_normal_2, atol=AFAR_DISTANCE_ATOL).all())
    print('PrimitiveSphere_vs_PrimitiveSphere_Penetrating: Distance, Contact Points, Normals: PASSED')


def testSphereVsBoxDistance(collisionScene):
    PrimitiveSphere_vs_PrimitiveBox_Distance = get_problem_initializer(collisionScene, '{exotica_examples}/test/resources/PrimitiveSphere_vs_PrimitiveBox_Distance.urdf')
    prob = exo.Setup.create_problem(PrimitiveSphere_vs_PrimitiveBox_Distance)
    prob.update(np.zeros(prob.N,))
    scene = prob.get_scene()

    assert(scene.is_state_valid(True) == True)
    print('PrimitiveSphere_vs_PrimitiveBox_Distance: is_state_valid(True): PASSED')

    p = scene.get_collision_distance("A", "B")
    debug_publish(p, scene)
    assert(len(p) == 1)
    assert(np.isclose(p[0].distance, 1.5))
    expected_contact_1 = np.array([-0.5, 0, 0])
    expected_contact_2 = np.array([1, 0, 0])
    expected_normal_1 = np.array([1, 0, 0])
    expected_normal_2 = np.array([-1, 0, 0])
    assert(np.isclose(p[0].contact_1, expected_contact_1, atol=AFAR_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].contact_2, expected_contact_2, atol=AFAR_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].normal_1, expected_normal_1, atol=AFAR_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].normal_2, expected_normal_2, atol=AFAR_DISTANCE_ATOL).all())
    print('PrimitiveSphere_vs_PrimitiveBox_Distance: Distance, Contact Points, Normals: PASSED')


def testSphereVsBoxTouching(collisionScene):
    PrimitiveSphere_vs_PrimitiveBox_Touching = get_problem_initializer(collisionScene, '{exotica_examples}/test/resources/PrimitiveSphere_vs_PrimitiveBox_Touching.urdf')
    prob = exo.Setup.create_problem(PrimitiveSphere_vs_PrimitiveBox_Touching)
    prob.update(np.zeros(prob.N,))
    scene = prob.get_scene()

    assert(scene.is_state_valid(True) == False)
    print('PrimitiveSphere_vs_PrimitiveBox_Touching: is_state_valid(True): PASSED')

    p = scene.get_collision_distance("A", "B")
    debug_publish(p, scene)
    assert(len(p) == 1)
    assert(np.isclose(p[0].distance, 0.))
    expected_contact_1 = np.array([0, 0, 0])
    expected_contact_2 = np.array([0, 0, 0])
    expected_normal_1 = np.array([-1, 0, 0])
    expected_normal_2 = np.array([1, 0, 0])
    assert(np.isclose(p[0].contact_1, expected_contact_1, atol=AFAR_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].contact_2, expected_contact_2, atol=AFAR_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].normal_1, expected_normal_1, atol=AFAR_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].normal_2, expected_normal_2, atol=AFAR_DISTANCE_ATOL).all())
    print('PrimitiveSphere_vs_PrimitiveBox_Touching: Distance, Contact Points, Normals: PASSED')


def testSphereVsBoxPenetrating(collisionScene):
    PrimitiveSphere_vs_PrimitiveBox_Penetrating = get_problem_initializer(collisionScene, '{exotica_examples}/test/resources/PrimitiveSphere_vs_PrimitiveBox_Penetrating.urdf')
    prob = exo.Setup.create_problem(PrimitiveSphere_vs_PrimitiveBox_Penetrating)
    prob.update(np.zeros(prob.N,))
    scene = prob.get_scene()

    assert(scene.is_state_valid(True) == False)
    print('PrimitiveSphere_vs_PrimitiveBox_Penetrating: is_state_valid(True): PASSED')

    p = scene.get_collision_distance("A", "B")
    debug_publish(p, scene)
    assert(len(p) == 1)
    assert(np.isclose(p[0].distance, -0.4, atol=PENETRATING_DISTANCE_ATOL))
    expected_contact_1 = np.array([0.2, 0, 0])
    expected_contact_2 = np.array([-0.2, 0, 0])
    expected_normal_1 = np.array([-1, 0, 0])
    expected_normal_2 = np.array([1, 0, 0])
    assert(np.isclose(p[0].contact_1, expected_contact_1, atol=AFAR_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].contact_2, expected_contact_2, atol=AFAR_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].normal_1, expected_normal_1, atol=AFAR_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].normal_2, expected_normal_2, atol=AFAR_DISTANCE_ATOL).all())
    print('PrimitiveSphere_vs_PrimitiveBox_Penetrating: Distance, Contact Points, Normals: PASSED')


def testSphereVsCylinderDistance(collisionScene):
    PrimitiveSphere_vs_PrimitiveCylinder_Distance = get_problem_initializer(collisionScene, '{exotica_examples}/test/resources/PrimitiveSphere_vs_PrimitiveCylinder_Distance.urdf')
    prob = exo.Setup.create_problem(PrimitiveSphere_vs_PrimitiveCylinder_Distance)
    prob.update(np.zeros(prob.N,))
    scene = prob.get_scene()

    assert(scene.is_state_valid(True) == True)
    print('PrimitiveSphere_vs_PrimitiveCylinder_Distance: is_state_valid(True): PASSED')

    p = scene.get_collision_distance("A", "B")
    debug_publish(p, scene)
    assert(len(p) == 1)
    assert(np.isclose(p[0].distance, 1.5))
    expected_contact_1 = np.array([-0.5, 0, 0])
    expected_contact_2 = np.array([1, 0, 0])
    expected_normal_1 = np.array([1, 0, 0])
    expected_normal_2 = np.array([-1, 0, 0])
    assert(np.isclose(p[0].contact_1, expected_contact_1, atol=AFAR_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].contact_2, expected_contact_2, atol=AFAR_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].normal_1, expected_normal_1, atol=AFAR_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].normal_2, expected_normal_2, atol=AFAR_DISTANCE_ATOL).all())
    print('PrimitiveSphere_vs_PrimitiveCylinder_Distance: Distance, Contact Points, Normals: PASSED')


def testSphereVsCylinderPenetrating(collisionScene):
    PrimitiveSphere_vs_PrimitiveCylinder_Penetrating = get_problem_initializer(collisionScene, '{exotica_examples}/test/resources/PrimitiveSphere_vs_PrimitiveCylinder_Penetrating.urdf')
    prob = exo.Setup.create_problem(PrimitiveSphere_vs_PrimitiveCylinder_Penetrating)
    prob.update(np.zeros(prob.N,))
    scene = prob.get_scene()

    assert(scene.is_state_valid(True) == False)
    print('PrimitiveSphere_vs_PrimitiveCylinder_Penetrating: is_state_valid(True): PASSED')

    p = scene.get_collision_distance("A", "B")
    debug_publish(p, scene)
    assert(len(p) == 1)
    assert(np.isclose(p[0].distance, -0.4, atol=PENETRATING_DISTANCE_ATOL))
    expected_contact_1 = np.array([0.2, 0, 0])
    expected_contact_2 = np.array([-0.2, 0, 0])
    expected_normal_1 = np.array([-1, 0, 0])
    expected_normal_2 = np.array([1, 0, 0])
    assert(np.isclose(p[0].contact_1, expected_contact_1, atol=AFAR_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].contact_2, expected_contact_2, atol=AFAR_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].normal_1, expected_normal_1, atol=AFAR_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].normal_2, expected_normal_2, atol=AFAR_DISTANCE_ATOL).all())
    print('PrimitiveSphere_vs_PrimitiveCylinder_Penetrating: Distance, Contact Points, Normals: PASSED')


def testSphereVsMeshDistance(collisionScene):
    PrimitiveSphere_vs_Mesh_Distance = get_problem_initializer(collisionScene, '{exotica_examples}/test/resources/PrimitiveSphere_vs_Mesh_Distance.urdf')
    prob = exo.Setup.create_problem(PrimitiveSphere_vs_Mesh_Distance)
    prob.update(np.zeros(prob.N,))
    scene = prob.get_scene()

    assert(scene.is_state_valid(True) == True)
    print('PrimitiveSphere_vs_Mesh_Distance: is_state_valid(True): PASSED')

    expectedDistance = 1
    expected_contact_1 = np.array([-0.5, 0, 0])
    expected_contact_2 = np.array([0.5, 0, 0])
    expected_normal_1 = np.array([1, 0, 0])
    expected_normal_2 = np.array([-1, 0, 0])

    p = scene.get_collision_distance("A", "B")
    print(p)
    debug_publish(p, scene)
    assert(len(p) == 1)
    assert(np.isclose(p[0].distance, expectedDistance))
    assert(np.isclose(p[0].contact_1, expected_contact_1).all())
    assert(np.isclose(p[0].contact_2, expected_contact_2).all())
    assert(np.isclose(p[0].normal_1, expected_normal_1).all())
    assert(np.isclose(p[0].normal_2, expected_normal_2).all())
    print('PrimitiveSphere_vs_Mesh_Distance: Distance, Contact Points, Normals: PASSED')

    # MeshVsSphere Test
    p = scene.get_collision_distance("B", "A")
    print(p)
    debug_publish(p, scene)
    assert(len(p) == 1)
    assert(np.isclose(p[0].distance, expectedDistance))
    assert(np.isclose(p[0].contact_1, expected_contact_2).all())
    assert(np.isclose(p[0].contact_2, expected_contact_1).all())
    assert(np.isclose(p[0].normal_1, expected_normal_2).all())
    assert(np.isclose(p[0].normal_2, expected_normal_1).all())
    print('Mesh_vs_PrimitiveSphere_Distance: Distance, Contact Points, Normals: PASSED')


def testSphereVsMeshPenetrating(collisionScene):
    PrimitiveSphere_vs_Mesh_Penetrating = get_problem_initializer(collisionScene, '{exotica_examples}/test/resources/PrimitiveSphere_vs_Mesh_Penetrating.urdf')
    prob = exo.Setup.create_problem(PrimitiveSphere_vs_Mesh_Penetrating)
    prob.update(np.zeros(prob.N,))
    scene = prob.get_scene()

    assert(scene.is_state_valid(True) == False)
    print('PrimitiveSphere_vs_Mesh_Penetrating: is_state_valid(True): PASSED')

    p = scene.get_collision_distance("A", "B")
    debug_publish(p, scene)
    print(p)
    expected_contact_1 = np.array([0.2, 0, 0])
    expected_contact_2 = np.array([-0.2, 0, 0])
    expected_normal_1 = np.array([-1, 0, 0])
    expected_normal_2 = np.array([1, 0, 0])
    assert(len(p) == 1)
    assert(np.isclose(p[0].distance, -.4, atol=PENETRATING_DISTANCE_ATOL))
    assert(np.isclose(p[0].contact_1, expected_contact_1).all())
    assert(np.isclose(p[0].contact_2, expected_contact_2).all())
    assert(np.isclose(p[0].normal_1, expected_normal_1).all())
    assert(np.isclose(p[0].normal_2, expected_normal_2).all())
    print('PrimitiveSphere_vs_Mesh_Penetrating: Distance, Contact Points, Normals: PASSED')

    # MeshVsSphere Test
    p = scene.get_collision_distance("B", "A")
    print(p)
    debug_publish(p, scene)
    assert(len(p) == 1)
    assert(np.isclose(p[0].distance, 1))
    assert(np.isclose(p[0].contact_1, expected_contact_2).all())
    assert(np.isclose(p[0].contact_2, expected_contact_1).all())
    assert(np.isclose(p[0].normal_1, expected_normal_2).all())
    assert(np.isclose(p[0].normal_2, expected_normal_1).all())
    print('Mesh_vs_PrimitiveSphere_Penetrating: Distance, Contact Points, Normals: PASSED')


def testBoxVsBoxDistance(collisionScene):
    PrimitiveBox_vs_PrimitiveBox_Distance = get_problem_initializer(collisionScene, '{exotica_examples}/test/resources/PrimitiveBox_vs_PrimitiveBox_Distance.urdf')
    prob = exo.Setup.create_problem(PrimitiveBox_vs_PrimitiveBox_Distance)
    prob.update(np.zeros(prob.N,))
    scene = prob.get_scene()

    assert(scene.is_state_valid(True) == True)
    print('PrimitiveBox_vs_PrimitiveBox_Distance: is_state_valid(True): PASSED')

    p = scene.get_collision_distance("A", "B")
    debug_publish(p, scene)
    assert(len(p) == 1)
    assert(np.isclose(p[0].distance, 1.))
    expected_contact_1 = np.array([-0.5, 0, 0])
    expected_contact_2 = np.array([0.5, 0, 0])
    expected_normal_1 = np.array([1, 0, 0])
    expected_normal_2 = np.array([-1, 0, 0])
    print(p)
    assert(np.isclose(p[0].contact_1[0], expected_contact_1[0], atol=CLOSE_DISTANCE_ATOL))
    assert(np.isclose(p[0].contact_2[0], expected_contact_2[0], atol=CLOSE_DISTANCE_ATOL))
    assert(np.isclose(p[0].normal_1, expected_normal_1, atol=CLOSE_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].normal_2, expected_normal_2, atol=CLOSE_DISTANCE_ATOL).all())
    print('PrimitiveBox_vs_PrimitiveBox_Distance: Distance, Contact Points, Normals: PASSED')


def testBoxVsBoxTouching(collisionScene):
    PrimitiveBox_vs_PrimitiveBox_Touching = get_problem_initializer(collisionScene, '{exotica_examples}/test/resources/PrimitiveBox_vs_PrimitiveBox_Touching.urdf')
    prob = exo.Setup.create_problem(PrimitiveBox_vs_PrimitiveBox_Touching)
    prob.update(np.zeros(prob.N,))
    scene = prob.get_scene()

    assert(scene.is_state_valid(True) == False)
    print('PrimitiveBox_vs_PrimitiveBox_Touching: is_state_valid(True): PASSED')

    p = scene.get_collision_distance("A", "B")
    debug_publish(p, scene)
    assert(len(p) == 1)
    assert(np.isclose(p[0].distance, 0.))
    expected_contact_1 = np.array([0, 0, 0])
    expected_contact_2 = np.array([0, 0, 0])
    expected_normal_1 = np.array([-1, 0, 0])
    expected_normal_2 = np.array([1, 0, 0])
    assert(np.isclose(p[0].contact_1-p[0].contact_2, np.zeros((3,1))).all())
    # Only check x-axis as this is face to face contact and thus undefined where y and z are.
    assert(np.isclose(p[0].contact_1[0], expected_contact_1[0]))
    assert(np.isclose(p[0].contact_2[0], expected_contact_2[0]))
    assert(np.isclose(p[0].normal_1, expected_normal_1).all())
    assert(np.isclose(p[0].normal_2, expected_normal_2).all())
    print('PrimitiveBox_vs_PrimitiveBox_Touching: Distance, Contact Points, Normals: PASSED')


def testBoxVsBoxPenetrating(collisionScene):
    PrimitiveBox_vs_PrimitiveBox_Penetrating = get_problem_initializer(collisionScene, '{exotica_examples}/test/resources/PrimitiveBox_vs_PrimitiveBox_Penetrating.urdf')
    prob = exo.Setup.create_problem(PrimitiveBox_vs_PrimitiveBox_Penetrating)
    prob.update(np.zeros(prob.N,))
    scene = prob.get_scene()

    assert(scene.is_state_valid(True) == False)
    print('PrimitiveBox_vs_PrimitiveBox_Penetrating: is_state_valid(True): PASSED')

    p = scene.get_collision_distance("A", "B")
    debug_publish(p, scene)
    assert(len(p) == 1)
    assert(np.isclose(p[0].distance, -.2))
    expected_contact_1 = np.array([0.1, 0, 0])
    expected_contact_2 = np.array([-0.1, 0, 0])
    expected_normal_1 = np.array([-1, 0, 0])
    expected_normal_2 = np.array([1, 0, 0])
    # Only test the x-component for the contacts as the faces are parallel.
    assert(np.isclose(p[0].contact_1[0], expected_contact_1[0]))
    assert(np.isclose(p[0].contact_2[0], expected_contact_2[0]))
    assert(np.isclose(p[0].normal_1, expected_normal_1).all())
    assert(np.isclose(p[0].normal_2, expected_normal_2).all())
    print('PrimitiveBox_vs_PrimitiveBox_Penetrating: Distance, Contact Points, Normals: PASSED')


def testBoxVsCylinderDistance(collisionScene):
    PrimitiveBox_vs_PrimitiveCylinder_Distance = get_problem_initializer(collisionScene, '{exotica_examples}/test/resources/PrimitiveBox_vs_PrimitiveCylinder_Distance.urdf')
    prob = exo.Setup.create_problem(PrimitiveBox_vs_PrimitiveCylinder_Distance)
    prob.update(np.zeros(prob.N,))
    scene = prob.get_scene()

    assert(scene.is_state_valid(True) == True)
    print('PrimitiveBox_vs_PrimitiveCylinder_Distance: is_state_valid(True): PASSED')

    p = scene.get_collision_distance("A", "B")
    debug_publish(p, scene)
    assert(len(p) == 1)
    assert(np.isclose(p[0].distance, 1.))
    expected_contact_1 = np.array([-0.5, 0, 0])
    expected_contact_2 = np.array([0.5, 0, 0])
    expected_normal_1 = np.array([1, 0, 0])
    expected_normal_2 = np.array([-1, 0, 0])
    assert(np.isclose(p[0].contact_1[0], expected_contact_1[0])) # TODO: face center issue
    assert(np.isclose(p[0].contact_2[0], expected_contact_2[0])) # TODO: face center issue
    assert(np.isclose(p[0].normal_1, expected_normal_1).all())
    assert(np.isclose(p[0].normal_2, expected_normal_2).all())
    print('PrimitiveBox_vs_PrimitiveCylinder_Distance: Distance, Contact Points, Normals: PASSED')


def testBoxVsCylinderPenetrating(collisionScene):
    PrimitiveBox_vs_PrimitiveCylinder_Penetrating = get_problem_initializer(collisionScene, '{exotica_examples}/test/resources/PrimitiveBox_vs_PrimitiveCylinder_Penetrating.urdf')
    prob = exo.Setup.create_problem(PrimitiveBox_vs_PrimitiveCylinder_Penetrating)
    prob.update(np.zeros(prob.N,))
    scene = prob.get_scene()

    assert(scene.is_state_valid(True) == False)
    print('PrimitiveBox_vs_PrimitiveCylinder_Penetrating: is_state_valid(True): PASSED')

    p = scene.get_collision_distance("A", "B")
    debug_publish(p, scene)
    assert(len(p) == 1)
    assert(np.isclose(p[0].distance, -.2, atol=PENETRATING_DISTANCE_ATOL))
    expected_contact_1 = np.array([0.1, 0, 0])
    expected_contact_2 = np.array([-0.1, 0, 0])
    expected_normal_1 = np.array([-1, 0, 0])
    expected_normal_2 = np.array([1, 0, 0])
    assert(np.isclose(p[0].contact_1, expected_contact_1, atol=AFAR_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].contact_2, expected_contact_2, atol=AFAR_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].normal_1, expected_normal_1, atol=AFAR_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].normal_2, expected_normal_2, atol=AFAR_DISTANCE_ATOL).all())
    print('PrimitiveBox_vs_PrimitiveCylinder_Penetrating: Distance, Contact Points, Normals: PASSED')


def testBoxVsMeshDistance(collisionScene):
    PrimitiveBox_vs_Mesh_Distance = get_problem_initializer(collisionScene, '{exotica_examples}/test/resources/PrimitiveBox_vs_Mesh_Distance.urdf')
    prob = exo.Setup.create_problem(PrimitiveBox_vs_Mesh_Distance)
    prob.update(np.zeros(prob.N,))
    scene = prob.get_scene()

    assert(scene.is_state_valid(True) == True)
    print('PrimitiveBox_vs_Mesh_Distance: is_state_valid(True): PASSED')

    expected_contact_1 = np.array([-0.5, 0, 0])
    expected_contact_2 = np.array([0.5, 0, 0])
    expected_normal_1 = np.array([1, 0, 0])
    expected_normal_2 = np.array([-1, 0, 0])

    p = scene.get_collision_distance("A", "B")
    debug_publish(p, scene)
    print(p)
    assert(len(p) == 1)
    assert(np.isclose(p[0].distance, 1))
    assert(np.isclose(p[0].contact_1, expected_contact_1).all())
    assert(np.isclose(p[0].contact_2, expected_contact_2).all())
    assert(np.isclose(p[0].normal_1, expected_normal_1).all())
    assert(np.isclose(p[0].normal_2, expected_normal_2).all())
    print('PrimitiveBox_vs_Mesh_Distance: Distance, Contact Points, Normals: PASSED')

    # MeshVsBox
    p = scene.get_collision_distance("B", "A")
    debug_publish(p, scene)
    assert(len(p) == 1)
    assert(np.isclose(p[0].distance, 1))
    assert(np.isclose(p[0].contact_1, expected_contact_2).all())
    assert(np.isclose(p[0].contact_2, expected_contact_1).all())
    assert(np.isclose(p[0].normal_1, expected_normal_2).all())
    assert(np.isclose(p[0].normal_2, expected_normal_1).all())
    print('Mesh_vs_PrimitiveBox_Distance: Distance, Contact Points, Normals: PASSED')


def testBoxVsMeshPenetrating(collisionScene):
    PrimitiveBox_vs_Mesh_Penetrating = get_problem_initializer(collisionScene, '{exotica_examples}/test/resources/PrimitiveBox_vs_Mesh_Penetrating.urdf')
    prob = exo.Setup.create_problem(PrimitiveBox_vs_Mesh_Penetrating)
    prob.update(np.zeros(prob.N,))
    scene = prob.get_scene()

    assert(scene.is_state_valid(True) == False)
    print('PrimitiveBox_vs_Mesh_Penetrating: is_state_valid(True): PASSED')

    expected_contact_1 = np.array([0.2, 0, 0])
    expected_contact_2 = np.array([-0.2, 0, 0])
    expected_normal_1 = np.array([-1, 0, 0])
    expected_normal_2 = np.array([1, 0, 0])

    p = scene.get_collision_distance("A", "B")
    debug_publish(p, scene)
    assert(len(p) == 1)
    print(p)
    assert(np.isclose(p[0].distance, -.2, atol=PENETRATING_DISTANCE_ATOL))
    assert(np.isclose(p[0].contact_1, expected_contact_1).all())
    assert(np.isclose(p[0].contact_2, expected_contact_2).all())
    assert(np.isclose(p[0].normal_1, expected_normal_1).all())
    assert(np.isclose(p[0].normal_2, expected_normal_2).all())
    print('PrimitiveBox_vs_Mesh_Penetrating: Distance, Contact Points, Normals: PASSED')
    
    # MeshVsBox
    p = scene.get_collision_distance("B", "A")
    debug_publish(p, scene)
    assert(len(p) == 1)
    print(p)
    assert(np.isclose(p[0].distance, -.2, atol=PENETRATING_DISTANCE_ATOL))
    assert(np.isclose(p[0].contact_1, expected_contact_2).all())
    assert(np.isclose(p[0].contact_2, expected_contact_1).all())
    assert(np.isclose(p[0].normal_1, expected_normal_2).all())
    assert(np.isclose(p[0].normal_2, expected_normal_1).all())
    print('Mesh_vs_PrimitiveBox_Penetrating: Distance, Contact Points, Normals: PASSED')


def testCylinderVsCylinderDistance(collisionScene):
    PrimitiveCylinder_vs_PrimitiveCylinder_Distance = get_problem_initializer(collisionScene, '{exotica_examples}/test/resources/PrimitiveCylinder_vs_PrimitiveCylinder_Distance.urdf')
    prob = exo.Setup.create_problem(PrimitiveCylinder_vs_PrimitiveCylinder_Distance)
    prob.update(np.zeros(prob.N,))
    scene = prob.get_scene()

    assert(scene.is_state_valid(True) == True)
    print('PrimitiveCylinder_vs_PrimitiveCylinder_Distance: is_state_valid(True): PASSED')

    p = scene.get_collision_distance("A", "B")
    debug_publish(p, scene)
    assert(len(p) == 1)
    assert(np.isclose(p[0].distance, 1.))
    expected_contact_1 = np.array([-0.5, 0, 0])
    expected_contact_2 = np.array([0.5, 0, 0])
    expected_normal_1 = np.array([1, 0, 0])
    expected_normal_2 = np.array([-1, 0, 0])
    assert(np.isclose(p[0].contact_1[0], expected_contact_1[0])) # TODO: face center issue
    assert(np.isclose(p[0].contact_2[0], expected_contact_2[0])) # TODO: face center issue
    assert(np.isclose(p[0].normal_1, expected_normal_1).all())
    assert(np.isclose(p[0].normal_2, expected_normal_2).all())
    print('PrimitiveCylinder_vs_PrimitiveCylinder_Distance: Distance, Contact Points, Normals: PASSED')


def testCylinderVsCylinderPenetrating(collisionScene):
    PrimitiveCylinder_vs_PrimitiveCylinder_Penetrating = get_problem_initializer(collisionScene, '{exotica_examples}/test/resources/PrimitiveCylinder_vs_PrimitiveCylinder_Penetrating.urdf')
    prob = exo.Setup.create_problem(PrimitiveCylinder_vs_PrimitiveCylinder_Penetrating)
    prob.update(np.zeros(prob.N,))
    scene = prob.get_scene()

    assert(scene.is_state_valid(True) == False)
    print('PrimitiveCylinder_vs_PrimitiveCylinder_Penetrating: is_state_valid(True): PASSED')

    p = scene.get_collision_distance("A", "B")
    debug_publish(p, scene)
    assert(len(p) == 1)
    assert(np.isclose(p[0].distance, -.2, atol=PENETRATING_DISTANCE_ATOL))
    expected_contact_1 = np.array([0.1, 0, 0])
    expected_contact_2 = np.array([-0.1, 0, 0])
    expected_normal_1 = np.array([-1, 0, 0])
    expected_normal_2 = np.array([1, 0, 0])
    assert(np.isclose(p[0].contact_1, expected_contact_1, atol=AFAR_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].contact_2, expected_contact_2, atol=AFAR_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].normal_1, expected_normal_1, atol=AFAR_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].normal_2, expected_normal_2, atol=AFAR_DISTANCE_ATOL).all())
    print('PrimitiveCylinder_vs_PrimitiveCylinder_Penetrating: Distance, Contact Points, Normals: PASSED')


def testCylinderVsMeshDistance(collisionScene):
    PrimitiveCylinder_vs_Mesh_Distance = get_problem_initializer(collisionScene, '{exotica_examples}/test/resources/PrimitiveCylinder_vs_Mesh_Distance.urdf')
    prob = exo.Setup.create_problem(PrimitiveCylinder_vs_Mesh_Distance)
    prob.update(np.zeros(prob.N,))
    scene = prob.get_scene()

    assert(scene.is_state_valid(True) == True)
    print('PrimitiveCylinder_vs_Mesh_Distance: is_state_valid(True): PASSED')

    expected_contact_1 = np.array([-1, 0, 0])
    expected_contact_2 = np.array([0.5, 0, 0])
    expected_normal_1 = np.array([1, 0, 0])
    expected_normal_2 = np.array([-1, 0, 0])

    p = scene.get_collision_distance("A", "B")
    debug_publish(p, scene)
    print(p)
    assert(len(p) == 1)
    assert(np.isclose(p[0].distance, 1.5))
    assert(np.isclose(p[0].contact_1, expected_contact_1).all())
    assert(np.isclose(p[0].contact_2, expected_contact_2).all())
    assert(np.isclose(p[0].normal_1, expected_normal_1).all())
    assert(np.isclose(p[0].normal_2, expected_normal_2).all())
    print('PrimitiveCylinder_vs_Mesh_Distance: Distance, Contact Points, Normals: PASSED')

    # Mesh vs Cylinder
    p = scene.get_collision_distance("B", "A")
    debug_publish(p, scene)
    assert(len(p) == 1)
    assert(np.isclose(p[0].distance, 1.5))
    assert(np.isclose(p[0].contact_1, expected_contact_2).all())
    assert(np.isclose(p[0].contact_2, expected_contact_1).all())
    assert(np.isclose(p[0].normal_1, expected_normal_2).all())
    assert(np.isclose(p[0].normal_2, expected_normal_1).all())
    print('Mesh_vs_PrimitiveCylinder_vs_Distance: Distance, Contact Points, Normals: PASSED')


def testCylinderVsMeshPenetrating(collisionScene):
    PrimitiveCylinder_vs_Mesh_Penetrating = get_problem_initializer(collisionScene, '{exotica_examples}/test/resources/PrimitiveCylinder_vs_Mesh_Penetrating.urdf')
    prob = exo.Setup.create_problem(PrimitiveCylinder_vs_Mesh_Penetrating)
    prob.update(np.zeros(prob.N,))
    scene = prob.get_scene()

    assert(scene.is_state_valid(True) == False)
    print('PrimitiveCylinder_vs_Mesh_Penetrating: is_state_valid(True): PASSED')

    expected_contact_1 = np.array([0.2, 0, 0])
    expected_contact_2 = np.array([-0.2, 0, 0])
    expected_normal_1 = np.array([-1, 0, 0])
    expected_normal_2 = np.array([1, 0, 0])

    p = scene.get_collision_distance("A", "B")
    debug_publish(p, scene)
    assert(len(p) == 1)
    print(p)
    assert(np.isclose(p[0].distance, -.1, atol=PENETRATING_DISTANCE_ATOL))
    assert(np.isclose(p[0].contact_1, expected_contact_1).all())
    assert(np.isclose(p[0].contact_2, expected_contact_2).all())
    assert(np.isclose(p[0].normal_1, expected_normal_1).all())
    assert(np.isclose(p[0].normal_2, expected_normal_2).all())
    print('PrimitiveCylinder_vs_Mesh_Penetrating: Distance, Contact Points, Normals: PASSED')

    # Mesh vs Cylinder
    p = scene.get_collision_distance("B", "A")
    debug_publish(p, scene)
    assert(len(p) == 1)
    print(p)
    assert(np.isclose(p[0].distance, -.1, atol=PENETRATING_DISTANCE_ATOL))
    assert(np.isclose(p[0].contact_1, expected_contact_2).all())
    assert(np.isclose(p[0].contact_2, expected_contact_1).all())
    assert(np.isclose(p[0].normal_1, expected_normal_2).all())
    assert(np.isclose(p[0].normal_2, expected_normal_1).all())
    print('Mesh_vs_PrimitiveCylinder_Penetrating: Distance, Contact Points, Normals: PASSED')


def testMeshVsMeshDistance(collisionScene):
    Mesh_vs_Mesh_Distance = get_problem_initializer(collisionScene, '{exotica_examples}/test/resources/Mesh_vs_Mesh_Distance.urdf')
    prob = exo.Setup.create_problem(Mesh_vs_Mesh_Distance)
    prob.update(np.zeros(prob.N,))
    scene = prob.get_scene()

    assert(scene.is_state_valid(True) == True)
    print('Mesh_vs_Mesh_Distance: is_state_valid(True): PASSED')

    p = scene.get_collision_distance("A", "B")
    debug_publish(p, scene)
    assert(len(p) == 1)
    assert(np.isclose(p[0].distance, 1))
    expected_contact_1 = np.array([-0.5, 0, 0])
    expected_contact_2 = np.array([0.5, 0, 0])
    expected_normal_1 = np.array([1, 0, 0])
    expected_normal_2 = np.array([-1, 0, 0])
    assert(np.isclose(p[0].contact_1, expected_contact_1, atol=CLOSE_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].contact_2, expected_contact_2, atol=CLOSE_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].normal_1, expected_normal_1, atol=CLOSE_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].normal_2, expected_normal_2, atol=CLOSE_DISTANCE_ATOL).all())
    print('Mesh_vs_Mesh_Distance: Distance, Contact Points, Normals: PASSED')


def testMeshVsMeshPenetrating(collisionScene):
    Mesh_vs_Mesh_Penetrating = get_problem_initializer(collisionScene, '{exotica_examples}/test/resources/Mesh_vs_Mesh_Penetrating.urdf')
    prob = exo.Setup.create_problem(Mesh_vs_Mesh_Penetrating)
    prob.update(np.zeros(prob.N,))
    scene = prob.get_scene()

    assert(scene.is_state_valid(True) == False)
    print('Mesh_vs_Mesh_Penetrating: is_state_valid(True): PASSED')

    p = scene.get_collision_distance("A", "B")
    debug_publish(p, scene)
    assert(len(p) == 1)
    print(p)
    assert(np.isclose(p[0].distance, -.1, atol=PENETRATING_DISTANCE_ATOL))
    expected_contact_1 = np.array([0.2, 0, 0])
    expected_contact_2 = np.array([-0.2, 0, 0])
    expected_normal_1 = np.array([-1, 0, 0])
    expected_normal_2 = np.array([1, 0, 0])
    assert(np.isclose(p[0].contact_1, expected_contact_1).all())
    assert(np.isclose(p[0].contact_2, expected_contact_2).all())
    assert(np.isclose(p[0].normal_1, expected_normal_1).all())
    assert(np.isclose(p[0].normal_2, expected_normal_2).all())
    print('Mesh_vs_Mesh_Penetrating: Distance, Contact Points, Normals: PASSED')


#########################################

# Cf. Issue #364 for tracking deactivated tests.
class TestClass(unittest.TestCase):
    collisionScene = 'CollisionSceneFCLLatest'
    def test_testSphereVsSphereDistance(self):
        testSphereVsSphereDistance(TestClass.collisionScene)

    def test_testSphereVsSpherePenetrating(self):
        testSphereVsSpherePenetrating(TestClass.collisionScene)

    def test_testSphereVsBoxDistance(self):
        testSphereVsBoxDistance(TestClass.collisionScene)

    def test_testSphereVsBoxTouching(self):
        testSphereVsBoxTouching(TestClass.collisionScene)

    def test_testSphereVsBoxPenetrating(self):
        testSphereVsBoxPenetrating(TestClass.collisionScene)

    def test_testSphereVsCylinderDistance(self):
        testSphereVsCylinderDistance(TestClass.collisionScene)

    def test_testSphereVsCylinderPenetrating(self):
        testSphereVsCylinderPenetrating(TestClass.collisionScene)

#     def test_testSphereVsMeshDistance(self):
#         testSphereVsMeshDistance(TestClass.collisionScene)  # includes mesh vs sphere (distance OK, points not)

#     def test_testSphereVsMeshPenetrating(self):
#         testSphereVsMeshPenetrating(TestClass.collisionScene)   # BROKEN with libccd (not implemented)

    def test_testBoxVsBoxDistance(self):
        testBoxVsBoxDistance(TestClass.collisionScene)

    def test_testBoxVsBoxTouching(self):
        testBoxVsBoxTouching(TestClass.collisionScene)

    def test_testBoxVsBoxPenetrating(self):
        testBoxVsBoxPenetrating(TestClass.collisionScene)

    def test_testBoxVsCylinderDistance(self):
        testBoxVsCylinderDistance(TestClass.collisionScene)

    def test_testBoxVsCylinderPenetrating(self):
        testBoxVsCylinderPenetrating(TestClass.collisionScene)

    def test_testBoxVsMeshDistance(self):
        testBoxVsMeshDistance(TestClass.collisionScene)  # includes mesh vs box

#     def test_testBoxVsMeshPenetrating(self):
#         testBoxVsMeshPenetrating(collisionScene)   # BROKEN with libccd (not implemented)

    def test_testCylinderVsCylinderDistance(self):
        testCylinderVsCylinderDistance(TestClass.collisionScene)

    def test_testCylinderVsCylinderPenetrating(self):
        testCylinderVsCylinderPenetrating(TestClass.collisionScene)

    def test_testCylinderVsMeshDistance(self):
        testCylinderVsMeshDistance(TestClass.collisionScene)  # includes mesh vs cylinder

#     def test_testCylinderVsMeshPenetrating(self):
#         testCylinderVsMeshPenetrating(collisionScene)   # BROKEN with libccd (not implemented)

    def test_testMeshVsMeshDistance(self):
        testMeshVsMeshDistance(TestClass.collisionScene)

#     def test_testMeshVsMeshPenetrating(self):
#         testMeshVsMeshPenetrating(collisionScene)    # BROKEN with libccd (very inaccurate distance)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'TestCollisionSceneDistance', TestClass)