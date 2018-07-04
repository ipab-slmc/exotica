#!/usr/bin/env python
from __future__ import print_function
import pyexotica as exo
import math
import time
import numpy as np

exo.Setup.initRos()

PENETRATING_DISTANCE_ATOL = 1e-2
AFAR_DISTANCE_ATOL = 1e-3
publishProxies = False

def getProblemInitializer(collisionScene, URDF):
    return ('exotica/UnconstrainedEndPoseProblem',
            {'Name': 'TestProblem',
             'PlanningScene': [('exotica/Scene',
                                {'CollisionScene': collisionScene,
                                 'JointGroup': 'group1',
                                 'Name': 'TestScene',
                                 'Debug': '0',
                                 'SRDF': '{exotica_examples}/tests/resources/A_vs_B.srdf',
                                 'SetRobotDescriptionRosParams': '1',
                                 'URDF': URDF})]})


def debugPublish(p, scene):
    if publishProxies:
        scene.getSolver().publishFrames()
        scene.publishProxies(p)
        time.sleep(.5)


def testSphereVsSphereDistance(collisionScene):
    PrimitiveSphere_vs_PrimitiveSphere_Distance = getProblemInitializer(collisionScene, '{exotica_examples}/tests/resources/PrimitiveSphere_vs_PrimitiveSphere_Distance.urdf')
    prob = exo.Setup.createProblem(PrimitiveSphere_vs_PrimitiveSphere_Distance)
    prob.update(np.zeros(prob.N,))
    scene = prob.getScene()

    assert(scene.isStateValid(True) == True)
    print('PrimitiveSphere_vs_PrimitiveSphere_Distance: isStateValid(True): PASSED')

    p = scene.getCollisionDistance("A", "B")
    debugPublish(p, scene)
    assert(len(p) == 1)
    assert(np.isclose(p[0].Distance, 1.))
    expectedContact1 = np.array([-0.5, 0, 0])
    expectedContact2 = np.array([0.5, 0, 0])
    expectedNormal1 = np.array([1, 0, 0])
    expectedNormal2 = np.array([-1, 0, 0])
    assert(np.isclose(p[0].Contact1, expectedContact1).all())
    assert(np.isclose(p[0].Contact2, expectedContact2).all())
    assert(np.isclose(p[0].Normal1, expectedNormal1).all())
    assert(np.isclose(p[0].Normal2, expectedNormal2).all())
    print('PrimitiveSphere_vs_PrimitiveSphere_Distance: Distance, Contact Points, Normals: PASSED')


def testSphereVsSpherePenetrating(collisionScene):
    PrimitiveSphere_vs_PrimitiveSphere_Penetrating = getProblemInitializer(collisionScene, '{exotica_examples}/tests/resources/PrimitiveSphere_vs_PrimitiveSphere_Penetrating.urdf')
    prob = exo.Setup.createProblem(PrimitiveSphere_vs_PrimitiveSphere_Penetrating)
    prob.update(np.zeros(prob.N,))
    scene = prob.getScene()

    assert(scene.isStateValid(True) == False)
    print('PrimitiveSphere_vs_PrimitiveSphere_Penetrating: isStateValid(True): PASSED')

    p = scene.getCollisionDistance("A", "B")
    debugPublish(p, scene)
    assert(len(p) == 1)
    assert(np.isclose(p[0].Distance, -.4, atol=PENETRATING_DISTANCE_ATOL))
    expectedContact1 = np.array([0.2, 0, 0])
    expectedContact2 = np.array([-0.2, 0, 0])
    expectedNormal1 = np.array([-1, 0, 0])
    expectedNormal2 = np.array([1, 0, 0])
    assert(np.isclose(p[0].Contact1, expectedContact1, atol=PENETRATING_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].Contact2, expectedContact2, atol=PENETRATING_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].Normal1, expectedNormal1, atol=PENETRATING_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].Normal2, expectedNormal2, atol=PENETRATING_DISTANCE_ATOL).all())
    print('PrimitiveSphere_vs_PrimitiveSphere_Penetrating: Distance, Contact Points, Normals: PASSED')


def testSphereVsBoxDistance(collisionScene):
    PrimitiveSphere_vs_PrimitiveBox_Distance = getProblemInitializer(collisionScene, '{exotica_examples}/tests/resources/PrimitiveSphere_vs_PrimitiveBox_Distance.urdf')
    prob = exo.Setup.createProblem(PrimitiveSphere_vs_PrimitiveBox_Distance)
    prob.update(np.zeros(prob.N,))
    scene = prob.getScene()

    assert(scene.isStateValid(True) == True)
    print('PrimitiveSphere_vs_PrimitiveBox_Distance: isStateValid(True): PASSED')

    p = scene.getCollisionDistance("A", "B")
    debugPublish(p, scene)
    assert(len(p) == 1)
    assert(np.isclose(p[0].Distance, 1.5))
    expectedContact1 = np.array([-0.5, 0, 0])
    expectedContact2 = np.array([1, 0, 0])
    expectedNormal1 = np.array([1, 0, 0])
    expectedNormal2 = np.array([-1, 0, 0])
    assert(np.isclose(p[0].Contact1, expectedContact1, atol=AFAR_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].Contact2, expectedContact2, atol=AFAR_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].Normal1, expectedNormal1, atol=AFAR_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].Normal2, expectedNormal2, atol=AFAR_DISTANCE_ATOL).all())
    print('PrimitiveSphere_vs_PrimitiveBox_Distance: Distance, Contact Points, Normals: PASSED')


def testSphereVsBoxTouching(collisionScene):
    PrimitiveSphere_vs_PrimitiveBox_Touching = getProblemInitializer(collisionScene, '{exotica_examples}/tests/resources/PrimitiveSphere_vs_PrimitiveBox_Touching.urdf')
    prob = exo.Setup.createProblem(PrimitiveSphere_vs_PrimitiveBox_Touching)
    prob.update(np.zeros(prob.N,))
    scene = prob.getScene()

    assert(scene.isStateValid(True) == False)
    print('PrimitiveSphere_vs_PrimitiveBox_Touching: isStateValid(True): PASSED')

    p = scene.getCollisionDistance("A", "B")
    print(p)
    debugPublish(p, scene)
    assert(len(p) == 1)
    assert(np.isclose(p[0].Distance, 0.))
    expectedContact1 = np.array([0.5, 0, 0])
    expectedContact2 = np.array([-1, 0, 0])
    expectedNormal1 = np.array([-1, 0, 0])
    expectedNormal2 = np.array([1, 0, 0])
    assert(np.isclose(p[0].Contact1, expectedContact1, atol=AFAR_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].Contact2, expectedContact2, atol=AFAR_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].Normal1, expectedNormal1, atol=AFAR_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].Normal2, expectedNormal2, atol=AFAR_DISTANCE_ATOL).all())
    print('PrimitiveSphere_vs_PrimitiveBox_Touching: Distance, Contact Points, Normals: PASSED')


def testSphereVsBoxPenetrating(collisionScene):
    PrimitiveSphere_vs_PrimitiveBox_Penetrating = getProblemInitializer(collisionScene, '{exotica_examples}/tests/resources/PrimitiveSphere_vs_PrimitiveBox_Penetrating.urdf')
    prob = exo.Setup.createProblem(PrimitiveSphere_vs_PrimitiveBox_Penetrating)
    prob.update(np.zeros(prob.N,))
    scene = prob.getScene()

    assert(scene.isStateValid(True) == False)
    print('PrimitiveSphere_vs_PrimitiveBox_Penetrating: isStateValid(True): PASSED')

    p = scene.getCollisionDistance("A", "B")
    debugPublish(p, scene)
    assert(len(p) == 1)
    assert(np.isclose(p[0].Distance, -0.4, atol=PENETRATING_DISTANCE_ATOL))
    expectedContact1 = np.array([0.2, 0, 0])
    expectedContact2 = np.array([-0.2, 0, 0])
    expectedNormal1 = np.array([-1, 0, 0])
    expectedNormal2 = np.array([1, 0, 0])
    assert(np.isclose(p[0].Contact1, expectedContact1, atol=PENETRATING_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].Contact2, expectedContact2, atol=PENETRATING_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].Normal1, expectedNormal1, atol=PENETRATING_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].Normal2, expectedNormal2, atol=PENETRATING_DISTANCE_ATOL).all())
    print('PrimitiveSphere_vs_PrimitiveBox_Penetrating: Distance, Contact Points, Normals: PASSED')


def testSphereVsCylinderDistance(collisionScene):
    PrimitiveSphere_vs_PrimitiveCylinder_Distance = getProblemInitializer(collisionScene, '{exotica_examples}/tests/resources/PrimitiveSphere_vs_PrimitiveCylinder_Distance.urdf')
    prob = exo.Setup.createProblem(PrimitiveSphere_vs_PrimitiveCylinder_Distance)
    prob.update(np.zeros(prob.N,))
    scene = prob.getScene()

    assert(scene.isStateValid(True) == True)
    print('PrimitiveSphere_vs_PrimitiveCylinder_Distance: isStateValid(True): PASSED')

    p = scene.getCollisionDistance("A", "B")
    debugPublish(p, scene)
    assert(len(p) == 1)
    assert(np.isclose(p[0].Distance, 1.5))
    expectedContact1 = np.array([-0.5, 0, 0])
    expectedContact2 = np.array([1, 0, 0])
    expectedNormal1 = np.array([1, 0, 0])
    expectedNormal2 = np.array([-1, 0, 0])
    assert(np.isclose(p[0].Contact1, expectedContact1, atol=AFAR_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].Contact2, expectedContact2, atol=AFAR_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].Normal1, expectedNormal1, atol=AFAR_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].Normal2, expectedNormal2, atol=AFAR_DISTANCE_ATOL).all())
    print('PrimitiveSphere_vs_PrimitiveCylinder_Distance: Distance, Contact Points, Normals: PASSED')


def testSphereVsCylinderPenetrating(collisionScene):
    PrimitiveSphere_vs_PrimitiveCylinder_Penetrating = getProblemInitializer(collisionScene, '{exotica_examples}/tests/resources/PrimitiveSphere_vs_PrimitiveCylinder_Penetrating.urdf')
    prob = exo.Setup.createProblem(PrimitiveSphere_vs_PrimitiveCylinder_Penetrating)
    prob.update(np.zeros(prob.N,))
    scene = prob.getScene()

    assert(scene.isStateValid(True) == False)
    print('PrimitiveSphere_vs_PrimitiveCylinder_Penetrating: isStateValid(True): PASSED')

    p = scene.getCollisionDistance("A", "B")
    debugPublish(p, scene)
    assert(len(p) == 1)
    assert(np.isclose(p[0].Distance, -0.4, atol=PENETRATING_DISTANCE_ATOL))
    expectedContact1 = np.array([0.2, 0, 0])
    expectedContact2 = np.array([-0.2, 0, 0])
    expectedNormal1 = np.array([-1, 0, 0])
    expectedNormal2 = np.array([1, 0, 0])
    assert(np.isclose(p[0].Contact1, expectedContact1, atol=PENETRATING_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].Contact2, expectedContact2, atol=PENETRATING_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].Normal1, expectedNormal1, atol=PENETRATING_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].Normal2, expectedNormal2, atol=PENETRATING_DISTANCE_ATOL).all())
    print('PrimitiveSphere_vs_PrimitiveCylinder_Penetrating: Distance, Contact Points, Normals: PASSED')


def testSphereVsMeshDistance(collisionScene):
    PrimitiveSphere_vs_Mesh_Distance = getProblemInitializer(collisionScene, '{exotica_examples}/tests/resources/PrimitiveSphere_vs_Mesh_Distance.urdf')
    prob = exo.Setup.createProblem(PrimitiveSphere_vs_Mesh_Distance)
    prob.update(np.zeros(prob.N,))
    scene = prob.getScene()

    assert(scene.isStateValid(True) == True)
    print('PrimitiveSphere_vs_Mesh_Distance: isStateValid(True): PASSED')

    expectedDistance = 1
    expectedContact1 = np.array([-0.5, 0, 0])
    expectedContact2 = np.array([0.5, 0, 0])
    expectedNormal1 = np.array([1, 0, 0])
    expectedNormal2 = np.array([-1, 0, 0])

    p = scene.getCollisionDistance("A", "B")
    debugPublish(p, scene)
    assert(len(p) == 1)
    assert(np.isclose(p[0].Distance, expectedDistance))
    assert(np.isclose(p[0].Contact1, expectedContact1).all())
    assert(np.isclose(p[0].Contact2, expectedContact2).all())
    assert(np.isclose(p[0].Normal1, expectedNormal1).all())
    assert(np.isclose(p[0].Normal2, expectedNormal2).all())
    print('PrimitiveSphere_vs_Mesh_Distance: Distance, Contact Points, Normals: PASSED')

    # MeshVsSphere Test
    p = scene.getCollisionDistance("B", "A")
    debugPublish(p, scene)
    assert(len(p) == 1)
    assert(np.isclose(p[0].Distance, expectedDistance))
    assert(np.isclose(p[0].Contact1, expectedContact2).all())
    assert(np.isclose(p[0].Contact2, expectedContact1).all())
    assert(np.isclose(p[0].Normal1, expectedNormal2).all())
    assert(np.isclose(p[0].Normal2, expectedNormal1).all())
    print('Mesh_vs_PrimitiveSphere_Distance: Distance, Contact Points, Normals: PASSED')


def testSphereVsMeshPenetrating(collisionScene):
    PrimitiveSphere_vs_Mesh_Penetrating = getProblemInitializer(collisionScene, '{exotica_examples}/tests/resources/PrimitiveSphere_vs_Mesh_Penetrating.urdf')
    prob = exo.Setup.createProblem(PrimitiveSphere_vs_Mesh_Penetrating)
    prob.update(np.zeros(prob.N,))
    scene = prob.getScene()

    assert(scene.isStateValid(True) == False)
    print('PrimitiveSphere_vs_Mesh_Penetrating: isStateValid(True): PASSED')

    p = scene.getCollisionDistance("A", "B")
    print(p)
    debugPublish(p, scene)
    expectedContact1 = np.array([0.2, 0, 0])
    expectedContact2 = np.array([-0.2, 0, 0])
    expectedNormal1 = np.array([-1, 0, 0])
    expectedNormal2 = np.array([1, 0, 0])
    assert(len(p) == 1)
    assert(np.isclose(p[0].Distance, -.4, atol=PENETRATING_DISTANCE_ATOL))
    assert(np.isclose(p[0].Contact1, expectedContact1).all())
    assert(np.isclose(p[0].Contact2, expectedContact2).all())
    assert(np.isclose(p[0].Normal1, expectedNormal1).all())
    assert(np.isclose(p[0].Normal2, expectedNormal2).all())
    print('PrimitiveSphere_vs_Mesh_Penetrating: Distance, Contact Points, Normals: PASSED')

    # MeshVsSphere Test
    p = scene.getCollisionDistance("B", "A")
    print(p)
    debugPublish(p, scene)
    assert(len(p) == 1)
    assert(np.isclose(p[0].Distance, 1))
    assert(np.isclose(p[0].Contact1, expectedContact2).all())
    assert(np.isclose(p[0].Contact2, expectedContact1).all())
    assert(np.isclose(p[0].Normal1, expectedNormal2).all())
    assert(np.isclose(p[0].Normal2, expectedNormal1).all())
    print('Mesh_vs_PrimitiveSphere_Penetrating: Distance, Contact Points, Normals: PASSED')


def testBoxVsBoxDistance(collisionScene):
    PrimitiveBox_vs_PrimitiveBox_Distance = getProblemInitializer(collisionScene, '{exotica_examples}/tests/resources/PrimitiveBox_vs_PrimitiveBox_Distance.urdf')
    prob = exo.Setup.createProblem(PrimitiveBox_vs_PrimitiveBox_Distance)
    prob.update(np.zeros(prob.N,))
    scene = prob.getScene()

    assert(scene.isStateValid(True) == True)
    print('PrimitiveBox_vs_PrimitiveBox_Distance: isStateValid(True): PASSED')

    p = scene.getCollisionDistance("A", "B")
    debugPublish(p, scene)
    assert(len(p) == 1)
    assert(np.isclose(p[0].Distance, 1.))
    expectedContact1 = np.array([-0.5, 0, 0])
    expectedContact2 = np.array([0.5, 0, 0])
    expectedNormal1 = np.array([1, 0, 0])
    expectedNormal2 = np.array([-1, 0, 0])
    # print(p) # Center of face problem!
    # assert(np.isclose(p[0].Contact1, expectedContact1, atol=AFAR_DISTANCE_ATOL).all())
    # assert(np.isclose(p[0].Contact2, expectedContact2, atol=AFAR_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].Contact1[0], expectedContact1[0], atol=AFAR_DISTANCE_ATOL)) # TODO: face center issue
    assert(np.isclose(p[0].Contact2[0], expectedContact2[0], atol=AFAR_DISTANCE_ATOL)) # TODO: face center issue
    assert(np.isclose(p[0].Normal1, expectedNormal1, atol=AFAR_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].Normal2, expectedNormal2, atol=AFAR_DISTANCE_ATOL).all())
    print('PrimitiveBox_vs_PrimitiveBox_Distance: Distance, Contact Points, Normals: PASSED')


def testBoxVsBoxTouching(collisionScene):
    PrimitiveBox_vs_PrimitiveBox_Touching = getProblemInitializer(collisionScene, '{exotica_examples}/tests/resources/PrimitiveBox_vs_PrimitiveBox_Touching.urdf')
    prob = exo.Setup.createProblem(PrimitiveBox_vs_PrimitiveBox_Touching)
    prob.update(np.zeros(prob.N,))
    scene = prob.getScene()

    assert(scene.isStateValid(True) == False)
    print('PrimitiveBox_vs_PrimitiveBox_Touching: isStateValid(True): PASSED')

    p = scene.getCollisionDistance("A", "B")
    print(p)
    debugPublish(p, scene)
    assert(len(p) == 1)
    assert(np.isclose(p[0].Distance, 0.))
    expectedContact1 = np.array([0.5, 0, 0])
    expectedContact2 = np.array([-0.5, 0, 0])
    expectedNormal1 = np.array([-1, 0, 0])
    expectedNormal2 = np.array([1, 0, 0])
    assert(np.isclose(p[0].Contact1, expectedContact1).all())
    assert(np.isclose(p[0].Contact2, expectedContact2).all())
    assert(np.isclose(p[0].Normal1, expectedNormal1).all())
    assert(np.isclose(p[0].Normal2, expectedNormal2).all())
    print('PrimitiveBox_vs_PrimitiveBox_Touching: Distance, Contact Points, Normals: PASSED')


def testBoxVsBoxPenetrating(collisionScene):
    PrimitiveBox_vs_PrimitiveBox_Penetrating = getProblemInitializer(collisionScene, '{exotica_examples}/tests/resources/PrimitiveBox_vs_PrimitiveBox_Penetrating.urdf')
    prob = exo.Setup.createProblem(PrimitiveBox_vs_PrimitiveBox_Penetrating)
    prob.update(np.zeros(prob.N,))
    scene = prob.getScene()

    assert(scene.isStateValid(True) == False)
    print('PrimitiveBox_vs_PrimitiveBox_Penetrating: isStateValid(True): PASSED')

    p = scene.getCollisionDistance("A", "B")
    debugPublish(p, scene)
    assert(len(p) == 1)
    assert(np.isclose(p[0].Distance, -.2))
    expectedContact1 = np.array([0.1, 0, 0])
    expectedContact2 = np.array([-0.1, 0, 0])
    expectedNormal1 = np.array([-1, 0, 0])
    expectedNormal2 = np.array([1, 0, 0])
    assert(np.isclose(p[0].Contact1, expectedContact1).all())
    assert(np.isclose(p[0].Contact2, expectedContact2).all())
    assert(np.isclose(p[0].Normal1, expectedNormal1).all())
    assert(np.isclose(p[0].Normal2, expectedNormal2).all())
    print('PrimitiveBox_vs_PrimitiveBox_Penetrating: Distance, Contact Points, Normals: PASSED')


def testBoxVsCylinderDistance(collisionScene):
    PrimitiveBox_vs_PrimitiveCylinder_Distance = getProblemInitializer(collisionScene, '{exotica_examples}/tests/resources/PrimitiveBox_vs_PrimitiveCylinder_Distance.urdf')
    prob = exo.Setup.createProblem(PrimitiveBox_vs_PrimitiveCylinder_Distance)
    prob.update(np.zeros(prob.N,))
    scene = prob.getScene()

    assert(scene.isStateValid(True) == True)
    print('PrimitiveBox_vs_PrimitiveCylinder_Distance: isStateValid(True): PASSED')

    p = scene.getCollisionDistance("A", "B")
    debugPublish(p, scene)
    assert(len(p) == 1)
    assert(np.isclose(p[0].Distance, 1.))
    expectedContact1 = np.array([-0.5, 0, 0])
    expectedContact2 = np.array([0.5, 0, 0])
    expectedNormal1 = np.array([1, 0, 0])
    expectedNormal2 = np.array([-1, 0, 0])
    assert(np.isclose(p[0].Contact1[0], expectedContact1[0])) # TODO: face center issue
    assert(np.isclose(p[0].Contact2[0], expectedContact2[0])) # TODO: face center issue
    assert(np.isclose(p[0].Normal1, expectedNormal1).all())
    assert(np.isclose(p[0].Normal2, expectedNormal2).all())
    print('PrimitiveBox_vs_PrimitiveCylinder_Distance: Distance, Contact Points, Normals: PASSED')


def testBoxVsCylinderPenetrating(collisionScene):
    PrimitiveBox_vs_PrimitiveCylinder_Penetrating = getProblemInitializer(collisionScene, '{exotica_examples}/tests/resources/PrimitiveBox_vs_PrimitiveCylinder_Penetrating.urdf')
    prob = exo.Setup.createProblem(PrimitiveBox_vs_PrimitiveCylinder_Penetrating)
    prob.update(np.zeros(prob.N,))
    scene = prob.getScene()

    assert(scene.isStateValid(True) == False)
    print('PrimitiveBox_vs_PrimitiveCylinder_Penetrating: isStateValid(True): PASSED')

    p = scene.getCollisionDistance("A", "B")
    debugPublish(p, scene)
    assert(len(p) == 1)
    assert(np.isclose(p[0].Distance, -.2, atol=PENETRATING_DISTANCE_ATOL))
    expectedContact1 = np.array([0.1, 0, 0])
    expectedContact2 = np.array([-0.1, 0, 0])
    expectedNormal1 = np.array([-1, 0, 0])
    expectedNormal2 = np.array([1, 0, 0])
    assert(np.isclose(p[0].Contact1, expectedContact1, atol=PENETRATING_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].Contact2, expectedContact2, atol=PENETRATING_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].Normal1, expectedNormal1, atol=1.5*PENETRATING_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].Normal2, expectedNormal2, atol=1.5*PENETRATING_DISTANCE_ATOL).all())
    print('PrimitiveBox_vs_PrimitiveCylinder_Penetrating: Distance, Contact Points, Normals: PASSED')


def testBoxVsMeshDistance(collisionScene):
    PrimitiveBox_vs_Mesh_Distance = getProblemInitializer(collisionScene, '{exotica_examples}/tests/resources/PrimitiveBox_vs_Mesh_Distance.urdf')
    prob = exo.Setup.createProblem(PrimitiveBox_vs_Mesh_Distance)
    prob.update(np.zeros(prob.N,))
    scene = prob.getScene()

    assert(scene.isStateValid(True) == True)
    print('PrimitiveBox_vs_Mesh_Distance: isStateValid(True): PASSED')

    expectedContact1 = np.array([-0.5, 0, 0])
    expectedContact2 = np.array([0.5, 0, 0])
    expectedNormal1 = np.array([1, 0, 0])
    expectedNormal2 = np.array([-1, 0, 0])

    p = scene.getCollisionDistance("A", "B")
    debugPublish(p, scene)
    assert(len(p) == 1)
    assert(np.isclose(p[0].Distance, 1))
    assert(np.isclose(p[0].Contact1, expectedContact1).all())
    assert(np.isclose(p[0].Contact2, expectedContact2).all())
    assert(np.isclose(p[0].Normal1, expectedNormal1).all())
    assert(np.isclose(p[0].Normal2, expectedNormal2).all())
    print('PrimitiveBox_vs_Mesh_Distance: Distance, Contact Points, Normals: PASSED')

    # MeshVsBox
    p = scene.getCollisionDistance("B", "A")
    debugPublish(p, scene)
    assert(len(p) == 1)
    assert(np.isclose(p[0].Distance, 1))
    assert(np.isclose(p[0].Contact1, expectedContact2).all())
    assert(np.isclose(p[0].Contact2, expectedContact1).all())
    assert(np.isclose(p[0].Normal1, expectedNormal2).all())
    assert(np.isclose(p[0].Normal2, expectedNormal1).all())
    print('Mesh_vs_PrimitiveBox_Distance: Distance, Contact Points, Normals: PASSED')


def testBoxVsMeshPenetrating(collisionScene):
    PrimitiveBox_vs_Mesh_Penetrating = getProblemInitializer(collisionScene, '{exotica_examples}/tests/resources/PrimitiveBox_vs_Mesh_Penetrating.urdf')
    prob = exo.Setup.createProblem(PrimitiveBox_vs_Mesh_Penetrating)
    prob.update(np.zeros(prob.N,))
    scene = prob.getScene()

    assert(scene.isStateValid(True) == False)
    print('PrimitiveBox_vs_Mesh_Penetrating: isStateValid(True): PASSED')

    expectedContact1 = np.array([0.2, 0, 0])
    expectedContact2 = np.array([-0.2, 0, 0])
    expectedNormal1 = np.array([-1, 0, 0])
    expectedNormal2 = np.array([1, 0, 0])

    p = scene.getCollisionDistance("A", "B")
    debugPublish(p, scene)
    assert(len(p) == 1)
    print(p)
    assert(np.isclose(p[0].Distance, -.2, atol=PENETRATING_DISTANCE_ATOL))
    assert(np.isclose(p[0].Contact1, expectedContact1).all())
    assert(np.isclose(p[0].Contact2, expectedContact2).all())
    assert(np.isclose(p[0].Normal1, expectedNormal1).all())
    assert(np.isclose(p[0].Normal2, expectedNormal2).all())
    print('PrimitiveBox_vs_Mesh_Penetrating: Distance, Contact Points, Normals: PASSED')
    
    # MeshVsBox
    p = scene.getCollisionDistance("B", "A")
    debugPublish(p, scene)
    assert(len(p) == 1)
    print(p)
    assert(np.isclose(p[0].Distance, -.2, atol=PENETRATING_DISTANCE_ATOL))
    assert(np.isclose(p[0].Contact1, expectedContact2).all())
    assert(np.isclose(p[0].Contact2, expectedContact1).all())
    assert(np.isclose(p[0].Normal1, expectedNormal2).all())
    assert(np.isclose(p[0].Normal2, expectedNormal1).all())
    print('Mesh_vs_PrimitiveBox_Penetrating: Distance, Contact Points, Normals: PASSED')


def testCylinderVsCylinderDistance(collisionScene):
    PrimitiveCylinder_vs_PrimitiveCylinder_Distance = getProblemInitializer(collisionScene, '{exotica_examples}/tests/resources/PrimitiveCylinder_vs_PrimitiveCylinder_Distance.urdf')
    prob = exo.Setup.createProblem(PrimitiveCylinder_vs_PrimitiveCylinder_Distance)
    prob.update(np.zeros(prob.N,))
    scene = prob.getScene()

    assert(scene.isStateValid(True) == True)
    print('PrimitiveCylinder_vs_PrimitiveCylinder_Distance: isStateValid(True): PASSED')

    p = scene.getCollisionDistance("A", "B")
    debugPublish(p, scene)
    assert(len(p) == 1)
    assert(np.isclose(p[0].Distance, 1.))
    expectedContact1 = np.array([-0.5, 0, 0])
    expectedContact2 = np.array([0.5, 0, 0])
    expectedNormal1 = np.array([1, 0, 0])
    expectedNormal2 = np.array([-1, 0, 0])
    assert(np.isclose(p[0].Contact1[0], expectedContact1[0])) # TODO: face center issue
    assert(np.isclose(p[0].Contact2[0], expectedContact2[0])) # TODO: face center issue
    assert(np.isclose(p[0].Normal1, expectedNormal1).all())
    assert(np.isclose(p[0].Normal2, expectedNormal2).all())
    print('PrimitiveCylinder_vs_PrimitiveCylinder_Distance: Distance, Contact Points, Normals: PASSED')


def testCylinderVsCylinderPenetrating(collisionScene):
    PrimitiveCylinder_vs_PrimitiveCylinder_Penetrating = getProblemInitializer(collisionScene, '{exotica_examples}/tests/resources/PrimitiveCylinder_vs_PrimitiveCylinder_Penetrating.urdf')
    prob = exo.Setup.createProblem(PrimitiveCylinder_vs_PrimitiveCylinder_Penetrating)
    prob.update(np.zeros(prob.N,))
    scene = prob.getScene()

    assert(scene.isStateValid(True) == False)
    print('PrimitiveCylinder_vs_PrimitiveCylinder_Penetrating: isStateValid(True): PASSED')

    p = scene.getCollisionDistance("A", "B")
    debugPublish(p, scene)
    assert(len(p) == 1)
    assert(np.isclose(p[0].Distance, -.2, atol=PENETRATING_DISTANCE_ATOL))
    expectedContact1 = np.array([0.1, 0, 0])
    expectedContact2 = np.array([-0.1, 0, 0])
    expectedNormal1 = np.array([-1, 0, 0])
    expectedNormal2 = np.array([1, 0, 0])
    assert(np.isclose(p[0].Contact1, expectedContact1, atol=1.5*PENETRATING_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].Contact2, expectedContact2, atol=1.5*PENETRATING_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].Normal1, expectedNormal1, atol=1.5*PENETRATING_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].Normal2, expectedNormal2, atol=1.5*PENETRATING_DISTANCE_ATOL).all())
    print('PrimitiveCylinder_vs_PrimitiveCylinder_Penetrating: Distance, Contact Points, Normals: PASSED')


def testCylinderVsMeshDistance(collisionScene):
    PrimitiveCylinder_vs_Mesh_Distance = getProblemInitializer(collisionScene, '{exotica_examples}/tests/resources/PrimitiveCylinder_vs_Mesh_Distance.urdf')
    prob = exo.Setup.createProblem(PrimitiveCylinder_vs_Mesh_Distance)
    prob.update(np.zeros(prob.N,))
    scene = prob.getScene()

    assert(scene.isStateValid(True) == True)
    print('PrimitiveCylinder_vs_Mesh_Distance: isStateValid(True): PASSED')

    expectedContact1 = np.array([-1, 0, 0])
    expectedContact2 = np.array([0.5, 0, 0])
    expectedNormal1 = np.array([1, 0, 0])
    expectedNormal2 = np.array([-1, 0, 0])

    p = scene.getCollisionDistance("A", "B")
    debugPublish(p, scene)
    assert(len(p) == 1)
    assert(np.isclose(p[0].Distance, 1.5))
    assert(np.isclose(p[0].Contact1, expectedContact1).all())
    assert(np.isclose(p[0].Contact2, expectedContact2).all())
    assert(np.isclose(p[0].Normal1, expectedNormal1).all())
    assert(np.isclose(p[0].Normal2, expectedNormal2).all())
    print('PrimitiveCylinder_vs_Mesh_Distance: Distance, Contact Points, Normals: PASSED')

    # Mesh vs Cylinder
    p = scene.getCollisionDistance("B", "A")
    debugPublish(p, scene)
    assert(len(p) == 1)
    assert(np.isclose(p[0].Distance, 1.5))
    assert(np.isclose(p[0].Contact1, expectedContact2).all())
    assert(np.isclose(p[0].Contact2, expectedContact1).all())
    assert(np.isclose(p[0].Normal1, expectedNormal2).all())
    assert(np.isclose(p[0].Normal2, expectedNormal1).all())
    print('Mesh_vs_PrimitiveCylinder_vs_Distance: Distance, Contact Points, Normals: PASSED')


def testCylinderVsMeshPenetrating(collisionScene):
    PrimitiveCylinder_vs_Mesh_Penetrating = getProblemInitializer(collisionScene, '{exotica_examples}/tests/resources/PrimitiveCylinder_vs_Mesh_Penetrating.urdf')
    prob = exo.Setup.createProblem(PrimitiveCylinder_vs_Mesh_Penetrating)
    prob.update(np.zeros(prob.N,))
    scene = prob.getScene()

    assert(scene.isStateValid(True) == False)
    print('PrimitiveCylinder_vs_Mesh_Penetrating: isStateValid(True): PASSED')

    expectedContact1 = np.array([0.2, 0, 0])
    expectedContact2 = np.array([-0.2, 0, 0])
    expectedNormal1 = np.array([-1, 0, 0])
    expectedNormal2 = np.array([1, 0, 0])

    p = scene.getCollisionDistance("A", "B")
    debugPublish(p, scene)
    assert(len(p) == 1)
    print(p)
    assert(np.isclose(p[0].Distance, -.1, atol=PENETRATING_DISTANCE_ATOL))
    assert(np.isclose(p[0].Contact1, expectedContact1).all())
    assert(np.isclose(p[0].Contact2, expectedContact2).all())
    assert(np.isclose(p[0].Normal1, expectedNormal1).all())
    assert(np.isclose(p[0].Normal2, expectedNormal2).all())
    print('PrimitiveCylinder_vs_Mesh_Penetrating: Distance, Contact Points, Normals: PASSED')

    # Mesh vs Cylinder
    p = scene.getCollisionDistance("B", "A")
    debugPublish(p, scene)
    assert(len(p) == 1)
    print(p)
    assert(np.isclose(p[0].Distance, -.1, atol=PENETRATING_DISTANCE_ATOL))
    assert(np.isclose(p[0].Contact1, expectedContact2).all())
    assert(np.isclose(p[0].Contact2, expectedContact1).all())
    assert(np.isclose(p[0].Normal1, expectedNormal2).all())
    assert(np.isclose(p[0].Normal2, expectedNormal1).all())
    print('Mesh_vs_PrimitiveCylinder_Penetrating: Distance, Contact Points, Normals: PASSED')


def testMeshVsMeshDistance(collisionScene):
    Mesh_vs_Mesh_Distance = getProblemInitializer(collisionScene, '{exotica_examples}/tests/resources/Mesh_vs_Mesh_Distance.urdf')
    prob = exo.Setup.createProblem(Mesh_vs_Mesh_Distance)
    prob.update(np.zeros(prob.N,))
    scene = prob.getScene()

    assert(scene.isStateValid(True) == True)
    print('Mesh_vs_Mesh_Distance: isStateValid(True): PASSED')

    p = scene.getCollisionDistance("A", "B")
    debugPublish(p, scene) ; print(p)
    assert(len(p) == 1)
    assert(np.isclose(p[0].Distance, 1))
    expectedContact1 = np.array([-0.5, 0, 0])
    expectedContact2 = np.array([0.5, 0, 0])
    expectedNormal1 = np.array([1, 0, 0])
    expectedNormal2 = np.array([-1, 0, 0])
    assert(np.isclose(p[0].Contact1, expectedContact1, atol=PENETRATING_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].Contact2, expectedContact2, atol=PENETRATING_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].Normal1, expectedNormal1, atol=PENETRATING_DISTANCE_ATOL).all())
    assert(np.isclose(p[0].Normal2, expectedNormal2, atol=PENETRATING_DISTANCE_ATOL).all())
    print('Mesh_vs_Mesh_Distance: Distance, Contact Points, Normals: PASSED')


def testMeshVsMeshPenetrating(collisionScene):
    Mesh_vs_Mesh_Penetrating = getProblemInitializer(collisionScene, '{exotica_examples}/tests/resources/Mesh_vs_Mesh_Penetrating.urdf')
    prob = exo.Setup.createProblem(Mesh_vs_Mesh_Penetrating)
    prob.update(np.zeros(prob.N,))
    scene = prob.getScene()

    assert(scene.isStateValid(True) == False)
    print('Mesh_vs_Mesh_Penetrating: isStateValid(True): PASSED')

    p = scene.getCollisionDistance("A", "B")
    debugPublish(p, scene)
    assert(len(p) == 1)
    print(p)
    assert(np.isclose(p[0].Distance, -.1, atol=PENETRATING_DISTANCE_ATOL))
    expectedContact1 = np.array([0.2, 0, 0])
    expectedContact2 = np.array([-0.2, 0, 0])
    expectedNormal1 = np.array([-1, 0, 0])
    expectedNormal2 = np.array([1, 0, 0])
    assert(np.isclose(p[0].Contact1, expectedContact1).all())
    assert(np.isclose(p[0].Contact2, expectedContact2).all())
    assert(np.isclose(p[0].Normal1, expectedNormal1).all())
    assert(np.isclose(p[0].Normal2, expectedNormal2).all())
    print('Mesh_vs_Mesh_Penetrating: Distance, Contact Points, Normals: PASSED')


#########################################

for collisionScene in ['CollisionSceneFCLLatest']:
    testSphereVsSphereDistance(collisionScene)
    testSphereVsSpherePenetrating(collisionScene)
    testSphereVsBoxDistance(collisionScene)
    testSphereVsBoxTouching(collisionScene)
    testSphereVsBoxPenetrating(collisionScene)
    testSphereVsCylinderDistance(collisionScene)
    testSphereVsCylinderPenetrating(collisionScene)
    testSphereVsMeshDistance(collisionScene)  # includes mesh vs sphere
    #testSphereVsMeshPenetrating(collisionScene)   # BROKEN with libccd
    testBoxVsBoxDistance(collisionScene)
    testBoxVsBoxTouching(collisionScene)
    testBoxVsBoxPenetrating(collisionScene)
    testBoxVsCylinderDistance(collisionScene)
    testBoxVsCylinderPenetrating(collisionScene)
    testBoxVsMeshDistance(collisionScene)  # includes mesh vs box
    #testBoxVsMeshPenetrating(collisionScene)   # BROKEN with libccd
    testCylinderVsCylinderDistance(collisionScene)
    testCylinderVsCylinderPenetrating(collisionScene)
    testCylinderVsMeshDistance(collisionScene)  # includes mesh vs cylinder
    #testCylinderVsMeshPenetrating(collisionScene)   # BROKEN with libccd
    testMeshVsMeshDistance(collisionScene)
    #testMeshVsMeshPenetrating(collisionScene)    # BROKEN with libccd

time.sleep(1)

print('>>SUCCESS<<')

exit(0)
