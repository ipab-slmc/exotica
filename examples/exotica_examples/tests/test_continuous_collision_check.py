#!/usr/bin/env python
from __future__ import print_function
import pyexotica as exo
import math
import time
import numpy as np

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

collisionScene = "CollisionSceneFCLLatest"

urdfs_to_test = ['{exotica_examples}/tests/resources/PrimitiveSphere_vs_PrimitiveSphere_Distance.urdf', '{exotica_examples}/tests/resources/Mesh_vs_Mesh_Distance.urdf']

for urdf in urdfs_to_test:
    print("Testing", urdf)
    
    initializer = getProblemInitializer(collisionScene, urdf)
    prob = exo.Setup.createProblem(initializer)
    prob.update(np.zeros(prob.N,))
    scene = prob.getScene()
    cs = scene.getCollisionScene()

    # Should collide at -2
    p = cs.continuousCollisionCheck(
            "A_collision_0", exo.KDLFrame([-3., 0.0, 0.0]), exo.KDLFrame([-1.0, 0.0, 0.0]),
            "B_collision_0", exo.KDLFrame([0, 0, 0]), exo.KDLFrame([0, 0, 0]))
    assert(p.InCollision == True)
    assert((p.TimeOfContact - 0.5) < 0.1)
    assert(np.isclose(p.ContactTransform1.getTranslation(), np.array([-2, 0, 0]), atol=0.15).all())
    assert(np.isclose(p.ContactTransform2.getTranslation(), np.array([0, 0, 0])).all())
    print(p)

print('>>SUCCESS<<')
exit(0)
