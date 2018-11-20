#!/usr/bin/env python
PKG = 'exotica_examples'
import roslib; roslib.load_manifest(PKG)  # This line is not needed with Catkin.

import unittest

def TestImport():
    global exo
    import pyexotica as exo

def TestSetup():
    global exo
    exo.Setup()

def TestGetters():
    global exo
    print(exo.Setup().getSolvers())
    print(exo.Setup().getProblems())
    print(exo.Setup().getMaps())
    exo.Setup().getInitializers()
    print(exo.Setup().getPackagePath('exotica_python'))

def TestROS():
    global exo
    exo.Setup().initRos()

def TestLoadXML():
    global exo
    (sol, prob)=exo.Initializers.loadXMLFull(exo.Setup.getPackagePath('exotica_examples')+'/resources/configs/ik_solver_demo.xml')
    (sol, prob)=exo.Initializers.loadXMLFull('{exotica_examples}/resources/configs/ik_solver_demo.xml')
    problem = exo.Setup.createProblem(prob)
    solver = exo.Setup.createSolver(sol)
    solver.specifyProblem(problem)
    solver.solve()

class TestClass(unittest.TestCase):
    def test_1_import(self):
        TestImport()

    def test_2_setup(self):
        TestSetup()

    def test_3_getters(self):
        TestGetters()

    def test_4_ros(self):
        TestROS()

    def test_5_xml(self):
        TestLoadXML()

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'TestCore', TestClass)