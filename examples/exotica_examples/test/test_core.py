#!/usr/bin/env python
import roslib
import unittest
PKG = 'exotica_examples'
roslib.load_manifest(PKG)  # This line is not needed with Catkin.


def TestImport():
    global exo
    import pyexotica as exo


def TestSetup():
    global exo
    exo.Setup()


def TestGetters():
    global exo
    print(exo.Setup().get_solvers())
    print(exo.Setup().get_problems())
    print(exo.Setup().get_maps())
    exo.Setup().get_initializers()
    print(exo.Setup().get_package_path('exotica_python'))


def TestROS():
    global exo
    exo.Setup().init_ros()


def TestLoadXML():
    global exo
    (sol, prob) = exo.Initializers.load_xml_full(exo.Setup.get_package_path(
        'exotica_examples')+'/resources/configs/ik_solver_demo.xml')
    (sol, prob) = exo.Initializers.load_xml_full(
        '{exotica_examples}/resources/configs/ik_solver_demo.xml')
    problem = exo.Setup.create_problem(prob)
    solver = exo.Setup.create_solver(sol)
    solver.specify_problem(problem)
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
