#!/usr/bin/env python

import pyexotica as exo
PKG = 'exotica_examples'
import roslib; roslib.load_manifest(PKG)  # This line is not needed with Catkin.

import unittest

def testScene(problemName, cleanScene=False):
    print('Loading '+problemName)
    (sol, prob)=exo.Initializers.loadXMLFull('{exotica_examples}/test/resources/test_scene.xml', problem_name=problemName)
    problem = exo.Setup.createProblem(prob)
    solver = exo.Setup.createSolver(sol)
    solver.specifyProblem(problem)

    print('Solving original scene ...')
    problem.update(problem.startState)

    print('Updating the scene ...')
    if cleanScene:
        problem.getScene().cleanScene()
        problem.getScene().loadSceneFile('{exotica_examples}/resources/scenes/example_moving_obstacle.scene')
    problem.getScene().loadSceneFile('{exotica_examples}/resources/scenes/example_distance.scene')
    print('Solving with updated scene ...')

    problem.update(problem.startState)
    print('Done')

class TestClass(unittest.TestCase):
    def test_MinimalProblem(self):
        testScene('MinimalProblem')
        
    def test_TrajectoryProblem(self):
        testScene('TrajectoryProblem')

    def test_CollisionProblem(self):
        testScene('CollisionProblem')

    def test_CustomLink1Problem(self):
        testScene('CustomLink1Problem')

    def test_CustomLink2Problem(self):
        testScene('CustomLink2Problem')

    def test_CustomLink3Problem(self):
        testScene('CustomLink3Problem')

    def test_FullProblem(self):
        testScene('FullProblem')

    def test_clean_MinimalProblem(self):
        testScene('MinimalProblem', True)

    def test_clean_MinimalProblem(self):
        testScene('MinimalProblem', True)

    def test_clean_TrajectoryProblem(self):
        testScene('TrajectoryProblem', True)

    def test_clean_CollisionProblem(self):
        testScene('CollisionProblem', True)

    def test_clean_CustomLink1Problem(self):
        testScene('CustomLink1Problem', True)

    def test_clean_CustomLink2Problem(self):
        testScene('CustomLink2Problem', True)

    def test_clean_CustomLink3Problem(self):
        testScene('CustomLink3Problem', True)

    def test_clean_FullProblem(self):
        testScene('FullProblem', True)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'TestSceneCreation', TestClass)