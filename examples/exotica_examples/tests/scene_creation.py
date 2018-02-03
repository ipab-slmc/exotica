#!/usr/bin/env python

import pyexotica as exo

def testScene(problemName, cleanScene=False):
    print('Loading '+problemName)
    (sol, prob)=exo.Initializers.loadXMLFull('{exotica_examples}/resources/configs/test_scene.xml', problem_name=problemName)
    problem = exo.Setup.createProblem(prob)
    solver = exo.Setup.createSolver(sol)
    solver.specifyProblem(problem)

    print('Solving original scene ...')
    problem.update(problem.startState)

    print('Updating the scene ...')
    if cleanScene:
        problem.getScene().cleanScene()
        problem.getScene().loadSceneFile('{exotica_examples}/resources/example_moving_obstacle.scene');
        problem.getScene().loadSceneFile('{exotica_examples}/resources/example_distance.scene');
    else:
        problem.getScene().loadSceneFile('{exotica_examples}/resources/example_distance.scene');
    print('Solving with updated scene ...')

    problem.update(problem.startState)
    print('Done')

testScene('MinimalProblem')
testScene('TrajectoryProblem')
testScene('CollisionProblem')
testScene('CustomLink1Problem')
testScene('CustomLink2Problem')
testScene('CustomLink3Problem')
testScene('FullProblem')

testScene('MinimalProblem', True)
testScene('MinimalProblem', True)
testScene('TrajectoryProblem', True)
testScene('CollisionProblem', True)
testScene('CustomLink1Problem', True)
testScene('CustomLink2Problem', True)
testScene('CustomLink3Problem', True)
testScene('FullProblem', True)

print('>>SUCCESS<<')
