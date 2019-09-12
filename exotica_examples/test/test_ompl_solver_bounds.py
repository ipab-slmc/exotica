import unittest

import pyexotica as exo

class OMPLLockedBoundsCase(unittest.TestCase):

    def test_lock_bounds(self):
        # self.assertTrue(True)
        ompl = exo.Setup.load_solver('{exotica_examples}/resources/configs/example_manipulate_ompl.xml')

        # set start and goal state
        ompl.get_problem().start_state = [1.5035205538438838, 0.8730168650583787, -1.6298590879018438, 1.7106630821349438, -0.8789956712153559, 0.1278222471656531, 0.0]
        ompl.get_problem().goal_state = [-1.5035205538442702, 0.8730168650583671, 1.6298590879018415, 1.7106630821349786, 0.8789956712153525, 0.12782224716566898, 0.0]

        # 1st solve call
        solution = None
        try:
            solution = ompl.solve()
        except Exception:
            pass
        self.assertTrue(solution is not None)

        # 2nd solve call
        solution = None
        try:
            solution = ompl.solve()
        except Exception:
            pass
        self.assertTrue(solution is not None)

        # get and change current bounds
        jl = ompl.get_problem().get_scene().get_kinematic_tree().get_joint_limits()
        jl[0] += 1
        ompl.get_problem().get_scene().get_kinematic_tree().set_joint_limits_lower(jl[:,0])
        ompl.get_problem().get_scene().get_kinematic_tree().set_joint_limits_upper(jl[:,1])

        # 3rd call with changed bounds
        solution = None
        failed = False
        try:
            solution = ompl.solve()
        except Exception:
            failed = True
            pass
        # the solve call with changed bounds should throw an exception
        self.assertTrue(failed)
        # ... and there should be no solution
        self.assertTrue(solution is None)

