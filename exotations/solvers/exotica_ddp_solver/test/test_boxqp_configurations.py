#!/usr/bin/env python
from __future__ import print_function, division

import matplotlib.pyplot as plt
import pyexotica as exo
from collections import OrderedDict


def test_solver(use_new_boxqp, use_polynomial_linesearch, use_cholesky):
    config = '{exotica_examples}/resources/configs/dynamic_time_indexed/13_control_limited_ddp_quadrotor.xml'
    # config = '{exotica_examples}/resources/configs/dynamic_time_indexed/07_control_limited_ddp_cartpole.xml'
    _, problem_config = exo.Initializers.load_xml_full(config)
    problem = exo.Setup.create_problem(problem_config)
    solver_config = ('exotica/ControlLimitedDDPSolver',
                     {'RegularizationRate': 1e-05,
                      'UseSecondOrderDynamics': False,
                      'MaxIterations': 100,
                      'UseNewBoxQP': use_new_boxqp,
                      'Name': u'Solver',
                      'BoxQPUsePolynomialLinesearch': use_polynomial_linesearch,
                      'FunctionTolerancePatience': 10,
                      'FunctionTolerance': 0.001,
                      'MinimumRegularization': 1e-12,
                      'ThresholdRegularizationDecrease': 0.5,
                      'BoxQPUseCholeskyFactorization': use_cholesky,
                      'MaximumRegularization': 1000.0,
                      'ClampControlsInForwardPass': True,
                      'ThresholdRegularizationIncrease': 0.01,
                      'Debug': False})
    solver = exo.Setup.create_solver(solver_config)
    #solver.max_iterations = 1000
    solver.specify_problem(problem)

    _ = solver.solve()

    print('Solver terminated with:', problem.termination_criterion)
    print('Solver took:', solver.get_planning_time())

    costs = problem.get_cost_evolution()
    return costs

if __name__ == "__main__":
    results = OrderedDict()

    for use_new_boxqp in [True, False]:
        for use_polynomial_linesearch in [True, False]:
            for use_cholesky in [True, False]:
                name = '{0}, {1}, {2}'.format('New' if use_new_boxqp else 'Old',
                                              'Polynomial Linesearch' if use_polynomial_linesearch else 'Linear Linesearch',
                                              'Cholesky' if use_cholesky else 'Regular Inverse')
                results[name] = test_solver(use_new_boxqp, use_polynomial_linesearch, use_cholesky)

    fig = plt.figure(1, (12,6))

    plt.subplot(1,2,1)
    for setting in results:
        plt.plot(results[setting][1], label=setting)
    plt.yscale('log')
    #plt.xscale('log')
    plt.ylabel('Cost')
    plt.xlabel('Iterations')
    plt.legend()

    plt.subplot(1,2,2)
    for setting in results:
        plt.plot(results[setting][0], results[setting][1], label=setting)
    plt.yscale('log')
    plt.ylabel('Cost')
    plt.xlabel('Time (s)')
    plt.legend()

    plt.tight_layout()
    fig.savefig('boxqp_configurations.png')
    plt.show()

