from __future__ import print_function, division
import numpy as np
from time import time
import matplotlib.pyplot as plt
from collections import OrderedDict

__all__ = ["check_trajectory_continuous_time",
           "check_whether_trajectory_is_collision_free_by_subsampling", "get_colliding_links"]

def check_trajectory_continuous_time(scene, trajectory):
    start_time = time()
    all_good = True
    robot_links = scene.get_collision_robot_links()
    world_links = scene.get_collision_world_links()
    for t in range(1, trajectory.shape[0]):
        q_t_1 = trajectory[t-1,:]
        q_t_2 = trajectory[t,:]
        for r_l in robot_links:
            for w_l in world_links:
                scene.update(q_t_1)
                T_r_l_1 = scene.fk(r_l)
                T_w_l_1 = scene.fk(w_l)
                scene.update(q_t_2)
                T_r_l_2 = scene.fk(r_l)
                T_w_l_2 = scene.fk(w_l)
                p = scene.get_collision_scene().continuous_collision_check(r_l, T_r_l_1, T_r_l_2, w_l, T_w_l_1, T_w_l_2)
                if p.in_collision:
                    print(t, p)
                    all_good = False
    end_time = time()
    print("Continuous-time collision verification took", end_time - start_time)
    return all_good


def check_whether_trajectory_is_collision_free_by_subsampling(scene, trajectory, num_subsamples=10, debug=False):
    '''
    num_subsamples specifies how many steps are checked between two configurations.
    Returns True if trajectory is collision-free, and False otherwise.

    TODO: Support setting time for Scene update.
    '''
    start_time = time()
    trajectory_length = trajectory.shape[0]
    for t in range(1, trajectory_length):
        q_t_1 = trajectory[t-1, :]
        q_t_2 = trajectory[t, :]
        q_t_interpolation = np.linspace(q_t_1, q_t_2, num_subsamples)
        for i in range(num_subsamples):
            scene.update(q_t_interpolation[i, :])
            if not scene.is_state_valid(True):
                return False
    end_time = time()
    if debug:
           print("Trajectory transition collision check took", end_time - start_time)
    return True


def get_colliding_links(scene, margin=0.0, safe_distance=0.0, check_self_collision=True, debug=False):
    robotLinks = scene.get_collision_robot_links()
    world_links = scene.get_collision_world_links()
    collisions = []
    for r_l in robotLinks:
        for w_l in world_links:
            if scene.is_allowed_to_collide(r_l, w_l, True):
                if not scene.is_collision_free(r_l, w_l, margin):
                    d=scene.get_collision_distance(r_l, w_l)
                    if abs(d[0].distance) > safe_distance:
                        collisions.append((r_l, w_l, d[0].distance))
                        if debug:
                            print(r_l,"-",w_l,"d=",d[0].distance)
        if check_self_collision:
            for w_l in robotLinks:
                if w_l != r_l:
                    if scene.is_allowed_to_collide(r_l, w_l, True):
                        if not scene.is_collision_free(r_l, w_l, margin):
                            d=scene.get_collision_distance(r_l,w_l)
                            if abs(d[0].distance) > safe_distance:
                                collisions.append((r_l, w_l, d[0].distance))
                                if debug:
                                    print(r_l,"-",w_l,d[0].distance)
    return collisions


def plot_task_cost_over_time(problem):	
    '''
    Plots the task cost (task maps) over time given a problem.
    '''
    costs = OrderedDict()	
    for task_name in problem.cost.task_maps:	
        costs[task_name] = np.zeros((problem.T,))	
        for t in range(problem.T):	
            ydiff = problem.cost.get_task_error(task_name, t)	
            S = problem.cost.get_S(task_name, t)	
            cost = np.dot(np.dot(ydiff, S), ydiff.T)
            costs[task_name][t] = cost	
    fig = plt.figure()	
    for task_name in costs:	
        plt.plot(costs[task_name], label=task_name)	
    plt.title('Task cost over time')	
    plt.legend()	
    plt.tight_layout()	
    plt.show()
