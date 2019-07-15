from __future__ import print_function, division
from time import time

__all__ = ["check_trajectory_continuous_time", "get_colliding_links"]

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

def get_colliding_links(scene, margin=0.0, safe_distance=0.0, check_self_collision=True):
    proxies = scene.get_collision_distance(True)
    robotLinks = scene.get_collision_robot_links()
    world_links = scene.get_collision_world_links()
    for r_l in robotLinks:
        for w_l in world_links:
            if scene.is_allowed_to_collide(r_l, w_l, True):
                if not scene.is_collision_free(r_l, w_l, margin):
                    d=scene.get_collision_distance(r_l, w_l)
                    if abs(d[0].distance) > safe_distance:
                        print(r_l,"-",w_l,"d=",d[0].distance)
        if check_self_collision:
            for w_l in robotLinks:
                if w_l != r_l:
                    if scene.is_allowed_to_collide(r_l, w_l, True):
                        if not scene.is_collision_free(r_l, w_l, margin):
                            d=scene.get_collision_distance(r_l,w_l)
                            if abs(d[0].distance) > safe_distance:
                                print(r_l,"-",w_l,d[0].distance)

