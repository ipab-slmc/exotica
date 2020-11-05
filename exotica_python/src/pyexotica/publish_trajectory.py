from __future__ import print_function, division
from time import sleep
import matplotlib.pyplot as plt
import signal

__all__ = ["sig_int_handler", "publish_pose", "publish_trajectory",
           "publish_time_indexed_trajectory", "plot"]


def sig_int_handler(signal, frame):
    raise KeyboardInterrupt


def publish_pose(q, problem, t=0.0):
    problem.get_scene().update(q, t)
    problem.get_scene().get_kinematic_tree().publish_frames()


def publish_trajectory(traj, T, problem, once=False):
    if len(traj) == 0:
        raise ValueError("Trajectory has zero elements")
    signal.signal(signal.SIGINT, sig_int_handler)
    print('Playing back trajectory ' + str(T) + 's')
    dt = float(T) / float(len(traj))
    t = 0
    while True:
        try:
            publish_pose(traj[t], problem, float(t) * dt)
            sleep(dt)
            if t >= len(traj) - 1 and once:
                return
            t = (t + 1) % len(traj)
        except KeyboardInterrupt:
            return False


def publish_time_indexed_trajectory(traj, Ts, problem, once=False):
    if len(traj) == 0:
        raise ValueError("Trajectory has zero elements")
    signal.signal(signal.SIGINT, sig_int_handler)
    print('Playing back trajectory ' + str(len(Ts)) +
          ' states in ' + str(Ts[len(Ts) - 1]))

    while True:
        try:
            for i in range(1, len(Ts) - 1):
                publish_pose(traj[i], problem, Ts[i])
                sleep(Ts[i] - Ts[i-1])
            if once:
                break
        except KeyboardInterrupt:
            return False
    return True


def plot(solution, labels=None, yscale=None):
    print('Plotting the solution')
    plt.plot(solution, '.-')
    if labels is not None:
        plt.legend(labels)
    if yscale is not None:
        plt.yscale(yscale)
    plt.show()
