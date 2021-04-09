cmake_minimum_required(VERSION 3.0.2)
project(exotica_superbuild)

# This project is a convenience wrapper to build Exotica using pure CMake

# Core & Python
add_subdirectory(../exotica_core)
add_subdirectory(../exotica_python) # Does not install

# Task Maps
add_subdirectory(../exotations/exotica_core_task_maps)

# Collision Scenes
# add_subdirectory(../exotations/exotica_collision_scene_fcl_latest)  # Requires FCL

# Motion Solvers
add_subdirectory(../exotations/solvers/exotica_aico_solver)
add_subdirectory(../exotations/solvers/exotica_ddp_solver)
add_subdirectory(../exotations/solvers/exotica_ik_solver)
add_subdirectory(../exotations/solvers/exotica_ilqg_solver)
add_subdirectory(../exotations/solvers/exotica_ilqr_solver)
add_subdirectory(../exotations/solvers/exotica_levenberg_marquardt_solver)
add_subdirectory(../exotations/solvers/exotica_ompl_control_solver) # Requires OMPL
add_subdirectory(../exotations/solvers/exotica_ompl_solver)         # Requires OMPL
add_subdirectory(../exotations/solvers/exotica_scipy_solver)
add_subdirectory(../exotations/solvers/exotica_time_indexed_rrt_connect_solver)

# Dynamics Solvers
add_subdirectory(../exotations/dynamics_solvers/exotica_cartpole_dynamics_solver)
add_subdirectory(../exotations/dynamics_solvers/exotica_double_integrator_dynamics_solver)
add_subdirectory(../exotations/dynamics_solvers/exotica_pendulum_dynamics_solver)
add_subdirectory(../exotations/dynamics_solvers/exotica_pinocchio_dynamics_solver) # Requires pinocchio
add_subdirectory(../exotations/dynamics_solvers/exotica_quadrotor_dynamics_solver)

# Examples
add_subdirectory(../exotica_examples)
