#!/usr/bin/env python
import numpy as np
import pyexotica as exo

# Exotica
(_, ik_prob) = exo.Initializers.loadXMLFull(
    '{exotica}/resources/configs/test_com_valkyrie.xml')
ik_problem = exo.Setup.createProblem(ik_prob)

# Test with zeros
print("============ ALL ZEROS ============")
ik_problem.update(np.zeros(38,))
drake_com = np.array(
    [-0.007956278619357, -0.000791781655122, -0.023887289533801])
exotica_com = ik_problem.Phi.data
print("Drake", drake_com)
print("Exotica", exotica_com)
assert np.allclose(drake_com, exotica_com)
print("")

# Test x,y,z movement
print("============ ALL ZEROS, x=1, y=2, z=3 ============")
test_vector = np.zeros(38,)
test_vector[0] = 1
test_vector[1] = 2
test_vector[2] = 3
ik_problem.update(test_vector)
exotica_test_vector = ik_problem.Phi.data
drake_test_vector = np.array(
    [0.992043721380643, 1.999208218344877, 2.976112710466198])
print("Drake", drake_test_vector)
print("Exotica", exotica_test_vector)
assert np.allclose(drake_test_vector, exotica_test_vector)
print("")

# Test with ones
print("============ ALL ONES ============")
drake_ones_com = np.array(
    [1.011827488179183, 0.984257082034692, 0.909060824252459])
ik_problem.update(np.ones(38,))
exotica_ones_com = ik_problem.Phi.data
print("Drake", drake_ones_com)
print("Exotica", exotica_ones_com)
assert np.allclose(drake_ones_com, exotica_ones_com)
print("")

# Test with ones, RPY=0
print("============ ALL ONES, rpy=0 ============")
drake_rpy_com = np.array(
    [1.072817938570169, 0.943826381822213, 0.985867234115696])
test_vector = np.ones(38,)
test_vector[3] = 0
test_vector[4] = 0
test_vector[5] = 0
ik_problem.update(test_vector)
exotica_rpy_com = ik_problem.Phi.data
print("Drake", drake_rpy_com)
print("Exotica", exotica_rpy_com)
assert np.allclose(drake_rpy_com, exotica_rpy_com)
print("")

# Test Roll
print("============ ALL ZEROS, xyz=1, roll=1.57 ============")
test_vector = np.zeros(38,)
test_vector[0] = 1
test_vector[1] = 1
test_vector[2] = 1
test_vector[3] = 1.57
ik_problem.update(test_vector)
exotica_test_vector = ik_problem.Phi.data
drake_test_vector = np.array(
    [0.992043721380643, 1.023886651443021, 0.999189196509224])
print("Drake", drake_test_vector)
print("Exotica", exotica_test_vector)
assert np.allclose(drake_test_vector, exotica_test_vector)
print("")

# Test Roll
print("============ ALL ZEROS, xyz=1, roll=1.57/3 ============")
test_vector = np.zeros(38,)
test_vector[0] = 1
test_vector[1] = 1
test_vector[2] = 1
test_vector[3] = 1.57/3
ik_problem.update(test_vector)
exotica_test_vector = ik_problem.Phi.data
drake_test_vector = np.array(
    [0.992043721380643, 1.011252345052587, 0.978914122017840])
print("Drake", drake_test_vector)
print("Exotica", exotica_test_vector)
assert np.allclose(drake_test_vector, exotica_test_vector)
print("")


# Test Pitch
print("============ ALL ZEROS, xyz=1, pitch=1.57 ============")
test_vector = np.zeros(38,)
test_vector[0] = 1
test_vector[1] = 1
test_vector[2] = 1
test_vector[4] = 1.57
ik_problem.update(test_vector)
exotica_test_vector = ik_problem.Phi.data
drake_test_vector = np.array(
    [0.976106382242915, 0.999208218344877, 1.007937254009971])
print("Drake", drake_test_vector)
print("Exotica", exotica_test_vector)
assert np.allclose(drake_test_vector, exotica_test_vector)
print("")

# Test Yaw
print("============ ALL ZEROS, xyz=1, yaw=1.57 ============")
test_vector = np.zeros(38,)
test_vector[0] = 1
test_vector[1] = 1
test_vector[2] = 1
test_vector[5] = 1.57
ik_problem.update(test_vector)
exotica_test_vector = ik_problem.Phi.data
drake_test_vector = np.array(
    [1.000785445606890, 0.992043093386445, 0.976112710466199])
print("Drake", drake_test_vector)
print("Exotica", exotica_test_vector)
assert np.allclose(drake_test_vector, exotica_test_vector)
print("")

# Test Yaw
print("============ ALL ZEROS, xyz=1, roll=1.57, pitch=1.57 ============")
test_vector = np.zeros(38,)
test_vector[0] = 1
test_vector[1] = 1
test_vector[2] = 1
test_vector[3] = 1.57
test_vector[4] = 1.57
ik_problem.update(test_vector)
exotica_test_vector = ik_problem.Phi.data
drake_test_vector = np.array(
    [0.999182860969121, 1.023886651443021, 1.007955630432197])
print("Drake", drake_test_vector)
print("Exotica", exotica_test_vector)
assert np.allclose(drake_test_vector, exotica_test_vector)
print("")

# Test RPY again
print("========== ALL ZEROS, roll=1.57/4, pitch=-1.57/4, yaw=1.57/4 =========")
test_vector = np.zeros(38,)
test_vector[3] = 1.57/4
test_vector[4] = -1.57/4
test_vector[5] = 1.57/4
ik_problem.update(test_vector)
exotica_test_vector = ik_problem.Phi.data
drake_test_vector = np.array(
    [-0.002100122545919, 0.008227677204985, -0.023715537144087])
print("Drake", drake_test_vector)
print("Exotica", exotica_test_vector)
assert np.allclose(drake_test_vector, exotica_test_vector)
print("")

print('>>SUCCESS<<')
