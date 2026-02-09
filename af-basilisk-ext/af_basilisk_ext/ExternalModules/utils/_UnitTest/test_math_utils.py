import numpy as np
from ExternalModules.utils.math_utils import *


def test_mrp_to_quat():
    """
    Tests `mrp_to_quat()` against hand calculated answers
    """
    ## Test null MRP. EXPECT: Receive JPL convention identity quaternion ([0, 0, 0, 1])
    mrp = np.zeros(3)
    quat = mrp_to_quat(mrp)
    ans = np.array([0.0, 0.0, 0.0, 1.0])
    assert np.all(quat == ans)

    ## The MRP vector is codirectional with the axis of rotation.
    ## Its magnitude is tan(theta / 4)

    ## This corresponds to a rotation about axis [0, 0, 1];
    ## with magnitude atan(1) * 4 = PI
    mrp = np.array([0.0, 0.0, 1.0])

    quat = mrp_to_quat(mrp)
    ## Quat is [ax * sin(theta / 2), ay * sin(theta / 2), az * sin(theta / 2), cos(theta / 2)]
    ## [0, 0, 1, 0]
    ans = np.array([0.0, 0.0, 1.0, 0.0])
    assert np.all(quat == ans)

    ## This corresponds to a rotation about axis [0, 1, 0];
    ## with magnitude atan(1) * 4 = PI
    mrp = np.array([0.0, 1.0, 0.0])

    quat = mrp_to_quat(mrp)
    ## Quat is [ax * sin(theta / 2), ay * sin(theta / 2), az * sin(theta / 2), cos(theta / 2)]
    ## [0, 1, 0, 0]
    ans = np.array([0.0, 1.0, 0.0, 0.0])
    assert np.all(quat == ans)


def test_quat_to_mrp():
    """
    Tests `quat_to_mrp()` against hand calculated answers
    """
    ## Test identity quaternion. EXPECT: Receive null MRP ([0, 0, 0])
    quat = np.array([0, 0, 0, 1])
    mrp = quat_to_mrp(quat)
    ans = np.array([0.0, 0.0, 0.0])
    assert np.all(mrp == ans)

    ## Quat is [ax * sin(theta / 2), ay * sin(theta / 2), az * sin(theta / 2), cos(theta / 2)]
    ## [0, 0, 1, 0]
    ## This is a rotation about axis [0, 0, 1];
    ## with magnitude atan(1) * 4 = PI
    quat = np.array([0.0, 0.0, 1.0, 0.0])

    ## The MRP vector is codirectional with the axis of rotation.
    ## Its magnitude is tan(theta / 4)
    ans = np.array([0.0, 0.0, 1.0])

    mrp = quat_to_mrp(quat)
    assert np.all(mrp == ans)

    ## Quat is [ax * sin(theta / 2), ay * sin(theta / 2), az * sin(theta / 2), cos(theta / 2)]
    ## [0, 1, 0, 0]
    ## This is to a rotation about axis [0, 1, 0];
    ## with magnitude atan(1) * 4 = PI
    quat = np.array([0.0, 1.0, 0.0, 0.0])

    ans = np.array([0.0, 1.0, 0.0])

    mrp = quat_to_mrp(quat)
    assert np.all(mrp == ans)


def create_random_quaternion() -> Quaternion:
    """
    @brief: Creates random unit quaternion
    """
    while True:
        quat_arr = np.random.random(4)
        if not np.allclose(np.linalg.norm(quat_arr), 0):
            break
    return Quaternion(quat_arr)


def test_quaternion_conjugate():
    """
    @brief: Tests quaternion conjugate function
    """
    quat = create_random_quaternion()
    quat_conj_ans = np.array([-quat[0], -quat[1], -quat[2], quat[3]])
    assert np.allclose(quat.conj().quat, quat_conj_ans)


def test_quaternion_inverse():
    """
    @brief: Tests quaternion inverse. q_inv * q should be identity
    """
    quat = create_random_quaternion()
    assert np.allclose((quat.inv() * quat).quat, np.array([0, 0, 0, 1]))


def test_quaternion_norm():
    """
    @brief: Tests quaternion norm function
    """
    quat = create_random_quaternion()
    assert np.allclose(quat.norm(), np.linalg.norm(quat.quat))


def test_quaternion_copy():
    """
    @brief: Tests quaternion copy function
    """
    quat = create_random_quaternion()
    assert np.allclose(quat.copy().quat, quat.quat)


def test_quaternion_math():
    """
    @brief: Tests quaternion math functions, from Quaternion object in math_utils
    """
    # Test 1 -- q x q.inv() == 1
    quat = Quaternion(np.array([1, 0, 0.1, 0.5]))
    test = quat * quat.inv()

    assert np.allclose(test.quat, np.array([0, 0, 0, 1]))

    # Test 2 -- q.inv() x q == 1
    quat = Quaternion(np.array([1, 0, 0.1, 0.5]))
    test = quat.inv() * quat

    assert np.allclose(test.quat, np.array([0, 0, 0, 1]))


def test_get_random_unit_vector():
    """
    @brief: Generates 100 random unit vectors, asserts that their norm is 1
    """
    N_RUNS = 100
    for _ in range(N_RUNS):
        vec = get_random_unit_vector()
        assert np.allclose(np.linalg.norm(vec), 1.0)


def test_get_perpendicular_vector():
    """
    @brief: Tests get perpendicular vector function, generating 100 random vectors,
    feeding them into the function,
    and ensuring that the function output is always perpendicular to the input
    """
    N_RUNS = 100
    for _ in range(N_RUNS):
        vec = get_random_unit_vector()
        perp = get_perpendicular_vector(vec)
        assert np.allclose(np.dot(vec, perp), 0.0)


def test_get_random_perpendicular_vector():
    """
    @brief: Tests get random perpendicular vector function, generating 100 random vectors,
    feeding them into the function,
    and ensuring that the function output is always perpendicular to the input
    """
    N_RUNS = 100
    for _ in range(N_RUNS):
        vec = get_random_unit_vector()
        perp = get_random_perpendicular_vector(vec)
        assert np.allclose(np.dot(vec, perp), 0.0)


def test_is_nearly_collinear():
    """
    @brief: Generates 100 random unit vectors, asserts that their norm is 1
    """
    vec_1 = np.array([0, 0, 1])
    vec_2 = np.array([0, 0, 3])

    assert is_nearly_colinear(vec_1, vec_2) == True

    vec_1 = np.array([-1, 0, 1])
    vec_2 = np.array([0, 0, 3])

    assert is_nearly_colinear(vec_1, vec_2) == False


def test_calc_miss_dist():
    """
    @brief: Evaluates function in math_utils: calc_miss_dist
    """
    ## Spacecraft is flying on x xis
    ## but asteroid is offset by 1.0 in y.
    ## Expect miss to be 1.0.
    rel_pos_n = np.array([10.0, 1.0, 0.0])
    rel_vel_n = np.array([1.0, 0.0, 0.0])
    miss = calc_miss_dist(rel_vel_n, rel_pos_n)
    assert np.allclose(miss, 1.0)


def test_rotate_a_to_b():
    """
    @brief: Evaluates function in math_utils: rotate_a_to_b
    """
    NUM_RUNS = 100

    for _ in range(NUM_RUNS):
        vec_a = np.random.random((3))
        vec_a /= np.linalg.norm(vec_a)
        vec_b = np.random.random((3))
        vec_b /= np.linalg.norm(vec_b)
        ang = (np.random.random() - 0.5) * 3.14
        output = rotate_a_to_b(vec_a, vec_b, ang)

        ## Check that vec is angle ang from a
        ang_to_a = np.abs(np.arccos(np.dot(output, vec_a)))
        at_correct_ang = np.allclose(ang_to_a, np.abs(ang))

        ## Check that a, b, and vec are in the same plane
        same_plane = np.allclose(np.dot(np.cross(vec_a, vec_b), output), 0)

        assert at_correct_ang and same_plane


def test_align_vectors():
    """
    @brief: Evaluates function in math_utils: align_vectors
    """

    def eval_answers(vec_a, vec_b):
        try:
            rot = align_vectors(vec_a, vec_b)
            assert np.allclose(rot.apply(vec_a).flatten(), vec_b)
        except ValueError:
            assert (not np.allclose(np.linalg.norm(vec_a), 1)) or (
                not np.allclose(np.linalg.norm(vec_b), 1)
            )

    ## Test 1
    vec_a = np.random.random((3))
    vec_a /= np.linalg.norm(vec_a)
    vec_b = np.array([1.0, 0, 0])
    eval_answers(vec_a, vec_b)

    ## Test 2
    vec_a = np.array([1.0, 0, 0])
    vec_b = np.random.random((3))
    vec_b /= np.linalg.norm(vec_a)
    eval_answers(vec_a, vec_b)

    ## Test 3
    vec_a = np.random.random((3))
    vec_a /= np.linalg.norm(vec_a)
    vec_b = -vec_a
    eval_answers(vec_a, vec_b)

    ## Test 4
    vec_a = np.random.random((3))
    vec_a /= np.linalg.norm(vec_a)
    vec_b = np.zeros((3))
    eval_answers(vec_a, vec_b)
