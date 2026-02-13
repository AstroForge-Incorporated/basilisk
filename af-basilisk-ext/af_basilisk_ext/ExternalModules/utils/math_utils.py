"""General FSW utils"""

from __future__ import annotations
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation as R


def mrp_to_quat(mrp: np.ndarray) -> np.ndarray:
    """
    Converts MRP to a JPL quaternion
    @params:
    - mrp: Modified Rodrigues Parameter, as a numpy array

    @returns:
    - quaternion, as a numpy array
    """
    rot = R.from_mrp(mrp)
    return rot.as_quat()


def quat_to_mrp(quat: np.ndarray) -> np.ndarray:
    """
    Converts JPL quaternion to an MRP
    @params:
    - quat: JPL quaternion, as a numpy array

    @returns:
    - mrp, as a numpy array
    """
    rot = R.from_quat(quat)
    return rot.as_mrp()


class Quaternion:
    """
    @brief: Orb Astro uses quaternion convention
        Q = [q_vec, q_scalar]. We use the same convention here.
    """

    def __init__(self, quat: np.ndarray[float, float, float, float]):
        self.quat = quat

        self.normalize()

    def __getitem__(self, key):
        return self.quat[key]

    @property
    def shape(self):
        """
        @brief: Returns shape of quaternion
        """
        return self.quat.shape

    def norm(self) -> np.floating[Any]:
        """
        @brief: Returns quaternion norm
        """
        return np.linalg.norm(self.quat)

    def vec(self) -> np.ndarray[float, float, float]:
        """
        @brief: Gets vector part of quaternion
        """
        return self.quat[0:3]

    def scalar(self) -> float:
        """
        @brief: Gets scalar part of quaternion
        """
        return self.quat[3]

    def normalize(self):
        """
        @brief: Normalizes quaternion object, in place
        """
        norm = self.norm()

        if np.allclose(norm, 0.0):
            raise ValueError("Invalid quaternion, has norm 0")

        self.quat = self.quat / self.norm()

    def __mul__(self, other: Quaternion) -> Quaternion:

        if not isinstance(other, Quaternion):
            raise TypeError("Can only multiply quaternion by another quaternion!")

        self_vec: np.ndarray = self.vec()
        other_vec: np.ndarray = other.vec()

        self_scal: float = self.scalar()
        other_scal: float = other.scalar()

        product_scal = self_scal * other_scal - np.dot(self_vec, other_vec)
        product_vec = (
            other_scal * self_vec
            + self_scal * other_vec
            - np.cross(self_vec, other_vec)
        )

        product = Quaternion(
            np.array([product_vec[0], product_vec[1], product_vec[2], product_scal])
        )

        return product

    def conj(self) -> Quaternion:
        """
        @brief: Creates conjugate quaternion object
        """
        conj_quat = np.array(
            [-self.quat[0], -self.quat[1], -self.quat[2], self.quat[3]]
        )

        return Quaternion(conj_quat)

    def inv(self) -> Quaternion:
        """
        @brief: Creates inverse quaternion object
        """
        inv_q = self.conj().quat / self.norm() ** 2
        return Quaternion(inv_q)

    def copy(self) -> Quaternion:
        """
        @brief: Returns copy of quaternion
        """
        return Quaternion(self.quat)

    def __repr__(self):
        return "Quaternion:[{}, {}, {}, {}]".format(
            self.quat[0], self.quat[1], self.quat[2], self.quat[3]
        )


def get_perpendicular_vector(vec: np.ndarray) -> np.ndarray:
    """
    @brief: Finds a unit vector perpendicular to input unit vector
    @details: Chooses a cross vector composed of three unique elements.
                If that vector is *not* parallel/anti-parallel to the input
                vector, we can get the perpendicular vector by simply
                crossing the cross and input vectors.
                If the cross vector *is* paralell / anti-parallel to the input,
                flips two of the elements of the cross vector (since the cross vector had three
                unique elements, this is guaranteed to create a unique vector)
    """

    assert not np.allclose(np.linalg.norm(vec), 0.0), "Input vec cannot have 0 norm"
    cross_vec = np.array([1.0, 2.0, 3.0])
    cross_vec = cross_vec / np.linalg.norm(cross_vec)

    if is_nearly_colinear(cross_vec, vec):
        cross_vec2 = np.copy(cross_vec)
        ## Flip values
        cross_vec[0] = cross_vec2[1]
        cross_vec[1] = cross_vec2[0]

    perp_vec = np.cross(cross_vec, vec)

    ## Normalize perpendicular vector
    perp_vec = perp_vec / np.linalg.norm(perp_vec)
    return perp_vec


def get_random_perpendicular_vector(vec: np.ndarray) -> np.ndarray:
    """
    @brief: Finds a random unit vector perpendicular to input unit vector.
    This function differes from get_perpendicular_vector in that
    it chooses a random input vector, at the cost of (potentially, but
    very very unlikely) having infinite runtime. For a given input,
    get_perpendicular_vector will always return the same value.

    @details: Chooses a random cross vector.
                If that vector is parallel/anti-parallel to the input
                vector, generates a new random cross vector. Continues
                until valid cross vector is found, and
                takes cross product of cross vector and input vector.
    """

    assert not np.allclose(np.linalg.norm(vec), 0.0), "Input vec cannot have 0 norm"

    while True:
        cross_vec = np.random.random(3) - 0.5
        cross_vec = cross_vec / np.linalg.norm(cross_vec)
        if not is_nearly_colinear(cross_vec, vec):
            break

    perp_vec = np.cross(cross_vec, vec)

    ## Normalize perpendiuclar vector
    perp_vec = perp_vec / np.linalg.norm(perp_vec)
    return perp_vec


def is_nearly_colinear(vec_1, vec_2, eps=1e-15):
    """
    @brief: Checks if two vectors are nearly parallel or anti parallel
    @details: If two vectors are nearly parallel, their cross product will have a norm of nearly 0
    """
    cross = np.cross(vec_1, vec_2)
    return np.linalg.norm(cross) < eps


def get_random_unit_vector() -> np.ndarray:
    """
    @brief: Finds random unit vector
    """
    phi = np.random.uniform(0, np.pi * 2)
    theta = np.arccos(np.random.uniform(-1, 1))
    x = np.sin(theta) * np.cos(phi)
    y = np.sin(theta) * np.sin(phi)
    z = np.cos(theta)

    vec = np.array([x, y, z])
    return vec


def rotate_a_to_b(vec_a: np.ndarray, vec_b: np.ndarray, ang: float) -> np.ndarray:
    """
    @brief; Generates vector that rotates input vec_a towards
    vec_b by angle ang
    @params:
    - ang: Radians, angle of rotation
    - vec_a: 3x0 unit vector, vector to be rotated
    - vec_b: 3x0 unit vector, vector to be rotated towards
    """

    if not np.allclose(np.linalg.norm(vec_a), 1.0) or not np.allclose(
        np.linalg.norm(vec_b), 1.0
    ):
        raise ValueError("Input vectors must be unit vectors!")

    b_tick = np.cross(np.cross(vec_a, vec_b), vec_a)
    b_tick /= np.linalg.norm(b_tick)

    rotated_vec = np.cos(ang) * vec_a + np.sin(ang) * b_tick
    return rotated_vec


def calc_miss_dist(rel_vel: np.ndarray, rel_pos: np.ndarray) -> float:
    """
    Calculates current spacecraft-asteroid miss distance.

    @params:
    - rel_vel: Asteroid velocity relative to spacecraft, in m/s
    - rel_pos: Asteroid position relative to spacecraft, in m

    @returns:
    - miss_dist: Miss distance, in meters
    """
    point = np.zeros(3)
    rel_vel_unit = rel_vel / np.linalg.norm(rel_vel)
    dist = (point - rel_pos) - (np.dot((point - rel_pos), rel_vel_unit) * rel_vel_unit)
    dist = np.linalg.norm(dist)
    return dist


def skew_sym(vec: np.ndarray) -> np.ndarray:
    """
    @brief: Calculates skew symmetric matrix from input vector
    """
    skew_mat = np.array(
        [[0, -vec[2], vec[1]], [vec[2], 0, -vec[0]], [-vec[1], vec[0], 0]]
    )
    return skew_mat


def align_vectors(vec_a: np.ndarray, vec_b: np.ndarray) -> R:
    """
    @brief: Find rotation object that rotates unit vector a to align with unit vector b
    @params:
        - vec_a : np.ndarray[3]: Unit vector
        - vec_b : np.ndarray[3]: Target unit vector
    @fixme: Replace with scipy align_vectors function?
    """
    if not np.allclose(np.linalg.norm(vec_a), 1.0) or not np.allclose(
        np.linalg.norm(vec_b), 1.0
    ):
        raise ValueError("Input vectors must be unit vectors!")

    cross_vec = np.cross(vec_a, vec_b)
    # Handle case where vec_a and vec_b point in exactly opposite directions
    if np.allclose(np.dot(vec_a, vec_b), -1.0):
        # Can replace with
        # https://math.stackexchange.com/questions/293116/rotating-one-3d-vector-to-another?
        target_rot, _ = R.align_vectors(vec_a[:, np.newaxis].T, vec_b[:, np.newaxis].T)
    else:
        dot_vec = np.dot(vec_a, vec_b)

        skew_mat = skew_sym(cross_vec)
        rot = np.eye(3) + skew_mat + skew_mat @ skew_mat * (1 / (1 + dot_vec))
        target_rot = R.from_matrix(rot)
    return target_rot
