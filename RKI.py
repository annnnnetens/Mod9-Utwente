import math
# When working on the microcontroller change the numpy to ulab since numpy is not available on the microcontroller
import numpy as np


def h_to_adjoint(h_matrix):
    """Turns H Matrix into Adjoint"""
    res = np.eye(3)
    res[1:3, 1:3] = h_matrix[0:2, 0:2]
    res[1, 0] = h_matrix[1, 2]
    res[2, 0] = h_matrix[0, 2] * -1
    return res


def adjoint_to_h(adjoint):
    """Turns adjoint matrix into H matrix"""
    res = np.eye(3)
    res[0:2, 0:2] = adjoint[1:3, 1:3]
    res[1, 2] = adjoint[1, 0]
    res[0, 2] = adjoint[2, 0] * -1
    return res


def create_h_matrix(angle, location):
    """Creates a H Matrix from a to b using the angle a makes with respect to b
    The location parameter is the origin of a expressed in frame b
    At least I think it does this :o
    My RKI imagination is not that good"""
    return np.array([
        [np.cos(angle * np.pi), -np.sin(angle * np.pi), location[0]],
        [np.sin(angle * np.pi), np.cos(angle * np.pi), location[1]],
        [0, 0, 1]
    ])


def get_angle_from_h_matrix(h_matrix):
    return np.arccos(h_matrix[0, 0]) / np.pi


def get_position_from_h_matrix(h_matrix):
    return h_matrix[0, 2], h_matrix[1, 2]


def brockett(h0_matrix, twistspairs):
    """
    Creates H matrix using brockett
    Inputs: H0_matrix is the H matrix in the reference configuration
    twistpairs is a list of tuples of a twist and the corresponding q (angle or translation
    If degrees is true it is assumed that q is in degrees (for example q = 45)
    If degrees is false it is assumed that q is in radians (for example q = 0.25) (so no multiplng with pi!)
    """
    res = np.eye(3)
    for twistpair in twistspairs:
        current_matrix = np.eye(3)
        twist = twistpair[0]
        q = twistpair[1]
        if twist[0] == 0:
            # Translational joint
            if (math.sqrt(twist[1] ** 2 + twist[2] ** 2) - 1) > 0.01:
                raise ValueError("There is a non unit twist in brockett")
            x = twist[1]
            y = twist[2]
            current_matrix[0, 2] = x * q
            current_matrix[1, 2] = y * q
            res = res @ current_matrix
        elif twist[0] == 1:
            # Rotational joint
            c = np.cos(q * np.pi)
            s = np.sin(q * np.pi)
            current_matrix[0:2, 0:2] = np.array([[c, -s], [s, c]])
            x = twist[2] * -1
            y = twist[1]
            current_matrix[0, 2] = x - x * c + y * s
            current_matrix[1, 2] = y - x * s - y * c
            res = res @ current_matrix
        else:
            raise ValueError("There is a non unit twist in brockett")
    res = res @ h0_matrix
    return res


def twist_to_tilde(twist):
    """Returns tilde form of twist"""
    return np.array([
        [0, -twist[0], twist[1]],
        [twist[0], 0, twist[2]],
        [0, 0, 0]
    ])


def tilde_to_twist(tilde):
    """Returns the twist form of a tilde twist"""
    return np.array([tilde[1, 0], tilde[0, 2], tilde[1, 2]])


def jacobian_twists(twists):
    """Calculate jacobian using the twists"""
    return np.array(twists).T


def jacobian_angles(q1):
    """Calculate jacobian using the angles
    Keep in mind that it only needs q1 and the length of the first part is assumed to be 240mm
    Currently only uses radians"""
    J = np.zeros((3, 2))
    J[:, 0] = (1, 0, 0)
    J[:, 1] = (1, 0.24 * np.cos(q1*np.pi), 0.24 * np.sin(q1*np.pi))
    return J


def unit_twist_tranlational(vx, vy):
    magnitude = math.sqrt(vx ** 2 + vy ** 2)
    if magnitude == 1:
        return np.array([0, vx, vy])
    else:
        return np.array([0, vx / magnitude, vy / magnitude]).T


def unit_twist_rotational(px, py):
    return np.array([1, py, -px]).T


def calculate_dq(q1, q2, x_desired, y_desired):
    L3 = 0.015
    reference_configuration = np.array([
        [1, 0, L3],
        [0, 1, 0.425],
        [0, 0, 1]
    ])
    reference_twist1 = unit_twist_rotational(0, 0)
    reference_twist2 = unit_twist_rotational(0, 0.24)
    He0 = brockett(reference_configuration, [(reference_twist1, q1), (reference_twist2, q2)])
    J = jacobian_angles(q1)

    pe0 = He0[:2, 2]
    ps = np.array([x_desired, y_desired])  # TODO: insert desired position or velocity?
    K = 10  # TODO: tune K to have good response
    F = K * (ps - pe0)
    # TODO: maybe constrict F like is done in exercise E
    Ws0 = np.array([pe0[0] * F[1] - pe0[1] * F[0], F[0], F[1]])
    tau = J.T @ Ws0

    b = [0.5, 0.25]  # TODO: tune b to have good response
    dq = tau / b
    return dq


if __name__ == "__main__":
    # For testing
    identity = np.eye(3)
    H1 = np.array([
        [1, 0, 0.5],
        [0, 1, 2],
        [0, 0, 1]
    ])
    H2 = np.array([
        [np.cos(0.25 * np.pi), -np.sin(0.25 * np.pi), 0],
        [np.sin(0.25 * np.pi), np.cos(0.25 * np.pi), 0],
        [0, 0, 1]
    ])
    H3 = np.array([
        [np.cos(1 / 3 * np.pi), -np.sin(1 / 3 * np.pi), 0.3],
        [np.sin(1 / 3 * np.pi), np.cos(1 / 3 * np.pi), 0.8],
        [0, 0, 1]
    ])
    H4 = np.array([
        [1, 0, 0],
        [0, 1, 3],
        [0, 0, 1]
    ])
    T1 = np.array([1, 0, 0]).T
    T2 = np.array([0, 0, 1]).T
    T3 = np.array([2, 0, 0]).T
    T4 = np.array([0, 1, 1]).T

    print(h_to_adjoint(identity))
    print(h_to_adjoint(H1))
    print(h_to_adjoint(H2))
    print(h_to_adjoint(H3))
    print(adjoint_to_h(h_to_adjoint(H3)))
    print("printing brocketts")
    print(brockett(H4, [(T1, 45), (T2, 0)]))
    print(brockett(H4, [(T1, 0.25), (T2, 0)]))
    print(brockett(H4, [(T1, 0.25), (T2, math.sqrt(2))]))
    try:
        brockett(H4, [(T3, 45), (T2, 0)])
    except ValueError:
        print("caught correct valueerror")
    try:
        brockett(H4, [(T1, 45), (T4, 0)])
    except ValueError:
        print("caught correct valueerror")
    print(jacobian_twists([unit_twist_rotational(0, 0), unit_twist_tranlational(2, 1)]))
