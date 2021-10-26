import math
import numpy as np


def HToAdjoint(H_matrix):
    res = np.eye(3)
    res[1:3, 1:3] = H_matrix[0:2, 0:2]
    res[1, 0] = H_matrix[1, 2]
    res[2, 0] = H_matrix[0, 2] * -1
    return res


def AdjointToH(Adjoint):
    res = np.eye(3)
    res[0:2, 0:2] = Adjoint[1:3, 1:3]
    res[1, 2] = Adjoint[1, 0]
    res[0, 2] = Adjoint[2, 0] * -1
    return res


def brockett(H0_matrix, *twistspairs, degrees=True):
    """
    Creates H matrix using brockett
    Inputs: H0_matrix is the H matrix in the reference configuration
    twistpairs are tuples of a twist and the corresponding q (angle or translation
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
            if (math.sqrt(twist[1]**2 + twist[2]**2) - 1) > 0.01:
                raise ValueError("There is a non unit twist in brockett")
            x = twist[1]
            y = twist[2]
            current_matrix[0, 2] = x * q
            current_matrix[1, 2] = y * q
            res = res @ current_matrix
        elif twist[0] == 1:
            # Rotational joint
            if degrees:
                c = np.cos(np.radians(q))
                s = np.sin(np.radians(q))
            else:
                c = np.cos(q*np.pi)
                s = np.sin(q*np.pi)
            current_matrix[0:2,0:2] = np.array([[c,-s],[s,c]])
            x = twist[2] * -1
            y = twist[1]
            current_matrix[0,2] = x - x*c + y*s
            current_matrix[1,2] = y - x*s - y*c
            res = res @ current_matrix
        else:
            raise ValueError("There is a non unit twist in brockett")
    res = res @ H0_matrix
    return res


def TwistToTilde(twist):
    """Returns tilde form of twist"""
    return np.array([
        [0, -twist[0], twist[1]],
        [twist[0], 0, twist[2]],
        [0, 0, 0]
    ])

def TildeToTwist(tilde):
    """Returns the twist form of a tilde twist"""
    return np.array([tilde[1,0],tilde[0,2],tilde[1,2]])

if __name__ == "__main__":
    # For testing
    identity = np.eye(3)
    H1 = np.array([
        [1,0,0.5],
        [0,1,2],
        [0,0,1]
    ])
    H2 = np.array([
        [np.cos(0.25*np.pi),-np.sin(0.25*np.pi),0],
        [np.sin(0.25*np.pi),np.cos(0.25*np.pi),0],
        [0,0,1]
    ])
    H3 = np.array([
        [np.cos(1/3*np.pi),-np.sin(1/3*np.pi),0.3],
        [np.sin(1/3*np.pi),np.cos(1/3*np.pi),0.8],
        [0,0,1]
    ])
    H4 = np.array([
        [1,0,0],
        [0,1,3],
        [0,0,1]
    ])
    T1 = np.array([1,0,0])
    T2 = np.array([0,0,1])
    T3 = np.array([2,0,0])
    T4 = np.array([0,1,1])


    print(HToAdjoint(identity))
    print(HToAdjoint(H1))
    print(HToAdjoint(H2))
    print(HToAdjoint(H3))
    print(AdjointToH(HToAdjoint(H3)))
    print("printing brocketts")
    print(brockett(H4, (T1, 45), (T2,0)))
    print(brockett(H4, (T1, 0.25), (T2, 0), degrees=False))
    print(brockett(H4, (T1, 0.25), (T2, math.sqrt(2)), degrees=False))
    try:
        brockett(H4, (T3, 45), (T2,0))
    except ValueError:
        print("caught correct valueerror")
    try:
        brockett(H4, (T1, 45), (T4, 0))
    except ValueError:
        print("caught correct valueerror")
