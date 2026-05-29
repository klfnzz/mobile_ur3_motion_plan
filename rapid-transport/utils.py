from openravepy import *
from pylab import *

import string
import numpy as np
import random

INF = np.infty
EPS = 1e-12
CLA_NOTHING = 0
FORWARD = 0
BACKWARD = 1
############################## POLYNOMIALS ##############################
"""
NB: we adopt the weak-term-first convention for inputs
"""

#_#_#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#_#_#all coefficient_list and trajectory_list are ascending
#_#_#but numpy.poly need descending coefficients, so use [::-1]
#_#_#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
def get_robot_base_z(robot_input):
    """Return the world-frame z height of the robot base_link."""
    base_link = robot_input.GetLink("base_link")
    if base_link is not None:
        return float(base_link.GetTransform()[2, 3])
    return float(robot_input.GetTransform()[2, 3])


def pose_to_homogeneous(pose, z=0):
    """
    将 [x, y, yaw] 转换为 4×4 齐次变换矩阵
    pose: iterable of length 3，格式 [x, y, yaw]
    返回: 4×4 numpy.ndarray
    """
    x, y, yaw = pose
    # 构造旋转矩阵
    c = np.cos(yaw)
    s = np.sin(yaw)
    R = np.array([[ c, -s, 0],
                  [ s,  c, 0],
                  [ 0,  0, 1]])
    # 构造齐次矩阵
    T = np.eye(4)
    T[0:3, 0:3] = R
    T[0:3, 3] = [x, y, z]
    return T


def homogeneous_to_pose2d(T):
    """Convert a homogeneous transform to [x, y, yaw]."""
    return np.array([
        T[0, 3],
        T[1, 3],
        np.arctan2(T[1, 0], T[0, 0]),
    ], dtype=float)


def set_robot_pose2d(robot_input, pose, z=None):
    """Set robot world pose from [x, y, yaw], preserving base height by default."""
    if z is None:
        z = get_robot_base_z(robot_input)
    T_world_base = pose_to_homogeneous(pose, z=z)
    robot_input.SetTransform(T_world_base)
    return T_world_base


def normalize(vector):
    vector_normal = np.linalg.norm(vector)
    assert(not vector_normal == 0)
    return vector/vector_normal


def interpolate_polynomial_1st(q_0, q_1):
    # 原理:已知2点(0,q_0)(1,q_1) q=F(s):
    #   1次多项式插值函数：q 约等于 P(s)=a0 + a1*s 
    #   满足条件：P(0)=q_0; P(1)=q_1. 则可求得以a1,a0：
    # Input:
    #   q_0, q_1: [np.ndarray(joint DOFs, )]
    # Return:
    #   a1, a0:   [np.ndarray(joint DOFs, )]
    a1 = q_1 - q_0
    a0 = q_0
    return a0, a1


def coefficient_set_1st(a0, a1):
    # 系数矩阵
    # Input:
    #   a0, a1:  [np.ndarray(joint DOFs, )]
    # Return:
    #   coefficient_array: [np.ndarray(joint DOFs,2)] 
    #           第1个维度表示系数的维度(=joint DOFs)，第2个维度表示多项式的维度即系数个数(取决于插值多项式的次数+1)
    A0 = np.array(a0)[:, np.newaxis]        # [np.ndarray(joint DOFs,1)]
    A1 = np.array(a1)[:, np.newaxis]
    coefficient_array = np.concatenate( (A0, A1), axis=1  ) # [np.ndarray(joint DOFs,2)] 用于数组拼接，axis=1指定沿着轴 1（即列方向）拼接
    return coefficient_array

def check_path_collision(robot_input, path_input, n_grid_input, direction_input=FORWARD):
    """Check_Path_Collision accepts a robot and a path object as its inputs.
       It returns True if any config along the path is IN-COLLISION.
    """
    env = robot_input.GetEnv()
    amount_dof = robot_input.GetDOF()
    grid_point_temp = np.delete(np.linspace(0, 1, n_grid_input, endpoint=False),[0])
    if direction_input == FORWARD:
        grid_point_set = grid_point_temp[::-1]
    if direction_input == BACKWARD:
        grid_point_set = grid_point_temp
    for s_hat in grid_point_set:
        with robot_input:
            robot_input.SetDOFValues( path_input.eval(s_hat)[3:9], 
                list(range(amount_dof)), CLA_NOTHING)
            T_start = set_robot_pose2d(robot_input, path_input.eval(s_hat)[:3])
            is_in_collision = ( 
                env.CheckCollision( robot_input, CollisionReport() ) 
                or robot_input.CheckSelfCollision( CollisionReport() ) )
            if (is_in_collision):
                return True
    return False

def check_configuration_collision(robot_input, q_input):
    env = robot_input.GetEnv()
    with robot_input:
        T_start = set_robot_pose2d(robot_input, q_input[:3])
        robot_input.SetActiveDOFValues(q_input[3:9])
        in_collision = (env.CheckCollision(robot_input, CollisionReport()) 
            or robot_input.CheckSelfCollision(CollisionReport()))
        return in_collision



