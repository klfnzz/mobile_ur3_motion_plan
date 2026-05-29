import numpy as np
import time
import string
import random
import pylab
from openravepy import CollisionReport
import utils
import heap
import copy
import toppra_sc as toppra

import ipdb

# suppress openrave complain
CLA_NOTHING = 0
X = np.array([1, 0, 0])
Y = np.array([0, 1, 0])
Z = np.array([0, 0, 1])
# global variables
FORWARD = 0
BACKWARD = 1
NOT_INTERSECT = -2
IN_COLLISION = -1
OK = 1
SMALL = 1e-9
is_BACKWARD_enable = True
grid_points = np.linspace(0, 1, 30)
'''
'Algorithm: BUILD_RRT
'Input: A starting configuration q_start
'Output: A tree T rooted at q_start
T.INITIALIZE(q_start)
FOR rep = 1 TO maxrep
  q_rand ← RANDOM_CONFIG()
  EXTEND(T, q_rand)

'Algorithm: EXTEND
'Input: A tree T and a target configuration q_rand
'Effect: Grow T by a new vertex in the direction of q_rand
q_near ← NEAREST_NEIGHBOR(T, q_rand)
IF q_new ← STEER(q_near, q_rand) succeeds THEN
  T.ADD_VERTEX(q_new)
  T.ADD_EDGE([q_near, q_new])

'Algorithm: STEER
'Input: The nearest configuration q_near of Tree in the direction of q_rand, a target configuration q_rand and a given distance r of q_near
'Effect: attempts making a straight motion from q_near towards q_rand.Three cases can happen
IF DISTANCE(q_rand,q_near) <= r THEN
  q_new ← q_rand
ELIF DISTANCE(q_rand,q_near) > r THEN
  q_new ←  the end of the segment of length r from  q_near
ELSE 
  Failure
'''
  
class Config(object):
    # configuration q:一组关节角 [np.ndarray(joint DOFs, )]
    def __init__(self, q):
        self.q = q

class Vertex(object):
    """
    Attributes:
        config          -- stores a Config object
        parent_index    -- **the parent for FORWARD vertex, the child for BACKWARD vertex**
        level           -- its level from the root of the tree (0 for the root)
        drawn           -- True if this vertex has been plotted via Vertex::Plot
    """
    def __init__(self, config, vertex_type = FORWARD):
        self.config = config
        self.type = vertex_type         # vertex_type = FORWARD/BACKWORD
        self.parent_index = None        # this is to be assigned when a vertex was added to a tree: = vertex_near.index
        self.index = 0                  # this is to be assigned after a vertex was added to a tree: = tree.length
        self.level = 0                  # this is to be assigned when a vertex was added to a tree: = vertex_near.level + 1
        self.coefficient_descend = []   # this is to be assigned when a vertex was added to a tree: 
                                        #   For tree_forward: path(vertex_near——>vertex_new) coefficients
                                        #   For tree_backward: path(vertex_new——>vertex_near) coefficients

class Tree(object):
    """
    Input: 
        tree_type = FORWARD     [int]
        vertex_root = None      [class `Vertex`]
        Tree(FORWARD, vertex_start)
    Attributes:
        vertex_list         -- stores all vertices added to the tree
        last_vertex_index   -- stores the index of last_vertex
    """
    def __init__(self, tree_type = FORWARD, vertex_root = None):
        self.vertex_list = []       # [list]: stores all vertices[class Vertex] added to the tree
        if vertex_root is not None:
            self.vertex_list.append(vertex_root)
            self.length = 1
        else:
            self.length = 0         # [int]: 
        self.type = tree_type       # stores the type of tree
        self.last_vertex_index = 0  # stores the index of last_vertex

    def __len__(self):
        '''
        return: the number of the present tree's vertices (include root_vertex)
        '''
        return len(self.vertex_list)

    def __getitem__(self, index):
        '''
        input: a vertex index
        return: the vertex [class Vertex]
        '''
        return self.vertex_list[index]

    def add_vertex(self, parent_index, vertex_new):
        '''
        input: parent_index [int]——> vertex_near[class Vertex], vertex_new[class Vertex]

        '''
        parent = self.vertex_list[parent_index]     # vertex_near
        vertex_new.parent_index = parent_index      # vertex_near.index
        vertex_new.level = parent.level + 1         # vertex_near.level + 1
        vertex_new.index = self.length
        self.vertex_list.append(vertex_new)
        self.length += 1
        # check soundness
        assert(self.length == len(self.vertex_list))

class RRTPlanner(object):
    REACHED = 1
    ADVANCED = 0
    TRAPPED = -1
    def __init__(self, vertex_start, vertex_goal, robot, fixed_base_yaw=None,
                 base_start=None, base_goal=None):
        """
        Initialize a planner. 
        RRTPlanner always has two trees. For a unidirectional planner, 
        the tree end will not be extended and always has only one vertex, 
        vertex_goal.
        """
        self.tree_forward = Tree(FORWARD, vertex_start)
        self.tree_backward = Tree(BACKWARD, vertex_goal)
        self.robot = robot
        self.connecting_string = ''
        self.running_time = 0.0
        #_#_# -1 means using the whole set of vertices as a nearest_neighbor set
        self.nearest_neighbor = -1
        self.amount_iteration = 0
        self.is_found = False
        self.RANDOM_NUMBER_GENERATOR = random.SystemRandom()        
        # default parameters
        self.STEP_SIZE = 0.3
        self.fixed_base_yaw = (None if fixed_base_yaw is None
                               else float(fixed_base_yaw))
        self.base_start = (None if base_start is None
                           else np.asarray(base_start, dtype=float).copy())
        self.base_goal = (None if base_goal is None
                          else np.asarray(base_goal, dtype=float).copy())
        self.base_goal_bias_prob = 0.25 if self.base_goal is not None else 0.0
        self.base_corridor_bias_prob = (
            0.60 if self.base_start is not None and self.base_goal is not None
            else 0.0
        )
        self.base_sampling_margin = 0.35
        if self.fixed_base_yaw is not None:
            self.tree_forward.vertex_list[0].config.q = self._apply_fixed_base_yaw(
                self.tree_forward.vertex_list[0].config.q)
            self.tree_backward.vertex_list[0].config.q = self._apply_fixed_base_yaw(
                self.tree_backward.vertex_list[0].config.q)

    def __str__(self):
        s = 'Total running time: {0} s.\n'.format(self.running_time)
        s += 'Total number of iterations: {0}\n'.format(self.amount_iteration)
        return s

    def extend(self, config_rand):
        raise RRTException("Virtual method not implemented.")

    def connect(self):
        raise RRTException("Virtual method not implemented.")

    def is_feasible_configuration(self, config_rand):
        """
        IsFeasibleConfig checks feasibility of the given Config object. 
        Feasibility conditions are to be determined by each RRT planner.
        """
        raise RRTException("Virtual method not implemented.")

    def is_feasible_trajectory(self):
        """
        IsFeasibleTrajectory checks feasibility of the given trajectorystring.
        Feasibility conditions are to be determined by each RRT planner.
        """
        raise RRTException("Virtual method not implemented.")

    def random_q_joint_values(self, amount_dof):
        # assign lower and upper limits of joint values
        [lower_limits, upper_limits] = self.robot.GetDOFLimits()

        q_rand = np.zeros(amount_dof)
        for dof_range in range(amount_dof):
            q_rand[dof_range] = self.RANDOM_NUMBER_GENERATOR.uniform(
                lower_limits[dof_range], upper_limits[dof_range])

        return q_rand
    def random_joint_values(self, amount_dof):
        """
        返回一个长度为 3 的数组：[x, y, yaw]，
        x ∈ [-100, 100], y ∈ [-100, 100], yaw ∈ [-6.28, 6.28]
        """
        assert amount_dof == 3, "只支持 3 自由度 (x, y, yaw)"
        q_rand = np.zeros(3)
        rand_scalar = self.RANDOM_NUMBER_GENERATOR.random()

        if self.base_goal is not None and rand_scalar < self.base_goal_bias_prob:
            q_rand[:] = self.base_goal[:3]
        elif (self.base_start is not None and self.base_goal is not None
              and rand_scalar < self.base_goal_bias_prob + self.base_corridor_bias_prob):
            xy_low = np.minimum(self.base_start[:2], self.base_goal[:2]) - self.base_sampling_margin
            xy_high = np.maximum(self.base_start[:2], self.base_goal[:2]) + self.base_sampling_margin
            q_rand[0] = self.RANDOM_NUMBER_GENERATOR.uniform(xy_low[0], xy_high[0])
            q_rand[1] = self.RANDOM_NUMBER_GENERATOR.uniform(xy_low[1], xy_high[1])
            if self.fixed_base_yaw is None:
                yaw_low = min(self.base_start[2], self.base_goal[2]) - np.deg2rad(20)
                yaw_high = max(self.base_start[2], self.base_goal[2]) + np.deg2rad(20)
                q_rand[2] = self.RANDOM_NUMBER_GENERATOR.uniform(yaw_low, yaw_high)
        else:
            # 定义各自由度的上下限
            limits = np.array([
                [-100.0,  100.0],   # x 范围
                [-100.0,  100.0],   # y 范围
                [  -6.28,    6.28]  # yaw 范围
            ])
            for i in range(3):
                low, high = limits[i]
                q_rand[i] = self.RANDOM_NUMBER_GENERATOR.uniform(low, high)
        if self.fixed_base_yaw is not None:
            q_rand[2] = self.fixed_base_yaw
        return q_rand

    def _apply_fixed_base_yaw(self, q_input):
        q_output = np.asarray(q_input, dtype=float).copy()
        if self.fixed_base_yaw is not None and q_output.shape[0] >= 3:
            q_output[2] = self.fixed_base_yaw
        return q_output

    def distance(self, config_1, config_2, metric_type=1):
        delta_q = config_1.q - config_2.q
        if (metric_type == 1):
            # norm-2 squared
            return np.dot(delta_q, delta_q)
        elif (metric_type == 2):
            # norm-1
            return np.linalg.norm(delta_q, 1)        

        else:
            raise RRTException("Unknown Distance Metric.")

    def nearest_neighbor_index(self, config_rand, tree_type, 
        custom_nearest_neighbor = 0):
        """
        nearest_neighbor_index returns index of self.nearest_neighbor nearest 
        neighbors of config_rand on the tree specified by treetype.
        """
        if (tree_type == FORWARD):
            tree = self.tree_forward
            amount_vertex = len(tree)
        else:
            tree = self.tree_backward
            amount_vertex = len(tree)
        # distance_list：计算vertex_rand和Tree中所有verteices的距离
        distance_list = [self.distance(config_rand, vertex_hat.config, 
            self.metric_type) for vertex_hat in tree.vertex_list]
        # 将distance_list从【list】转化成【二叉堆】结构
        distance_heap = heap.Heap(distance_list)
        if (custom_nearest_neighbor == 0):
            nearest_neighbor = self.nearest_neighbor # self.nearest_neighbor=-1
        else:
            nearest_neighbor = custom_nearest_neighbor
        if (nearest_neighbor == -1):
            nearest_neighbor = amount_vertex
        else:
            nearest_neighbor = min(self.nearest_neighbor, amount_vertex)
        # def distance_heap.extract_min()：从堆中提取最小值并移除
        #   Return:一个包含两个元素的元组(index, min_element)，其中第一个元素是最小值的索引，第二个元素是最小值
        nearest_neighbor_index = [ distance_heap.extract_min()[0] for 
            i_range in range(nearest_neighbor) ]
        return nearest_neighbor_index

    def run(self, allotted_time):
        if (self.is_found):
            print ("The planner has already found a path.")
            return True
        time_sum = 0.0 # total running time for this run
        prev_iter = self.amount_iteration
        amount_dof = 9
        while (time_sum < allotted_time):
            self.amount_iteration += 1
            print ("\033[1;34m iteration:", self.amount_iteration, "\033[0m")
            #_#_#Bold_Blue="\[\033[1;34m\]" 
            #_#_#Color_Off="\[\033[0m\]"
            running_time_begin = time.time()
            q_rand_c = self.random_joint_values(amount_dof-6)
            q_rand_r = self.random_q_joint_values(amount_dof-3)
            q_rand = np.hstack((q_rand_c, q_rand_r))  # [x,y,yaw,q1,q2,q3,q4,q5,q6]
            q_rand = self._apply_fixed_base_yaw(q_rand)
            config_rand = Config(q_rand)
            if (self.extend(config_rand) != self.TRAPPED):  # 扩展节点config_rand
                print ("\033[1;32mTree start : ", len(self.tree_forward.vertex_list),) 
                print ("; Tree end : ", len(self.tree_backward.vertex_list), "\033[0m")
                if (self.connect() == self.REACHED): # 连接两棵树
                    print ("\033[1;32mPath found")
                    print ("    Total number of iteration: {0}".format(
                        self.amount_iteration))
                    running_time_end = time.time()
                    time_sum += running_time_end - running_time_begin
                    self.running_time += time_sum
                    print ("    Total running time: {0} s.\033[0m".format(
                        self.running_time))
                    self.is_found = True
                    return True # running result
                    
            running_time_end = time.time()
            time_sum += running_time_end - running_time_begin
            self.running_time += running_time_end - running_time_begin
        print ("\033[1;31mAllotted time ({0} s. is exhausted after {1} \
        amount_iteration\033[0m".format(allotted_time, 
            self.amount_iteration - prev_iter))
        return False
def pose_to_homogeneous(pose, z=0):
    return utils.pose_to_homogeneous(pose, z=z)



class BiRRTPlanner(RRTPlanner):
    def __init__(self, vertex_start, vertex_goal, robot, all_constraints, 
        nearest_neighbor = -1, metric_type = 1, fixed_base_yaw=None,
        base_start=None, base_goal=None):

        super(BiRRTPlanner, self).__init__(
            vertex_start, vertex_goal, robot, fixed_base_yaw=fixed_base_yaw,
            base_start=base_start, base_goal=base_goal)

        self.all_constraints = all_constraints
        self.nearest_neighbor = nearest_neighbor
        self.metric_type = metric_type
        self._max_repeat = -1
        self.handles_vertex_c = []
        self.handles_vertex_r = []
        self.handles_edge_c = []
        self.handles_edge_r = []
        self.handles_plot = []
        self.connected_coefficient_descend = []

    def plot_vertex(self, q_input):
        with self.robot:
            T_start = utils.set_robot_pose2d(self.robot, q_input[:3])
            self.robot.SetActiveDOFValues(q_input[3:9])
            print ("Plot vertex at: ", q_input)
            vertex_transform = self.robot.GetLink('ee_link').GetTransform()
        vertex_position_c = T_start[0:3, 3]
        self.handles_vertex_c.append( self.robot.GetEnv().plot3( 
            points=np.array(vertex_position_c ), pointsize=5, 
            colors=np.array( (0,0,1) ), drawstyle=1) )
        vertex_position_r = vertex_transform[0:3, 3]
        self.handles_vertex_r.append( self.robot.GetEnv().plot3( 
            points=np.array(vertex_position_r ), pointsize=5, 
            colors=np.array( (0,0,1) ), drawstyle=1) )

    def plot_edge(self, path_input):
        edge_transform_set_c = []
        edge_transform_set_r = []
        for s_grid in np.linspace(0, 1, 100):
            with self.robot:
                T_start = utils.set_robot_pose2d(self.robot, path_input.eval(s_grid)[:3])
                self.robot.SetActiveDOFValues(path_input.eval(s_grid)[3:9])
                transform = self.robot.GetLink('ee_link').GetTransform()
                position_c = T_start[0:3, 3]
                position_r = transform[0:3, 3]
                edge_transform_set_c.append(position_c)
                edge_transform_set_r.append(position_r)
        self.handles_edge_c.append(self.robot.GetEnv().drawlinestrip(
            points=np.array(edge_transform_set_c),linewidth=0.5,
            colors=np.array((0,1,0)) ) )
        self.handles_edge_r.append(self.robot.GetEnv().drawlinestrip(
            points=np.array(edge_transform_set_r),linewidth=0.5,
            colors=np.array((0,1,0)) ) )

    def plot_parameterization(self, reachable_set_input=None, 
        controllable_set_input=None, feasible_set_input=None, s_d_set=None):
        pylab.ion()
        if reachable_set_input is not None:
            pylab.plot(reachable_set_input[:, 0], 'g--', label="reachable sets")
            pylab.plot(reachable_set_input[:, 1], 'g--')
        if reachable_set_input is not None:
            pylab.plot(controllable_set_input[:, 0], 'k.', 
                label="controllable sets")
            pylab.plot(controllable_set_input[:, 1], 'k.',)
        if feasible_set_input is not None:
            pylab.plot(feasible_set_input[:, 0], 'r:', label="feasible sets")
            pylab.plot(feasible_set_input[:, 0], 'r:')
        if s_d_set is not None:
            pylab.plot(s_d_vec, label="Velocity profile")
        pylab.title("Path-position path-velocity plot")
        pylab.xlabel("Path position")
        pylab.ylabel("Path velocity square")
        pylab.legend()
        pylab.ioff()
        pylab.show()

    def extend(self, config_rand):
        if not is_BACKWARD_enable:
            return self.extend_forward(config_rand)
        #_#_#if there is no is_BACKWARD_enable, 
        #_#_#then will do ExtendFORWARD & ExtendBACKWARD alternatively
        # 根据迭代次数的奇偶性，交替地执行前向和后向的路径扩展操作，以确保在搜索过程中在前向和后向之间进行均衡。
        if (np.mod(self.amount_iteration - 1, 2) == FORWARD):
            return self.extend_forward(config_rand)
        else:
            return self.extend_backward(config_rand)

    def extend_forward(self, config_rand):
        amount_dof = self.robot.GetActiveDOF()
        status = self.TRAPPED # -1
        #_#_#the nearest vertex in tree_forward to the vertex_rand
        nearest_neighbor_index = self.nearest_neighbor_index(config_rand, FORWARD)
        for n_n_i_hat in nearest_neighbor_index:
            print ("extend_forward from index = {0}".format(n_n_i_hat))
            vertex_near = self.tree_forward.vertex_list[n_n_i_hat]
            q_beg = vertex_near.config.q

            #_#_# check if config_rand is too far from vertex_near
            delta = self.distance(vertex_near.config, config_rand)
            if (delta <= self.STEP_SIZE):
                q_end = config_rand.q
                status = self.REACHED
            else:
                # 单位向量：(config_rand.q - q_beg)/np.sqrt(delta)
                q_end = q_beg + self.STEP_SIZE*(config_rand.q - 
                    q_beg)/np.sqrt(delta)
                status = self.ADVANCED
            q_end = self._apply_fixed_base_yaw(q_end)
            

            config_new = Config(q_end)
            if not (delta <= self.STEP_SIZE):
                delta = self.distance(vertex_near.config, config_new)
            n_check_grid = int(delta * 20)

            import importlib, inspect, sys
            try:
                m = importlib.import_module('utils')
                print("utils loaded from:", getattr(m, '__file__', None))
                print("has attr check_configuration_collision:", hasattr(m, 'check_configuration_collision'))
                if hasattr(m, 'check_configuration_collision'):
                    print("signature:", inspect.signature(m.check_configuration_collision))
                    try:
                        src = inspect.getsource(m.check_configuration_collision)
                        print("source head:\n", "\n".join(src.splitlines()[:20]))
                    except Exception as e:
                        print("cannot get source:", e)
            except Exception as e:
                print("import utils failed:", type(e), e)

            #_#_#print "\textend_forward : Check_Configuration_Collision"
            config_in_collision = utils.check_configuration_collision(
                self.robot, config_new.q)
            if config_in_collision:
                status = self.TRAPPED
                #_#_#print "\t!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                continue

            a0, a1 = utils.interpolate_polynomial_1st(q_beg, q_end) # [np.ndarray(joint DOF, )] tree_forward:vertex_near——>vertex_new
            # coefficient_ascend: [np.ndarray(joint DOFs,2)] 
            #   第1个维度表示系数的维度(=joint DOFs)，第2个维度表示多项式的维度即系数个数(取决于插值多项式的次数+1)
            coefficient_ascend = utils.coefficient_set_1st(a0, a1)  # For tree_forward: path(vertex_near——>vertex_new) coefficients

            path_instance = toppra.PolynomialPath(coefficient_ascend)
            # print "\textend_forward : check_path_collision"
            path_in_collision = utils.check_path_collision(
                self.robot, path_instance, n_check_grid, direction_input=FORWARD)
            if path_in_collision:
                #_#_#print "\t!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                del path_instance
                continue

            vertex_new = Vertex(config_new)

            vertex_new.level = vertex_near.level + 1
            coefficient_descend = utils.coefficient_set_1st(a1, a0)
            vertex_new.coefficient_descend = coefficient_descend    # For tree_forward: path(vertex_near——>vertex_new) coefficients
            self.tree_forward.add_vertex(n_n_i_hat, vertex_new)
            self.plot_vertex(vertex_new.config.q)
            self.plot_edge(path_instance)
            status = self.REACHED

            del path_instance
            print ("extend_forward : Successful extension")
            return status
        return status

    def extend_backward(self, config_rand):
        amount_dof = self.robot.GetActiveDOF()
        status = self.TRAPPED
        #_#_#the nearest vertex in tree_backward to the vertex_rand
        nearest_neighbor_index = self.nearest_neighbor_index(config_rand, BACKWARD)
        for n_n_i_hat in nearest_neighbor_index:
            print ("extend_backward from index = {0}".format(n_n_i_hat))
            vertex_near = self.tree_backward.vertex_list[n_n_i_hat]
            q_end = vertex_near.config.q

            #_#_# check if config_rand is too far from vertex_near
            delta = self.distance(vertex_near.config, config_rand)
            if (delta <= self.STEP_SIZE):
                q_beg = config_rand.q
                status = self.REACHED
            else:
                q_beg = q_end + self.STEP_SIZE*(config_rand.q - 
                    q_end)/np.sqrt(delta)
                status = self.ADVANCED
            q_beg = self._apply_fixed_base_yaw(q_beg)

            config_new = Config(q_beg)
            if not (delta <= self.STEP_SIZE):
                delta = self.distance(vertex_near.config, config_new)
            n_check_grid = int(delta * 20)

            #_#_#print "\textend_backward : Check_Configuration_Collision"
            config_in_collision = utils.check_configuration_collision(
                self.robot, config_new.q)
            if config_in_collision:
                status = self.TRAPPED
                #_#_#print "\t!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                continue

            a0, a1 = utils.interpolate_polynomial_1st(q_beg, q_end)
            coefficient_ascend = utils.coefficient_set_1st(a0, a1)  # For tree_backward: path(vertex_new——>vertex_near) coefficients

            path_instance = toppra.PolynomialPath(coefficient_ascend)
            #_#_#print "\textend_backward : check_path_collision"
            path_in_collision = utils.check_path_collision(
                self.robot, path_instance, n_check_grid, direction_input=BACKWARD)
            if path_in_collision:
                #_#_#print "\t!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                del path_instance
                continue

            vertex_new = Vertex(config_new)

            vertex_new.level = vertex_near.level + 1
            coefficient_descend = utils.coefficient_set_1st(a1, a0)
            vertex_new.coefficient_descend = coefficient_descend    # For tree_backward: path(vertex_new——>vertex_near) coefficients
            self.tree_backward.add_vertex(n_n_i_hat, vertex_new)
            self.plot_vertex(vertex_new.config.q)
            self.plot_edge(path_instance)
            status = self.REACHED
            del path_instance
            print ("extend_backward : Successful extension")
            return status
        status = self.TRAPPED
        return status
   
    def connect(self):
        if not is_BACKWARD_enable:
            return self.connect_backward()
        # 根据迭代次数的奇偶性，交替地执行前向和后向的路径连接操作，以确保在搜索过程中在前向和后向之间进行均衡。
        if (np.mod(self.amount_iteration - 1, 2) == FORWARD):
            return self.connect_backward()
        else:
            return self.connect_forward()

    def connect_backward(self):
        vertex_test = self.tree_forward.vertex_list[-1]
        #_#_#the nearest vertex in tree_backward to the last vertex in tree_forward
        #_#_#if is_BACKWARD_enable == False, tree_backward has only one vertex
        nearest_neighbor_index = self.nearest_neighbor_index(vertex_test.config, BACKWARD)
        status = self.TRAPPED
        for n_n_i_hat in nearest_neighbor_index:
            print ("connect_backward from tree_forward.index={0} to tree_backward.index={1}".format(vertex_test.index, n_n_i_hat))
            vertex_near = self.tree_backward.vertex_list[n_n_i_hat]
            q_end = vertex_near.config.q
            
            q_beg = vertex_test.config.q

            a0, a1 = utils.interpolate_polynomial_1st(q_beg, q_end)
            coefficient_ascend = utils.coefficient_set_1st(a0, a1)
            
            #_#_#print "\tconnect_backward : check_dof_limit"

            path_instance = toppra.PolynomialPath(coefficient_ascend)
            #_#_#print "\tconnect_backward : check_path_collision"
            path_in_collision = utils.check_path_collision(
                self.robot, path_instance, 100)
            if path_in_collision:
                #_#_#print "\t!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                del path_instance
                continue

            status = self.REACHED
            coefficient_descend = utils.coefficient_set_1st(a1, a0)
            self.connected_coefficient_descend = coefficient_descend
            self.tree_forward.last_vertex_index = vertex_test.index
            self.tree_backward.last_vertex_index = vertex_near.index
            self.plot_edge(path_instance)
            del path_instance
            return status
        status = self.TRAPPED
        return status

    def connect_forward(self):
        vertex_test = self.tree_backward.vertex_list[-1]
        #_#_#the nearest vertex in tree_forward to the last vertex in tree_backward
        #_#_#if is_BACKWARD_enable == False, tree_backward has only one vertex
        nearest_neighbor_index = self.nearest_neighbor_index(vertex_test.config, FORWARD)
        status = self.TRAPPED
        for n_n_i_hat in nearest_neighbor_index:
            print ("connect_forward from tree_forward.index={0} to tree_backward.index={1}".format(n_n_i_hat, vertex_test.index))
            vertex_near = self.tree_forward.vertex_list[n_n_i_hat]
            q_end = vertex_test.config.q

            q_beg = vertex_near.config.q

            a0, a1 = utils.interpolate_polynomial_1st(q_beg, q_end)
            coefficient_ascend = utils.coefficient_set_1st(a0, a1)

            path_instance = toppra.PolynomialPath(coefficient_ascend)
            #_#_#print "\tconnect_forward : check_path_collision"
            path_in_collision = utils.check_path_collision(
                self.robot, path_instance, 100)
            if path_in_collision:
                #_#_#print "\t!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                del path_instance
                continue

            status = self.REACHED
            coefficient_descend = utils.coefficient_set_1st(a1, a0)
            self.connected_coefficient_descend = coefficient_descend
            self.tree_forward.last_vertex_index = vertex_near.index
            self.tree_backward.last_vertex_index = vertex_test.index
            self.plot_edge(path_instance)
            del path_instance
            return status
        status = self.TRAPPED
        return status

    def generate_final_coefficient(self):
        connected_coefficient_descend = self.connected_coefficient_descend  # [list[???]]包含两棵树连接路径的下降的系数
        coefficient_set_temp = np.array(
            connected_coefficient_descend)[np.newaxis, :, :]    # [3维 np.ndarray(1,joint DOFs,2)]
            
        # 用来收集从 forward 树回溯到根的 waypoints
        waypoints_forward = []
        # first, 把 forward 树的最后一个顶点也收进来
        waypoints_forward.append(self.tree_forward.vertex_list[self.tree_forward.last_vertex_index].config.q)    

        amount_segment = 1
        vertex_index = self.tree_forward.last_vertex_index
        while vertex_index != 0:    #while vertex_index is not 0:
            
            coefficient_temp_1 = self.tree_forward.vertex_list[
                vertex_index].coefficient_descend   # [np.ndarray(joint DOFs,2)]
            coefficient_temp_2 = np.array(coefficient_temp_1)[np.newaxis, :, :] # [3维 np.ndarray(1,joint DOFs,2)]
            coefficient_set_temp = np.concatenate(
                (coefficient_temp_2, coefficient_set_temp), axis=0 ) # 沿着轴0拼接 [np.ndarray(+1,joint DOFs,2)]
            vertex_index = self.tree_forward.vertex_list[vertex_index].parent_index     # **the parent for FORWARD vertex**
            waypoints_forward.append(self.tree_forward.vertex_list[vertex_index].config.q)
            amount_segment += 1
        # coefficient_set_temp: 
        #   [np.ndarray(1 + tree_forward.vertex_list[tree_forward.last_vertex_index].level, joint DOFs, 2)]
        # amount_segment: [int] 
        #   = 1 + tree_forward.vertex_list[tree_forward.last_vertex_index].level
        
        waypoints_backward = []
        waypoints_backward.append(self.tree_backward.vertex_list[self.tree_backward.last_vertex_index].config.q)


        vertex_index = self.tree_backward.last_vertex_index
        while vertex_index != 0:    #while vertex_index is not 0:
            coefficient_temp_1 = self.tree_backward.vertex_list[
                vertex_index].coefficient_descend
            coefficient_temp_2 = np.array(coefficient_temp_1)[np.newaxis, :, :]
            coefficient_set_temp = np.concatenate(
                (coefficient_set_temp, coefficient_temp_2), axis=0 )
            vertex_index = self.tree_backward.vertex_list[vertex_index].parent_index    # **the child for BACKWARD vertex**
            waypoints_backward.append(self.tree_backward.vertex_list[vertex_index].config.q)
            amount_segment += 1
        # coefficient_set_temp: 
        #   [np.ndarray(1 + tree_forward.vertex_list[tree_forward.last_vertex_index].level + tree_backward.vertex_list[tree_backward.last_vertex_index].level, joint DOFs, 2)]
        #   第1个维度表示路径的段数(tree的层数)
        #   第2个维度是系数的维度(=joint DOFs)
        #   第3个维度是每个段插值多项式维度即系数个数(取决于插值多项式的次数+1)
        # amount_segment:   [int] 
        #   = 1 + tree_forward.vertex_list[tree_forward.last_vertex_index].level + tree_backward.vertex_list[tree_backward.last_vertex_index].level
        shape = np.array(coefficient_set_temp.shape)
        coefficient_set = np.zeros((shape[2], shape[0], shape[1]))  
        for i in range(shape[1]):
            for j in range(shape[0]):
                for k in range(shape[2]):
                    coefficient_set[k, j, i] = coefficient_set_temp[j, i, k]
        # 从tree_forward的root到tree_backward的root的所有段的路径系数集
        # [np.ndarray(2, amount_segment, joint DOFs)]
        #   第1个维度是每个段插值多项式维度即系数个数(取决于插值多项式的次数+1)
        #   第2个维度是从前向树root到后向树root的路径的段数(=amount_segment)
        #   第3个维度是系数的维度(=joint DOFs)
        waypoints_forward = waypoints_forward[::-1]

        waypoints = waypoints_forward + waypoints_backward  # 去掉 meeting 重复点
        waypoints = np.array(waypoints)
        return coefficient_set, amount_segment, waypoints

class RRTException(Exception):
    """Base class for exceptions for RRT planners"""
    pass
