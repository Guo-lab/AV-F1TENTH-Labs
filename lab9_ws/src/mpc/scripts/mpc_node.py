#!/usr/bin/env python3
"""
Dynamic Single Track MPC waypoint tracker

@ref: f1tenth planning
"""

import math
from dataclasses import dataclass, field
from weakref import ref

import cvxpy
import numpy as np
import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray, Marker
import tf_transformations

from scipy.linalg import block_diag
from scipy.sparse import block_diag, csc_matrix, diags
from scipy.interpolate import splev, splprep
from scipy.spatial.transform import Rotation as R

from utils import (
    nearest_point,
    construct_full_path,
    construct_trajectory,
    construct_mpc_path,
    construct_trajectory_trees,
)

# TODO CHECK: include needed ROS msg type headers and libraries

"""NOTE: 
    Could not rotate: horizon too short
    reverse rotation:
"""


@dataclass
class mpc_config:
    state_len: int = 4  # length of kinematic state vector: z = [x, y, v, yaw]
    input_len: int = 2  # length of input vector: u = = [steering speed, acceleration]

    # ---------------------------------------------------
    #
    # TODO: you may need to tune the following matrices
    #
    # input cost matrix, penalty for inputs - [accel, steering_speed] # field(default_factory=lambda: np.diag([0.01, 100.0]))
    Rk: list = field(default_factory=lambda: np.diag([0.01, 100.0]))
    # Rk: list = field(default_factory=lambda: np.diag([0.01, 100.0]))
    #
    # input difference cost matrix, penalty for change of inputs - [accel, steering_speed] # field(default_factory=lambda: np.diag([0.01, 100.0]))
    Rdk: list = field(default_factory=lambda: np.diag([0.01, 100.0]))
    #
    # state error cost matrix, for the the next (T) prediction time steps [x, y, delta, v, yaw, yaw-rate, beta] # field(default_factory=lambda: np.diag([13.5, 13.5, 5.5, 13.0]))
    # Qk: list = field(default_factory=lambda: np.diag([70, 70, 24.0, 45]))
    Qk: list = field(default_factory=lambda: np.diag([19.5, 19.5, 8.5, 17.5]))
    #
    # final state error matrix, penalty  for the final state constraints: [x, y, delta, v, yaw, yaw-rate, beta] # field(default_factory=lambda: np.diag([13.5, 13.5, 5.5, 13.0]))
    # Qfk: list = field(default_factory=lambda: np.diag([40, 40, 25, 50]))
    Qfk: list = field(default_factory=lambda: np.diag([23.5, 23.5, 5.5, 18.0]))
    #
    # ---------------------------------------------------

    N_IND_SEARCH: int = 24  # Search index number

    finite_time_horizon: int = 17  # finite time horizon length kinematic
    time_step: float = 0.05  # time step [s] kinematic
    dist_step: float = 0.03  # dist step [m] kinematic

    LENGTH: float = 0.58  # Length of the vehicle [m]
    WIDTH: float = 0.31  # Width of the vehicle [m]
    WB: float = 0.33  # Wheelbase [m]

    MIN_STEER: float = -0.4189  # minimum steering angle [rad]
    MAX_STEER: float = 0.4189  # maximum steering angle [rad]
    MAX_DSTEER: float = np.deg2rad(180.0)  # maximum steering speed [rad/s]

    MAX_SPEED: float = 1.2  # maximum speed [m/s]
    MIN_SPEED: float = 0.0  # minimum backward speed [m/s]
    MAX_ACCEL: float = 0.9  # maximum acceleration [m/ss]


@dataclass
class State:
    x: float = 0.0
    y: float = 0.0
    delta: float = 0.0
    v: float = 0.0
    yaw: float = 0.0
    yawrate: float = 0.0
    beta: float = 0.0


class MPC(Node):
    """
    Implement Kinematic MPC on the car. All vehicle pose used by the planner should be in the map frame.
    """

    def __init__(self):
        super().__init__("mpc_node")
        # TODO: create ROS subscribers and publishers
        #       use the MPC as a tracker (similar to pure pursuit)

        waypoints_file_path = "./src/mpc/data/waypoints.csv"

        odom_topic = "/ego_racecar/odom"

        drive_topic = "/drive"
        full_path_topic = "/full_path"
        mpc_horizon_topic = "/horizon"

        self.pose_sub_ = self.create_subscription(Odometry, odom_topic, self.pose_callback, 1)

        self.drive_pub_ = self.create_publisher(AckermannDriveStamped, drive_topic, 1)
        self.waypoints_pub_ = self.create_publisher(Path, full_path_topic, 1)
        self.path_pub_ = self.create_publisher(MarkerArray, mpc_horizon_topic, 1)
        self.mpc_path_pub_ = self.create_publisher(Path, "/mpc_path", 1)
        self.mpc_traj_tree_pub_ = self.create_publisher(MarkerArray, "/traj_trees", 1)

        # TODO: get waypoints here
        self.waypoints = np.genfromtxt(waypoints_file_path, delimiter=",")
        self.get_logger().info(f"waypoints loaded: {self.waypoints.shape}")
        assert self.waypoints.shape[1] == 4, "waypoints should be [N x 4]: x, y, yaw, velocity"

        self.config = mpc_config()
        self.odelta_v = None
        self.odelta = None
        self.oa = None

        self.init_flag = 0
        self.debug = True  # True # False

        self.mpc_prob_init()
        self.log_timer = self.create_timer(1.0, self.log_state)
        self.ref_state_for_logging = None
        self.vehicle_state_for_logging = None

        full_path_to_publish = construct_full_path(self.waypoints)
        self.waypoints_pub_.publish(full_path_to_publish)

    def mpc_prob_init(self):
        """
        Create MPC quadratic optimization problem using cvxpy, solver: OSQP
            Will be solved every iteration for control.

        More MPC problem information here: https://osqp.org/docs/examples/mpc.html
            More QP example in CVXPY here: https://www.cvxpy.org/examples/basic/quadratic_program.html

        @ref: Public Repo https://github.com/f1tenth/f1tenth_planning/

        """
        # Initialize and create vectors for the optimization problem

        # Vehicle State Vector (4x25)
        self.xk = cvxpy.Variable((self.config.state_len, self.config.finite_time_horizon + 1))

        # Control Input vector (2x24)
        self.uk = cvxpy.Variable((self.config.input_len, self.config.finite_time_horizon))

        objective = 0.0  # Objective value of the optimization problem
        constraints = []  # Create constraints array

        # Initialize reference vectors
        self.x0k = cvxpy.Parameter((self.config.state_len,))
        self.x0k.value = np.zeros((self.config.state_len,))

        # Initialize reference trajectory parameter
        self.ref_traj_k = cvxpy.Parameter(
            (self.config.state_len, self.config.finite_time_horizon + 1)
        )
        self.ref_traj_k.value = np.zeros(
            (self.config.state_len, self.config.finite_time_horizon + 1)
        )

        # Initializes block diagonal form of R = [R, R, ..., R] (input_len*T, input_len*T)
        R_block = block_diag(tuple([self.config.Rk] * self.config.finite_time_horizon))

        # Initializes block diagonal form of Rd = [Rd, ..., Rd] (input_len*(T-1), input_len*(T-1))
        Rd_block = block_diag(tuple([self.config.Rdk] * (self.config.finite_time_horizon - 1)))

        # Initializes block diagonal form of Q = [Q, Q, ..., Qf] (NX*T, NX*T)
        Q_block = [self.config.Qk] * (self.config.finite_time_horizon)
        Q_block.append(self.config.Qfk)
        Q_block = block_diag(tuple(Q_block))

        # Formulate and create the finite-horizon optimal control problem (objective function)
        # The FTOCP has the horizon of T timesteps

        # --------------------------------------------------------
        # TODO: fill in the objectives here, you should be using cvxpy.quad_form() somehwhere

        # TODO: Objective part 1: Influence of the control inputs: Inputs u multiplied by the penalty R
        objective += cvxpy.quad_form(cvxpy.vec(self.uk), R_block)

        # TODO: Objective part 2: Deviation of the vehicle from the reference trajectory weighted by Q, including final Timestep T weighted by Qf
        objective += cvxpy.quad_form(cvxpy.vec(self.xk - self.ref_traj_k), Q_block)

        # TODO: Objective part 3: Difference from one control input to the next control input weighted by Rd
        objective += cvxpy.quad_form(cvxpy.vec(cvxpy.diff(self.uk, axis=1)), Rd_block)

        # --------------------------------------------------------

        # Constraints 1: Calculate the future vehicle behavior/states based on the vehicle dynamics model matrices
        # Evaluate vehicle Dynamics for next T timesteps
        A_block = []
        B_block = []
        C_block = []
        # init path to zeros
        path_predict = np.zeros((self.config.state_len, self.config.finite_time_horizon + 1))
        for t in range(self.config.finite_time_horizon):
            A, B, C = self.get_model_matrix(path_predict[2, t], path_predict[3, t], 0.0)
            A_block.append(A)
            B_block.append(B)
            C_block.extend(C)

        A_block = block_diag(tuple(A_block))
        B_block = block_diag(tuple(B_block))
        C_block = np.array(C_block)

        # [AA] Sparse matrix to CVX parameter for proper stuffing
        # Reference: https://github.com/cvxpy/cvxpy/issues/1159#issuecomment-718925710
        m, n = A_block.shape
        self.Annz_k = cvxpy.Parameter(A_block.nnz)
        data = np.ones(self.Annz_k.size)
        rows = A_block.row * n + A_block.col
        cols = np.arange(self.Annz_k.size)
        Indexer = csc_matrix((data, (rows, cols)), shape=(m * n, self.Annz_k.size))
        self.Annz_k.value = A_block.data  # Setting sparse matrix data
        self.Ak_ = cvxpy.reshape(Indexer @ self.Annz_k, (m, n), order="C")  # sparse A_block

        # Same as A
        m, n = B_block.shape
        self.Bnnz_k = cvxpy.Parameter(B_block.nnz)
        data = np.ones(self.Bnnz_k.size)
        rows = B_block.row * n + B_block.col
        cols = np.arange(self.Bnnz_k.size)
        Indexer = csc_matrix((data, (rows, cols)), shape=(m * n, self.Bnnz_k.size))
        self.Bnnz_k.value = B_block.data
        self.Bk_ = cvxpy.reshape(Indexer @ self.Bnnz_k, (m, n), order="C")

        # No need for sparse matrices for C as most values are parameters
        self.Ck_ = cvxpy.Parameter(C_block.shape)
        self.Ck_.value = C_block

        # -------------------------------------------------------------
        # TODO: Constraint part 1:
        #       Add dynamics constraints to the optimization problem. This constraint should be based on a few variables:
        #       self.xk, self.Ak_, self.Bk_, self.uk, and self.Ck_
        # Future vehicle states must follow the linearized vehicle dynamic model.
        constraints += [
            cvxpy.vec(self.xk[:, 1:])
            == self.Ak_ @ cvxpy.vec(self.xk[:, :-1])
            + self.Bk_ @ cvxpy.vec(self.uk)
            + (self.Ck_)
        ]

        # TODO: Constraint part 2:
        #       Add constraints on steering, change in steering angle cannot exceed steering angle speed limit. Should be based on:
        #       self.uk, self.config.MAX_DSTEER, self.config.time_step
        constraints += [
            cvxpy.abs(cvxpy.diff(self.uk[1, :]))
            <= self.config.MAX_DSTEER * self.config.time_step
        ]

        # TODO: Constraint part 3:
        #       Add constraints on upper and lower bounds of states and inputs and initial state constraint, should be based on:
        #       self.xk, self.x0k, self.config.MAX_SPEED, self.config.MIN_SPEED,
        #       self.uk, self.config.MAX_ACCEL, self.config.MAX_STEER
        # Initial state in the plan for the current horizon must match current vehicle state.
        # Inputs generated must be within vehicle limits.
        constraints += [self.xk[:, 0] == self.x0k]

        constraints += [self.xk[2, :] <= self.config.MAX_SPEED]
        constraints += [self.xk[2, :] >= self.config.MIN_SPEED]

        constraints += [cvxpy.abs(self.uk[0, :]) <= self.config.MAX_ACCEL]
        constraints += [cvxpy.abs(self.uk[1, :]) <= self.config.MAX_STEER]

        # -------------------------------------------------------------

        # Create the optimization problem in CVXPY and setup the workspace
        # Optimization goal: minimize the objective function
        self.MPC_prob = cvxpy.Problem(cvxpy.Minimize(objective), constraints)

    def log_state(self):
        if not self.debug:
            self.get_logger().info(
                f"state: {self.vehicle_state_for_logging.x}, {self.vehicle_state_for_logging.y}, {self.vehicle_state_for_logging.v}, {self.vehicle_state_for_logging.yaw}"
            )
            self.get_logger().info(
                f"ref_x: {self.ref_state_for_logging.x}, ref_y: {self.ref_state_for_logging.y}, ref_v: {self.ref_state_for_logging.v}, ref_yaw: {self.ref_state_for_logging.yaw}"
            )

    def pose_callback(self, pose_msg):
        # TODO: extract pose from ROS msg
        velocity_2d = np.sqrt(
            pose_msg.twist.twist.linear.x**2 + pose_msg.twist.twist.linear.y**2
        )

        orientation_q = pose_msg.pose.pose.orientation
        quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(quaternion)

        vehicle_state = State(
            x=pose_msg.pose.pose.position.x,
            y=pose_msg.pose.pose.position.y,
            v=velocity_2d,
            yaw=yaw,
        )
        if vehicle_state.yaw < 0:
            vehicle_state.yaw = vehicle_state.yaw + 2 * np.pi
        self.vehicle_state_for_logging = vehicle_state

        # TODO: Calculate the next reference trajectory for the next T steps with current vehicle pose.
        #       ref_x, ref_y, ref_yaw, ref_v are columns of self.waypoints
        ref_x = self.waypoints[:, 0]
        ref_y = self.waypoints[:, 1]

        ref_yaw = self.waypoints[:, 2]
        angle_flip_idx = np.where(ref_yaw < 0)
        ref_yaw[angle_flip_idx] = ref_yaw[angle_flip_idx] + 2 * np.pi

        ref_v = self.waypoints[:, 3]
        self.ref_state_for_logging = State(x=ref_x[0], y=ref_y[0], v=ref_v[0], yaw=ref_yaw[0])

        # [x, y, v, yaw]
        ref_path = self.calc_ref_trajectory(vehicle_state, ref_x, ref_y, ref_yaw, ref_v)
        if not self.debug:
            self.get_logger().info(f"ref_path: {ref_path}")
            self.get_logger().info(f"its shape: {ref_path.shape}")

        trajectory = construct_trajectory(ref_path)
        self.path_pub_.publish(trajectory)

        x0 = [vehicle_state.x, vehicle_state.y, vehicle_state.v, vehicle_state.yaw]

        # TODO: solve the MPC control problem
        self.oa, self.odelta_v, ox, oy, oyaw, ov, state_predict = self.linear_mpc_control(
            ref_path, x0, self.oa, self.odelta_v
        )
        mpc_path = construct_mpc_path(ox, oy)
        self.mpc_path_pub_.publish(mpc_path)
        traj_trees = construct_trajectory_trees(state_predict)
        self.mpc_traj_tree_pub_.publish(traj_trees)

        # print()
        # print(state_predict.shape)
        # print(state_predict)
        if self.debug:
            import matplotlib.pyplot as plt

            plt.cla()
            plt.axis(
                [
                    vehicle_state.x - 6,
                    vehicle_state.x + 4.5,
                    vehicle_state.y - 2.5,
                    vehicle_state.y + 2.5,
                ]
            )
            plt.plot(
                ref_x,
                ref_y,
                linestyle="solid",
                linewidth=2,
                color="#005293",
                label="Raceline",
            )
            plt.plot(
                vehicle_state.x,
                vehicle_state.y,
                marker="o",
                markersize=12,
                color="#FF7F50",
                label="CoG",
            )
            plt.scatter(
                ref_path[0],
                ref_path[1],
                marker="x",
                linewidth=4,
                color="#87CEEB",
                label="MPC Input: Ref. Trajectory for T steps",
            )
            plt.scatter(
                ox,
                oy,
                marker="o",
                linewidth=4,
                color="#FFD700",
                label="MPC Output: Trajectory for T steps",
            )
            plt.legend()
            plt.pause(0.000001)
            plt.axis("equal")

        # TODO: publish drive message.
        steer_output = self.odelta_v[0]
        speed_output = vehicle_state.v + self.oa[0] * self.config.time_step

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = speed_output
        drive_msg.drive.steering_angle = steer_output

        self.drive_pub_.publish(drive_msg)

    def calc_ref_trajectory(self, state, cx, cy, cyaw, sp):
        """
        calc referent trajectory ref_traj in T steps: [x, y, v, yaw]
            using the current velocity, calc the T points along the reference path

        :param cx: Course X-Position
        :param cy: Course y-Position
        :param cyaw: Course Heading
        :param sp: speed profile
        :dl: distance step
        :pind: Setpoint Index

        :return: reference trajectory ref_traj, reference steering angle

        """
        # Create placeholder Arrays for the reference trajectory for T steps
        ref_traj = np.zeros((self.config.state_len, self.config.finite_time_horizon + 1))
        ncourse = len(cx)

        # Find nearest index/setpoint from where the trajectories are calculated
        _, _, _, ind = nearest_point(np.array([state.x, state.y]), np.array([cx, cy]).T)

        # Load the initial parameters from the setpoint into the trajectory
        ref_traj[0, 0] = cx[ind]
        ref_traj[1, 0] = cy[ind]
        ref_traj[2, 0] = sp[ind]
        ref_traj[3, 0] = cyaw[ind]

        # based on current velocity, distance traveled on the ref line between time steps
        travel = abs(state.v) * self.config.time_step

        dind = travel / self.config.dist_step

        ind_list = int(ind) + np.insert(
            np.cumsum(np.repeat(dind, self.config.finite_time_horizon)), 0, 0
        ).astype(int)
        ind_list[ind_list >= ncourse] -= ncourse

        ref_traj[0, :] = cx[ind_list]
        ref_traj[1, :] = cy[ind_list]
        ref_traj[2, :] = sp[ind_list]
        cyaw[cyaw - state.yaw > 4.5] = np.abs(cyaw[cyaw - state.yaw > 4.5] - (2 * np.pi))
        cyaw[cyaw - state.yaw < -4.5] = np.abs(cyaw[cyaw - state.yaw < -4.5] + (2 * np.pi))
        ref_traj[3, :] = cyaw[ind_list]

        return ref_traj

    def predict_motion(self, x0, oa, od, xref):
        path_predict = xref * 0.0
        for i, _ in enumerate(x0):
            path_predict[i, 0] = x0[i]

        state = State(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2])
        for ai, di, i in zip(oa, od, range(1, self.config.finite_time_horizon + 1)):
            state = self.update_state(state, ai, di)
            path_predict[0, i] = state.x
            path_predict[1, i] = state.y
            path_predict[2, i] = state.v
            path_predict[3, i] = state.yaw

        return path_predict

    def update_state(self, state, a, delta):

        # input check
        if delta >= self.config.MAX_STEER:
            delta = self.config.MAX_STEER
        elif delta <= -self.config.MAX_STEER:
            delta = -self.config.MAX_STEER

        state.x = state.x + state.v * math.cos(state.yaw) * self.config.time_step
        state.y = state.y + state.v * math.sin(state.yaw) * self.config.time_step
        state.yaw = (
            state.yaw + (state.v / self.config.WB) * math.tan(delta) * self.config.time_step
        )

        state.v = state.v + a * self.config.time_step

        if state.v > self.config.MAX_SPEED:
            state.v = self.config.MAX_SPEED
        elif state.v < self.config.MIN_SPEED:
            state.v = self.config.MIN_SPEED

        return state

    def get_model_matrix(self, v, phi, delta):
        """
        Calc linear and discrete time dynamic model-> Explicit discrete time-invariant
            Linear System: Xdot = Ax +Bu + C
            State vector: x=[x, y, v, yaw]

        :param v: speed
        :param phi: heading angle of the vehicle
        :param delta: steering angle: delta_bar

        :return: A, B, C

        """

        # State (or system) matrix A, 4x4
        A = np.zeros((self.config.state_len, self.config.state_len))
        A[0, 0] = 1.0
        A[1, 1] = 1.0
        A[2, 2] = 1.0
        A[3, 3] = 1.0
        A[0, 2] = self.config.time_step * math.cos(phi)
        A[0, 3] = -self.config.time_step * v * math.sin(phi)
        A[1, 2] = self.config.time_step * math.sin(phi)
        A[1, 3] = self.config.time_step * v * math.cos(phi)
        A[3, 2] = self.config.time_step * math.tan(delta) / self.config.WB

        # Input Matrix B; 4x2
        B = np.zeros((self.config.state_len, self.config.input_len))
        B[2, 0] = self.config.time_step
        B[3, 1] = self.config.time_step * v / (self.config.WB * math.cos(delta) ** 2)

        C = np.zeros(self.config.state_len)
        C[0] = self.config.time_step * v * math.sin(phi) * phi
        C[1] = -self.config.time_step * v * math.cos(phi) * phi
        C[3] = -self.config.time_step * v * delta / (self.config.WB * math.cos(delta) ** 2)

        return A, B, C

    def mpc_prob_solve(self, ref_traj, path_predict, x0):
        """
        Args:
            ref_traj: reference trajectory (desired trajectory: [x, y, v, yaw])
            path_predict: predicted states in T steps
            x0: initial state

        Returns:
            _type_: _description_
        """
        self.x0k.value = x0

        A_block = []
        B_block = []
        C_block = []
        for t in range(self.config.finite_time_horizon):
            A, B, C = self.get_model_matrix(path_predict[2, t], path_predict[3, t], 0.0)
            A_block.append(A)
            B_block.append(B)
            C_block.extend(C)

        A_block = block_diag(tuple(A_block))
        B_block = block_diag(tuple(B_block))
        C_block = np.array(C_block)

        self.Annz_k.value = A_block.data
        self.Bnnz_k.value = B_block.data
        self.Ck_.value = C_block

        self.ref_traj_k.value = ref_traj

        # Solve the optimization problem in CVXPY
        # Solver selections: cvxpy.OSQP; cvxpy.GUROBI
        self.MPC_prob.solve(solver=cvxpy.OSQP, verbose=False, warm_start=True)

        if (
            self.MPC_prob.status == cvxpy.OPTIMAL
            or self.MPC_prob.status == cvxpy.OPTIMAL_INACCURATE
        ):
            ox = np.array(self.xk.value[0, :]).flatten()
            oy = np.array(self.xk.value[1, :]).flatten()
            ov = np.array(self.xk.value[2, :]).flatten()
            oyaw = np.array(self.xk.value[3, :]).flatten()
            oa = np.array(self.uk.value[0, :]).flatten()
            odelta = np.array(self.uk.value[1, :]).flatten()

        else:
            print("Error: Cannot solve mpc..")
            oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None

        # print(oa)
        # print()
        # print(odelta)
        # print()
        # print(ox)
        # print()
        # print(oy)
        # print()
        # print(oyaw)
        # print()
        # print(ov)
        return oa, odelta, ox, oy, oyaw, ov

    def linear_mpc_control(self, ref_path, x0, oa, od):
        """
        MPC contorl with updating operational point iteraitvely

        :param ref_path: reference trajectory in T steps
        :param x0: initial state vector
        :param oa: acceleration of T steps of last time
        :param od: delta of T steps of last time

        """
        if oa is None or od is None:
            oa = [0.0] * self.config.finite_time_horizon
            od = [0.0] * self.config.finite_time_horizon

        # Call the Motion Prediction function: Predict the vehicle motion for x-steps
        path_predict = self.predict_motion(x0, oa, od, ref_path)
        poa, pod = oa[:], od[:]

        # Run the MPC optimization: Create and solve the optimization problem
        mpc_a, mpc_delta, mpc_x, mpc_y, mpc_yaw, mpc_v = self.mpc_prob_solve(
            ref_path, path_predict, x0
        )

        return mpc_a, mpc_delta, mpc_x, mpc_y, mpc_yaw, mpc_v, path_predict


def main(args=None):
    rclpy.init(args=args)
    print("|====================================================================|")
    print("|                                                                    |")
    print("|                      MPC Node Initialized                          |")
    print("|                                                                    |")
    print("|====================================================================|")
    print()
    mpc_node = MPC()

    rclpy.spin(mpc_node)
    # Destroy the node explicitly, otherwise it will be done automatically when the garbage collector destroys the node object
    mpc_node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
