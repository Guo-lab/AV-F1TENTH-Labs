# MIT License

# Copyright (c) Hongrui Zheng, Johannes Betz

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
Utility functions for Kinematic Single Track MPC waypoint tracker

Author: Hongrui Zheng, Johannes Betz, Ahmad Amine
Last Modified: 12/27/22

"""
import math
import numpy as np

# from numba import njit
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import rclpy
from transforms3d.euler import euler2quat


# @njit(cache=True)
def nearest_point(point, trajectory):
    """
    Return the nearest point along the given piecewise linear trajectory.

    Args:
        point (numpy.ndarray, (2, )): (x, y) of current pose
        trajectory (numpy.ndarray, (N, 2)): array of (x, y) trajectory waypoints
            NOTE: points in trajectory must be unique. If they are not unique, a divide by 0 error will destroy the world

    Returns:
        nearest_point (numpy.ndarray, (2, )): nearest point on the trajectory to the point
        nearest_dist (float): distance to the nearest point
        t (float): nearest point's location as a segment between 0 and 1 on the vector formed by the closest two points on the trajectory. (p_i---*-------p_i+1)
        i (int): index of nearest point in the array of trajectory waypoints

    """
    diffs = trajectory[1:, :] - trajectory[:-1, :]

    l2s = diffs[:, 0] ** 2 + diffs[:, 1] ** 2
    l2s = np.where(l2s == 0, np.finfo(float).eps, l2s)  # NOTE: Avoid divide by 0

    dots = np.empty((trajectory.shape[0] - 1,))
    for i in range(dots.shape[0]):
        dots[i] = np.dot((point - trajectory[i, :]), diffs[i, :])

    t = dots / l2s
    t[t < 0.0] = 0.0
    t[t > 1.0] = 1.0

    projections = trajectory[:-1, :] + (t * diffs.T).T

    dists = np.empty((projections.shape[0],))
    for i in range(dists.shape[0]):
        temp = point - projections[i]
        dists[i] = np.sqrt(np.sum(temp * temp))

    min_dist_segment = np.argmin(dists)

    return (
        projections[min_dist_segment],
        dists[min_dist_segment],
        t[min_dist_segment],
        min_dist_segment,
    )


"""
    =====================================================================
   
                               For Visualization          

    =====================================================================
    
"""


def construct_trajectory(path):
    """
    Construct a tree of the trajectory waypoints

    Args:
        path (numpy.ndarray, (4, 8+1)): array of (x, y) trajectory waypoints

    Returns:
        Path to visualize the path in rviz
    """
    horizon_path = MarkerArray()
    assert path.shape[0] == 4, "Path must be of shape (4, N+1)"

    for i in range(path.shape[1]):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = "mpc_horizon"
        marker.id = i
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = path[0, i]
        marker.pose.position.y = path[1, i]
        marker.pose.position.z = 0.3
        quat = euler2quat(0, 0, path[2, i])
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]

        marker.scale.x = 0.09
        marker.scale.y = 0.09
        marker.scale.z = 0.09
        marker.color.a = 0.8
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        horizon_path.markers.append(marker)

    return horizon_path


def construct_full_path(waypoints):
    """
    Construct a full path from the given waypoints

    Args:
        waypoints (numpy.ndarray, (N, 4)): array of (x, y) trajectory waypoints

    Returns:
        Path to visualize the path in rviz
    """

    full_path = Path()
    full_path.header.frame_id = "map"

    for i in range(waypoints.shape[0]):

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = waypoints[i, 0]
        pose.pose.position.y = waypoints[i, 1]
        pose.pose.position.z = 0.0

        full_path.poses.append(pose)

    return full_path


def construct_mpc_path(ox, oy):
    """
    Construct a full path from the given waypoints

    Args:
        waypoints (numpy.ndarray, (N, 4)): array of (x, y) trajectory waypoints

    Returns:
        MarkerArray to visualize the path in rviz
    """
    mpc_path = Path()
    mpc_path.header.frame_id = "map"

    for i in range(ox.shape[0]):
        marker = PoseStamped()
        marker.header.frame_id = "map"

        marker.pose.position.x = ox[i]
        marker.pose.position.y = oy[i]
        marker.pose.position.z = 0.0

        mpc_path.poses.append(marker)

    return mpc_path


def construct_trajectory_trees(all_mpc_path):
    trajectory_trees = MarkerArray()

    for i in range(all_mpc_path.shape[1]):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = "mpc_traj_trees"
        marker.id = i
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = all_mpc_path[0, i]
        marker.pose.position.y = all_mpc_path[1, i]
        marker.pose.position.z = 0.1
        quat = euler2quat(0, 0, all_mpc_path[2, i])
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]

        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.scale.z = 0.08
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        trajectory_trees.markers.append(marker)

    return trajectory_trees
