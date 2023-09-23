"""
Implementation of reference QoS profile getters for visualization topics.

Roberto Masocco <robmasocco@gmail.com>
Intelligent Systems Lab <isl.torvergata@gmail.com>

September 23, 2023
"""

# This is free software.
# You can redistribute it and/or modify this file under the
# terms of the GNU General Public License as published by the Free Software
# Foundation; either version 3 of the License, or (at your option) any later
# version.
#
# This file is distributed in the hope that it will be useful, but WITHOUT ANY
# WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
# A PARTICULAR PURPOSE. See the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along with
# this file; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA.

import rclpy.qos

from typing import TypeVar

QoSProfile = TypeVar('QoSProfile')


def get_datum_qos(depth: int = 5) -> QoSProfile:
    """
    Returns the QoS profile for regular data topics.

    :returns: The QoS profile.
    """
    return rclpy.qos.QoSProfile(
        depth=depth,
        history=rclpy.qos.HistoryPolicy.KEEP_LAST,
        reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
        durability=rclpy.qos.DurabilityPolicy.VOLATILE)


def get_command_qos(depth: int = 5) -> QoSProfile:
    """
    Returns the QoS profile for command topics.

    :returns: The QoS profile.
    """
    return rclpy.qos.QoSProfile(
        depth=depth,
        history=rclpy.qos.HistoryPolicy.KEEP_LAST,
        reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
        durability=rclpy.qos.DurabilityPolicy.VOLATILE)


def get_scan_qos(depth: int = 1) -> QoSProfile:
    """
    Returns the QoS profile for scan topics, like pointclouds or laser scans.

    :returns: The QoS profile.
    """
    return rclpy.qos.QoSProfile(
        depth=depth,
        history=rclpy.qos.HistoryPolicy.KEEP_LAST,
        reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
        durability=rclpy.qos.DurabilityPolicy.VOLATILE)


def get_image_qos(depth: int = 1) -> QoSProfile:
    """
    Returns the QoS profile for image topics.

    :returns: The QoS profile.
    """
    return rclpy.qos.QoSProfile(
        depth=depth,
        history=rclpy.qos.HistoryPolicy.KEEP_LAST,
        reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
        durability=rclpy.qos.DurabilityPolicy.VOLATILE)


def get_marker_qos(depth: int = 1) -> QoSProfile:
    """
    Returns the QoS profile for marker topics.

    :returns: The QoS profile.
    """
    return rclpy.qos.QoSProfile(
        depth=depth,
        history=rclpy.qos.HistoryPolicy.KEEP_LAST,
        reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
        durability=rclpy.qos.DurabilityPolicy.VOLATILE)
