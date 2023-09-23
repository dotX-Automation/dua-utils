"""
Simple rclpy service client wrapper.

Roberto Masocco <robmasocco@gmail.com>
Intelligent Systems Lab <isl.torvergata@gmail.com>

September 11, 2022
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

import rclpy
from rclpy.node import Node
from rclpy.task import Future

from typing import TypeVar

ServiceType = TypeVar('ServiceType')
ReqType = TypeVar('ReqType')
RespType = TypeVar('RespType')


class SimpleServiceClient():
    """
    Wraps a service client providing fully a/synchronous, transparent operation.
    The request is either handled in a single call that directly returns the final
    result, but internally runs the back-end by spinning the node when
    necessary, or in an asynchronous way, returning a Future object.
    """

    def __init__(
            self,
            node: Node,
            type: ServiceType,
            service_name: str,
            wait: bool = True) -> None:
        """
        Creates a new SimpleServiceClient.
        Can wait for the server to become active.

        :param node: Reference to the ROS 2 node to use.
        :param type: Service interface type.
        :param service_name: Name of the service to look for.
        :param wait: Indicates whether to wait for the server immediately.
        """
        # Create the ROS 2 service client and link the provided node
        self._node = node
        self._client = self._node.create_client(type, service_name)

        # Wait for the server to come up
        while wait and not self._client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self._node.get_logger().fatal(
                    "Middleware crashed while waiting for service {}".format(self._client.srv_name))
                raise RuntimeError(
                    "Middleware crashed while waiting for service {}".format(self._client.srv_name))
            self._node.get_logger().warn(
                "Service {} not available...".format(self._client.srv_name))

        self._node.get_logger().info(
            "Initialized client for service {}".format(self._client.srv_name))

    def call_sync(self, request: ReqType) -> RespType:
        """
        Calls the service, returns only when the request has been completed.
        Spins the node while waiting.

        :param request: Service request to send.
        :returns: Service response.
        """
        resp_future = self._client.call_async(request)
        rclpy.spin_until_future_complete(self._node, resp_future)
        return resp_future.result()

    def call_async(self, request: ReqType) -> Future:
        """
        Calls the service, returns immediately.
        Does not spin the node.

        :param request: Service request to send.
        :returns: Future object for the response.
        """
        return self._client.call_async(request)
