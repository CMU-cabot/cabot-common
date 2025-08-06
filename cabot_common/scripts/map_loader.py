#!/usr/bin/env python3
# Copyright (c) 2020  Carnegie Mellon University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os.path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import sys
import traceback
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String
from nav2_msgs.srv import LoadMap
from diagnostic_updater import Updater
from diagnostic_msgs.msg import DiagnosticStatus


class MapLoaderNode(Node):
    def __init__(self):
        super().__init__('map_loader_node')

        self.declare_parameter('map_servers', ['/map_server'])
        service_timeout_sec = self.declare_parameter('timeout', 3.0).get_parameter_value().double_value
        self.service_timeout = rclpy.duration.Duration(seconds=service_timeout_sec)

        latched_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'current_map_filename', self.map_filename_callback, latched_qos)
        self.current_map_filename = None

        self.updater = Updater(self)
        self.updater.add("ROS2 Map Loader", self.check_status)
        self.error_message = None

        self.create_timer(1.0, self.check_update)
        self.needs_update = False
        self.futures = []

        self.get_logger().info("MapLoaderNode is launched")

    def get_filename(self, url):
        PACKAGE_PREFIX = 'package://'
        mod_url = url
        if url.find(PACKAGE_PREFIX) == 0:
            mod_url = url[len(PACKAGE_PREFIX):]
            pos = mod_url.find('/')
            if pos == -1:
                raise Exception(f"Could not parse package:// format into file:// format for {url}")

            package = mod_url[0:pos]
            mod_url = mod_url[pos:]
            package_path = get_package_share_directory(package)

            mod_url = package_path + mod_url
        return mod_url

    def map_filename_callback(self, msg):
        if self.current_map_filename != msg.data:
            self.current_map_filename = msg.data
            self.needs_update = True

    def check_update(self):
        if not self.needs_update:
            expired = list(filter(lambda x: self.get_clock().now() - x[0] > self.service_timeout, self.futures))
            if expired:
                for expire in expired:
                    self.get_logger().info(f"Removing expired futures: {expire}")
                    expire[2].cancel()
                self.needs_update = True
                self.futures = []
            return

        filename = self.get_filename(self.current_map_filename)
        self.get_logger().info(f"{self.current_map_filename=}")
        self.get_logger().info(f"{self.needs_update=}")
        self.get_logger().info(f"{self.futures=}")
        self.get_logger().info(f"{filename=}")

        if not os.path.exists(filename):
            self.error_message = f"{filename} is not found"
            return

        servers = self.get_parameter('map_servers').get_parameter_value().string_array_value

        self.get_logger().info(str(servers))

        for server in servers:
            cli = self.create_client(LoadMap, f'{server}/load_map')
            req = LoadMap.Request()
            req.map_url = filename  # need to specify file path not url

            if cli.wait_for_service(timeout_sec=10.0):
                self.get_logger().info(server + ' service is ready')
                future = cli.call_async(req)

                def done_callback(future):
                    self.futures = list(filter(lambda x: x[2] != future, self.futures))
                    if future.result().result != LoadMap.Response.RESULT_SUCCESS:
                        self.get_logger().error('LoadMap request fails. Try again...')
                        self.needs_update = True
                    else:
                        self.get_logger().info('LoadMap request succeeds')
                        self.error_message = None
                future.add_done_callback(done_callback)
                self.futures.append((self.get_clock().now(), server, future))
            else:
                self.error_message = server + ' service is not available'
                self.get_logger().info(server + ' service is not available')
                return

        self.needs_update = False

    def check_status(self, stat):
        if self.error_message:
            stat.summary(DiagnosticStatus.ERROR, self.error_message)
            return stat
        stat.summary(DiagnosticStatus.OK, "working")
        return stat


def main():
    rclpy.init()
    node = MapLoaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down map_loader_node")
    except:  # noqa: E722
        node.get_logger().error(traceback.format_exc())
    sys.exit(0)


if __name__ == '__main__':
    main()
