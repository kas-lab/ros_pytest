# Copyright 2023 Gustavo Rezende Silva
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from threading import Thread

import rclpy
from rclpy.node import Node

from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.srv import GetState


class HelperTestNode(Node):

    def __init__(self, name='test_node'):
        super().__init__(name)

    def start_node(self, node):
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        Thread(target=executor.spin).start()

    def change_lc_node_state(self, node_name, transition_id):
        srv = self.create_client(
            ChangeState, node_name + '/change_state')
        change_state_req = ChangeState.Request()
        change_state_req.transition.id = transition_id
        return self.call_service(srv, change_state_req)

    def get_lc_node_state(self, node_name):
        get_state_srv = self.create_client(
            GetState, node_name + '/get_state')
        get_state_req = GetState.Request()
        return self.call_service(get_state_srv, get_state_req)

    def call_service(self, cli, request):
        if cli.wait_for_service(timeout_sec=5.0) is False:
            self.get_logger().error(
                'service not available {}'.format(cli.srv_name))
            return None
        future = cli.call_async(request)
        if self.executor.spin_until_future_complete(
                future, timeout_sec=5.0) is False:
            self.get_logger().error(
                'Future not completed {}'.format(cli.srv_name))
            return None

        return future.result()

    def activate_lc_node(self, node_name):
        self.change_lc_node_state(node_name, 1)
        self.change_lc_node_state(node_name, 3)

    def lc_configure(self, node_name):
        configure_res = self.change_lc_node_state(
            node_name, 1)
        get_inactive_state_res = self.get_lc_node_state(
            node_name)
        return configure_res.success is True and \
            get_inactive_state_res.current_state.id == 2

    def lc_activate(self, node_name):
        activate_res = self.change_lc_node_state(
            node_name, 3)
        get_active_state_res = self.get_lc_node_state(
            node_name)
        return activate_res.success is True and \
            get_active_state_res.current_state.id == 3

    def lc_configure_activate(self, node_name):
        return self.lc_configure(node_name) and self.lc_activate(node_name)
