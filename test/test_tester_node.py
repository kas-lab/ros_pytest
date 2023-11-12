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
import launch
import launch_pytest
import launch_ros

from pathlib import Path

import pytest

import sys

from ros_pytest.fixture import tester_node

test_node = 'test_helper_node'
tested_node = 'mock_node'


@launch_pytest.fixture
def generate_test_description():
    path_to_test = Path(__file__).parents[0]

    mock_node = launch_ros.actions.Node(
        executable=sys.executable,
        arguments=[str(path_to_test / 'mock_node' / 'mock_lc_node.py')],
        additional_env={'PYTHONUNBUFFERED': '1'},
        name=tested_node,
        output='screen',
    )
    return launch.LaunchDescription([
        mock_node,
    ])


@pytest.mark.launch(fixture=generate_test_description)
@pytest.mark.usefixtures(fixture=tester_node)
def test_lc_configure(tester_node):
    assert tester_node.lc_configure(tested_node)


@pytest.mark.launch(fixture=generate_test_description)
@pytest.mark.usefixtures(fixture=tester_node)
def test_lc_configure_activate(tester_node):
    assert tester_node.lc_configure_activate(tested_node)
