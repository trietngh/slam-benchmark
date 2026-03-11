# Copyright 2025 Stereolabs
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    LogInfo
)
from launch.substitutions import (
    LaunchConfiguration,
    TextSubstitution
)
from launch_ros.actions import (
    ComposableNodeContainer,
    LoadComposableNodes
)

from launch_ros.descriptions import (
    ComposableNode
)
# Enable colored output
os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"


def launch_setup(context, *args, **kwargs):
    # Get the path to the AprilTag configuration file
    apriltag_config_path = os.path.join(
        get_package_share_directory('apriltag_launch'),
        'config',
        'apriltag.yaml'
    )

    # List of actions to be launched
    actions = []

    namespace_val = 'apriltag'
    
    # ROS 2 Component Container
    container_name = 'apriltag_container'
    info = '* Starting Composable node container: ' + namespace_val + '/' + container_name
    actions.append(LogInfo(msg=TextSubstitution(text=info)))

    # Note: It is crucial that the 'executable' field is set to be 'component_container_mt'
    #  so that the created nodes can be started and communicated correctly within the same process.

    apriltag_container = ComposableNodeContainer(
        name=container_name,
        namespace=namespace_val,
        package='rclcpp_components',
        executable='component_container_mt',
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen',
    )
    actions.append(apriltag_container)

    # Isaac ROS Node to convert from ZED BGRA8 image to BGR8 required by AprilTag
    isaac_converter_node = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        name='zed_image_converter',
        namespace=namespace_val,
        parameters=[
            {
                'image_width': 540,
                'image_height': 960,
                'encoding_desired': 'bgr8',
                'num_blocks': 40
            }
        ],
        remappings=[
            ('image_raw', '/zed/zed_node/rgb/color/rect/image'),
            ('image', '/zed/zed_node/rgb/color/rect/image_bgr8')
        ]
    )

    # add parameters for conversion node -> https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_image_pipeline/isaac_ros_image_proc/index.html#imageformatconverternode

    # AprilTag detection node
    isac_apriltag_node = ComposableNode(
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        name='apriltag',
        namespace=namespace_val,
        remappings=[
                ('image', '/zed/zed_node/rgb/color/rect/image_bgr8'),
                ('camera_info', '/zed/zed_node/rgb/color/rect/camera_info')
        ],
        parameters=[apriltag_config_path]
    )

    container_full_name = namespace_val + '/' + container_name
    # Load the Converter node into the container
    load_converter_node = LoadComposableNodes(
        composable_node_descriptions=[isaac_converter_node],
        target_container=container_full_name
    )
    actions.append(load_converter_node)

    # Load the AprilTag node into the container
    load_april_tag_node = LoadComposableNodes(
        composable_node_descriptions=[isac_apriltag_node],
        target_container=container_full_name
    )
    actions.append(load_april_tag_node)

    return actions

def generate_launch_description():
    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup)
        ]
    )