import copy
from launch import LaunchDescription, LaunchContext
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.absolute()))
from launch.actions import TimerAction
import rs_launch

local_parameters = [
    {'name': 'camera_name1', 'default': 'camera1', 'description': 'camera1 unique name'},
    {'name': 'camera_name2', 'default': 'camera2', 'description': 'camera2 unique name'},
    {'name': 'camera_namespace1', 'default': 'camera1', 'description': 'camera1 namespace'},
    {'name': 'camera_namespace2', 'default': 'camera2', 'description': 'camera2 namespace'},
]

def duplicate_params(general_params, suffix):
    local_params = copy.deepcopy(general_params)
    for param in local_params:
        param['original_name'] = param['name']
        param['name'] += suffix
    return local_params

def set_configurable_parameters(local_params):
    return dict([(param['original_name'], LaunchConfiguration(param['name'])) for param in local_params])

def create_camera_container(context: LaunchContext, index: str):
    params = set_configurable_parameters(duplicate_params(rs_launch.configurable_parameters, index))
    config_file = LaunchConfiguration(f'config_file{index}').perform(context)
    params_from_file = {} if config_file == "''" else rs_launch.yaml_to_dict(config_file)

    return [
        TimerAction(
        period=1.0,
        actions=[
            ComposableNodeContainer(
                name=f'realsense_container{index}',
                namespace='',
                package='rclcpp_components',
                executable='component_container_mt',
                output='screen',
                emulate_tty=True,
                composable_node_descriptions=[
                    ComposableNode(
                        package='realsense2_camera',
                        plugin='realsense2_camera::RealSenseNodeFactory',
                        name=LaunchConfiguration(f'camera_name{index}'),
                        namespace=LaunchConfiguration(f'camera_namespace{index}'),
                        parameters=[params, params_from_file],
                        extra_arguments=[{'use_intra_process_comms': False}],
                    )
                ],
            )
        ],
        )
    ]

def launch_static_transform_publisher_node(context: LaunchContext):
    camera_tf1 = None
    camera_tf2 = None

    # Determine frame names based on device types
    if context.launch_configurations['device_type1'] == 't265':
        camera_tf1 = context.launch_configurations['camera_name1'] + '_pose_frame'
    else:
        camera_tf2 = context.launch_configurations['camera_name1'] + '_link'

    if context.launch_configurations['device_type2'] == 't265':
        # Only set if not already set
        if camera_tf1 is None:
            camera_tf1 = context.launch_configurations['camera_name2'] + '_pose_frame'
    else:
        # Only set if not already set
        if camera_tf2 is None:
            camera_tf2 = context.launch_configurations['camera_name2'] + '_link'
    node = Node(
            package = "tf2_ros",
            executable = "static_transform_publisher",
            condition=IfCondition(LaunchConfiguration('static_transform')),
                arguments=[
                    "--x", "0",
                    "--y", "0.2",
                    "--z", "0",
                    "--roll", "0",
                    "--pitch", "0",
                    "--yaw", "0",
                    "--frame-id", camera_tf1,
                    "--child-frame-id", camera_tf2]
                )
    return [
        ComposableNodeContainer(
            name='tf_publisher_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            output='screen',
            composable_node_descriptions=[],
        ),
        node
    ]

def generate_launch_description():
    params1 = duplicate_params(rs_launch.configurable_parameters, '1')
    params2 = duplicate_params(rs_launch.configurable_parameters, '2')

    return LaunchDescription(
        rs_launch.declare_configurable_parameters(local_parameters) +
        rs_launch.declare_configurable_parameters(params1) +
        rs_launch.declare_configurable_parameters(params2) + [
            OpaqueFunction(function=create_camera_container, kwargs={'index': '1'}),
            OpaqueFunction(function=create_camera_container, kwargs={'index': '2'}),
            OpaqueFunction(function=launch_static_transform_publisher_node),
        ]
    )

