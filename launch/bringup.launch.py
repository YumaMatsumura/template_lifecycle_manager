from launch_ros.actions import Node
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription

def generate_launch_description():
    container = Node(
        package='rclcpp_components',
        executable='component_container'
    )
    
    components = LoadComposableNodes(
        target_container='ComponentManager',
        composable_node_descriptions=[
            ComposableNode(
                package='template_lifecycle_manager',
                plugin='template_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager',
                extra_arguments=[
                    {'use_intra_process_comms': True}
                ]
            )
        ], 
    )
    
    return LaunchDescription([
        container,
        components
    ])
