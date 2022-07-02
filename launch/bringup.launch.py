from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription

def generate_launch_description():
    components = LoadComposableNodes(
        target_container='lifecycle_manager_container',
        composable_node_descriptions=[
            ComposableNode(
                package='template_lifecycle_manager',
                plugin='template_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager'
            )
        ]
    )
    
    return LaunchDescription([
        components
    ])
