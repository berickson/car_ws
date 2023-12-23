import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, launch_description_sources
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
import launch_ros.descriptions


def generate_launch_description():
    cone_detector_path = get_package_share_directory('cone_detector')


    default_resources_path = os.path.join(cone_detector_path,
                                'resources')
    tf_prefix    = LaunchConfiguration('tf_prefix',     default = 'oak')
    base_frame   = LaunchConfiguration('base_frame',    default = 'oak-d_frame')
    parent_frame = LaunchConfiguration('parent_frame',  default = 'oak-d-base-frame')
    spatial_camera = LaunchConfiguration('spatial_camera',  default = True)

    camera_param_uri   = LaunchConfiguration('camera_param_uri',  default = 'package://cone_detector/params/camera')
    sync_nn            = LaunchConfiguration('sync_nn',           default = True)

    nnName             = LaunchConfiguration('nnName', default = "x")
    resourceBaseFolder = LaunchConfiguration('resourceBaseFolder', default = default_resources_path)
    monoResolution     = LaunchConfiguration('monoResolution',  default = '400p')


    namespace = LaunchConfiguration('namespace')
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='car/oakd',
        description='Namespace for all topics, defaults to car/oakd')

    yolov8_cone_node = launch_ros.actions.Node(
            package='cone_detector', 
            executable='cone_detector_node',
            namespace=namespace,
            output='screen',
            parameters=[{'tf_prefix': tf_prefix},
                        {'camera_param_uri': camera_param_uri},
                        {'sync_nn': sync_nn},
                        {'nnName': nnName},
                        {'resourceBaseFolder': resourceBaseFolder},
                        {'monoResolution': monoResolution},
                        {'spatial_camera': spatial_camera}])

    ld = LaunchDescription([namespace_arg])

    ld.add_action(yolov8_cone_node)

    return ld

