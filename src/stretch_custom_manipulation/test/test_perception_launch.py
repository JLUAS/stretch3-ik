"""Resolve real bringup actions without executing a node or touching hardware."""
import importlib.util
from pathlib import Path
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

path=Path(__file__).parents[1]/'launch/bringup.launch.py'
spec=importlib.util.spec_from_file_location('perception_bringup',path)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

def test_real_bringup_replaces_sam_and_reuses_one_camera():
    context=LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action,DeclareLaunchArgument):action.execute(context)
    context.launch_configurations.update(sim='false',scenario='fixture',rviz='false',
        dry_run='true',execution_enabled='false',start_robot_driver='false',
        start_robot_camera='false',start_robot_lidar='false',start_nav2='false',
        collision_environment='primitive',octomap_enabled='false',gripper_enabled='false')
    actions=module.setup(context)
    includes=[a for a in actions if isinstance(a,IncludeLaunchDescription)]
    for action in includes:
        action.launch_description_source.get_launch_description(context)
    found=[a for a in includes if 'stretch_rgbd_perception' in a.launch_description_source.location]
    assert len(found)==1
    args=dict(found[0].launch_arguments)
    assert args['start_camera']=='false'
    assert args['pose_source'].perform(context)=='tf'
    assert all('sam2' not in str(a) for a in actions)
