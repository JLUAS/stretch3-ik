#!/usr/bin/env python3
"""End-to-end segmented lift action test using an isolated mock controller."""
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters
from rclpy.action import (ActionClient, ActionServer, CancelResponse,
                          GoalResponse)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from stretch_guarded_executor.action import ExecuteLiftTarget


ROOT = Path(__file__).resolve().parents[1]
assert os.environ.get("ROS_DOMAIN_ID") == "20"
assert os.environ.get("ROS_LOCALHOST_ONLY") == "1"

rclpy.init()
group = ReentrantCallbackGroup()
driver = Node("stretch_driver", start_parameter_services=False)
client_node = Node("lift_transport_test")
retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      reliability=ReliabilityPolicy.RELIABLE)
executor_status = []
client_node.create_subscription(
    String, "/guarded_executor/status",
    lambda message: executor_status.append(message.data), retained,
    callback_group=group)

description = driver.create_publisher(String, "/robot_description", retained)
joints = driver.create_publisher(JointState, "/stretch/joint_states", 20)
homed = driver.create_publisher(Bool, "/is_homed", 20)
runstopped = driver.create_publisher(Bool, "/is_runstopped", 20)
mode = driver.create_publisher(String, "/mode", 20)
streaming = driver.create_publisher(Bool, "/is_streaming_position", 20)
diagnostics = driver.create_publisher(DiagnosticArray, "/diagnostics", 20)

state_lock = threading.Lock()
measured_lift = .45
shutdown = False
goals = []
results = []


def publish_state():
    while not shutdown:
        now = driver.get_clock().now().to_msg()
        with state_lock:
            lift = measured_lift
        message = JointState()
        message.header.stamp = now
        message.name = ["joint_lift", "joint_arm_l3", "joint_arm_l2",
                        "joint_arm_l1", "joint_arm_l0", "joint_wrist_yaw",
                        "joint_wrist_pitch", "joint_wrist_roll"]
        message.position = [lift, .05, .05, .05, .05, 0., -.2, 0.]
        joints.publish(message)
        homed.publish(Bool(data=True))
        runstopped.publish(Bool(data=False))
        mode.publish(String(data="trajectory"))
        streaming.publish(Bool(data=False))
        diagnostic = DiagnosticArray()
        diagnostic.header.stamp = now
        diagnostic.status = [DiagnosticStatus(
            level=DiagnosticStatus.ERROR,
            name="stretch_driver: Pimu/Cliff",
            message="fixture cliff_range=75 out of range [-50,20]")]
        diagnostics.publish(diagnostic)
        time.sleep(.02)


def get_parameters(request, response):
    response.values = [ParameterValue(
        type=ParameterType.PARAMETER_BOOL, bool_value=True)
        for _ in request.names]
    return response


def stop_service(_request, response):
    response.success = True
    response.message = "mock actuators stopped"
    return response


def runstop_service(_request, response):
    response.success = True
    response.message = "mock runstop"
    return response


driver.create_service(GetParameters, "/stretch_driver/get_parameters",
                      get_parameters, callback_group=group)
driver.create_service(Trigger, "/stop_the_robot", stop_service,
                      callback_group=group)
driver.create_service(SetBool, "/runstop", runstop_service,
                      callback_group=group)


def goal_callback(goal):
    assert goal.trajectory.joint_names == ["joint_lift"]
    assert len(goal.trajectory.points) == 31
    target = goal.trajectory.points[-1].positions[0]
    goals.append({"target": target,
                  "joint_names": list(goal.trajectory.joint_names),
                  "points": len(goal.trajectory.points)})
    return GoalResponse.ACCEPT


def cancel_callback(_goal):
    return CancelResponse.ACCEPT


def seconds(duration):
    return duration.sec + duration.nanosec * 1e-9


def execute_callback(handle):
    global measured_lift
    trajectory = handle.request.trajectory
    began = time.monotonic()
    previous = 0.
    for point in trajectory.points:
        due = seconds(point.time_from_start)
        delay = due - previous
        if delay > 0:
            time.sleep(delay)
        previous = due
        if handle.is_cancel_requested:
            handle.canceled()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            result.error_string = "mock cancellation confirmed"
            return result
        with state_lock:
            measured_lift = point.positions[0]
        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = ["joint_lift"]
        feedback.actual.positions = [point.positions[0]]
        handle.publish_feedback(feedback)
    elapsed = time.monotonic() - began
    assert elapsed >= 2.8
    handle.succeed()
    result = FollowJointTrajectory.Result()
    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
    result.error_string = "mock segment complete"
    results.append(trajectory.points[-1].positions[0])
    return result


action_server = ActionServer(
    driver, FollowJointTrajectory,
    "/stretch_controller/follow_joint_trajectory",
    execute_callback=execute_callback, goal_callback=goal_callback,
    cancel_callback=cancel_callback, callback_group=group)

description.publish(String(data=(ROOT / "docs/evidence/robot.urdf").read_text()))
publisher_thread = threading.Thread(target=publish_state, daemon=True)
publisher_thread.start()
executor = MultiThreadedExecutor(num_threads=6)
executor.add_node(driver)
executor.add_node(client_node)
spin_thread = threading.Thread(target=executor.spin, daemon=True)
spin_thread.start()

log_path = ROOT / "build/stretch_guarded_executor/lift_transport.log"
command = [
    "ros2", "run", "stretch_guarded_executor", "guarded_executor", "--ros-args",
    "-r", "__ns:=/guarded_executor",
    "--params-file",
    str(ROOT / "src/stretch_guarded_executor/config/executor_real_test.yaml"),
    "-p", "dry_run:=false", "-p", "execution_enabled:=true",
    "-p", "transport_test_mode:=true",
]
process = None


def wait_future(future, timeout):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        time.sleep(.01)
    assert future.done(), "future timeout"
    return future.result()


def send_target(client, target, timeout=70):
    feedback = []
    future = client.send_goal_async(
        ExecuteLiftTarget.Goal(target_position=target),
        feedback_callback=lambda item: feedback.append(item.feedback))
    handle = wait_future(future, 5)
    assert handle.accepted, f"target {target} rejected"
    wrapped = wait_future(handle.get_result_async(), timeout)
    return handle, wrapped, feedback


try:
    with open(log_path, "w") as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        action = ActionClient(client_node, ExecuteLiftTarget,
                              "/guarded_executor/execute_lift_target",
                              callback_group=group)
        assert action.wait_for_server(timeout_sec=10)
        deadline = time.monotonic() + 12
        required = ["state=DISARMED", "action_server_available=1",
                    "mode=trajectory", "startpoint_guard=1"]
        while (not executor_status or
               not all(item in executor_status[-1] for item in required)):
            assert time.monotonic() < deadline, (
                "executor preflight unavailable: " +
                (executor_status[-1] if executor_status else "no status"))
            time.sleep(.05)

        first_future = action.send_goal_async(
            ExecuteLiftTarget.Goal(target_position=.53))
        first_handle = wait_future(first_future, 5)
        assert first_handle.accepted
        first_result_future = first_handle.get_result_async()
        time.sleep(.4)
        if first_result_future.done():
            early = first_result_future.result()
            raise AssertionError(
                "first goal terminated before concurrency check: " +
                early.result.message + "; status=" + executor_status[-1])
        double_handle = wait_future(action.send_goal_async(
            ExecuteLiftTarget.Goal(target_position=.60)), 5)
        assert not double_handle.accepted, (
            "overlapping action goal accepted; status=" + executor_status[-1])
        first_result = wait_future(first_result_future, 70)
        assert first_result.status == GoalStatus.STATUS_SUCCEEDED
        assert first_result.result.success
        assert math.isclose(first_result.result.final_position, .53,
                            abs_tol=.001)
        assert [round(item["target"], 9) for item in goals] == [
            .48, .51, .53]
        assert all(item["joint_names"] == ["joint_lift"] for item in goals)
        assert all(item["points"] == 31 for item in goals)

        upward_count = len(goals)
        _handle, downward, feedback = send_target(action, .45)
        assert downward.status == GoalStatus.STATUS_SUCCEEDED
        assert downward.result.success
        assert math.isclose(downward.result.final_position, .45,
                            abs_tol=.001)
        assert [round(item["target"], 9)
                for item in goals[upward_count:]] == [
                    .50, .47, .45]
        assert feedback and feedback[-1].total_segments == 3
        assert all(item["joint_names"] == ["joint_lift"] for item in goals)

        cancel_future = action.send_goal_async(
            ExecuteLiftTarget.Goal(target_position=.47))
        cancel_handle = wait_future(cancel_future, 5)
        assert cancel_handle.accepted
        time.sleep(.5)
        cancel_response = wait_future(cancel_handle.cancel_goal_async(), 5)
        assert cancel_response.goals_canceling
        canceled = wait_future(cancel_handle.get_result_async(), 10)
        assert canceled.status == GoalStatus.STATUS_CANCELED
        assert not canceled.result.success
        assert "confirmed stop" in canceled.result.message

        print({
            "upward_segments": [x["target"] for x in goals[:upward_count]],
            "downward_segments": [x["target"] for x in goals[upward_count:16]],
            "joint_names": sorted({tuple(x["joint_names"]) for x in goals}),
            "controller_success_results": results,
            "cancellation": canceled.result.message,
        })
finally:
    if process and process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
    shutdown = True
    publisher_thread.join(timeout=1)
    executor.shutdown()
    spin_thread.join(timeout=1)
    action_server.destroy()
    driver.destroy_node()
    client_node.destroy_node()
    rclpy.shutdown()
