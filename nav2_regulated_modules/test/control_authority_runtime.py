"""Real navigator + synthetic feedback and FollowPath server, isolated DDS only.

No physical robot or production launch is used. Refuse the default DDS domain.
"""
import os
from pathlib import Path
import subprocess
import threading
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import FollowPath
from byd_custom_msgs.action import NavigationService
from byd_custom_msgs.msg import ChassisControl, ControlRes, MotionState, NaviSegment
from tf2_ros import TransformBroadcaster


def require(value, message):
    if not value:
        raise AssertionError(message)


def main():
    require(int(os.environ.get('ROS_DOMAIN_ID', '0')) >= 100,
            'Run only in an explicitly isolated ROS_DOMAIN_ID >= 100')
    require(os.environ.get('ROS_LOCALHOST_ONLY') == '1', 'Require localhost DDS')
    rclpy.init()
    node = Node('control_authority_test')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    group = ReentrantCallbackGroup()
    feedback = node.create_publisher(MotionState, '/motion_state', 10)
    navigation = node.create_publisher(Twist, '/cmd_vel', 10)
    remote = node.create_publisher(ChassisControl, '/downstream/chassis_control', 10)
    tf = TransformBroadcaster(node)
    samples = []
    subscription = node.create_subscription(ControlRes, '/control_to_uart',
        lambda m: samples.append((time.monotonic(), m.v, m.w)), 100)
    state = dict(feedback=True, measured=0.0, angular=0.0, nav=None, remote=None,
                 result=True, delay=0.0, reject=False, follows=0, cancels=0)

    def tick():
        if state['feedback']:
            m = MotionState()
            m.header.stamp = node.get_clock().now().to_msg()
            m.v_car, m.w_car = state['measured'], state['angular']
            feedback.publish(m)
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = 'map', 'base_link'
        t.header.stamp = node.get_clock().now().to_msg()
        t.transform.rotation.w = 1.0
        tf.sendTransform(t)
        if state['nav'] is not None:
            m = Twist()
            m.linear.x = state['nav']
            navigation.publish(m)
        if state['remote'] is not None:
            remote.publish(state['remote'])

    timer = node.create_timer(0.01, tick)

    def goal_callback(_):
        state['follows'] += 1
        time.sleep(state['delay'])
        return GoalResponse.REJECT if state['reject'] else GoalResponse.ACCEPT

    def cancel_callback(_):
        state['cancels'] += 1
        return CancelResponse.ACCEPT

    def execute(handle):
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline:
            if handle.is_cancel_requested and state['result']:
                handle.canceled()
                return FollowPath.Result()
            time.sleep(0.01)
        handle.abort()
        return FollowPath.Result()

    server = ActionServer(node, FollowPath, '/follow_path', execute,
        goal_callback=goal_callback, cancel_callback=cancel_callback, callback_group=group)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    log_path = Path(os.environ.get('AUTHORITY_TEST_LOG', '/tmp/control-authority-node.log'))
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log = log_path.open('w')
    executable = Path(get_package_prefix('nav2_regulated_modules')) / 'lib/nav2_regulated_modules/regulated_navigator_node'
    process = subprocess.Popen([str(executable), '--ros-args', '-p', 'operation_mode:=fixed_path',
        '-p', 'tf_diagnostics_enabled:=false', '-p', 'legacy_velocity_logging:=false',
        '-p', 'velocity_odom_topic:=/motion_state'], stdout=log, stderr=subprocess.STDOUT)

    def wait(predicate, message, timeout=4):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            require(process.poll() is None, f'navigator exited: {log_path}')
            time.sleep(0.01)
        raise AssertionError(message)

    def future_result(future):
        wait(future.done, 'ROS future did not complete')
        return future.result()

    def transition(value):
        client = node.create_client(ChangeState, '/regulated_navigator/change_state')
        require(client.wait_for_service(timeout_sec=8), 'lifecycle service available')
        request = ChangeState.Request()
        request.transition.id = value
        require(future_result(client.call_async(request)).success, f'lifecycle transition {value}')
        node.destroy_client(client)

    action = ActionClient(node, NavigationService, '/navigation_service')

    def path_goal():
        g = NavigationService.Goal()
        g.task_id = 'authority-test'
        s = NaviSegment()
        s.segment_type, s.motion_direction = 1, 1
        s.node2.x, s.max_speed = 2.0, 0.8
        g.navi_segment = [s]
        return future_result(action.send_goal_async(g))

    def command(op=0, speed=0.25):
        m = ChassisControl()
        m.op, m.linear_velocity = op, speed
        return m

    def assert_zero(duration=0.2):
        start = time.monotonic()
        time.sleep(duration)
        window = [s for s in samples if s[0] > start + 0.04]
        require(len(window) >= 4, 'zero output heartbeat exists')
        require(all(abs(v) < 1e-6 and abs(w) < 1e-6 for _, v, w in window),
                f'expected zero output: {window[-5:]}')

    def has_speed(predicate):
        return bool(samples) and samples[-1][0] > time.monotonic() - 0.1 and predicate(samples[-1][1])

    checks = []
    try:
        transition(1)
        transition(3)
        require(action.wait_for_server(timeout_sec=4), 'NavigationService available')
        wait(lambda: len(samples) > 8, 'unified zero output available')
        require(node.count_publishers('/control_to_uart') == 1, 'exactly one output publisher')
        assert_zero()
        checks.append('startup idle and unique publisher')

        state['feedback'] = False
        time.sleep(0.3)
        require(not path_goal().accepted, 'missing fresh feedback rejects path')
        state['feedback'] = True
        time.sleep(0.15)
        checks.append('missing feedback rejects navigation')

        first = path_goal()
        require(first.accepted, 'fresh stopped state accepts path')
        first_result = first.get_result_async()
        wait(lambda: state['follows'] == 1, 'FollowPath dispatched')
        state['nav'] = 0.8
        wait(lambda: has_speed(lambda v: abs(v - 0.8) < 1e-5), 'navigation is not remote-clamped')
        checks.append('navigation preserves speed above remote limit')

        state['remote'] = command(op=255)
        time.sleep(0.08)
        require(has_speed(lambda v: v > 0.7), 'invalid op cannot take over')
        state['remote'] = command(speed=float('nan'))
        time.sleep(0.08)
        require(has_speed(lambda v: v > 0.7), 'nonfinite command cannot take over')
        state['remote'] = None
        checks.append('invalid commands do not take over')

        state['result'], state['measured'] = False, 0.2
        state['remote'] = command()
        wait(lambda: state['cancels'] >= 1, 'remote cancels downstream FollowPath')
        require(future_result(first_result).status == 6, 'interrupted outer goal is ABORTED')
        assert_zero()
        require(not path_goal().accepted, 'remote handover rejects path')
        state['measured'] = 0.0
        assert_zero()
        checks.append('cancel acceptance alone cannot release remote motion')
        state['result'] = True
        wait(lambda: has_speed(lambda v: v > 0.02), 'remote motion after downstream result and stopped feedback')
        require(not path_goal().accepted, 'active remote rejects path')
        wait(lambda: has_speed(lambda v: abs(v - 0.25) < 1e-3), 'remote reaches target')
        checks.append('remote takeover after downstream terminal result')

        state['remote'] = command(speed=1.5)
        wait(lambda: has_speed(lambda v: abs(v - 0.3) < 1e-4), 'remote speed is limited to 0.3')
        checks.append('remote speed clamp')
        state['remote'], state['measured'] = None, 0.1
        wait(lambda: has_speed(lambda v: abs(v) < 1e-6), 'timeout ramps to zero', timeout=3)
        require(not path_goal().accepted, 'zero command without measured stop cannot release')
        state['measured'] = 0.0
        time.sleep(0.2)
        assert_zero()
        require(state['follows'] == 1, 'old navigation never resumes')
        checks.append('timeout, measured stop and no old path resume')

        state['nav'] = None
        time.sleep(0.25)
        second = path_goal()
        require(second.accepted, 'new path after remote release is accepted')
        second_result = second.get_result_async()
        wait(lambda: state['follows'] == 2, 'new FollowPath dispatched')
        state['nav'] = 0.6
        wait(lambda: has_speed(lambda v: abs(v - 0.6) < 1e-5), 'new navigation owns output')
        state['feedback'] = False
        time.sleep(0.3)
        assert_zero()
        require(future_result(second_result).status == 6, 'feedback loss aborts navigation')
        state['feedback'], state['nav'] = True, None
        time.sleep(0.25)
        checks.append('new path and ongoing feedback-loss stop')

        stream_goal = path_goal()
        require(stream_goal.accepted, 'path for stream timeout accepted')
        stream_result = stream_goal.get_result_async()
        wait(lambda: state['follows'] == 3, 'stream timeout FollowPath dispatched')
        state['nav'] = 0.5
        wait(lambda: has_speed(lambda v: v > 0.49), 'navigation stream active')
        state['nav'] = None
        time.sleep(0.3)
        assert_zero()
        require(future_result(stream_result).status == 6, 'missing navigation stream aborts task')
        time.sleep(0.2)
        checks.append('navigation stream timeout stops and aborts')

        # 接管发生在下游接受响应返回前，迟到句柄仍需取消并等待终态。
        state['delay'], state['result'] = 0.6, False
        third = path_goal()
        require(third.accepted, 'late acceptance scenario path accepted')
        wait(lambda: state['follows'] == 4, 'delayed FollowPath request received')
        previous_cancels = state['cancels']
        state['remote'] = command(op=1)
        wait(lambda: state['cancels'] > previous_cancels, 'late FollowPath handle is canceled')
        assert_zero()
        state['result'] = True
        wait(lambda: has_speed(lambda v: v < -0.02), 'reverse remote after delayed goal terminates')
        state['remote'] = None
        time.sleep(0.9)
        checks.append('late accepted FollowPath is canceled; reverse remote works')

        # 交接期间单帧命令过期，终态到达后不得执行旧遥控目标。
        state['delay'], state['result'] = 0.0, False
        expired_goal = path_goal()
        require(expired_goal.accepted, 'expiry scenario path accepted')
        wait(lambda: state['follows'] == 5, 'expiry scenario FollowPath dispatched')
        old_cancels = state['cancels']
        remote.publish(command())
        wait(lambda: state['cancels'] > old_cancels, 'single remote frame cancels path')
        time.sleep(0.25)
        state['result'] = True
        assert_zero(0.35)
        checks.append('command expires during handover and cannot move later')

        # 下游拒绝 Goal 也必须释放 outstanding 计数。
        state['delay'], state['reject'] = 0.0, True
        rejected = path_goal()
        require(rejected.accepted, 'outer goal before downstream rejection')
        wait(lambda: state['follows'] == 6, 'downstream reject scenario dispatched')
        time.sleep(0.2)
        state['remote'] = command()
        wait(lambda: has_speed(lambda v: v > 0.02), 'downstream rejection does not leak pending count')
        checks.append('downstream rejection releases pending request')

        # 运行中的遥控同样不能在旧反馈下继续行驶。
        state['feedback'] = False
        time.sleep(0.3)
        assert_zero()
        state['remote'], state['feedback'] = None, True
        time.sleep(0.25)
        checks.append('remote feedback loss inhibits motion')

        state['remote'] = command(op=2, speed=0.0)
        state['remote'].angular_velocity = 0.3
        wait(lambda: samples and samples[-1][2] > 0.02, 'left-turn remote controls angular output')
        state['remote'] = command(speed=0.0)
        wait(lambda: samples and abs(samples[-1][2]) < 1e-6, 'zero remote command stops rotation')
        require(not path_goal().accepted, 'fresh zero remote commands retain ownership')
        state['remote'] = None
        time.sleep(0.25)
        checks.append('angular remote and explicit zero command ownership')

        extra = node.create_publisher(ControlRes, '/control_to_uart', 10)
        time.sleep(0.2)
        require(not path_goal().accepted, 'extra output publisher inhibits navigation')
        assert_zero()
        node.destroy_publisher(extra)
        time.sleep(0.2)
        checks.append('extra output publisher inhibits control')
        transition(4)
        checks.append('lifecycle deactivation')
        print('\n'.join(f'PASS: {item}' for item in checks), flush=True)
        print(f'PASS: {len(checks)} isolated runtime scenarios', flush=True)
    finally:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3)
        log.close()
        executor.shutdown(timeout_sec=3)
        thread.join(timeout=3)
        action.destroy()
        server.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
