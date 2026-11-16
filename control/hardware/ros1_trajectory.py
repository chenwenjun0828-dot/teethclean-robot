"""ROS 1 FollowJointTrajectory transport. Imports ROS only upon construction."""
import threading
import time

import numpy as np


def quaternion_matrix(quaternion):
    x, y, z, w = quaternion
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def quaternion_product(a, b):
    av, bv, aw, bw = np.asarray(a[:3]), np.asarray(b[:3]), a[3], b[3]
    return np.r_[aw*bv+bw*av+np.cross(av, bv), aw*bw-av@bv]


def matrix_quaternion(matrix):
    # Eigenvector form remains stable for rotations close to 180 degrees.
    r = np.asarray(matrix)
    k = np.array([[r[0, 0]-r[1, 1]-r[2, 2], r[0, 1]+r[1, 0], r[0, 2]+r[2, 0], r[2, 1]-r[1, 2]],
                  [r[0, 1]+r[1, 0], r[1, 1]-r[0, 0]-r[2, 2], r[1, 2]+r[2, 1], r[0, 2]-r[2, 0]],
                  [r[0, 2]+r[2, 0], r[1, 2]+r[2, 1], r[2, 2]-r[0, 0]-r[1, 1], r[1, 0]-r[0, 1]],
                  [r[2, 1]-r[1, 2], r[0, 2]-r[2, 0], r[1, 0]-r[0, 1], np.trace(r)]])/3
    _, vectors = np.linalg.eigh(k)
    return vectors[:, -1]


def fill_goal(curve, goal, point_factory, duration_factory):
    """Message conversion is separate so it can be tested without ROS installed."""
    goal.trajectory.joint_names = list(curve.names)
    for p in curve.record['points']:
        point = point_factory()
        point.positions = list(p['positions'])
        point.velocities = list(p['velocities'])
        point.accelerations = list(p['accelerations'])
        nanoseconds = round(p['time_from_start']*1e9)
        point.time_from_start = duration_factory(nanoseconds//10**9, nanoseconds % 10**9)
        goal.trajectory.points.append(point)
    return goal


class Ros1TrajectoryBackend:
    def __init__(self, config, curve):
        import actionlib
        import rospy
        import tf2_ros
        from control_msgs.msg import (FollowJointTrajectoryAction, FollowJointTrajectoryGoal,
                                      JointTolerance, JointTrajectoryControllerState)
        from controller_manager_msgs.srv import ListControllers
        from geometry_msgs.msg import WrenchStamped
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool, Float64, UInt64
        from std_srvs.srv import Trigger
        from trajectory_msgs.msg import JointTrajectoryPoint
        from ur_dashboard_msgs.srv import GetRobotMode, GetSafetyMode, IsProgramRunning
        self.rospy, self.config, self.curve = rospy, config, curve
        self.lock, self.closed = threading.RLock(), threading.Event()
        self.values, self.received, self.stamp = {}, {}, {}
        self.error, self.sent = None, False
        self.counter, self.UInt64 = 0, UInt64
        if not rospy.core.is_initialized():
            rospy.init_node('dental_saved_trajectory', anonymous=True, disable_signals=True)
        if rospy.get_param('/use_sim_time', False):
            raise ValueError('Real hardware execution requires wall ROS time')
        ros = config['ros']
        self.client = actionlib.SimpleActionClient(ros['action'], FollowJointTrajectoryAction)
        self.goal = fill_goal(curve, FollowJointTrajectoryGoal(), JointTrajectoryPoint, rospy.Duration)
        for name in curve.names:
            self.goal.path_tolerance.append(JointTolerance(name=name, position=config['limits']['joint_tracking_rad']))
            self.goal.goal_tolerance.append(JointTolerance(name=name, position=config['limits']['joint_tracking_rad']))
        if not self.client.wait_for_server(rospy.Duration(config['limits']['max_feedback_age_s'])):
            raise RuntimeError('FollowJointTrajectory server unavailable')
        self.tf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf)
        self.subscribers = [
            rospy.Subscriber(ros['joint_states_topic'], JointState, self._joints, queue_size=1),
            rospy.Subscriber(ros['wrench_topic'], WrenchStamped, self._wrench, queue_size=1),
            rospy.Subscriber(ros['speed_scaling_topic'], Float64,
                             lambda msg: self._put('speed_scaling', float(msg.data)), queue_size=1),
            rospy.Subscriber(ros['watchdog_ready_topic'], Bool,
                             lambda msg: self._put('watchdog', bool(msg.data)), queue_size=1),
            rospy.Subscriber(ros['controller_state_topic'], JointTrajectoryControllerState,
                             self._controller_state, queue_size=1)]
        self.publisher = rospy.Publisher(ros['heartbeat_topic'], UInt64, queue_size=1, latch=False)
        self.stop_proxy = rospy.ServiceProxy(ros['stop_service'], Trigger)
        # Dashboard modes are latched topics in the reference driver. Poll its
        # read-only services to distinguish unchanged status from lost updates.
        services = [('controllers', ListControllers, ros['list_controllers_service'], lambda r: r.controller),
                    ('robot_mode', GetRobotMode, ros['robot_mode_service'], lambda r: r.robot_mode.mode),
                    ('safety_mode', GetSafetyMode, ros['safety_mode_service'], lambda r: r.safety_mode.mode),
                    ('program_running', IsProgramRunning, ros['program_running_service'], lambda r: r.program_running)]
        for name, service_type, path, extract in services:
            proxy = rospy.ServiceProxy(path, service_type)
            threading.Thread(target=self._poll, args=(name, proxy, extract), daemon=True).start()

    def _put(self, name, value, stamp=None):
        with self.lock:
            self.values[name], self.received[name] = value, time.monotonic()
            if stamp is not None:
                self.stamp[name] = float(stamp)

    def _poll(self, name, proxy, extract):
        while not self.closed.is_set():
            try:
                response = proxy()
                if hasattr(response, 'success') and not response.success:
                    raise ValueError('Dashboard read failed: '+name)
                self._put(name, extract(response))
            except Exception:
                pass  # Existing receive time ages out; failed reads never refresh it.
            self.closed.wait(min(.1, self.config['limits']['max_state_age_s']/3))

    def _joints(self, msg):
        try:
            if len(set(msg.name)) != len(msg.name):
                raise ValueError('Duplicate joint state names')
            order = [msg.name.index(name) for name in self.curve.names]
            self._put('joints', ([msg.position[i] for i in order], [msg.velocity[i] for i in order]),
                      msg.header.stamp.to_sec())
        except Exception as error:
            self.error = 'invalid_joint_state: '+str(error)

    def _wrench(self, msg):
        if msg.header.frame_id != self.config['ros']['wrench_frame']:
            self.error = 'wrench_frame_mismatch'
            return
        f, t = msg.wrench.force, msg.wrench.torque
        self._put('wrench', [f.x, f.y, f.z, t.x, t.y, t.z], msg.header.stamp.to_sec())

    def _controller_state(self, msg):
        try:
            if list(msg.joint_names) != self.curve.names:
                raise ValueError('Controller feedback joint order mismatch')
            progress, desired = float(msg.actual.time_from_start.to_sec()), list(msg.desired.positions)
            if len(desired) != 6 or not np.isfinite([progress]+desired).all():
                raise ValueError('Invalid controller state clock or reference')
            # In ros_control, desired.time_from_start can denote the segment
            # boundary. actual.time_from_start carries the sampled trajectory clock.
            self._put('feedback', (progress, desired), msg.header.stamp.to_sec())
        except Exception as error:
            self.error = str(error)

    def heartbeat(self):
        self.counter += 1
        self.publisher.publish(self.UInt64(data=self.counter))

    def snapshot(self):
        ros = self.config['ros']
        try:
            tf = self.tf.lookup_transform(ros['base_frame'], ros['tip_frame'], self.rospy.Time(0))
            p, q = tf.transform.translation, tf.transform.rotation
            world = np.asarray(self.config['planning_world_from_base_m'])
            tcp = np.r_[world[:3, :3]@np.array([p.x, p.y, p.z])+world[:3, 3],
                        quaternion_product(matrix_quaternion(world[:3, :3]), [q.x, q.y, q.z, q.w])].tolist()
            self._put('tcp', tcp, tf.header.stamp.to_sec())
        except Exception:
            pass
        with self.lock:
            now, ros_now = time.monotonic(), self.rospy.Time.now().to_sec()
            ages = {key: now-received for key, received in self.received.items()}
            for key, stamp in self.stamp.items():
                age = ros_now-stamp
                ages[key] = max(ages.get(key, float('inf')), age) if stamp > 0 and age >= -.01 else float('inf')
            q, dq = self.values.get('joints', ([], []))
            progress, desired = self.values.get('feedback', (0., self.curve.q[0].tolist()))
            controllers = self.values.get('controllers', [])
            own = [c for c in controllers if c.name == ros['controller']]
            conflicting = []
            for c in controllers:
                if c.name == ros['controller'] or c.state != 'running':
                    continue
                for resource in c.claimed_resources:
                    if ('Joint' in resource.hardware_interface and 'State' not in resource.hardware_interface
                            and set(resource.resources) & set(self.curve.names)):
                        conflicting.append(c.name)
            status = self.client.get_state()
            states = {0: 'pending', 1: 'active', 2: 'preempted', 3: 'succeeded', 4: 'aborted',
                      5: 'rejected', 6: 'preempting', 7: 'recalling', 8: 'recalled', 9: 'lost'}
            result = self.client.get_result()
            ages['controller_state'] = ages.get('feedback', float('inf'))
            if not self.sent or self.stamp.get('feedback', 0.) < self.goal.trajectory.header.stamp.to_sec():
                ages['feedback'] = float('inf')
                progress, desired = 0., self.curve.q[0].tolist()
            elif progress >= self.curve.duration and np.allclose(desired, self.curve.q[-1], atol=1e-6, rtol=0):
                # State publications continue while holding the completed goal.
                progress = self.curve.duration
            return {'q': q, 'dq': dq, 'tcp': self.values.get('tcp', []),
                    'wrench': self.values.get('wrench', []), 'progress_s': progress, 'desired_q': desired,
                    'ages': ages, 'controller_running': bool(own and own[0].state == 'running'
                         and own[0].type == 'position_controllers/ScaledJointTrajectoryController'),
                    'conflicting_controllers': conflicting, 'robot_mode': self.values.get('robot_mode'),
                    'safety_mode': self.values.get('safety_mode'), 'program_running': self.values.get('program_running'),
                    'speed_scaling': self.values.get('speed_scaling'), 'watchdog_ready': self.values.get('watchdog'),
                    'action_state': states.get(status, 'unknown'),
                    'result_code': getattr(result, 'error_code', None), 'error': self.error}

    def send(self, curve):
        if curve is not self.curve:
            raise ValueError('Prepared goal differs from verified trajectory')
        self.goal.trajectory.header.stamp = self.rospy.Time.now()+self.rospy.Duration(self.config['limits']['goal_lead_s'])
        self.sent = True
        self.client.send_goal(self.goal)

    def cancel(self):
        self.client.cancel_goal()

    def stop(self):
        finished, errors = threading.Event(), []
        def invoke():
            try:
                result = self.stop_proxy()
                if not result.success:
                    errors.append(result.message)
            except Exception as error:
                errors.append(str(error))
            finally:
                finished.set()
        threading.Thread(target=invoke, daemon=True).start()
        if not finished.wait(self.config['limits']['stop_timeout_s']):
            raise RuntimeError('Stop service timed out; independent watchdog required')
        if errors:
            raise RuntimeError('; '.join(errors))

    def close(self):
        self.closed.set()
        for subscriber in self.subscribers:
            subscriber.unregister()
        self.publisher.unregister()
