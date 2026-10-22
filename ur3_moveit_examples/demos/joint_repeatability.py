"""Encoder/TF repeatability experiment; not physical or ISO-certified metrology."""
import argparse
import csv
import json
import math
from pathlib import Path
import statistics
import tempfile
import time

import rclpy
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from ur3_moveit_examples.core.ur3_moveit_controller import UR3MoveItController, UR_JOINT_NAMES

TARGETS = {
    'home': (-.024909,-.674317,-1.823105,-.669480,1.524261,.967475),
    'a': (-.783081,-1.970703,-1.823500,.711616,1.631625,.967499),
    'b': (1.349597,-1.998401,-.797771,-.399316,1.504543,.967499),
}


def positive(value):
    n = float(value)
    if not math.isfinite(n) or n <= 0:
        raise argparse.ArgumentTypeError('must be positive and finite')
    return n


def endpoint_errors(requested, planned, actual):
    if any(len(v) != 6 or not all(map(math.isfinite, v)) for v in (requested, planned, actual)):
        raise ValueError('Expected six finite joints')
    return ([p-r for r,p in zip(requested,planned)],
            [a-p for p,a in zip(planned,actual)])


class RepeatabilityController(UR3MoveItController):
    """Observe the validated plan; never edit or replay its trajectory."""
    planned_endpoint = None

    def _validate_planned_joint_goal(self, result, target, tolerance):
        self.planned_endpoint = None
        if not super()._validate_planned_joint_goal(result, target, tolerance):
            return False
        trajectory = result.planned_trajectory.joint_trajectory
        if not trajectory.points:
            return False
        mapping = dict(zip(trajectory.joint_names, trajectory.points[-1].positions))
        endpoint = [mapping[n] for n in UR_JOINT_NAMES]
        errors, _ = endpoint_errors(target, endpoint, endpoint)
        if max(map(abs, errors)) > tolerance:
            self.get_logger().error('Plan endpoint exceeds repeatability tolerance; no execution')
            return False
        self.planned_endpoint = endpoint
        return True


def stationary(records, now):
    records = [(t,q) for t,q in records if 0 <= (now-t)*1e-9 <= 1.2]
    if len(records)<5 or (records[-1][0]-records[0][0])*1e-9 < .9:
        raise RuntimeError('Insufficient stationary samples')
    if any(not 0 < (b[0]-a[0])*1e-9 <= .2 for a,b in zip(records,records[1:])):
        raise RuntimeError('Joint timestamp gap/reversal')
    if any(max(q[i] for _,q in records)-min(q[i] for _,q in records)>.001 for i in range(6)):
        raise RuntimeError('Joints moving: excursion > 0.001 rad')
    if (now-records[-1][0])*1e-9 > .2:
        raise RuntimeError('Stale joint state')
    return records[-1]


def measure(robot, inbox, label, settle):
    inbox.clear()
    deadline = time.monotonic()+settle
    while rclpy.ok() and time.monotonic()<deadline:
        rclpy.spin_once(robot,timeout_sec=.02)
    stamp,joints = stationary(inbox,robot.get_clock().now().nanoseconds)
    deadline = time.monotonic()+2
    while rclpy.ok() and time.monotonic()<deadline:
        rclpy.spin_once(robot,timeout_sec=.02)
        if robot._tf_buffer.can_transform('base_link','tool0',Time(nanoseconds=stamp)):
            tf = robot._tf_buffer.lookup_transform('base_link','tool0',Time(nanoseconds=stamp))
            p,q = tf.transform.translation,tf.transform.rotation
            values = [p.x,p.y,p.z,q.x,q.y,q.z,q.w]
            error = max(abs(x-y) for x,y in zip(joints,TARGETS[label]))
            if not all(map(math.isfinite,values)) or error>robot.verify_joint_tolerance:
                raise RuntimeError('Invalid settled pose/joint target error')
            return [stamp,*joints,*values,error]
    raise RuntimeError('No TF at joint timestamp; latest TF not substituted')


def summarize(rows):
    groups = {}
    for prev,label in [('home','a'),('a','b'),('b','a')]:
        selected = [r for r in rows if r[1:3]==[label,prev]]
        if selected:
            axes = list(zip(*(r[10:13] for r in selected)))
            groups[f'{prev}->{label}'] = {'count':len(selected),
                'mean_xyz_m':[statistics.mean(a) for a in axes],
                'range_xyz_mm':[(max(a)-min(a))*1000 for a in axes],
                'sample_std_xyz_mm':[statistics.stdev(a)*1000 if len(a)>1 else None for a in axes]}
    return groups


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--plan-target',choices=TARGETS,default='home')
    parser.add_argument('--cycles',type=int,default=1)
    parser.add_argument('--settle-time',type=positive,default=2.)
    parser.add_argument('--endpoint-tolerance',type=positive,default=0.000001,
                        help='Joint endpoint bound [rad]; not an exactly identical endpoint')
    parser.add_argument('--max-joint-travel',type=positive,default=2.20)
    parser.add_argument('--velocity-scaling',type=positive,default=.03)
    parser.add_argument('--acceleration-scaling',type=positive,default=.03)
    parser.add_argument('--output-dir',default=str(Path.home()/'ur3_repeatability'))
    cli = parser.parse_args()
    if cli.cycles<1 or cli.settle_time<1.5 or max(cli.velocity_scaling,cli.acceleration_scaling)>1:
        parser.error('cycles>=1, settle-time>=1.5, scaling<=1 required')
    rclpy.init()
    robot = RepeatabilityController(node_name='joint_repeatability',velocity_scaling=cli.velocity_scaling,
        acceleration_scaling=cli.acceleration_scaling,max_joint_travel=cli.max_joint_travel,planning_time=10.,
        joint_goal_tolerance=cli.endpoint_tolerance)
    inbox = []
    def receive(msg):
        mapping = dict(zip(msg.name,msg.position))
        if all(n in mapping and math.isfinite(mapping[n]) for n in UR_JOINT_NAMES):
            inbox.append((msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,[mapping[n] for n in UR_JOINT_NAMES]))
            del inbox[:-1000]
    robot.create_subscription(JointState,'/joint_states',receive,qos_profile_sensor_data)
    out, status, error = None, 'failed', None
    rows = []
    try:
        if not cli.execute:
            robot.get_logger().warn(f'PLAN ONLY current -> {cli.plan_target}; other legs NOT checked')
            if not robot.move_to_joint(list(TARGETS[cli.plan_target]),execute=False):
                raise RuntimeError('Plan rejected')
            return
        root = Path(cli.output_dir).expanduser()
        root.mkdir(parents=True,exist_ok=True)
        out = Path(tempfile.mkdtemp(prefix=time.strftime('%Y%m%d_%H%M%S_'),dir=root))
        (out/'settings.json').write_text(json.dumps({'args':vars(cli),'targets':TARGETS,
            'base_frame':'base_link','tcp_frame':'tool0'},indent=2))
        robot.get_logger().warn(f'EXECUTE HOME -> A -> (B -> A) x {cli.cycles}; output={out}')
        with (out/'samples.csv').open('w',newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['cycle','target','predecessor','joint_stamp_ns',*UR_JOINT_NAMES,
                'x_m','y_m','z_m','qx','qy','qz','qw','max_joint_error_rad',
                *[f'{prefix}_{name}' for prefix in ('requested','planned','planning_error','execution_error')
                  for name in UR_JOINT_NAMES]])
            if not robot.move_to_joint(list(TARGETS['home']),execute=True):
                raise RuntimeError('HOME failed')
            prev = 'home'
            for cycle,label in [(0,'a')]+[(i,t) for i in range(1,cli.cycles+1) for t in ('b','a')]:
                if not robot.move_to_joint(list(TARGETS[label]),execute=True):
                    raise RuntimeError(f'{prev}->{label} failed, cycle={cycle}')
                row = [cycle,label,prev,*measure(robot,inbox,label,cli.settle_time)]
                planned = robot.planned_endpoint
                if planned is None:
                    raise RuntimeError('Missing validated plan endpoint')
                planning_error, execution_error = endpoint_errors(TARGETS[label],planned,row[4:10])
                row.extend([*TARGETS[label],*planned,*planning_error,*execution_error])
                writer.writerow(row)
                stream.flush()
                rows.append(row)
                robot.get_logger().info(f'Saved cycle={cycle} {prev}->{label}')
                prev = label
        status = 'completed'
    except KeyboardInterrupt:
        status,error = 'interrupted','Verify robot stopped on pendant; no automatic return'
        raise SystemExit(error)
    except Exception as exc:
        error = str(exc)
        robot.get_logger().error(error)
        raise SystemExit(1)
    finally:
        try:
            if out is not None:
                (out/'summary.json').write_text(json.dumps({'status':status,'error':error,
                    'groups':summarize(rows),'note':'Encoder/FK only; not physical accuracy or ISO test'},indent=2))
        finally:
            robot.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()

