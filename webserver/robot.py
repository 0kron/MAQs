"""Synthetic Go2 telemetry over a rosbridge-compatible WebSocket.

Run python3 robot.py, then open http://localhost:8000/main.html.
Requires websockets>=12,<17. No ROS installation required.
Kinematic sensor demonstration, not a dynamics simulator or a ROS/DDS node.
"""

import argparse
import asyncio
import base64
import functools
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import random
from pathlib import Path
import struct
import threading
import time
import xml.etree.ElementTree as ET

import websockets

ROOT = Path(__file__).resolve().parent
URDF_PATH = ROOT / 'assets/go2_description/urdf/go2_description.urdf'
MOTOR_LEGS = ('FR', 'FL', 'RR', 'RL')
JOINT_NAMES = tuple(f'{leg}_{joint}_joint' for leg in MOTOR_LEGS for joint in ('hip', 'thigh', 'calf'))
RATE, LIDAR_RATE = 30, 10
BODY_HEIGHT, RADIUS, YAW_RATE = 0.32, 1.0, 0.22
SPEED = RADIUS * YAW_RATE
LEG_LENGTH, HIP_OFFSET, STEP_PERIOD = 0.213, 0.0955, 0.8
ROOM = ((-3.5, -3.0, 0.0), (3.5, 3.0, 2.2))
BOXES = (((1.7, 0.4, 0.0), (2.2, 1.1, 0.85)), ((-2.4, -1.6, 0.0), (-1.6, -0.9, 1.1)))
TOPICS = {
    '/tf': 'tf2_msgs/msg/TFMessage', '/tf_static': 'tf2_msgs/msg/TFMessage',
    '/utlidar/robot_odom': 'nav_msgs/msg/Odometry',
    '/utlidar/robot_pose': 'geometry_msgs/msg/PoseStamped',
    '/sportmodestate': 'unitree_go/msg/SportModeState',
    '/lf/sportmodestate': 'unitree_go/msg/SportModeState',
    '/lowstate': 'unitree_go/msg/LowState', '/lf/lowstate': 'unitree_go/msg/LowState',
    '/utlidar/imu': 'sensor_msgs/msg/Imu', '/utlidar/lidar_state': 'unitree_go/msg/LidarState',
    '/utlidar/cloud': 'sensor_msgs/msg/PointCloud2',
    '/utlidar/cloud_base': 'sensor_msgs/msg/PointCloud2',
    '/utlidar/cloud_deskewed': 'sensor_msgs/msg/PointCloud2',
    '/uslam/cloud_map': 'sensor_msgs/msg/PointCloud2',
    '/utlidar/voxel_map': 'sensor_msgs/msg/PointCloud2',
}


def vec(x=0.0, y=0.0, z=0.0):
    return {'x': x, 'y': y, 'z': z}


def quat(roll=0.0, pitch=0.0, yaw=0.0):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return {'x': sr*cp*cy - cr*sp*sy, 'y': cr*sp*cy + sr*cp*sy,
            'z': cr*cp*sy - sr*sp*cy, 'w': cr*cp*cy + sr*sp*sy}


def rotate(q, point):
    """Rotate an XYZ vector by a unit XYZW quaternion."""
    x, y, z = point
    a, b, c, w = q['x'], q['y'], q['z'], q['w']
    tx, ty, tz = 2*(b*z-c*y), 2*(c*x-a*z), 2*(a*y-b*x)
    return (x+w*tx+b*tz-c*ty, y+w*ty+c*tx-a*tz, z+w*tz+a*ty-b*tx)


def inverse(q):
    return {'x': -q['x'], 'y': -q['y'], 'z': -q['z'], 'w': q['w']}


def multiply(a, b):
    ax, ay, az, aw = (a[k] for k in ('x', 'y', 'z', 'w'))
    bx, by, bz, bw = (b[k] for k in ('x', 'y', 'z', 'w'))
    return {'x': aw*bx+ax*bw+ay*bz-az*by, 'y': aw*by-ax*bz+ay*bw+az*bx,
            'z': aw*bz+ax*by-ay*bx+az*bw, 'w': aw*bw-ax*bx-ay*by-az*bz}


def timestamp(now):
    sec = int(now)
    return {'sec': sec, 'nanosec': int((now-sec)*1e9)}


def header(stamp, frame):
    return {'stamp': stamp, 'frame_id': frame}


def diagonal(size, value):
    return [value if i//size == i%size else 0.0 for i in range(size*size)]


def joint_definitions():
    result = []
    for joint in ET.parse(URDF_PATH).getroot().findall('joint'):
        origin = joint.find('origin')
        xyz = tuple(map(float, origin.get('xyz', '0 0 0').split()))
        rpy = tuple(map(float, origin.get('rpy', '0 0 0').split()))
        axis = joint.find('axis')
        result.append({'name': joint.get('name'), 'fixed': joint.get('type') == 'fixed',
                       'parent': joint.find('parent').get('link'), 'child': joint.find('child').get('link'),
                       'xyz': xyz, 'quaternion': quat(*rpy),
                       'axis': tuple(map(float, axis.get('xyz', '0 0 0').split())) if axis is not None else (0, 0, 0)})
    return result


JOINTS = joint_definitions()
LIDAR_MOUNT = next(j for j in JOINTS if j['name'] == 'radar_joint')


def transform(stamp, parent, child, xyz, q):
    return {'header': header(stamp, parent), 'child_frame_id': child,
            'transform': {'translation': vec(*xyz), 'rotation': q}}


def static_tf(stamp):
    return {'transforms': [transform(stamp, j['parent'], j['child'], j['xyz'], j['quaternion'])
                           for j in JOINTS if j['fixed'] and 'camera' not in j['name']]}


def gait(t):
    """Inverse kinematics for an alternating diagonal trot.

    Stance feet stay in the world while the body moves and turns. Swing feet
    interpolate to the next contact with a small lift. Positions use base.
    """
    angles, feet, forces = [], [], []
    for leg in MOTOR_LEGS:
        side = 1 if leg.endswith('L') else -1
        hx = 0.1934 if leg.startswith('F') else -0.1934
        hy = side*0.0465
        offset = 0.0 if leg in ('FR', 'RL') else 0.5
        phase = (t/STEP_PERIOD + offset) % 1
        swing = phase < 0.5
        half_step = STEP_PERIOD*0.25
        if swing:
            p = phase*2
            blend = (1-math.cos(math.pi*p))/2
            dt = (2*blend-1)*half_step
            lift = 0.045*math.sin(math.pi*p)
        else:
            dt = (0.75-phase)*STEP_PERIOD
            lift = 0.0
        # Exact planar rigid-body inverse motion around mid-stance.
        a = YAW_RATE*dt
        anchor_x, anchor_y = hx, hy + side*HIP_OFFSET
        fx = math.cos(a)*anchor_x - math.sin(a)*(anchor_y-RADIUS)
        fy = math.sin(a)*anchor_x + math.cos(a)*(anchor_y-RADIUS) + RADIUS
        foot_x, foot_y, foot_z = fx-hx, fy-hy, -BODY_HEIGHT+lift
        vertical = math.sqrt(foot_y**2 + foot_z**2 - HIP_OFFSET**2)
        hip = math.atan2(foot_y, -foot_z) - math.atan2(side*HIP_OFFSET, vertical)
        calf = -math.acos(max(-1, min(1, (foot_x**2+vertical**2-2*LEG_LENGTH**2)/(2*LEG_LENGTH**2))))
        thigh = math.atan2(-foot_x, vertical) - calf/2
        angles.extend((hip, thigh, calf))
        feet.extend((fx, fy, foot_z))
        forces.append(0 if swing else 65)
    return angles, feet, forces


def state(t):
    theta = YAW_RATE*t
    yaw = theta + math.pi/2
    xyz = (RADIUS*math.cos(theta), RADIUS*math.sin(theta), BODY_HEIGHT)
    return xyz, quat(yaw=yaw), yaw


def telemetry(t, now, speed=1.0):
    stamp = timestamp(now)
    xyz, q, yaw = state(t)
    angles, feet, forces = gait(t)
    eps = 0.001
    before, feet_before, _ = gait(t-eps)
    after, feet_after, _ = gait(t+eps)
    rates = [speed*(a-b)/(2*eps) for a, b in zip(after, before)]
    accelerations = [speed**2*(a-2*c+b)/eps**2 for a, b, c in zip(after, before, angles)]
    foot_rates = [speed*(a-b)/(2*eps) for a, b in zip(feet_after, feet_before)]
    pose = {'position': vec(*xyz), 'orientation': q}
    # Odometry twist is in child_frame_id, not in the world frame.
    odom = {'header': header(stamp, 'odom'), 'child_frame_id': 'base',
            'pose': {'pose': pose, 'covariance': diagonal(6, 0.001)},
            'twist': {'twist': {'linear': vec(SPEED*speed, 0, 0), 'angular': vec(0, 0, YAW_RATE*speed)}, 'covariance': diagonal(6, 0.001)}}
    imu_body = {'quaternion': [q[k] for k in ('w', 'x', 'y', 'z')],
                'gyroscope': [0.0, 0.0, YAW_RATE*speed], 'accelerometer': [0.0, SPEED*YAW_RATE*speed**2, 9.81],
                'rpy': [0.0, 0.0, math.atan2(math.sin(yaw), math.cos(yaw))], 'temperature': 30}
    sport = {'stamp': stamp, 'error_code': 0, 'imu_state': imu_body, 'mode': 3,
             'progress': 0.0, 'gait_type': 1, 'foot_raise_height': 0.045, 'position': list(xyz),
             'body_height': BODY_HEIGHT,
             'velocity': [-SPEED*speed*math.sin(YAW_RATE*t), SPEED*speed*math.cos(YAW_RATE*t), 0.0],
             'yaw_speed': YAW_RATE*speed, 'range_obstacle': [2.0]*4,
             'foot_force': forces, 'foot_position_body': feet, 'foot_speed_body': foot_rates}
    motors = [{'mode': 1, 'q': angles[i] if i < 12 else 0.0,
               'dq': rates[i] if i < 12 else 0.0, 'ddq': accelerations[i] if i < 12 else 0.0,
               'tau_est': 0.0, 'q_raw': angles[i] if i < 12 else 0.0,
               'dq_raw': rates[i] if i < 12 else 0.0, 'ddq_raw': accelerations[i] if i < 12 else 0.0,
               'temperature': 30, 'lost': 0, 'reserve': [0, 0]} for i in range(20)]
    low = {'head': base64.b64encode(bytes((0xFE, 0xEF))).decode(), 'level_flag': 0xFF,
           'frame_reserve': 0, 'sn': [0, 0], 'version': [0, 0], 'bandwidth': 0,
           'imu_state': imu_body, 'motor_state': motors,
           'bms_state': {'version_high': 0, 'version_low': 0, 'status': 1, 'soc': 90, 'current': -1000,
                         'cycle': 0, 'bq_ntc': [25, 25], 'mcu_ntc': [25, 25], 'cell_vol': [4000]*15},
           'foot_force': forces, 'foot_force_est': forces, 'tick': int(t*1000) % (2**32),
           'wireless_remote': base64.b64encode(bytes(40)).decode(), 'bit_flag': 0,
           'adc_reel': 0.0, 'temperature_ntc1': 25, 'temperature_ntc2': 25, 'power_v': 28.0,
           'power_a': 1.0, 'fan_frequency': [0]*4, 'reserve': 0, 'crc': 0}
    angle_by_name = dict(zip(JOINT_NAMES, angles))
    dynamic = [transform(stamp, 'odom', 'base', xyz, q)]
    for j in JOINTS:
        if j['fixed']:
            continue
        angle = angle_by_name[j['name']]
        s = math.sin(angle/2)
        rotation = dict(zip(('x', 'y', 'z', 'w'), [v*s for v in j['axis']] + [math.cos(angle/2)]))
        dynamic.append(transform(stamp, j['parent'], j['child'], j['xyz'], multiply(j['quaternion'], rotation)))
    lidar_q = multiply(q, LIDAR_MOUNT['quaternion'])
    gyro = rotate(inverse(LIDAR_MOUNT['quaternion']), (0, 0, YAW_RATE*speed))
    # Specific acceleration includes the sensor's centripetal offset.
    accel = rotate(inverse(LIDAR_MOUNT['quaternion']), (-YAW_RATE**2*speed**2*LIDAR_MOUNT['xyz'][0], SPEED*YAW_RATE*speed**2, 9.81))
    imu = {'header': header(stamp, 'radar'), 'orientation': lidar_q,
           'orientation_covariance': diagonal(3, 0.001), 'angular_velocity': vec(*gyro),
           'angular_velocity_covariance': diagonal(3, 0.001), 'linear_acceleration': vec(*accel),
           'linear_acceleration_covariance': diagonal(3, 0.01)}
    return {'/tf': {'transforms': dynamic}, '/utlidar/robot_odom': odom,
            '/utlidar/robot_pose': {'header': header(stamp, 'odom'), 'pose': pose},
            '/sportmodestate': sport, '/lf/sportmodestate': sport,
            '/lowstate': low, '/lf/lowstate': low, '/utlidar/imu': imu}


def ray_box(origin, direction, bounds):
    """Nearest positive ray/AABB hit; inside rays return the exit face."""
    near, far = -math.inf, math.inf
    for o, d, lo, hi in zip(origin, direction, *bounds):
        if abs(d) < 1e-10:
            if o < lo or o > hi:
                return None
            continue
        a, b = (lo-o)/d, (hi-o)/d
        near, far = max(near, min(a, b)), min(far, max(a, b))
        if near > far:
            return None
    hit = near if near > 0.05 else far
    return hit if 0.05 < hit < 10.0 else None


# The topic type does not specify the physical sensor's actual beam pattern.
RAYS = tuple((math.cos(e)*math.cos(a), math.cos(e)*math.sin(a), math.sin(e))
             for e in (math.radians(-80+k*160/23) for k in range(24))
             for a in (k*2*math.pi/120 for k in range(120)))
RAYS_BASE = tuple(rotate(LIDAR_MOUNT['quaternion'], ray) for ray in RAYS)


# SDK PointType: aligned XYZ (16 bytes), float intensity, uint16 ring,
# float relative time, rounded to 32 bytes by PCL/Eigen's 16-byte alignment.
# This public SDK profile is not a capture of Go2's built-in firmware topics.
POINT_STRUCT = struct.Struct('<fff4xfH2xf4x')
POINT_FIELDS = (('x', 0, 7), ('y', 4, 7), ('z', 8, 7),
                ('intensity', 16, 7), ('ring', 20, 4), ('time', 24, 7))


def point_cloud(points, stamp, frame):
    data = b''.join(POINT_STRUCT.pack(*p) for p in points)
    return {'header': header(stamp, frame), 'height': 1, 'width': len(points),
            'fields': [{'name': name, 'offset': offset, 'datatype': datatype, 'count': 1}
                       for name, offset, datatype in POINT_FIELDS],
            'is_bigendian': False, 'point_step': POINT_STRUCT.size, 'row_step': POINT_STRUCT.size*len(points),
            'data': base64.b64encode(data).decode(), 'is_dense': True}


def scan(t, now):
    xyz, q, yaw = state(t)
    mount = LIDAR_MOUNT
    offset = rotate(q, mount['xyz'])
    origin = tuple(a+b for a, b in zip(xyz, offset))
    cy, sy = math.cos(yaw), math.sin(yaw)
    local_points, base_points, world_points = [], [], []
    rng = random.Random(math.floor(t*LIDAR_RATE))
    for index, (ray, body_ray) in enumerate(zip(RAYS, RAYS_BASE)):
        # The simulated body rotates only around Z. Cache the fixed sensor
        # rotation rather than multiplying quaternions for every ray.
        direction = (cy*body_ray[0]-sy*body_ray[1], sy*body_ray[0]+cy*body_ray[1], body_ray[2])
        hits = [hit for b in (ROOM, *BOXES) if (hit := ray_box(origin, direction, b)) is not None]
        if not hits:
            continue
        # Repeatable range noise; surfaces stay fixed in the world.
        distance = min(hits) + 0.003*math.sin(index*1.37+t*2)
        point = tuple(distance*v for v in ray)
        bp = tuple(distance*v for v in body_ray)
        bp = tuple(a+b for a, b in zip(bp, mount['xyz']))
        wp = tuple(o+distance*d for o, d in zip(origin, direction))
        # Random demo values, not calibrated reflectivity. Reproducible per scan;
        # each view retains the identical intensity for the same physical hit.
        intensity = rng.uniform(0.0, 255.0)
        ring, relative_time = index//120, 0.0  # Virtual rings; instantaneous scan.
        attributes = (intensity, ring, relative_time)
        local_points.append((*point, *attributes)); base_points.append((*bp, *attributes)); world_points.append((*wp, *attributes))
    stamp = timestamp(now)
    return {'/utlidar/cloud': point_cloud(local_points, stamp, 'radar'),
            '/utlidar/cloud_base': point_cloud(base_points, stamp, 'base'),
            '/utlidar/cloud_deskewed': point_cloud(world_points, stamp, 'odom')}, world_points


def lidar_state(t, now, count):
    # State/status values describe this mock; they are not hardware readings.
    return {'stamp': now, 'firmware_version': 'synthetic', 'software_version': 'synthetic',
            'sdk_version': 'synthetic', 'sys_rotation_speed': 0.0, 'com_rotation_speed': 0.0,
            'error_state': 0, 'cloud_frequency': float(LIDAR_RATE), 'cloud_packet_loss_rate': 0.0,
            'cloud_size': count, 'cloud_scan_num': int(t*LIDAR_RATE), 'imu_frequency': float(RATE),
            'imu_packet_loss_rate': 0.0, 'imu_rpy': [0.0, 2.8782, YAW_RATE*t+math.pi/2],
            'serial_recv_stamp': now, 'serial_buffer_size': 0, 'serial_buffer_read': 0}


class Simulator:
    def __init__(self, speed=1.0):
        if not math.isfinite(speed) or speed < 0:
            raise ValueError('Simulation speed must be finite and nonnegative')
        self.speed = speed
        self.clients = {}
        self.started = time.monotonic()
        self.map_voxels = {}
        self.latest_static = static_tf(timestamp(time.time()))

    async def client(self, websocket):
        # Topic -> (minimum interval, last send time). Read-only rosbridge
        # subset: subscribe and unsubscribe, with static transforms latched.
        subscriptions = {}
        self.clients[websocket] = subscriptions
        try:
            async for raw in websocket:
                try:
                    request = json.loads(raw)
                    topic = request.get('topic')
                    if request.get('op') == 'subscribe':
                        if topic not in TOPICS:
                            await websocket.send(json.dumps({'op': 'status', 'level': 'error', 'msg': f'Unsupported synthetic topic: {topic}'}))
                            continue
                        subscriptions[topic] = [max(0, float(request.get('throttle_rate', 0)))/1000, -math.inf]
                        if topic == '/tf_static':
                            await websocket.send(json.dumps({'op': 'publish', 'topic': topic, 'msg': self.latest_static}))
                    elif request.get('op') == 'unsubscribe':
                        subscriptions.pop(topic, None)
                except (ValueError, TypeError, AttributeError):
                    await websocket.send(json.dumps({'op': 'status', 'level': 'error', 'msg': 'Invalid subscription JSON'}))
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.clients.pop(websocket, None)

    async def broadcast(self, messages, clock):
        wanted = set().union(*(set(subs) for subs in self.clients.values()))
        encoded = {topic: json.dumps({'op': 'publish', 'topic': topic, 'msg': msg}, separators=(',', ':'), allow_nan=False)
                   for topic, msg in messages.items() if topic in wanted}

        async def send(websocket, subscriptions):
            try:
                for topic, payload in encoded.items():
                    rule = subscriptions.get(topic)
                    if rule is not None and clock-rule[1]+1e-6 >= rule[0]:
                        # Drop slow clients instead of stopping the clock.
                        await asyncio.wait_for(websocket.send(payload), timeout=1.0)
                        rule[1] = clock
            except (asyncio.TimeoutError, websockets.exceptions.ConnectionClosed):
                self.clients.pop(websocket, None)
                await websocket.close()
        await asyncio.gather(*(send(ws, subs) for ws, subs in list(self.clients.items())))

    async def run(self):
        tick = 0
        next_scan = next_map = 0.0
        while True:
            await asyncio.sleep(max(0, self.started+tick/RATE-time.monotonic()))
            now = time.time()
            elapsed = time.monotonic()-self.started
            t = elapsed*self.speed
            if self.clients:
                messages = telemetry(t, now, speed=self.speed)
                if tick % 6:
                    messages.pop('/lf/lowstate'); messages.pop('/lf/sportmodestate')
                if elapsed >= next_scan:
                    next_scan = elapsed + 1/LIDAR_RATE
                    clouds, points = scan(t, now)
                    messages.update(clouds)
                    # Bounded 10 cm map accumulated from actual scan hits.
                    for p in points:
                        self.map_voxels[tuple(round(v/0.1) for v in p[:3])] = p
                    if len(self.map_voxels) > 20000:
                        self.map_voxels = dict(list(self.map_voxels.items())[-20000:])
                    if elapsed >= next_map:
                        next_map = elapsed + 1.0
                        cloud_map = point_cloud(list(self.map_voxels.values()), timestamp(now), 'odom')
                        messages['/uslam/cloud_map'] = cloud_map; messages['/utlidar/voxel_map'] = cloud_map
                        messages['/utlidar/lidar_state'] = lidar_state(t, now, len(points))
                # Delivery cadence uses wall time; only the motion clock slows down.
                await self.broadcast(messages, elapsed)
            # Skip missed ticks rather than sending a backlog in bursts.
            tick = max(tick+1, math.floor((time.monotonic()-self.started)*RATE)+1)


class PageHandler(SimpleHTTPRequestHandler):
    def log_message(self, format, *args):
        pass


async def capture_real_topics(url, output, seconds=10.0):
    """Save real rosbridge messages for firmware/layout verification. Read-only."""
    captured, statuses, tf_edges = {}, [], {'/tf': {}, '/tf_static': {}}
    async with websockets.connect(url, open_timeout=5, max_size=32_000_000) as ws:
        for topic, msg_type in TOPICS.items():
            await ws.send(json.dumps({'op': 'subscribe', 'topic': topic, 'type': msg_type,
                                      'queue_length': 1, 'throttle_rate': 100, 'compression': 'none'}))
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            try:
                payload = json.loads(await asyncio.wait_for(ws.recv(), deadline-time.monotonic()))
            except asyncio.TimeoutError:
                break
            if payload.get('op') == 'status':
                statuses.append(payload)
            elif payload.get('op') == 'publish' and payload.get('topic') in TOPICS:
                topic, msg = payload['topic'], payload['msg']
                if topic in tf_edges:
                    for edge in msg['transforms']:
                        tf_edges[topic][(edge['header']['frame_id'], edge['child_frame_id'])] = edge
                    captured[topic] = {'op': 'publish', 'topic': topic,
                                       'msg': {'transforms': list(tf_edges[topic].values())}}
                else:
                    captured.setdefault(topic, payload)
    missing = sorted(set(TOPICS)-set(captured))
    report = {'endpoint': url, 'captured_at': time.time(), 'messages': list(captured.values()),
              'missing_topics': missing, 'bridge_status': statuses}
    Path(output).write_text(json.dumps(report, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(f'Saved {len(captured)} topics to {Path(output).resolve()}', flush=True)
    if missing:
        print('No messages received for: '+', '.join(missing), flush=True)
    return report


async def main(args):
    simulator = Simulator(speed=args.speed)
    server = None
    try:
        if not args.no_http:
            server = ThreadingHTTPServer((args.host, args.http_port), functools.partial(PageHandler, directory=str(ROOT)))
            threading.Thread(target=server.serve_forever, daemon=True).start()
        async with websockets.serve(simulator.client, args.host, args.port, max_size=1_000_000):
            print(f'Synthetic Go2 topics: ws://{args.host}:{args.port}', flush=True)
            if server:
                print(f'Open http://localhost:{args.http_port}/main.html?port={args.port}', flush=True)
            print('Ctrl+C stops the simulator. No ROS or physical robot connection.', flush=True)
            print(f'Simulation speed: {args.speed:g}x (1 = original speed)', flush=True)
            await simulator.run()
    finally:
        if server:
            server.shutdown(); server.server_close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--http-port', type=int, default=8000)
    parser.add_argument('--speed', type=float, default=0.5, help='Simulation motion speed: 0 pauses, 0.5 is half speed, 1 is original speed')
    parser.add_argument('--no-http', action='store_true')
    parser.add_argument('--dump-one', action='store_true', help='Print a synthetic frame as rosbridge JSON and exit')
    parser.add_argument('--capture', metavar='WS_URL', help='Read real rosbridge topics into a local verification file, then exit')
    parser.add_argument('--capture-output', default=str(ROOT/'live_sample.json'))
    parser.add_argument('--capture-seconds', type=float, default=10.0)
    args = parser.parse_args()
    if not math.isfinite(args.speed) or args.speed < 0:
        parser.error('--speed must be finite and nonnegative')
    if args.capture:
        if not math.isfinite(args.capture_seconds) or args.capture_seconds <= 0:
            parser.error('--capture-seconds must be positive and finite')
        try:
            asyncio.run(capture_real_topics(args.capture, args.capture_output, args.capture_seconds))
        except (OSError, TimeoutError, websockets.exceptions.WebSocketException) as error:
            parser.exit(1, f'Cannot capture from {args.capture}: {error}\n')
    elif args.dump_one:
        now = time.time()
        messages = {'/tf_static': static_tf(timestamp(now)), **telemetry(0.25, now)}
        clouds, points = scan(0.25, now); messages.update(clouds)
        messages['/utlidar/lidar_state'] = lidar_state(0.25, now, len(points))
        messages['/uslam/cloud_map'] = messages['/utlidar/cloud_deskewed']
        messages['/utlidar/voxel_map'] = messages['/utlidar/cloud_deskewed']
        print(json.dumps([{'op': 'publish', 'topic': topic, 'msg': msg} for topic, msg in messages.items()], indent=2, allow_nan=False))
    else:
        try:
            asyncio.run(main(args))
        except KeyboardInterrupt:
            print('\nSimulator stopped.')
        except OSError as error:
            parser.exit(1, f'{error}\nChoose another --port or --http-port if the port is occupied.\n')
