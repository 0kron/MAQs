"""Geometry and transport invariants: python3 -m unittest -v test_robot.py."""
import asyncio
import base64
import json
import math
import struct
import time
import tempfile
import unittest
import xml.etree.ElementTree as ET

import websockets
import robot


class GeometryTests(unittest.TestCase):
    def test_slow_motion_keeps_velocities_consistent_with_wall_time(self):
        speed, elapsed, dt = 0.5, 2.8, 1e-4
        t = elapsed*speed
        messages = robot.telemetry(t, 12345.25, speed=speed)
        odom = messages['/utlidar/robot_odom']
        earlier, _, _ = robot.state((elapsed-dt)*speed)
        later, _, _ = robot.state((elapsed+dt)*speed)
        actual_velocity = tuple((b-a)/(2*dt) for a, b in zip(earlier, later))
        reported_velocity = robot.rotate(odom['pose']['pose']['orientation'],
                                         tuple(odom['twist']['twist']['linear'][k] for k in ('x', 'y', 'z')))
        self.assertLess(math.dist(actual_velocity, reported_velocity), 1e-8)
        earlier_angles = robot.gait((elapsed-dt)*speed)[0]
        later_angles = robot.gait((elapsed+dt)*speed)[0]
        for a, b, motor in zip(earlier_angles, later_angles, messages['/lowstate']['motor_state']):
            self.assertAlmostEqual((b-a)/(2*dt), motor['dq'], delta=2e-4)
        self.assertEqual(messages['/sportmodestate']['yaw_speed'], robot.YAW_RATE*speed)
        self.assertEqual(messages['/sportmodestate']['imu_state']['accelerometer'][2], 9.81)

    def test_simulation_speed_validation_and_paused_telemetry(self):
        for speed in (-1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                robot.Simulator(speed=speed)
        paused = robot.telemetry(0, 12345.25, speed=0)
        self.assertEqual(paused['/sportmodestate']['velocity'], [0, 0, 0])
        self.assertTrue(all(m['dq'] == 0 and m['ddq'] == 0 for m in paused['/lowstate']['motor_state']))

    def test_sdk_point_layout_and_intensity_consistency(self):
        messages, points = robot.scan(1.37, 12345.25)
        clouds = []
        for message in messages.values():
            self.assertEqual(message['point_step'], 32)
            expected = [('x', 0, 7), ('y', 4, 7), ('z', 8, 7), ('intensity', 16, 7), ('ring', 20, 4), ('time', 24, 7)]
            self.assertEqual([(f['name'], f['offset'], f['datatype']) for f in message['fields']], expected)
            clouds.append(list(struct.iter_unpack('<fff4xfH2xf4x', base64.b64decode(message['data']))))
        intensities = [p[3] for p in clouds[0]]
        self.assertLess(min(intensities), 5)
        self.assertGreater(max(intensities), 250)
        self.assertTrue(all(0 <= p[3] <= 255 and 0 <= p[4] < 24 and p[5] == 0 for p in clouds[0]))
        for cloud in clouds[1:]:
            self.assertEqual([p[3:] for p in cloud], [p[3:] for p in clouds[0]])
        repeated, _ = robot.scan(1.37, 12345.25)
        self.assertEqual(messages, repeated)

    def test_urdf_joint_limits_and_forward_kinematics(self):
        limits = {j.get('name'): tuple(float(j.find('limit').get(k)) for k in ('lower', 'upper'))
                  for j in ET.parse(robot.URDF_PATH).getroot().findall('joint') if j.get('type') == 'revolute'}
        for sample in range(160):
            angles, feet, _ = robot.gait(sample * 0.031)
            for name, angle in zip(robot.JOINT_NAMES, angles):
                lower, upper = limits[name]
                self.assertTrue(lower <= angle <= upper, (name, angle))
            for i, leg in enumerate(robot.MOTOR_LEGS):
                hip, thigh, calf = angles[3*i:3*i+3]
                side = 1 if leg.endswith('L') else -1
                hx = 0.1934 if leg.startswith('F') else -0.1934
                hy = side*0.0465
                x = -robot.LEG_LENGTH*(math.sin(thigh)+math.sin(thigh+calf))
                z = -robot.LEG_LENGTH*(math.cos(thigh)+math.cos(thigh+calf))
                y = side*robot.HIP_OFFSET
                actual = (hx+x, hy+math.cos(hip)*y-math.sin(hip)*z, math.sin(hip)*y+math.cos(hip)*z)
                for a, b in zip(actual, feet[3*i:3*i+3]):
                    self.assertAlmostEqual(a, b, places=10)

    def test_stance_contact_stays_fixed_in_world(self):
        for t in (0.43, 0.51, 0.63):
            contacts = []
            for instant in (t, t+0.01):
                _, feet, force = robot.gait(instant)
                xyz, q, _ = robot.state(instant)
                contact = robot.rotate(q, feet[:3])
                contacts.append(tuple(a+b for a, b in zip(xyz, contact)))
                self.assertGreater(force[0], 0)
            for a, b in zip(*contacts):
                self.assertAlmostEqual(a, b, places=10)

    def test_cloud_frames_have_same_world_hits(self):
        t, now = 1.37, 12345.25
        messages, world = robot.scan(t, now)
        xyz, q, _ = robot.state(t)
        for topic, message in messages.items():
            data = base64.b64decode(message['data'], validate=True)
            self.assertEqual(len(data), message['row_step']*message['height'])
            self.assertEqual(message['width'], len(world))
            self.assertEqual(message['header']['stamp'], robot.timestamp(now))
            for index, point in enumerate(struct.iter_unpack('<fff4xfH2xf4x', data)):
                p = point[:3]
                if topic.endswith('/cloud'):
                    p = robot.rotate(robot.LIDAR_MOUNT['quaternion'], p)
                    p = tuple(a+b for a, b in zip(p, robot.LIDAR_MOUNT['xyz']))
                if not topic.endswith('deskewed'):
                    p = robot.rotate(q, p)
                    p = tuple(a+b for a, b in zip(p, xyz))
                self.assertLess(math.dist(p, world[index][:3]), 1e-6)

    def test_ray_hits_are_first_surface_and_occluded(self):
        self.assertAlmostEqual(robot.ray_box((0, 0, .5), (1, 0, 0), robot.ROOM), 3.5)
        self.assertAlmostEqual(robot.ray_box((0, .6, .5), (1, 0, 0), robot.BOXES[0]), 1.7)
        self.assertIsNone(robot.ray_box((0, 2, .5), (1, 0, 0), robot.BOXES[0]))
        self.assertAlmostEqual(robot.ray_box((0, 0, .5), (0, 0, -1), robot.ROOM), .5)

    def test_pose_velocity_and_quaternions_agree(self):
        t, dt = 1.4, 1e-4
        messages = robot.telemetry(t, 12345.25)
        odom = messages['/utlidar/robot_odom']
        sport = messages['/sportmodestate']
        q = odom['pose']['pose']['orientation']
        self.assertAlmostEqual(sum(v*v for v in q.values()), 1.0)
        self.assertEqual(sport['imu_state']['quaternion'], [q[k] for k in ('w', 'x', 'y', 'z')])
        earlier, _, _ = robot.state(t-dt); later, _, _ = robot.state(t+dt)
        velocity = tuple((a-b)/(2*dt) for a, b in zip(later, earlier))
        body_v = odom['twist']['twist']['linear']
        world_v = robot.rotate(q, tuple(body_v[k] for k in ('x', 'y', 'z')))
        self.assertLess(math.dist(velocity, world_v), 1e-8)
        self.assertLess(math.dist(velocity, sport['velocity']), 1e-8)
        low = messages['/lowstate']
        self.assertEqual(len(low['motor_state']), 20)
        self.assertEqual(len(base64.b64decode(low['wireless_remote'])), 40)
        json.dumps(messages, allow_nan=False)


class TransportTests(unittest.IsolatedAsyncioTestCase):
    async def test_paused_motion_still_delivers_fresh_maps_and_telemetry(self):
        simulator = robot.Simulator(speed=0)
        async with websockets.serve(simulator.client, '127.0.0.1', 0) as server:
            producer = asyncio.create_task(simulator.run())
            try:
                port = server.sockets[0].getsockname()[1]
                async with websockets.connect(f'ws://127.0.0.1:{port}') as ws:
                    topics = ['/uslam/cloud_map', '/utlidar/voxel_map', '/utlidar/robot_odom']
                    for topic in topics:
                        await ws.send(json.dumps({'op': 'subscribe', 'topic': topic, 'throttle_rate': 100}))
                    received = {topic: [] for topic in topics}
                    deadline = time.monotonic()+3
                    while min(len(received[t]) for t in topics[:2]) < 2:
                        payload = json.loads(await asyncio.wait_for(ws.recv(), deadline-time.monotonic()))
                        received[payload['topic']].append(payload['msg'])
                    for topic in topics[:2]:
                        maps = received[topic]
                        self.assertGreater(maps[0]['width'], 0)
                        self.assertNotEqual(maps[0]['header']['stamp'], maps[1]['header']['stamp'])
                    odom = received[topics[2]]
                    self.assertGreater(len(odom), 5)
                    self.assertTrue(all(m['pose'] == odom[0]['pose'] for m in odom))
            finally:
                producer.cancel()
                await asyncio.gather(producer, return_exceptions=True)

    async def test_real_capture_collects_messages_without_starting_a_second_simulator(self):
        simulator = robot.Simulator()
        async with websockets.serve(simulator.client, '127.0.0.1', 0) as server:
            producer = asyncio.create_task(simulator.run())
            try:
                with tempfile.TemporaryDirectory() as folder:
                    output = robot.Path(folder)/'sample.json'
                    port = server.sockets[0].getsockname()[1]
                    report = await robot.capture_real_topics(f'ws://127.0.0.1:{port}', output, 0.4)
                    saved = json.loads(output.read_text())
                    self.assertEqual(saved, report)
                    by_topic = {p['topic']: p['msg'] for p in saved['messages']}
                    self.assertIn('/tf_static', by_topic)
                    self.assertIn('/utlidar/robot_odom', by_topic)
                    self.assertIn('/utlidar/cloud', by_topic)
                    self.assertEqual(by_topic['/utlidar/cloud']['point_step'], 32)
                    self.assertEqual(saved['bridge_status'], [])
            finally:
                producer.cancel()
                await asyncio.gather(producer, return_exceptions=True)

    async def test_subscriptions_latched_tf_throttle_and_unsubscribe(self):
        simulator = robot.Simulator()
        async with websockets.serve(simulator.client, '127.0.0.1', 0) as server:
            port = server.sockets[0].getsockname()[1]
            producer = asyncio.create_task(simulator.run())
            try:
                async with websockets.connect(f'ws://127.0.0.1:{port}') as ws:
                    # No messages are sent before subscribing.
                    with self.assertRaises(asyncio.TimeoutError):
                        await asyncio.wait_for(ws.recv(), 0.1)
                    await ws.send(json.dumps({'op': 'subscribe', 'topic': '/tf_static'}))
                    first = json.loads(await asyncio.wait_for(ws.recv(), 1))
                    self.assertEqual(first['topic'], '/tf_static')
                    self.assertIn('radar', [t['child_frame_id'] for t in first['msg']['transforms']])
                    await ws.send(json.dumps({'op': 'subscribe', 'topic': '/utlidar/robot_odom', 'throttle_rate': 200}))
                    stamps = []
                    for _ in range(4):
                        msg = json.loads(await asyncio.wait_for(ws.recv(), 2))
                        self.assertEqual(msg['topic'], '/utlidar/robot_odom')
                        stamp = msg['msg']['header']['stamp']
                        stamps.append(stamp['sec']+stamp['nanosec']/1e9)
                    self.assertTrue(all(b-a >= .19 for a, b in zip(stamps, stamps[1:])))
                    await ws.send(json.dumps({'op': 'unsubscribe', 'topic': '/utlidar/robot_odom'}))
                    # An unsupported-topic response acts as a processing barrier.
                    await ws.send(json.dumps({'op': 'subscribe', 'topic': '/not-a-topic'}))
                    while json.loads(await asyncio.wait_for(ws.recv(), 1))['op'] != 'status':
                        pass
                    with self.assertRaises(asyncio.TimeoutError):
                        await asyncio.wait_for(ws.recv(), .3)
            finally:
                producer.cancel()
                await asyncio.gather(producer, return_exceptions=True)


if __name__ == '__main__':
    unittest.main()
