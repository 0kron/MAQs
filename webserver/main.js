import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import URDFLoader from './vendor/urdf/URDFLoader.js';
import { decodeCloud, intensityColors } from './pointcloud.mjs';
import { VoxelMap } from './voxelmap.mjs';

const $ = id => document.getElementById(id);
THREE.Object3D.DEFAULT_UP.set(0, 0, 1);
const scene = new THREE.Scene();
scene.background = new THREE.Color(0xffffff);
const camera = new THREE.PerspectiveCamera(50, 1, 0.02, 100);
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
$('viewport').appendChild(renderer.domElement);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.minDistance = 0.6;
controls.maxDistance = 30;
scene.add(new THREE.HemisphereLight(0xffffff, 0x888888, 2));
const light = new THREE.DirectionalLight(0xffffff, 2);
light.position.set(3, -2, 5);
scene.add(light);
const grid = new THREE.GridHelper(10, 20, 0xbbbbbb, 0xeeeeee);
grid.rotation.x = Math.PI / 2;
grid.position.z = -0.025;
scene.add(grid);
const cloudGeometry = new THREE.BufferGeometry();
const cloud = new THREE.Points(cloudGeometry, new THREE.PointsMaterial({
  color: 0xffffff, vertexColors: true, size: 0.022, sizeAttenuation: true,
}));
scene.add(cloud);

let robot, modelFailed = false, robotAligned = false, socket, generation = 0;
let reconnectTimer, connectionTimer, reconnectAttempt = 0;
let pose = null, motors = null, pendingCloud = null;
let lastCloudMessage = null, cloudSample = null, fitSensorView = false;
let cloudSubscribedAt = 0;
let latestOdom = null, latestCloudHeader = null, lastOdomAt = 0;
let modelTarget = null, floorFrame = null, lastFullJointAt = 0;
let cameraBodyPosition = null;
const jointTargets = new Map();
const localMap = new VoxelMap();
let lastMapInputAt = 0, lastMapUpdateAt = 0;
const localMapSelected = () => $('cloud-topic').value === 'viewer/map';
const cloudSource = (topic = $('cloud-topic').value) => topic === 'viewer/map' ? '/utlidar/cloud_base' : topic;
const cloudLabel = () => localMapSelected() ? 'Mapa local' : isMapTopic($('cloud-topic').value) ? 'Mapa' : 'LiDAR';
function updateAlignmentData() {
  $('alignment-data').textContent = JSON.stringify({ nube: latestCloudHeader,
    odometria: latestOdom && { header: latestOdom.header, child_frame_id: latestOdom.child_frame_id, pose: latestOdom.pose.pose } }, null, 2);
}
const isMapTopic = topic => ['viewer/map', '/uslam/cloud_map', '/utlidar/voxel_map'].includes(topic);
const sensorView = () => $('cloud-view').value === 'sensor';
let lastPoseAt = 0, lastJointAt = 0, lastCloudAt = 0, lastImuAt = 0;
const transforms = new Map();
const history = new Map();
const frameName = name => (name || '').replace(/^\/+/, '');
const stampTime = stamp => (stamp?.sec ?? stamp?.secs ?? 0) + (stamp?.nanosec ?? stamp?.nsecs ?? 0) / 1e9;
const vector = v => new THREE.Vector3(v.x, v.y, v.z);
const quaternion = q => new THREE.Quaternion(q.x, q.y, q.z, q.w).normalize();
const poseMatrix = p => new THREE.Matrix4().compose(vector(p.position), quaternion(p.orientation), new THREE.Vector3(1, 1, 1));
const tfMatrix = t => new THREE.Matrix4().compose(vector(t.translation), quaternion(t.rotation), new THREE.Vector3(1, 1, 1));
const typeByTopic = {
  '/tf': 'tf2_msgs/msg/TFMessage', '/tf_static': 'tf2_msgs/msg/TFMessage',
  '/utlidar/robot_odom': 'nav_msgs/msg/Odometry',
  '/utlidar/robot_pose': 'geometry_msgs/msg/PoseStamped',
  '/sportmodestate': 'unitree_go/msg/SportModeState',
  '/lf/sportmodestate': 'unitree_go/msg/SportModeState',
  '/lowstate': 'unitree_go/msg/LowState', '/lf/lowstate': 'unitree_go/msg/LowState',
  '/utlidar/imu': 'sensor_msgs/msg/Imu',
};
const baseTopics = Object.keys(typeByTopic);
const motorOrder = ['FR', 'FL', 'RR', 'RL']; // Unitree motor indices 0–11.

function resetView() {
  cameraBodyPosition = null;
  if (sensorView() || (isMapTopic($('cloud-topic').value) && !robotAligned)) {
    const sphere = cloudGeometry.boundingSphere;
    controls.target.copy(sphere?.center ?? new THREE.Vector3());
    if (!sensorView()) controls.target.applyMatrix4(cloud.matrix);
    const distance = Math.max(2, (sphere?.radius ?? 1) * 2.8);
    controls.maxDistance = Math.max(30, distance * 3);
    camera.far = Math.max(100, distance * 6);
    camera.updateProjectionMatrix();
    camera.position.copy(controls.target).add(new THREE.Vector3(1, -1, 0.7).normalize().multiplyScalar(distance));
    controls.update();
    return;
  }
  controls.maxDistance = 30;
  const p = robot?.position ?? new THREE.Vector3(0, 0, 0.3);
  controls.target.set(p.x, p.y, p.z + 0.03);
  camera.position.set(p.x + 1.8, p.y - 2.2, p.z + 1.3);
  if (robotAligned) cameraBodyPosition = p.clone();
  controls.update();
}
$('reset-view').onclick = resetView;
resetView();
const manager = new THREE.LoadingManager();
manager.onError = url => { modelFailed = true; $('model-status').textContent = 'URDF: error al cargar un archivo'; $('error').textContent = `No se pudo cargar ${url}`; };
manager.onLoad = () => {
  if (!robot || modelFailed) return;
  $('model-status').textContent = 'URDF: cargado';
  applyRobotState();
};
const loader = new URDFLoader(manager);
loader.packages = { go2_description: new URL('./assets/go2_description/', import.meta.url).href };
loader.load('./assets/go2_description/urdf/go2_description.urdf', model => {
  robot = model;
  robot.visible = false;
  robot.position.z = 0.32;
  for (const leg of motorOrder) {
    robot.setJointValue(`${leg}_thigh_joint`, 0.8);
    robot.setJointValue(`${leg}_calf_joint`, -1.6);
  }
  scene.add(robot);
  applyRobotState();
}, undefined, error => { modelFailed = true; $('model-status').textContent = 'URDF: error'; $('error').textContent = String(error); });

function updateRobotVisibility() {
  if (robot) robot.visible = !sensorView() && robotAligned;
  grid.visible = !sensorView() && robotAligned;
}

function updateFloorReference() {
  if (!robotAligned || !motors || floorFrame === pose.frame) return;
  robot.updateMatrixWorld(true);
  const heights = motorOrder.map(leg => robot.links[`${leg}_foot`]?.localToWorld(new THREE.Vector3(-0.002, 0, 0)).z).filter(Number.isFinite);
  if (heights.length) {
    // Set the reference once in world coordinates. Following each foot or the
    // body would move the floor with the gait and hide the robot's translation.
    grid.position.set(Math.round(robot.position.x * 2) / 2, Math.round(robot.position.y * 2) / 2, Math.min(...heights) - 0.022 - 0.003);
    floorFrame = pose.frame;
  }
}
$('reset-floor').onclick = () => { floorFrame = null; updateFloorReference(); };

function applyRobotState() {
  if (!robot) return;
  robotAligned = false;
  if (pose && performance.now() - lastPoseAt <= 1500) {
    // Odometry describes its child frame, which need not be the URDF's base.
    let offset = relativeTransform('base', pose.child, pose.stamp);
    // Explicit visual anchoring of the URDF body, not a ROS TF alias or sensor calibration.
    const bodyReference = !offset && $('model-anchor').value === 'odom' && pose.source === '/utlidar/robot_odom';
    if (bodyReference) offset = new THREE.Matrix4();
    if (offset) {
      const firstPose = !modelTarget || modelTarget.frame !== pose.frame;
      modelTarget = { position: new THREE.Vector3(), quaternion: new THREE.Quaternion(), scale: new THREE.Vector3(), frame: pose.frame };
      poseMatrix(pose).multiply(offset).decompose(modelTarget.position, modelTarget.quaternion, modelTarget.scale);
      if (firstPose) {
        robot.position.copy(modelTarget.position); robot.quaternion.copy(modelTarget.quaternion); robot.scale.copy(modelTarget.scale);
        floorFrame = null;
      }
      robotAligned = true;
      if (firstPose && !sensorView()) resetView();
      const p = modelTarget.position;
      const yaw = new THREE.Euler().setFromQuaternion(modelTarget.quaternion, 'ZYX').z;
      $('pose-status').textContent = `Posición: ${p.x.toFixed(2)}, ${p.y.toFixed(2)}, ${p.z.toFixed(2)} m · giro Z ${(yaw * 180 / Math.PI).toFixed(0)}°${bodyReference ? ` · cuerpo ${pose.child} (referencia visual)` : ''}`;
    } else $('pose-status').textContent = `Posición: esperando TF base → ${pose.child}; modelo oculto hasta alinear`;
  }
  if (motors) {
    for (let leg = 0; leg < 4; leg++) for (let joint = 0; joint < 3; joint++) {
      const q = motors[leg * 3 + joint]?.q;
      const name = `${motorOrder[leg]}_${['hip', 'thigh', 'calf'][joint]}_joint`;
      if (Number.isFinite(q)) {
        const limits = robot.joints[name].limit;
        const angle = THREE.MathUtils.clamp(q, limits.lower, limits.upper);
        if (!jointTargets.has(name)) robot.setJointValue(name, angle);
        jointTargets.set(name, angle);
      }
    }
  }
  updateRobotVisibility();
  updateFloorReference();
}
function animateRobot(dt) {
  if (!robot || !robotAligned || !modelTarget) return;
  // Smooth only the drawing. Cloud registration always uses original odometry.
  const alpha = -Math.expm1(-dt / 0.055);
  robot.position.lerp(modelTarget.position, alpha);
  robot.quaternion.slerp(modelTarget.quaternion, alpha);
  robot.scale.copy(modelTarget.scale);
  for (const [name, target] of jointTargets) {
    const current = robot.joints[name].jointValue[0];
    robot.setJointValue(name, current + (target - current) * alpha);
  }
}
function followRobotCamera() {
  if (sensorView() || !robotAligned || !robot) { cameraBodyPosition = null; return; }
  if (cameraBodyPosition) {
    // Translate the orbit with the body, preserving the user's angle, zoom and pan.
    const delta = robot.position.clone().sub(cameraBodyPosition);
    camera.position.add(delta); controls.target.add(delta);
  }
  cameraBodyPosition = robot.position.clone();
}
$('model-anchor').onchange = () => { applyRobotState(); resetView(); };

function storeTransform(parent, child, matrix, stamp, fixed = false) {
  parent = frameName(parent); child = frameName(child);
  if (!parent || !child || parent === child) return;
  const entry = { parent, matrix, stamp, fixed };
  transforms.set(child, entry);
  if (!fixed) {
    const samples = history.get(child) || [];
    samples.push(entry);
    if (samples.length > 150) samples.shift();
    history.set(child, samples);
  }
}

// Resolve both directions of TF chains at the cloud timestamp. Dynamic edges
// use the nearest sample; our simulator emits an exact pose for every scan.
function relativeTransform(source, target, stamp) {
  source = frameName(source); target = frameName(target);
  if (source === target) return new THREE.Matrix4();
  const edges = [];
  for (const [child, latest] of transforms) {
    let edge = latest;
    if (!latest.fixed && stamp) {
      const samples = history.get(child) || [];
      edge = samples.reduce((best, sample) => Math.abs(sample.stamp - stamp) < Math.abs(best.stamp - stamp) ? sample : best, latest);
      if (Math.abs(edge.stamp - stamp) > 0.25) continue;
    }
    edges.push({ from: child, to: edge.parent, matrix: edge.matrix });
    edges.push({ from: edge.parent, to: child, matrix: edge.matrix.clone().invert() });
  }
  const queue = [{ frame: source, matrix: new THREE.Matrix4() }], visited = new Set([source]);
  for (let index = 0; index < queue.length; index++) {
    const item = queue[index];
    for (const edge of edges) {
      if (edge.from !== item.frame || visited.has(edge.to)) continue;
      const matrix = edge.matrix.clone().multiply(item.matrix);
      if (edge.to === target) return matrix;
      visited.add(edge.to); queue.push({ frame: edge.to, matrix });
    }
  }
  return null;
}

function setCloudSample(message, decoded) {
  cloudSample = { message, ...decoded };
  cloudGeometry.setAttribute('position', new THREE.BufferAttribute(decoded.positions, 3));
  cloudGeometry.setAttribute('color', new THREE.BufferAttribute(intensityColors(decoded.intensities, decoded.range), 3));
  cloudGeometry.computeBoundingSphere();
}

function showCloud() {
  if (!pendingCloud) return;
  const message = pendingCloud;
  try {
    if (cloudSample?.message !== message) {
      setCloudSample(message, decodeCloud(message));
    }
    const { positions, range } = cloudSample;
    $('intensity-status').textContent = range ? 'Intensidad (escala automática):' : 'Intensidad: no disponible (gris)';
    $('intensity-scale').hidden = !range;
    if (range) {
      $('intensity-min').textContent = range[0].toFixed(1);
      $('intensity-max').textContent = range[1].toFixed(1);
    }
    // Sensor view preserves raw coordinates. It never overlays an uncalibrated robot.
    const target = localMapSelected() ? localMap.frame : pose?.frame || 'odom';
    const matrix = sensorView() ? new THREE.Matrix4()
      : relativeTransform(message.header.frame_id, target, stampTime(message.header.stamp));
    if (!matrix) {
      cloud.visible = false;
      $('cloud-status').textContent = `${cloudLabel()}: ${positions.length / 3} puntos recibidos; esperando TF ${message.header.frame_id} → ${target}. Usa Solo LiDAR.`;
      return;
    }
    cloud.matrixAutoUpdate = false; cloud.matrix.copy(matrix); cloud.matrixWorldNeedsUpdate = true;
    cloud.visible = positions.length > 0;
    $('cloud-status').textContent = `${cloudLabel()}: ${positions.length / 3} puntos (${message.header.frame_id})${sensorView() ? ' · coordenadas del mensaje' : ''}`;
    if (fitSensorView && positions.length) { resetView(); fitSensorView = false; }
    pendingCloud = null;
  } catch (error) { cloud.visible = false; $('error').textContent = error.message; pendingCloud = null; }
}

function changeView() {
  grid.visible = !sensorView();
  cloud.material.size = sensorView() ? 0.05 : isMapTopic($('cloud-topic').value) ? 0.035 : 0.022;
  updateRobotVisibility();
  $('view-note').hidden = !sensorView();
  $('pose-status').hidden = sensorView();
  fitSensorView = sensorView() || isMapTopic($('cloud-topic').value);
  cloud.visible = false;
  if (lastCloudMessage && (isMapTopic($('cloud-topic').value) || performance.now() - lastCloudAt <= 1500)) {
    pendingCloud = lastCloudMessage;
    showCloud();
  }
  resetView();
}
$('cloud-view').onchange = changeView;

function accumulateCloud(msg, now) {
  lastMapInputAt = now;
  if (!latestOdom || now - lastOdomAt > 1500) {
    $('cloud-status').textContent = 'Mapa local: esperando odometría reciente para acumular';
    return;
  }
  const target = frameName(latestOdom.header.frame_id);
  const matrix = relativeTransform(msg.header.frame_id, target, stampTime(msg.header.stamp));
  if (!matrix) {
    $('cloud-status').textContent = `Mapa local: esperando transformación ${msg.header.frame_id} → ${target}`;
    return;
  }
  if (now - lastMapUpdateAt < 200) return;
  const decoded = decodeCloud(msg);
  localMap.add(decoded.positions, decoded.intensities, matrix.elements, target);
  const message = { header: { frame_id: target, stamp: msg.header.stamp } };
  setCloudSample(message, localMap.snapshot());
  pendingCloud = lastCloudMessage = message;
  lastCloudAt = lastMapUpdateAt = now;
  showCloud();
}

$('clear-map').onclick = () => {
  localMap.reset(); lastMapUpdateAt = lastCloudAt = 0;
  pendingCloud = lastCloudMessage = cloudSample = null; cloud.visible = false;
  $('cloud-status').textContent = 'Mapa local: vacío; esperando el siguiente escaneo';
};

function receive(topic, msg) {
  const now = performance.now();
  if (topic === '/tf' || topic === '/tf_static') {
    for (const t of msg.transforms) storeTransform(t.header.frame_id, t.child_frame_id, tfMatrix(t.transform), stampTime(t.header.stamp), topic === '/tf_static');
    applyRobotState(); showCloud();
  } else if (topic === '/utlidar/robot_odom' || topic === '/utlidar/robot_pose') {
    if (topic === '/utlidar/robot_odom') { latestOdom = msg; lastOdomAt = now; updateAlignmentData(); }
    // Prefer odometry. PoseStamped provides a fallback if odometry is absent.
    if (topic.endsWith('robot_pose') && pose?.source === '/utlidar/robot_odom' && now - lastPoseAt < 1000) return;
    const p = topic.endsWith('robot_odom') ? msg.pose.pose : msg.pose;
    const child = frameName(topic.endsWith('robot_odom') ? msg.child_frame_id : latestOdom?.child_frame_id);
    pose = { ...p, frame: frameName(msg.header.frame_id), child, stamp: stampTime(msg.header.stamp), source: topic };
    storeTransform(pose.frame, child, poseMatrix(p), stampTime(msg.header.stamp));
    lastPoseAt = now; applyRobotState(); showCloud();
  } else if (topic.endsWith('lowstate')) {
    // Do not alternate the same joints between full-rate and delayed LF streams.
    if (topic === '/lf/lowstate' && now - lastFullJointAt < 1000) return;
    if (topic === '/lowstate') lastFullJointAt = now;
    motors = msg.motor_state; lastJointAt = now; applyRobotState();
    $('joint-status').textContent = 'Articulaciones: en vivo';
  } else if (topic.endsWith('sportmodestate')) {
    // Sport state has no declared world/body frames: don't mix it into a cloud map.
    if (!pose || now - lastPoseAt > 1500) $('pose-status').textContent = 'Posición: esperando odometría con marcos de referencia';
  } else if (topic === '/utlidar/imu') {
    lastImuAt = now;
    $('imu-status').textContent = `IMU: giro Z ${msg.angular_velocity.z.toFixed(2)} rad/s`;
  } else if (topic === cloudSource()) {
    latestCloudHeader = msg.header; updateAlignmentData();
    if (localMapSelected()) { accumulateCloud(msg, now); return; }
    pendingCloud = lastCloudMessage = msg; lastCloudAt = now; showCloud();
  }
}

function subscribe(topic) {
  if (topic === cloudSource()) cloudSubscribedAt = performance.now();
  socket.send(JSON.stringify({ op: 'subscribe', topic, type: typeByTopic[topic] || 'sensor_msgs/msg/PointCloud2', queue_length: 1, throttle_rate: topic.includes('cloud') || topic.includes('voxel') ? 100 : 0, compression: 'none' }));
}
let selectedCloud = $('cloud-topic').value;
function closeCurrentSocket() {
  clearTimeout(connectionTimer);
  const previous = socket;
  if (previous?.readyState === WebSocket.OPEN) {
    for (const topic of [...baseTopics, cloudSource(selectedCloud)]) {
      previous.send(JSON.stringify({ op: 'unsubscribe', topic }));
    }
  }
  previous?.close();
}
$('disconnect').onclick = () => {
  clearTimeout(reconnectTimer); ++generation; closeCurrentSocket();
  $('connection').textContent = 'Desconectado';
  robotAligned = false; updateRobotVisibility(); cloud.visible = false;
};
window.addEventListener('beforeunload', () => { ++generation; closeCurrentSocket(); });
$('cloud-topic').onchange = () => {
  if (socket?.readyState === WebSocket.OPEN) {
    socket.send(JSON.stringify({ op: 'unsubscribe', topic: cloudSource(selectedCloud) }));
    subscribe(cloudSource());
  }
  selectedCloud = $('cloud-topic').value; pendingCloud = lastCloudMessage = cloudSample = null; cloud.visible = false;
  $('clear-map').disabled = !localMapSelected();
  lastMapUpdateAt = lastMapInputAt = 0;
  cloud.material.size = sensorView() ? 0.05 : isMapTopic(selectedCloud) ? 0.035 : 0.022;
  fitSensorView = sensorView() || isMapTopic(selectedCloud);
  $('cloud-status').textContent = 'LiDAR: esperando datos'; lastCloudAt = 0;
  $('intensity-status').textContent = 'Intensidad: esperando datos'; $('intensity-scale').hidden = true;
};

function connect(endpoint = $('endpoint').value) {
  clearTimeout(reconnectTimer);
  const id = ++generation;
  closeCurrentSocket();
  transforms.clear(); history.clear(); pose = motors = pendingCloud = null;
  latestOdom = latestCloudHeader = null; updateAlignmentData();
  modelTarget = null; floorFrame = null; lastFullJointAt = -Infinity; jointTargets.clear();
  cameraBodyPosition = null;
  localMap.reset(); lastOdomAt = lastMapInputAt = lastMapUpdateAt = 0;
  robotAligned = false; updateRobotVisibility();
  lastCloudMessage = cloudSample = null; fitSensorView = sensorView() || isMapTopic($('cloud-topic').value);
  cloudSubscribedAt = 0;
  cloud.visible = false;
  lastPoseAt = lastJointAt = lastCloudAt = lastImuAt = 0;
  for (const [key, label] of [['pose-status', 'Posición'], ['joint-status', 'Articulaciones'], ['cloud-status', 'LiDAR'], ['imu-status', 'IMU']]) $(key).textContent = `${label}: esperando datos`;
  $('error').textContent = ''; $('connection').textContent = 'Conectando…';
  $('intensity-status').textContent = 'Intensidad: esperando datos'; $('intensity-scale').hidden = true;
  try { socket = new WebSocket(endpoint); }
  catch (error) { $('connection').textContent = 'Error de conexión'; $('error').textContent = error.message; return; }
  connectionTimer = setTimeout(() => {
    if (id !== generation || socket.readyState !== WebSocket.CONNECTING) return;
    $('error').textContent = 'rosbridge no completó la conexión en 15 s. Comprueba su terminal y la dirección WebSocket.';
    socket.close();
  }, 15000);
  socket.onopen = () => {
    if (id !== generation) return;
    clearTimeout(connectionTimer);
    reconnectAttempt = 0;
    $('connection').textContent = 'Conectado';
    for (const topic of [...baseTopics, cloudSource()]) subscribe(topic);
  };
  socket.onmessage = event => {
    if (id !== generation) return;
    try {
      const payload = JSON.parse(event.data);
      if (payload.op === 'publish') receive(payload.topic, payload.msg);
      else if (payload.op === 'status' && payload.level === 'error') $('error').textContent = `rosbridge: ${payload.msg}`;
    }
    catch (error) { $('error').textContent = `Error de mensaje: ${error.message}`; }
  };
  socket.onerror = () => { if (id === generation) $('error').textContent = 'No se pudo conectar al WebSocket. Comprueba que robot.py o rosbridge esté en ejecución.'; };
  socket.onclose = () => {
    if (id !== generation) return;
    clearTimeout(connectionTimer);
    $('connection').textContent = 'Desconectado · reintentando…';
    robotAligned = false; updateRobotVisibility();
    cloud.visible = false; $('intensity-scale').hidden = true; $('intensity-status').textContent = 'Intensidad: esperando datos';
    reconnectTimer = setTimeout(() => connect(endpoint), Math.min(8000, 1000 * 2 ** reconnectAttempt++));
  };
}
$('connection-form').onsubmit = event => { event.preventDefault(); connect(); };
const params = new URLSearchParams(location.search);
if (params.get('view') === 'sensor') $('cloud-view').value = 'sensor';
if ([...$('cloud-topic').options].some(option => option.value === params.get('topic'))) {
  $('cloud-topic').value = selectedCloud = params.get('topic');
  $('clear-map').disabled = !localMapSelected();
}
changeView();
$('endpoint').value = params.get('ws') || `${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.hostname || 'localhost'}:${params.get('port') || '9090'}`;
connect();

new ResizeObserver(() => {
  const width = $('viewport').clientWidth, height = $('viewport').clientHeight;
  camera.aspect = width / height; camera.updateProjectionMatrix(); renderer.setSize(width, height);
}).observe($('viewport'));
setInterval(() => {
  const now = performance.now();
  if (lastPoseAt && now - lastPoseAt > 1500) { robotAligned = false; updateRobotVisibility(); }
  const mapSelected = isMapTopic($('cloud-topic').value);
  for (const [id, stamp, label] of [['pose-status', lastPoseAt, 'Posición'], ['joint-status', lastJointAt, 'Articulaciones'], ['cloud-status', lastCloudAt, 'LiDAR'], ['imu-status', lastImuAt, 'IMU']]) {
    if (id === 'cloud-status' && mapSelected) continue;
    if (stamp && now - stamp > 1500) $(id).textContent = `${label}: sin datos recientes`;
  }
  if (mapSelected && lastCloudAt && now - lastCloudAt > 1500 && cloud.visible) {
    $('cloud-status').textContent = `${cloudLabel()}: ${cloudSample.positions.length / 3} puntos conservados · ${Math.floor((now - lastCloudAt) / 1000)} s desde la última actualización`;
  }
  if (mapSelected && !lastCloudAt && cloudSubscribedAt && now - cloudSubscribedAt > 5000 && socket?.readyState === WebSocket.OPEN && (!localMapSelected() || now - lastMapInputAt > 1500)) {
    $('cloud-status').textContent = `${cloudLabel()}: no llegan mensajes de ${cloudSource()}. Comprueba la suscripción y la publicación del mapa.`;
  }
  if (!mapSelected && lastCloudAt && now - lastCloudAt > 1500) { cloud.visible = false; $('intensity-scale').hidden = true; $('intensity-status').textContent = 'Intensidad: sin datos recientes'; }
}, 500);
let lastRenderAt = performance.now();
renderer.setAnimationLoop(() => {
  const now = performance.now();
  animateRobot(Math.min(0.1, (now - lastRenderAt) / 1000)); lastRenderAt = now;
  followRobotCamera();
  controls.update(); renderer.render(scene, camera);
});
