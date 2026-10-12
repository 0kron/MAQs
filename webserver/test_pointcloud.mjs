import test from 'node:test';
import assert from 'node:assert/strict';
import { decodeCloud, intensityColors } from './pointcloud.mjs';

function fixture(fields, points, { stride = 32, width = points.length, padding = 0, big = false, array = false } = {}) {
  const rowStride = stride * width + padding, height = points.length / width;
  const bytes = new Uint8Array(rowStride * height), view = new DataView(bytes.buffer);
  const writers = { 2: 'setUint8', 3: 'setInt16', 4: 'setUint16', 7: 'setFloat32', 8: 'setFloat64' };
  points.forEach((point, i) => {
    const offset = Math.floor(i / width) * rowStride + (i % width) * stride;
    fields.forEach(field => view[writers[field.datatype]](offset + field.offset, point[field.name], !big));
  });
  return { fields, width, height, point_step: stride, row_step: rowStride, is_bigendian: big,
    data: array ? Array.from(bytes) : Buffer.from(bytes).toString('base64') };
}
const field = (name, offset, datatype = 7) => ({ name, offset, datatype, count: 1 });
const sdkFields = [field('x', 0), field('y', 4), field('z', 8), field('intensity', 16), field('ring', 20, 4), field('time', 24)];

test('Unitree SDK padded XYZ/intensity/ring/time profile preserves points and colors', () => {
  const decoded = decodeCloud(fixture(sdkFields, [
    { x: 1, y: 2, z: 3, intensity: 10, ring: 2, time: 0.03 },
    { x: 4, y: 5, z: 6, intensity: 250, ring: 3, time: 0.04 },
  ]));
  assert.deepEqual([...decoded.positions], [1, 2, 3, 4, 5, 6]);
  assert.deepEqual(decoded.range, [10, 250]);
  assert.deepEqual([...intensityColors(decoded.intensities, decoded.range)], [0, 0, 1, 1, 0, 0]);
});

test('packed legacy XYZI byte arrays work with intensity at offset 12', () => {
  const decoded = decodeCloud(fixture([field('x', 0), field('y', 4), field('z', 8), field('intensity', 12)],
    [{ x: -1, y: 0, z: 0.5, intensity: 80 }], { stride: 16, array: true }));
  assert.deepEqual([...decoded.positions], [-1, 0, 0.5]);
  assert.deepEqual(decoded.range, [80, 80]);
  assert.deepEqual([...intensityColors(decoded.intensities, decoded.range)], [0, 1, 0]);
});

test('big endian, reordered fields, uint16 intensity, float64 XYZ and row padding', () => {
  const fields = [field('z', 0, 8), field('y', 8, 3), field('x', 12), field('intensities', 16, 4)];
  const decoded = decodeCloud(fixture(fields, [
    { x: 1, y: -2, z: 3.5, intensities: 1000 },
    { x: 2, y: -3, z: 4.5, intensities: 2000 },
    { x: 3, y: -4, z: 5.5, intensities: 3000 },
    { x: 4, y: -5, z: 6.5, intensities: 4000 },
  ], { stride: 20, width: 2, padding: 12, big: true }));
  assert.deepEqual([...decoded.positions], [1, -2, 3.5, 2, -3, 4.5, 3, -4, 5.5, 4, -5, 6.5]);
  assert.deepEqual([...decoded.intensities], [1000, 2000, 3000, 4000]);
  assert.deepEqual(decoded.range, [1000, 4000]);
});

test('discarded invalid XYZ does not misalign intensity or affect the displayed range', () => {
  const decoded = decodeCloud(fixture(sdkFields, [
    { x: NaN, y: 0, z: 0, intensity: 9999, ring: 0, time: 0 },
    { x: 2, y: 3, z: 4, intensity: 100, ring: 0, time: 0 },
    { x: 5, y: 6, z: 7, intensity: NaN, ring: 0, time: 0 },
  ]));
  assert.deepEqual([...decoded.positions], [2, 3, 4, 5, 6, 7]);
  assert.equal(decoded.intensities[0], 100);
  assert.ok(Number.isNaN(decoded.intensities[1]));
  assert.deepEqual(decoded.range, [100, 100]);
  const colors = intensityColors(decoded.intensities, decoded.range);
  assert.deepEqual([...colors.slice(0, 3)], [0, 1, 0]);
  assert.equal(colors[3], colors[4]); assert.equal(colors[4], colors[5]);
});

test('clouds without intensity still display gray points', () => {
  const decoded = decodeCloud(fixture([field('x', 0), field('y', 4), field('z', 8)], [{ x: 1, y: 2, z: 3 }], { stride: 12 }));
  assert.deepEqual([...decoded.positions], [1, 2, 3]);
  assert.equal(decoded.range, null);
  const colors = intensityColors(decoded.intensities, decoded.range);
  assert.ok([...colors].every(value => Number.isFinite(value) && value === colors[0]));
});

test('truncated data and invalid XYZ offsets fail with a useful error', () => {
  const cloud = fixture(sdkFields, [{ x: 1, y: 2, z: 3, intensity: 0, ring: 0, time: 0 }]);
  assert.throws(() => decodeCloud({ ...cloud, data: [] }), /longitud de bytes/);
  assert.throws(() => decodeCloud({ ...cloud, fields: [field('x', 32), field('y', 4), field('z', 8)] }), /Campo x/);
});
