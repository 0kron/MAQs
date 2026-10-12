import test from 'node:test';
import assert from 'node:assert/strict';
import { VoxelMap } from './voxelmap.mjs';
const identity = [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1];

test('rotated and translated scans merge at the same world surface', () => {
  const map = new VoxelMap();
  const transform = [0,1,0,0, -1,0,0,0, 0,0,1,0, 2,0,0,1];
  map.add([1,0,0], [10], transform, 'odom');
  map.add([2,1,0], [20], identity, 'odom');
  assert.equal(map.voxels.size, 1);
  assert.deepEqual([...map.snapshot().positions], [2,1,0]);
  assert.deepEqual(map.snapshot().range, [20,20]);
});
test('negative voxel coordinates, finite points and bounded memory', () => {
  const map = new VoxelMap({ maxPoints: 2 });
  map.add([-.01,0,0, .01,0,0, NaN,0,0], null, identity, 'odom');
  assert.equal(map.voxels.size, 2);
  map.add([2,0,0], null, identity, 'odom');
  assert.equal(map.voxels.size, 2);
  assert.equal(map.snapshot().range, null);
  assert.ok([...map.snapshot().positions].every(Number.isFinite));
});
test('changing world frames clears old geometry and reset clears all points', () => {
  const map = new VoxelMap();
  map.add([1,0,0], [10], identity, 'odom');
  map.add([4,0,0], [30], identity, 'map');
  assert.deepEqual([...map.snapshot().positions], [4,0,0]);
  map.reset();
  assert.equal(map.snapshot().positions.length, 0);
  assert.throws(() => map.add([0,0,0], null, identity, ''), /marco/);
});
