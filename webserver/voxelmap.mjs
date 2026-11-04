// Accumulate clouds only after transforming them into one declared world frame.
export class VoxelMap {
  constructor({ voxelSize = 0.1, maxPoints = 80000 } = {}) {
    this.voxelSize = voxelSize;
    this.maxPoints = maxPoints;
    this.reset();
  }
  reset(frame = null) { this.frame = frame; this.voxels = new Map(); }
  add(positions, intensities, matrix, frame) {
    if (!frame) throw new Error('Falta el marco de referencia del mapa');
    if (this.frame !== frame) this.reset(frame);
    for (let n = 0; n < positions.length / 3; n++) {
      const i = n * 3, x = positions[i], y = positions[i + 1], z = positions[i + 2];
      const p = [matrix[0]*x + matrix[4]*y + matrix[8]*z + matrix[12],
        matrix[1]*x + matrix[5]*y + matrix[9]*z + matrix[13],
        matrix[2]*x + matrix[6]*y + matrix[10]*z + matrix[14]];
      if (!p.every(Number.isFinite)) continue;
      const key = p.map(v => Math.floor(v / this.voxelSize)).join(',');
      this.voxels.delete(key);
      this.voxels.set(key, [...p, intensities?.[n] ?? NaN]);
      if (this.voxels.size > this.maxPoints) this.voxels.delete(this.voxels.keys().next().value);
    }
  }
  snapshot() {
    const positions = new Float32Array(this.voxels.size * 3);
    const intensities = new Float32Array(this.voxels.size);
    let n = 0, low = Infinity, high = -Infinity;
    for (const point of this.voxels.values()) {
      positions.set(point.slice(0, 3), n * 3);
      const intensity = intensities[n++] = point[3];
      if (Number.isFinite(intensity)) { low = Math.min(low, intensity); high = Math.max(high, intensity); }
    }
    return { positions, intensities, range: Number.isFinite(low) ? [low, high] : null };
  }
}
