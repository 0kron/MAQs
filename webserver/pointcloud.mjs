// PointCloud2 describes its own binary layout. Never assume XYZ/intensity offsets.
const readers = {
  1: ['getInt8', 1], 2: ['getUint8', 1], 3: ['getInt16', 2],
  4: ['getUint16', 2], 5: ['getInt32', 4], 6: ['getUint32', 4],
  7: ['getFloat32', 4], 8: ['getFloat64', 8],
};

export function decodeCloud(message) {
  let bytes;
  if (typeof message.data === 'string') bytes = Uint8Array.from(atob(message.data), c => c.charCodeAt(0));
  else if (Array.isArray(message.data)) bytes = Uint8Array.from(message.data);
  else throw new Error('PointCloud2.data debe ser base64 o un arreglo de bytes');
  const { width, height, point_step: stride, row_step: rowStride } = message;
  if (![width, height, stride, rowStride].every(Number.isSafeInteger) || width < 0 || height < 0 || stride <= 0 || rowStride < width * stride || bytes.length < rowStride * height) {
    throw new Error('Dimensiones o longitud de bytes inválidas en PointCloud2');
  }
  if (!Array.isArray(message.fields)) throw new Error('Faltan los campos de PointCloud2');
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const little = !message.is_bigendian;
  function fieldReader(name, required = false) {
    const field = message.fields.find(f => f.name === name);
    const reader = readers[field?.datatype];
    if (!reader || !Number.isInteger(field.count) || field.count < 1 || !Number.isInteger(field.offset) || field.offset < 0 || field.offset + reader[1] > stride) {
      if (required) throw new Error(`Campo ${name} inválido o ausente`);
      return null;
    }
    return offset => view[reader[0]](offset + field.offset, little);
  }
  const axes = ['x', 'y', 'z'].map(name => fieldReader(name, true));
  const intensityReader = fieldReader('intensity') || fieldReader('intensities');
  const step = Math.max(1, Math.ceil(width * height / 100000));
  const positions = [], intensities = [];
  let minimum = Infinity, maximum = -Infinity;
  for (let i = 0; i < width * height; i += step) {
    const offset = Math.floor(i / width) * rowStride + (i % width) * stride;
    const point = axes.map(read => read(offset));
    if (!point.every(Number.isFinite)) continue;
    positions.push(...point);
    const value = intensityReader ? intensityReader(offset) : NaN;
    intensities.push(value);
    if (Number.isFinite(value)) { minimum = Math.min(minimum, value); maximum = Math.max(maximum, value); }
  }
  return {
    positions: new Float32Array(positions), intensities: new Float64Array(intensities),
    range: minimum === Infinity ? null : [minimum, maximum],
  };
}

// Match the scale shown in the page: blue -> cyan -> green -> yellow -> red.
// Range is measured from the points displayed in this message, not assumed 0–255.
export function intensityColors(intensities, range) {
  const stops = [[0, 0, 1], [0, 1, 1], [0, 1, 0], [1, 1, 0], [1, 0, 0]];
  const colors = new Float32Array(intensities.length * 3);
  for (let i = 0; i < intensities.length; i++) {
    if (!range || !Number.isFinite(intensities[i])) {
      colors.set([0.091, 0.091, 0.091], i * 3); // Neutral gray for absent/invalid intensity.
      continue;
    }
    const u = range[1] === range[0] ? 0.5 : Math.max(0, Math.min(1, (intensities[i] - range[0]) / (range[1] - range[0])));
    const segment = Math.min(3, Math.floor(u * 4)), fraction = u * 4 - segment;
    for (let channel = 0; channel < 3; channel++) {
      const srgb = stops[segment][channel] * (1 - fraction) + stops[segment + 1][channel] * fraction;
      colors[i * 3 + channel] = srgb <= 0.04045 ? srgb / 12.92 : ((srgb + 0.055) / 1.055) ** 2.4;
    }
  }
  return colors;
}
