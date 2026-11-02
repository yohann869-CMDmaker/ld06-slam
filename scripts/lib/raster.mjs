// Rendu d'images PNG sans dépendance (figures du compte rendu, sortie de la CLI).
import { deflateSync } from 'node:zlib';
import { occupancyRGB } from '../../src/core/gridmap.js';

const CRC_TABLE = new Uint32Array(256).map((_, n) => {
  let c = n;
  for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1;
  return c >>> 0;
});

function crc32(buf) {
  let c = 0xffffffff;
  for (const b of buf) c = CRC_TABLE[(c ^ b) & 0xff] ^ (c >>> 8);
  return (c ^ 0xffffffff) >>> 0;
}

function chunk(type, data) {
  const len = Buffer.alloc(4);
  len.writeUInt32BE(data.length);
  const td = Buffer.concat([Buffer.from(type, 'ascii'), data]);
  const crc = Buffer.alloc(4);
  crc.writeUInt32BE(crc32(td));
  return Buffer.concat([len, td, crc]);
}

/** Encode une image RGB (Uint8Array w*h*3) en PNG. */
export function encodePNG(width, height, rgb) {
  const raw = Buffer.alloc((width * 3 + 1) * height);
  for (let y = 0; y < height; y++) {
    raw[y * (width * 3 + 1)] = 0; // filtre « None »
    raw.set(rgb.subarray(y * width * 3, (y + 1) * width * 3), y * (width * 3 + 1) + 1);
  }
  const ihdr = Buffer.alloc(13);
  ihdr.writeUInt32BE(width, 0);
  ihdr.writeUInt32BE(height, 4);
  ihdr[8] = 8; // bits par canal
  ihdr[9] = 2; // RGB
  return Buffer.concat([
    Buffer.from([0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a]),
    chunk('IHDR', ihdr),
    chunk('IDAT', deflateSync(raw)),
    chunk('IEND', Buffer.alloc(0)),
  ]);
}

export const hex = (h) => [parseInt(h.slice(1, 3), 16), parseInt(h.slice(3, 5), 16), parseInt(h.slice(5, 7), 16)];

/**
 * Image en coordonnées monde : `bounds` (m) et `scale` (pixels par mètre),
 * axe Y vers le haut comme dans les cartes.
 */
export class Raster {
  constructor(bounds, scale, background = '#fbfbf9') {
    this.b = bounds;
    this.scale = scale;
    this.width = Math.ceil((bounds.maxX - bounds.minX) * scale);
    this.height = Math.ceil((bounds.maxY - bounds.minY) * scale);
    this.rgb = new Uint8Array(this.width * this.height * 3);
    const [r, g, b] = hex(background);
    for (let k = 0; k < this.width * this.height; k++) this.rgb.set([r, g, b], k * 3);
  }

  X(x) {
    return (x - this.b.minX) * this.scale;
  }

  Y(y) {
    return (this.b.maxY - y) * this.scale;
  }

  /** Mélange un pixel (coordonnées image) avec une couleur et une opacité. */
  blend(px, py, color, alpha = 1) {
    const x = Math.floor(px);
    const y = Math.floor(py);
    if (x < 0 || y < 0 || x >= this.width || y >= this.height) return;
    const k = (y * this.width + x) * 3;
    for (let c = 0; c < 3; c++) this.rgb[k + c] = Math.round(this.rgb[k + c] * (1 - alpha) + color[c] * alpha);
  }

  /** Disque plein en coordonnées image, bords adoucis. */
  discPx(cx, cy, r, color, alpha = 1) {
    for (let y = Math.floor(cy - r - 1); y <= cy + r + 1; y++) {
      for (let x = Math.floor(cx - r - 1); x <= cx + r + 1; x++) {
        const d = Math.hypot(x + 0.5 - cx, y + 0.5 - cy);
        const cover = Math.max(0, Math.min(1, r + 0.5 - d));
        if (cover > 0) this.blend(x, y, color, alpha * cover);
      }
    }
  }

  dot(x, y, r, color, alpha = 1) {
    this.discPx(this.X(x), this.Y(y), r, color, alpha);
  }

  /** Segment monde, épaisseur en pixels, pointillés optionnels [plein, vide] en pixels. */
  line(x1, y1, x2, y2, color, width = 1, dash = null, alpha = 1) {
    const ax = this.X(x1);
    const ay = this.Y(y1);
    const bx = this.X(x2);
    const by = this.Y(y2);
    const len = Math.hypot(bx - ax, by - ay);
    const steps = Math.max(1, Math.ceil(len * 2));
    const seen = new Set();
    for (let s = 0; s <= steps; s++) {
      const d = (s / steps) * len;
      if (dash && d % (dash[0] + dash[1]) > dash[0]) continue;
      const cx = ax + ((bx - ax) * s) / steps;
      const cy = ay + ((by - ay) * s) / steps;
      const r = width / 2;
      for (let y = Math.floor(cy - r - 1); y <= cy + r + 1; y++) {
        for (let x = Math.floor(cx - r - 1); x <= cx + r + 1; x++) {
          const key = y * this.width + x;
          if (seen.has(key)) continue;
          const cover = Math.max(0, Math.min(1, r + 0.5 - Math.hypot(x + 0.5 - cx, y + 0.5 - cy)));
          if (cover >= 0.5) {
            seen.add(key);
            this.blend(x, y, color, alpha);
          }
        }
      }
    }
  }

  /** Robot : disque et trait de cap. */
  robot(pose, color, size = 7) {
    const a = (pose.theta * Math.PI) / 180;
    const L = (size * 2.2) / this.scale;
    this.line(pose.x, pose.y, pose.x + L * Math.cos(a), pose.y + L * Math.sin(a), hex('#1d2327'), 2.5);
    this.dot(pose.x, pose.y, size, hex('#ffffff'));
    this.dot(pose.x, pose.y, size - 2, color);
  }

  /** Grid map (blanc libre, gris-bleu inconnu, encre occupé), couleurs de occupancyRGB. */
  grid(g) {
    for (let j = 0; j < g.height; j++) {
      for (let i = 0; i < g.width; i++) {
        const color = occupancyRGB(g, g.index(i, j));
        const x0 = this.X(g.originX + i * g.res);
        const y0 = this.Y(g.originY + (j + 1) * g.res);
        const x1 = this.X(g.originX + (i + 1) * g.res);
        const y1 = this.Y(g.originY + j * g.res);
        for (let y = Math.round(y0); y < Math.round(y1); y++) {
          for (let x = Math.round(x0); x < Math.round(x1); x++) this.blend(x, y, color);
        }
      }
    }
  }

  /** Quadrillage métrique. */
  metric(step = 1) {
    const c = hex('#1d2327');
    for (let x = Math.ceil(this.b.minX / step) * step; x <= this.b.maxX; x += step) {
      this.line(x, this.b.minY, x, this.b.maxY, c, 1, null, x === 0 ? 0.35 : 0.08);
    }
    for (let y = Math.ceil(this.b.minY / step) * step; y <= this.b.maxY; y += step) {
      this.line(this.b.minX, y, this.b.maxX, y, c, 1, null, y === 0 ? 0.35 : 0.08);
    }
  }

  png() {
    return encodePNG(this.width, this.height, this.rgb);
  }
}
