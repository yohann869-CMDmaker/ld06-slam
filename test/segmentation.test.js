import { test } from 'node:test';
import assert from 'node:assert/strict';
import { clusterize, extractSegments, fitLine } from '../src/core/segmentation.js';
import { rectangle, simulateScan } from '../src/core/sim.js';
import { scanToWorldXY } from '../src/core/scan.js';
import { angleBetweenDeg, pointLineDistance } from '../src/core/geometry.js';

const room = rectangle(0, 0, 4, 3);
const walls = [
  { x1: 0, y1: 0, x2: 4, y2: 0 },
  { x1: 4, y1: 0, x2: 4, y2: 3 },
  { x1: 0, y1: 3, x2: 4, y2: 3 },
  { x1: 0, y1: 0, x2: 0, y2: 3 },
];

/** Le segment s est-il porté par le mur w (angle et distance) ? */
const onWall = (s, w, tolDist) =>
  angleBetweenDeg(s, w) < 1.5 && pointLineDistance(w, s.x1, s.y1) < tolDist && pointLineDistance(w, s.x2, s.y2) < tolDist;

test('fitLine : droite verticale (cas où y = ax + b échouerait)', () => {
  const f = fitLine([0, 1, 2, 3].map((y) => ({ x: 2, y })));
  assert.ok(Math.abs(Math.abs(f.uy) - 1) < 1e-12);
  assert.ok(f.rms < 1e-12);
  assert.equal(f.cx, 2);
});

test('pièce sans bruit : 4 segments, un par mur', () => {
  const scan = simulateScan(room, { x: 1.7, y: 1.2, theta: 25 }, { noiseRange: 0, dropRate: 0 });
  const { segments } = extractSegments(scanToWorldXY(scan));
  assert.equal(segments.length, 4);
  for (const w of walls) assert.ok(segments.some((s) => onWall(s, w, 0.005)), `mur ${JSON.stringify(w)} non trouvé`);
});

test('pièce avec bruit de 1 cm : toujours un segment par mur, rms de l\'ordre du bruit', () => {
  const scan = simulateScan(room, { x: 1.7, y: 1.2, theta: 25 }, { noiseRange: 0.01, dropRate: 0.02, seed: 3 });
  const { segments } = extractSegments(scanToWorldXY(scan));
  for (const w of walls) assert.ok(segments.some((s) => onWall(s, w, 0.02)), `mur ${JSON.stringify(w)} non trouvé`);
  for (const s of segments) assert.ok(s.rms < 0.02);
});

test('le mur qui contient l\'angle 0° n\'est pas coupé en deux (raccord 360° -> 0°)', () => {
  // robot orienté vers le mur de droite : les angles du mur vont de ~-35° à ~+35°
  const scan = simulateScan(room, { x: 2, y: 1.5, theta: 0 }, { noiseRange: 0, dropRate: 0 });
  const pts = scanToWorldXY(scan);
  const clusters = clusterize(pts);
  assert.equal(clusters.length, 1); // pièce fermée : un seul objet continu
  const { segments } = extractSegments(pts);
  assert.equal(segments.filter((s) => onWall(s, walls[1], 0.005)).length, 1);
});

test('deux objets à des distances différentes donnent deux clusters', () => {
  const pole = rectangle(1.0, -0.1, 1.2, 0.1);
  const far = [{ x1: 3, y1: -2, x2: 3, y2: 2 }];
  const scan = simulateScan([...pole, ...far], { x: 0, y: 0, theta: 0 }, { noiseRange: 0, dropRate: 0, rays: 720 });
  const clusters = clusterize(scanToWorldXY(scan));
  assert.ok(clusters.length >= 3, `${clusters.length} clusters`); // mur, poteau, mur (de part et d'autre du poteau)
});

test('coin en L : deux segments perpendiculaires', () => {
  const corner = [
    { x1: 2, y1: -1, x2: 2, y2: 1 },
    { x1: 2, y1: 1, x2: -1, y2: 1 },
  ];
  const scan = simulateScan(corner, { x: 0, y: 0, theta: 0 }, { noiseRange: 0, dropRate: 0 });
  const { segments } = extractSegments(scanToWorldXY(scan));
  assert.equal(segments.length, 2);
  assert.ok(Math.abs(angleBetweenDeg(segments[0], segments[1]) - 90) < 1);
});
