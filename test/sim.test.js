import { test } from 'node:test';
import assert from 'node:assert/strict';
import { rayCast, rectangle, simulateScan } from '../src/core/sim.js';
import { scanToWorldXY } from '../src/core/scan.js';

test('rayCast trouve le mur le plus proche', () => {
  const room = rectangle(0, 0, 4, 2);
  const hit = rayCast(room, 1, 1, 1, 0);
  assert.ok(Math.abs(hit.t - 3) < 1e-12);
  assert.equal(rayCast(room, 1, 1, 0, 1).t, 1);
});

test('rayCast ignore un rayon parallèle au segment', () => {
  assert.equal(rayCast([{ x1: 0, y1: 1, x2: 5, y2: 1 }], 0, 0, 1, 0), null);
});

test('scan simulé sans bruit : chaque point retombe sur un mur, quelle que soit la pose', () => {
  const room = rectangle(0, 0, 4, 3);
  const pose = { x: 1.3, y: 0.9, theta: 37 };
  const scan = simulateScan(room, pose, { noiseRange: 0, dropRate: 0, rays: 360 });
  const pts = scanToWorldXY(scan);
  assert.equal(pts.length, 360);
  for (const p of pts) {
    const dWall = Math.min(Math.abs(p.x), Math.abs(p.x - 4), Math.abs(p.y), Math.abs(p.y - 3));
    // arrondis : distance au mm, angle au 0,01° -> moins de 2 mm d'écart à 4 m
    assert.ok(dWall < 0.002, `point (${p.x}, ${p.y}) à ${dWall} m du mur`);
  }
});

test('le simulateur est reproductible à graine égale', () => {
  const room = rectangle(0, 0, 4, 3);
  const a = simulateScan(room, { x: 2, y: 1, theta: 0 }, { seed: 7 });
  const b = simulateScan(room, { x: 2, y: 1, theta: 0 }, { seed: 7 });
  assert.deepEqual(a.points, b.points);
});
