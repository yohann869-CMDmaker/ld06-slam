import { test } from 'node:test';
import assert from 'node:assert/strict';
import { updateVectorMap, VectorMap, vectorMapFromScan } from '../src/core/vectormap.js';
import { rectangle, simulateScan } from '../src/core/sim.js';
import { length } from '../src/core/geometry.js';

const room = rectangle(0, 0, 4, 3);
const noNoise = { noiseRange: 0, dropRate: 0 };

test('carte depuis un scan : un segment par mur', () => {
  const scan = simulateScan(room, { x: 1.5, y: 1, theta: 0 }, noNoise);
  const m = vectorMapFromScan(scan);
  assert.equal(m.segments.length, 4);
});

test('mise à jour depuis une autre pose : fusion, pas de doublon', () => {
  const m1 = vectorMapFromScan(simulateScan(room, { x: 1, y: 1, theta: 0 }, { seed: 1 }));
  const m2 = updateVectorMap(m1, simulateScan(room, { x: 3, y: 2, theta: 90 }, { seed: 2 }));
  assert.equal(m2.segments.length, 4);
  assert.ok(m2.segments.every((s) => s.observations === 2));
  assert.equal(m1.segments[0].observations, 1, 'la carte d\'origine n\'est pas modifiée');
});

test('fusion : un mur vu partiellement depuis deux poses est reconstruit en entier', () => {
  const hidden = [...room, ...rectangle(1.8, 2.0, 2.2, 2.4)]; // poteau qui masque une partie du mur du haut
  const m = new VectorMap();
  m.integrateScan(simulateScan(hidden, { x: 1.0, y: 1.0, theta: 0 }, noNoise));
  m.integrateScan(simulateScan(hidden, { x: 3.0, y: 1.0, theta: 0 }, noNoise));
  const top = m.segments.filter((s) => Math.abs(s.y1 - 3) < 0.02 && Math.abs(s.y2 - 3) < 0.02);
  assert.equal(top.length, 1);
  assert.ok(length(top[0]) > 3.9, `mur du haut : ${length(top[0]).toFixed(2)} m`);
});

test('absence : un obstacle retiré est découpé puis supprimé', () => {
  const box = rectangle(2.5, 1.3, 2.7, 1.5);
  const pose = { x: 1, y: 1.4, theta: 0 };
  const m = vectorMapFromScan(simulateScan([...room, ...box], pose, noNoise));
  const boxFace = (s) => Math.abs(s.x1 - 2.5) < 0.02 && Math.abs(s.x2 - 2.5) < 0.02;
  assert.equal(m.segments.filter(boxFace).length, 1);
  const st = m.integrateScan(simulateScan(room, pose, noNoise));
  assert.ok(st.cuts > 0);
  assert.equal(m.segments.filter(boxFace).length, 0);
});

test('absence : les murs vus ne sont pas découpés par leurs propres faisceaux', () => {
  const m = vectorMapFromScan(simulateScan(room, { x: 1.5, y: 1, theta: 0 }, { seed: 4 }));
  const st = m.integrateScan(simulateScan(room, { x: 2.5, y: 2, theta: 30 }, { seed: 5 }));
  assert.equal(st.cuts, 0);
  assert.equal(m.segments.length, 4);
});

test('export TXT puis import : mêmes segments', () => {
  const m = vectorMapFromScan(simulateScan(room, { x: 1.5, y: 1, theta: 0 }, noNoise));
  const back = VectorMap.fromTXT(m.toTXT());
  assert.equal(back.segments.length, m.segments.length);
  for (let k = 0; k < m.segments.length; k++) {
    assert.ok(Math.abs(back.segments[k].x1 - m.segments[k].x1) < 1e-4);
  }
});
