import { test } from 'node:test';
import assert from 'node:assert/strict';
import { cleanGrid, cleanVectorMap, dominantDirection, segmentsNearFloor } from '../src/core/cleanup.js';
import { GridMap } from '../src/core/gridmap.js';
import { orientationDeg } from '../src/core/geometry.js';

const S = (x1, y1, x2, y2) => ({ x1, y1, x2, y2 });

test('direction dominante : murs tournés de 12°', () => {
  const r = (x, y, a) => [x * Math.cos(a) - y * Math.sin(a), x * Math.sin(a) + y * Math.cos(a)];
  const a = (12 * Math.PI) / 180;
  const segs = [
    [0, 0, 4, 0],
    [4, 0, 4, 3],
    [0, 3, 4, 3],
  ].map(([x1, y1, x2, y2]) => S(...r(x1, y1, a), ...r(x2, y2, a)));
  assert.ok(Math.abs(dominantDirection(segs) - 12) < 0.5);
});

test('carte vectorielle propre : murs redressés, morceaux fusionnés, coin fermé', () => {
  const segs = [
    S(0.05, 0.02, 1.9, 0.06), // mur du bas en deux morceaux, légèrement penchés
    S(2.05, 0.04, 3.9, 0.01),
    S(4.02, 0.12, 3.98, 2.9), // mur de droite qui s'arrête 12 cm avant le coin
    S(1.0, 1.0, 1.1, 1.05), // poussière de 11 cm
  ];
  const { segments, dominant } = cleanVectorMap(segs);
  assert.equal(segments.length, 2);
  // les deux murs suivent exactement la direction dominante (≈ 0,4° ici) et sa perpendiculaire
  const bottom = segments.find((s) => Math.round(orientationDeg(s)) === 0);
  const right = segments.find((s) => Math.round(orientationDeg(s)) === 90);
  assert.ok(Math.abs(orientationDeg(bottom) - dominant) < 1e-9);
  assert.ok(Math.abs(orientationDeg(right) - (dominant + 90)) < 1e-9);
  // le mur de droite démarrait 12 cm trop haut : son extrémité basse rejoint maintenant le mur du bas
  const low = right.y1 < right.y2 ? [right.x1, right.y1] : [right.x2, right.y2];
  const end = bottom.x2 > bottom.x1 ? [bottom.x2, bottom.y2] : [bottom.x1, bottom.y1];
  assert.ok(Math.hypot(low[0] - end[0], low[1] - end[1]) < 1e-9);
});

test('grid map propre : un faisceau étroit qui sort de la pièce est retiré, la pièce reste', () => {
  const g = new GridMap({ resolution: 0.1, offI: 0, offJ: 0, width: 60, height: 30 });
  const set = (i, j, l) => (g.logOdds[g.index(i, j)] = l);
  for (let j = 5; j < 25; j++) for (let i = 5; i < 25; i++) set(i, j, -4); // pièce libre
  for (let i = 4; i <= 25; i++) {
    set(i, 4, 4);
    set(i, 25, 4);
  }
  for (let j = 4; j <= 25; j++) {
    set(4, j, 4);
    set(25, j, 4);
  }
  for (let i = 25; i < 58; i++) set(i, 15, -4); // faisceau d'une cellule de large à travers une vitre
  const c = cleanGrid(g, [{ x: 1.5, y: 1.5, theta: 0 }]);
  assert.equal(c.classify(c.index(15, 15)), -1);
  assert.equal(c.classify(c.index(45, 15)), 0, 'faisceau retiré');
  assert.equal(c.classify(c.index(4, 15)), 1, 'mur gardé');
});

test('segments hors de la pièce retirés, segment débordant raccourci', () => {
  const g = new GridMap({ resolution: 0.1, offI: 0, offJ: 0, width: 50, height: 30 });
  for (let j = 0; j < 20; j++) for (let i = 0; i < 20; i++) g.logOdds[g.index(i, j)] = -4;
  const kept = segmentsNearFloor([S(0, 2.1, 2, 2.1), S(4, 0, 4, 2), S(1, 2.05, 3.5, 2.05)], g);
  assert.equal(kept.length, 2);
  assert.ok(kept[1].x2 < 2.4, `segment raccourci à ${kept[1].x2}`);
});
