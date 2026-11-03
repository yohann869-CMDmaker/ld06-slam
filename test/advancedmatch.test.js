import { test } from 'node:test';
import assert from 'node:assert/strict';
import { localize, gridReference, scanReference, vectorReference } from '../src/core/localization.js';
import { ndtGrids, orientedPoints } from '../src/core/advancedmatch.js';
import { rectangle, simulateScan } from '../src/core/sim.js';
import { scanToLocalXY, scanToWorldXY } from '../src/core/scan.js';
import { gridFromScan } from '../src/core/gridmap.js';

// salle 8 m × 6 m avec une table et un pilier (pas de symétrie parfaite)
const ROOM = [...rectangle(0, 0, 8, 6), ...rectangle(3, 2.5, 5, 3.5), ...rectangle(6.5, 1, 6.8, 1.3)];
const TRUTH = { x: 2, y: 1.5, theta: 15 };
const pts = scanToLocalXY(simulateScan(ROOM, TRUTH, { seed: 21 }));
const err = (p) => ({ d: Math.hypot(p.x - TRUTH.x, p.y - TRUTH.y), a: Math.abs(p.theta - TRUTH.theta) });

test('normales des points de référence : perpendiculaires au mur, coins écartés', () => {
  const wall = [];
  for (let k = 0; k <= 100; k++) wall.push({ x: k * 0.02, y: 0 });
  const o = orientedPoints(wall);
  assert.ok(o[50].ok);
  assert.ok(Math.abs(Math.abs(o[50].ny) - 1) < 1e-6, 'normale verticale');
  const corner = [...wall, ...Array.from({ length: 50 }, (_, k) => ({ x: 0, y: k * 0.02 }))];
  assert.equal(orientedPoints(corner)[0].ok, false, 'au coin, le voisinage n\'est pas rectiligne');
});

test('grille NDT : une cellule le long d\'un mur a une gaussienne allongée selon le mur', () => {
  const wall = [];
  for (let k = 0; k < 50; k++) wall.push({ x: 0.1 + k * 0.01, y: 0.2 + 0.001 * (k % 3) });
  const cell = ndtGrids(wall, 1).grids[0].cells.get('0,0');
  assert.ok(cell);
  assert.ok(Math.abs(cell.my - 0.201) < 0.002);
  assert.ok(cell.cyy > 10 * cell.cxx, 'inverse de covariance : forte en travers du mur (y), faible le long (x)');
});

for (const method of ['plicp', 'ndt', 'gaussnewton']) {
  test(`${method} : pose retrouvée sur les trois références depuis 20 cm et 8° d'écart`, () => {
    const refScan = simulateScan(ROOM, { x: 2.3, y: 1.8, theta: 0 }, { seed: 5 });
    const refs = {
      vector: vectorReference(ROOM),
      scan: scanReference(scanToWorldXY(refScan)),
      grid: gridReference(gridFromScan(refScan, refScan.pose)),
    };
    const init = { x: TRUTH.x + 0.15, y: TRUTH.y - 0.13, theta: TRUTH.theta + 8 };
    for (const [kind, ref] of Object.entries(refs)) {
      const e = err(localize(ref, pts, init, method).pose);
      assert.ok(e.d < 0.04 && e.a < 0.5, `${kind} : ${(100 * e.d).toFixed(1)} cm, ${e.a.toFixed(2)}°`);
    }
  });
}

test('export ROS : PGM binaire (libre 254, occupé 0, inconnu 205) et YAML map_server', () => {
  const g = gridFromScan(simulateScan(ROOM, TRUTH, { seed: 3 }), TRUTH);
  const { pgm, yaml } = g.toROS('salle');
  const head = new TextDecoder().decode(pgm.slice(0, 40)).split('\n');
  assert.equal(head[0], 'P5');
  assert.equal(head[2], `${g.width} ${g.height}`);
  const values = new Set(pgm.slice(pgm.length - g.width * g.height));
  assert.deepEqual([...values].sort((a, b) => a - b), [0, 205, 254]);
  assert.match(yaml, /^image: salle\.pgm$/m);
  assert.match(yaml, new RegExp(`^resolution: ${g.res}$`, 'm'));
  assert.match(yaml, /^origin: \[-?\d+\.\d+, -?\d+\.\d+, 0\.0\]$/m);
});
