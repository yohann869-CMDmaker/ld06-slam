import { test } from 'node:test';
import assert from 'node:assert/strict';
import { filterClutter, reconstructRoom } from '../src/core/reconstruct.js';
import { gridFromScan } from '../src/core/gridmap.js';
import { VectorMap } from '../src/core/vectormap.js';
import { rectangle, simulateScan } from '../src/core/sim.js';

// Salle en L (8 × 6 m moins le coin 3 × 3 m en haut à gauche, 39 m²), une table et des pieds de chaises.
const W = (x1, y1, x2, y2) => ({ x1, y1, x2, y2 });
const room = [W(0, 0, 8, 0), W(8, 0, 8, 6), W(8, 6, 3, 6), W(3, 6, 3, 3), W(3, 3, 0, 3), W(0, 3, 0, 0)];
const table = rectangle(5, 2, 6.2, 2.8);
const legs = [[1.5, 1.2], [2.0, 1.6], [4.2, 1.0], [4.5, 4.5], [6.8, 4.2], [7.1, 1.1]].flatMap(([x, y]) => rectangle(x, y, x + 0.04, y + 0.04));
const scene = [...room, ...table, ...legs];
const poses = [
  { x: 1, y: 1.5, theta: 0 },
  { x: 2.5, y: 0.8, theta: 30 },
  { x: 4, y: 1.5, theta: 60 },
  { x: 5.5, y: 1.2, theta: 0 },
  { x: 7, y: 2, theta: 90 },
  { x: 7, y: 4.5, theta: 150 },
  { x: 5, y: 5, theta: 180 },
  { x: 3.8, y: 4, theta: -90 },
];
const scans = poses.map((p, k) => simulateScan(scene, p, { seed: k + 1 }));
const grid = gridFromScan(scans[0], poses[0]);
scans.slice(1).forEach((s, k) => grid.integrateScan(s, poses[k + 1]));
const vm = new VectorMap([], { absence: false });
scans.forEach((s, k) => vm.integrateScan(s, poses[k]));

test('filtrage 1 : les pieds de chaises sont retirés, les murs et la table restent', () => {
  const c = filterClutter(grid);
  for (const [x, y] of [[1.52, 1.22], [4.52, 4.52], [6.82, 4.22]]) {
    const [i, j] = c.grid.worldToCell(x, y);
    assert.notEqual(c.grid.classify(c.grid.index(i, j)), 1, `pied de chaise gardé en ${x}, ${y}`);
  }
  for (const [x, y] of [[4, 0], [8, 3], [5.6, 2]]) {
    const [i, j] = c.grid.worldToCell(x, y);
    let near = false;
    for (let dj = -2; dj <= 2; dj++) for (let di = -2; di <= 2; di++) if (c.grid.classify(c.grid.index(i + di, j + dj)) === 1) near = true;
    assert.ok(near, `mur ou table perdu en ${x}, ${y}`);
  }
});

test('reconstruction : la salle en L, sans le coin extérieur, murs au bon endroit', () => {
  const c = filterClutter(grid);
  const r = reconstructRoom(c.grid, vm.segments, { seeds: poses });
  assert.ok(Math.abs(r.area - 39) < 4, `surface ${r.area.toFixed(1)} m² au lieu de 39`);
  // chaque mur reconstruit est sur un mur de la salle ou sur la table
  const dist = (p, w) => {
    const dx = w.x2 - w.x1;
    const dy = w.y2 - w.y1;
    const t = Math.max(0, Math.min(1, ((p.x - w.x1) * dx + (p.y - w.y1) * dy) / (dx * dx + dy * dy)));
    return Math.hypot(p.x - w.x1 - t * dx, p.y - w.y1 - t * dy);
  };
  const truth = [...room, ...table];
  for (const w of r.walls) {
    const m = { x: (w.x1 + w.x2) / 2, y: (w.y1 + w.y2) / 2 };
    const d = Math.min(...truth.map((t) => dist(m, t)));
    assert.ok(d < 0.15, `mur à ${(100 * d).toFixed(0)} cm de la salle : ${JSON.stringify(w)}`);
  }
  // le coin extérieur du L n'est pas dans la salle
  const ang = (r.angle * Math.PI) / 180;
  const u = Math.cos(ang) * 1.5 + Math.sin(ang) * 4.5;
  const v = -Math.sin(ang) * 1.5 + Math.cos(ang) * 4.5;
  const inRoom = r.rects.some((q) => q.label !== 'extérieur' && u >= q.x0 && u < q.x1 && v >= q.y0 && v < q.y1);
  assert.equal(inRoom, false, 'coin extérieur du L compté dans la salle');
});
