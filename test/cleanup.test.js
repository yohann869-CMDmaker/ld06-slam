import { test } from 'node:test';
import assert from 'node:assert/strict';
import { classifySegments, cleanGrid, cleanMaps, cleanVectorMap, dominantDirection, drawObjects, isPartlyOccupied, markRaised, objectGroups, segmentsNearFloor } from '../src/core/cleanup.js';
import { GridMap, gridFromScan } from '../src/core/gridmap.js';
import { VectorMap } from '../src/core/vectormap.js';
import { rectangle, simulateScan } from '../src/core/sim.js';
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

// Pièce propre de 2 m × 2 m (cellules de 10 cm) : sol libre, murs occupés, extérieur inconnu,
// et une table au milieu dont l'intérieur est inconnu (jamais vu) mais entouré de sol.
function roomWithTable() {
  const g = new GridMap({ resolution: 0.1, offI: 0, offJ: 0, width: 30, height: 30 });
  const set = (i, j, l) => (g.logOdds[g.index(i, j)] = l);
  for (let j = 5; j < 25; j++) for (let i = 5; i < 25; i++) set(i, j, -4);
  for (let k = 4; k <= 25; k++) [set(k, 4, 4), set(k, 25, 4), set(4, k, 4), set(25, k, 4)];
  for (let j = 12; j < 15; j++) for (let i = 12; i < 16; i++) set(i, j, 0);
  return g;
}

test('murs et objets fixes : côté de table vu par 2 scans gardé, face accrochée par un coin, fouillis retiré', () => {
  const g = roomWithTable();
  const out = classifySegments(
    [
      { ...S(0.5, 0.5, 2.5, 0.5), observations: 1 }, // mur du bas, vu une fois : reste un mur
      { ...S(1.2, 1.2, 1.6, 1.2), observations: 2 }, // côté de la table vu par 2 scans
      { ...S(1.6, 1.2, 1.6, 1.5), observations: 1 }, // autre côté, vu une fois, relié par le coin
      { ...S(0.8, 2.0, 1.1, 2.0), observations: 1 }, // chaise vue une fois, isolée
    ],
    g,
  );
  assert.deepEqual(out.map((s) => s.kind), ['mur', 'objet', 'objet']);
  const drawn = drawObjects(g, out);
  const [i, j] = g.worldToCell(1.4, 1.2);
  assert.equal(drawn.classify(drawn.index(i, j)), 1, 'côté de la table dessiné dans la grid');
  assert.notEqual(g.classify(g.index(i, j)), 1, 'grid d\'origine inchangée');
});

test('cartes propres : la table d\'une salle simulée devient un objet fixe, les murs restent des murs', () => {
  const scene = [...rectangle(0, 0, 6, 4), ...rectangle(2.5, 1.5, 3.5, 2.1)];
  const poses = [{ x: 1, y: 1, theta: 0 }, { x: 5, y: 3, theta: 90 }, { x: 1, y: 3, theta: -30 }];
  const scans = poses.map((p, k) => simulateScan(scene, p, { seed: k + 1 }));
  const grid = gridFromScan(scans[0], poses[0]);
  scans.slice(1).forEach((s, k) => grid.integrateScan(s, poses[k + 1]));
  const vm = new VectorMap([], { absence: false });
  scans.forEach((s, k) => vm.integrateScan(s, poses[k]));
  const c = cleanMaps(grid, vm.segments, poses);
  const mid = (s) => ({ x: (s.x1 + s.x2) / 2, y: (s.y1 + s.y2) / 2 });
  const objects = c.segments.filter((s) => s.kind === 'objet');
  const walls = c.segments.filter((s) => s.kind === 'mur');
  assert.equal(objects.length, 4, 'les 4 côtés de la table');
  for (const s of objects) {
    const m = mid(s);
    assert.ok(m.x > 2.4 && m.x < 3.6 && m.y > 1.4 && m.y < 2.2, `objet hors de la table : ${m.x}, ${m.y}`);
  }
  assert.equal(walls.length, 4, 'les 4 murs');
  for (const s of walls) {
    const m = mid(s);
    assert.ok(m.x < 0.2 || m.x > 5.8 || m.y < 0.2 || m.y > 3.8, `mur hors du bord : ${m.x}, ${m.y}`);
  }
});

// Robot en `pose` ; `stop` impacts sur le meuble (y = 1) et `under` impacts 1 m derrière (y = 2) :
// ces faisceaux-là sont passés sous le meuble.
const view = (pose, x0, stop, under) => ({
  pose,
  points: [
    ...Array.from({ length: stop }, (_, k) => ({ x: x0 + 0.2 + 0.06 * k, y: 1 })),
    ...Array.from({ length: under }, (_, k) => ({ x: x0 + 0.2 + 0.06 * k, y: 2 })),
  ],
});
const objet = (x1, y1, x2, y2) => ({ x1, y1, x2, y2, kind: 'objet' });

test('meubles : regroupés par leurs coins (les côtés d\'une table forment un seul meuble)', () => {
  const segs = [objet(0, 0, 1, 0), objet(1, 0, 1, 1), { ...S(5, 0, 6, 0), kind: 'mur' }, objet(3, 0, 4, 0)];
  assert.deepEqual(objectGroups(segs), [[0, 1], [3]]);
});

test('meubles surélevés : la table laisse passer des faisceaux dessous, l\'armoire les arrête', () => {
  const table = objet(1, 1, 2, 1);
  const armoire = objet(4, 1, 5, 1);
  const out = markRaised([table, armoire], [view({ x: 1.5, y: 0, theta: 90 }, 1, 6, 4), view({ x: 4.5, y: 0, theta: 90 }, 4, 10, 0)]);
  assert.equal(out[0].raised, true);
  assert.ok(Math.abs(out[0].under - 0.4) < 1e-9, `table : ${out[0].under}`);
  assert.equal(out[1].raised, false);
  assert.equal(out[1].under, 0);
  assert.equal(table.raised, undefined, 'segments d\'origine inchangés');
});

test('meubles surélevés : un robot garé dans l\'emprise du meuble ne compte pas', () => {
  // armoire en L de 1 m × 1 m ; un robot placé à l'intérieur voit à travers ses faces
  const segs = [objet(6, 1, 7, 1), objet(7, 1, 7, 2)];
  const inside = { pose: { x: 6.5, y: 1.5, theta: -90 }, points: [{ x: 6.5, y: 0.2 }, { x: 6.6, y: 0.2 }, { x: 6.7, y: 0.2 }] };
  const out = markRaised(segs, [inside, view({ x: 6.5, y: 0, theta: 90 }, 6, 5, 0)]);
  assert.deepEqual(out.map((s) => s.raised), [false, false]);
});

test('grid map propre : meuble surélevé occupé en partie, meuble posé au sol occupé', () => {
  const g = roomWithTable();
  const drawn = drawObjects(g, [{ ...objet(0.8, 1.0, 1.2, 1.0), raised: true }, objet(0.8, 2.0, 1.2, 2.0)]);
  const [i1, j1] = g.worldToCell(1.0, 1.0);
  const [i2, j2] = g.worldToCell(1.0, 2.0);
  assert.equal(drawn.classify(drawn.index(i1, j1)), 1, 'reste un obstacle');
  assert.equal(isPartlyOccupied(drawn, drawn.index(i1, j1)), true);
  assert.equal(isPartlyOccupied(drawn, drawn.index(i2, j2)), false);
});

test('murs vus d\'en haut : ils ferment la salle derrière une armoire, qui reste un objet', () => {
  // pièce de 2 m : le mur de droite (i = 25) n'est pas vu du sol derrière l'armoire (j 12..16)
  const g = new GridMap({ resolution: 0.1, offI: 0, offJ: 0, width: 30, height: 30 });
  const set = (i, j, l) => (g.logOdds[g.index(i, j)] = l);
  for (let j = 5; j < 25; j++) for (let i = 5; i < 21; i++) set(i, j, -4);
  for (let j = 5; j < 25; j++) for (let i = 21; i < 25; i++) if (j < 12 || j > 16) set(i, j, -4);
  for (let k = 4; k <= 25; k++) [set(k, 4, 4), set(k, 25, 4), set(4, k, 4)];
  for (let j = 4; j <= 25; j++) if (j < 12 || j > 16) set(25, j, 4);
  const armoire = [
    { ...S(2.1, 1.2, 2.1, 1.7), observations: 2 },
    { ...S(2.1, 1.7, 2.5, 1.7), observations: 2 },
    { ...S(2.1, 1.2, 2.5, 1.2), observations: 2 },
  ];
  assert.deepEqual(classifySegments(armoire, g).map((s) => s.kind), ['mur', 'mur', 'mur'], 'sans le mur caché : frontière');
  const out = classifySegments(armoire, g, { barriers: [S(2.55, 0.45, 2.55, 2.55)] });
  assert.deepEqual(out.map((s) => s.kind), ['objet', 'objet', 'objet', 'mur']);
  assert.equal(out[3].seenFromAbove, true);
});
