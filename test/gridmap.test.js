import { test } from 'node:test';
import assert from 'node:assert/strict';
import { bresenham, GridMap, gridFromScan, updateGrid } from '../src/core/gridmap.js';
import { rectangle, simulateScan } from '../src/core/sim.js';

const line = (i0, j0, i1, j1) => {
  const cells = [];
  bresenham(i0, j0, i1, j1, (i, j) => cells.push([i, j]));
  return cells;
};

/** Scan minimal : un seul faisceau. */
const oneRay = (angle, r) => ({ name: 'ray', pose: { x: 0, y: 0, theta: 0 }, points: [{ angle, r, intensity: 200 }] });

test('bresenham : extrémités incluses, une cellule par pas sur l\'axe dominant', () => {
  assert.deepEqual(line(0, 0, 3, 0), [[0, 0], [1, 0], [2, 0], [3, 0]]);
  assert.deepEqual(line(0, 0, 0, -2), [[0, 0], [0, -1], [0, -2]]);
  assert.deepEqual(line(0, 0, 2, 2), [[0, 0], [1, 1], [2, 2]]);
  const steep = line(0, 0, 2, 7);
  assert.equal(steep.length, 8);
  assert.deepEqual(steep.at(-1), [2, 7]);
  // chaque pas est un déplacement vers une cellule voisine (8-connexité)
  for (let k = 1; k < steep.length; k++) {
    assert.ok(Math.abs(steep[k][0] - steep[k - 1][0]) <= 1 && steep[k][1] - steep[k - 1][1] === 1);
  }
});

test('worldToCell / cellCenter, y compris en coordonnées négatives', () => {
  const g = new GridMap({ resolution: 0.1, offI: -10, offJ: -5, width: 30, height: 20 });
  assert.deepEqual(g.worldToCell(-1, -0.5), [0, 0]);
  assert.deepEqual(g.worldToCell(-0.95, -0.41), [0, 0]);
  assert.deepEqual(g.worldToCell(0, 0), [10, 5]);
  const c = g.cellCenter(10, 5);
  assert.ok(Math.abs(c.x - 0.05) < 1e-12 && Math.abs(c.y - 0.05) < 1e-12);
});

test('un faisceau : cellules traversées libres, cellule d\'impact occupée, le reste inconnu', () => {
  const g = new GridMap({ resolution: 0.1, offI: -5, offJ: -5, width: 30, height: 10 });
  // robot en (0, 0) dans la cellule (5, 5), impact à 1,05 m sur +X -> cellule (15, 5)
  g.integrateScan(oneRay(0, 1.05), { x: 0.01, y: 0.01, theta: 0 });
  for (let i = 5; i < 15; i++) assert.equal(g.classify(g.index(i, 5)), -1, `cellule ${i} libre`);
  assert.equal(g.classify(g.index(15, 5)), 1);
  assert.equal(g.classify(g.index(16, 5)), 0);
  assert.equal(g.classify(g.index(10, 7)), 0);
});

test('une cellule n\'est mise à jour qu\'une fois par scan', () => {
  const g = new GridMap({ resolution: 0.1, offI: -5, offJ: -5, width: 30, height: 10 });
  const scan = { name: 'x', pose: { x: 0, y: 0, theta: 0 }, points: [] };
  for (let a = -2; a <= 2; a += 0.5) scan.points.push({ angle: (a + 360) % 360, r: 1.5, intensity: 200 });
  g.integrateScan(scan, { x: 0.05, y: 0.05, theta: 0 });
  // la cellule du robot est traversée par tous les faisceaux mais ne reçoit qu'un lFree
  assert.ok(Math.abs(g.logOdds[g.index(5, 5)] - g.params.lFree) < 1e-6);
});

test('une cellule déjà occupée par un faisceau n\'est pas libérée par un autre dans le même scan', () => {
  const g = new GridMap({ resolution: 0.1, offI: -5, offJ: -5, width: 30, height: 10 });
  const scan = { name: 'x', pose: { x: 0, y: 0, theta: 0 }, points: [
    { angle: 0, r: 0.5, intensity: 200 }, // impact en (0,55 ; 0,05) -> cellule (10, 5)
    { angle: 0, r: 1.2, intensity: 200 }, // traverse la cellule (10, 5)
  ] };
  g.integrateScan(scan, { x: 0.05, y: 0.05, theta: 0 });
  assert.equal(g.classify(g.index(10, 5)), 1);
});

test('scan simulé dans une pièce : murs occupés, intérieur libre, extérieur inconnu', () => {
  const room = rectangle(0, 0, 4, 3);
  const scan = simulateScan(room, { x: 1.5, y: 1.2, theta: 20 }, { noiseRange: 0, dropRate: 0 });
  const g = gridFromScan(scan, scan.pose, { resolution: 0.05 });
  const at = (x, y) => g.classify(g.index(...g.worldToCell(x, y)));
  assert.equal(at(2, 1.5), -1);
  assert.equal(at(3.9, 2.9), -1);
  assert.equal(at(4.6, 1.5), 0);
  // chaque point du mur de droite (x = 4) tombe dans une cellule occupée, juste d'un côté ou de l'autre
  for (let y = 0.2; y < 2.9; y += 0.1) {
    assert.ok(at(3.975, y) === 1 || at(4.025, y) === 1, `mur x=4 non occupé en y=${y.toFixed(2)}`);
  }
});

test('la grille s\'agrandit pour couvrir un scan pris ailleurs sans perdre le contenu', () => {
  const room = rectangle(0, 0, 4, 3);
  const s1 = simulateScan(room, { x: 1, y: 1, theta: 0 }, { noiseRange: 0, dropRate: 0 });
  const g = gridFromScan(s1, s1.pose, { resolution: 0.05 });
  const before = g.value(g.index(...g.worldToCell(2, 1)));
  const shifted = { x: -6, y: -4, theta: 0 }; // même scan placé loin : force un agrandissement vers le bas-gauche
  const grown = updateGrid(g, s1, shifted);
  assert.ok(grown.width > g.width && grown.height > g.height);
  assert.equal(grown.value(grown.index(...grown.worldToCell(2, 1))), before);
});

test('mise à jour : un objet retiré finit par redevenir libre', () => {
  const room = rectangle(0, 0, 4, 3);
  // face avant du carton en x = 2,52 : au milieu d'une cellule de 5 cm, pas sur une frontière
  const box = rectangle(2.52, 1.32, 2.72, 1.52);
  const pose = { x: 1, y: 1.4, theta: 0 };
  let g = gridFromScan(simulateScan([...room, ...box], pose, { noiseRange: 0, dropRate: 0 }), pose);
  const boxCell = g.index(...g.worldToCell(2.53, 1.42));
  assert.equal(g.classify(boxCell), 1);
  for (let k = 0; k < 6; k++) g = updateGrid(g, simulateScan(room, pose, { noiseRange: 0, dropRate: 0, seed: k }), pose);
  assert.equal(g.classify(boxCell), -1);
});

test('décroissance temporelle et filtrage des cellules isolées', () => {
  const g = new GridMap({ resolution: 0.1, offI: 0, offJ: 0, width: 10, height: 10 });
  g.logOdds[g.index(5, 5)] = 4; // cellule occupée isolée
  g.logOdds[g.index(1, 1)] = 4;
  g.logOdds[g.index(2, 1)] = 4; // deux cellules voisines
  assert.equal(g.removeIsolated(), 1);
  assert.equal(g.classify(g.index(5, 5)), 0);
  assert.equal(g.classify(g.index(1, 1)), 1);
  g.scanCount = 10;
  g.decay(0.5, 1);
  assert.equal(g.logOdds[g.index(1, 1)], 2);
});

test('export CSV puis import : même grille', () => {
  const room = rectangle(0, 0, 2, 1.5);
  const scan = simulateScan(room, { x: 0.7, y: 0.6, theta: 0 }, { noiseRange: 0, dropRate: 0 });
  const g = gridFromScan(scan, scan.pose, { resolution: 0.1 });
  const back = GridMap.fromCSV(g.toCSV());
  assert.equal(back.width, g.width);
  assert.equal(back.offI, g.offI);
  for (let k = 0; k < g.logOdds.length; k++) assert.equal(back.value(k), g.value(k));
});

test('import d\'une grille MapTool (binaire, origine en 0,0, lignes du haut vers le bas)', () => {
  const csv = '# Grid map 3x2, cell=0.1 m ; 1=occupied,0=free\n1,0,0\n0,0,1\n';
  const g = GridMap.fromCSV(csv);
  assert.equal(g.width, 3);
  assert.equal(g.height, 2);
  assert.equal(g.classify(g.index(0, 1)), 1); // première ligne = rangée du haut
  assert.equal(g.classify(g.index(2, 0)), 1);
  assert.equal(g.classify(g.index(1, 0)), -1);
});
