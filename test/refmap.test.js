import { test } from 'node:test';
import assert from 'node:assert/strict';
import { makeReferenceMap, parseReferenceMap, referenceBounds, serializeReferenceMap } from '../src/core/refmap.js';
import { gridFromScan } from '../src/core/gridmap.js';
import { rectangle, simulateScan } from '../src/core/sim.js';

test('carte de référence : enregistrement puis relecture identiques', () => {
  const room = rectangle(0, 0, 4, 3);
  const scan = simulateScan(room, { x: 1.5, y: 1.2, theta: 30 }, { seed: 3 });
  const grid = gridFromScan(scan);
  const ref = makeReferenceMap({
    name: 'salle test',
    grid,
    segments: [{ x1: 0, y1: 0, x2: 4, y2: 0, weight: 12 }],
    corners: [{ x: 0, y: 0, theta: 45 }],
    scans: [{ name: 'scan_01.csv', pose: scan.pose }],
    clean: true,
  });
  const text = serializeReferenceMap(ref);
  const back = parseReferenceMap(text);
  assert.equal(back.name, 'salle test');
  assert.equal(back.clean, true);
  assert.equal(back.grid.width, grid.width);
  assert.equal(back.grid.offI, grid.offI);
  for (let k = 0; k < grid.logOdds.length; k++) assert.equal(back.grid.classify(k), grid.classify(k));
  assert.deepEqual(back.segments, [{ x1: 0, y1: 0, x2: 4, y2: 0 }]);
  assert.deepEqual(back.scans[0].pose, { x: 1.5, y: 1.2, theta: 30 });
  // compression par plages : bien plus petit qu'une valeur par cellule
  assert.ok(JSON.parse(text).grid.runs.length < grid.width * grid.height / 4);
});

test('carte de référence : fichier d\'un autre format refusé', () => {
  assert.throws(() => parseReferenceMap('{"format":"autre"}'), /pas une carte de référence/);
});

test('carte de référence : bornes des cellules connues', () => {
  const room = rectangle(0, 0, 4, 3);
  const grid = gridFromScan(simulateScan(room, { x: 2, y: 1.5, theta: 0 }, { noiseRange: 0, dropRate: 0 }));
  const b = referenceBounds(makeReferenceMap({ grid }), 0);
  assert.ok(Math.abs(b.minX - 0) < 0.1 && Math.abs(b.maxX - 4) < 0.1);
  assert.ok(Math.abs(b.minY - 0) < 0.1 && Math.abs(b.maxY - 3) < 0.1);
});

test('carte de référence : les objets fixes gardent leur type, posés au sol ou surélevés', () => {
  const grid = gridFromScan(simulateScan(rectangle(0, 0, 4, 3), { x: 2, y: 1.5, theta: 0 }, { seed: 1 }));
  const ref = makeReferenceMap({
    grid,
    segments: [
      { x1: 0, y1: 0, x2: 4, y2: 0, kind: 'mur' },
      { x1: 1, y1: 1, x2: 2, y2: 1, kind: 'objet' },
      { x1: 1, y1: 2, x2: 2, y2: 2, kind: 'objet', raised: true },
    ],
  });
  const text = serializeReferenceMap(ref);
  assert.match(text, /"objet surélevé"/);
  assert.deepEqual(parseReferenceMap(text).segments, [
    { x1: 0, y1: 0, x2: 4, y2: 0 },
    { x1: 1, y1: 1, x2: 2, y2: 1, kind: 'objet', raised: false },
    { x1: 1, y1: 2, x2: 2, y2: 2, kind: 'objet', raised: true },
  ]);
});
