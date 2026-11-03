// Carte de référence (jour 2) : la carte de la salle construite au jour 1, figée et enregistrée,
// sur laquelle on localise ensuite de nouveaux scans.
//
// Elle réunit dans un seul fichier JSON :
//   - la grid map (valeurs 0..255 par cellule, compressées par plages de valeurs identiques) ;
//   - la carte vectorielle (segments de murs et d'objets fixes) et ses coins ;
//   - les scans qui ont servi à la construire et leurs poses (traçabilité).
// La carte ne change plus ensuite : localiser un scan ne la modifie pas.

import { GridMap } from './gridmap.js';

export const REFMAP_FORMAT = 'ld06-slam/carte-reference';

/** Compression par plages : [valeur, nombre, valeur, nombre, ...]. Efficace : la carte est faite de grandes zones uniformes. */
function runLength(values) {
  const out = [];
  for (let k = 0; k < values.length; ) {
    const v = values[k];
    let n = 1;
    while (k + n < values.length && values[k + n] === v) n++;
    out.push(v, n);
    k += n;
  }
  return out;
}

function unRunLength(runs, size) {
  const out = new Uint8Array(size);
  let k = 0;
  for (let i = 0; i < runs.length; i += 2) {
    out.fill(runs[i], k, k + runs[i + 1]);
    k += runs[i + 1];
  }
  if (k !== size) throw new Error('Grid map de référence incomplète');
  return out;
}

/**
 * Crée une carte de référence.
 * o = { name, grid, segments, corners, scans: [{ name, pose }], clean (version propre ou brute) }
 */
export function makeReferenceMap(o) {
  return {
    format: REFMAP_FORMAT,
    name: o.name ?? 'Carte de référence',
    date: o.date ?? new Date().toISOString(),
    clean: Boolean(o.clean),
    grid: o.grid,
    segments: (o.segments ?? []).map(({ x1, y1, x2, y2, kind }) => (kind === 'objet' ? { x1, y1, x2, y2, kind } : { x1, y1, x2, y2 })),
    corners: (o.corners ?? []).map(({ x, y, theta }) => ({ x, y, theta })),
    scans: (o.scans ?? []).map((s) => ({ name: s.name, pose: { ...s.pose } })),
  };
}

/** Texte JSON de la carte de référence. */
export function serializeReferenceMap(ref) {
  const g = ref.grid;
  const values = new Array(g.width * g.height);
  for (let k = 0; k < values.length; k++) values[k] = g.value(k);
  const round = (v) => Math.round(v * 10000) / 10000;
  return JSON.stringify({
    format: REFMAP_FORMAT,
    version: 1,
    name: ref.name,
    date: ref.date,
    clean: ref.clean,
    grid: {
      resolution: g.res,
      offI: g.offI,
      offJ: g.offJ,
      width: g.width,
      height: g.height,
      encoding: 'valeurs 0..255 (0 libre, 128 inconnu, 255 occupé), lignes de bas en haut, plages [valeur, nombre]',
      runs: runLength(values),
    },
    // murs : [x1, y1, x2, y2] ; objets fixes (meubles) : [x1, y1, x2, y2, "objet"]
    segments: ref.segments.map((s) => [s.x1, s.y1, s.x2, s.y2].map(round).concat(s.kind === 'objet' ? ['objet'] : [])),
    corners: ref.corners.map((c) => [round(c.x), round(c.y), Math.round(c.theta * 10) / 10]),
    scans: ref.scans.map((s) => ({ name: s.name, pose: [round(s.pose.x), round(s.pose.y), Math.round(s.pose.theta * 100) / 100] })),
  });
}

/** Relit une carte de référence enregistrée par serializeReferenceMap. */
export function parseReferenceMap(text) {
  const d = JSON.parse(text);
  if (d.format !== REFMAP_FORMAT) throw new Error("Ce fichier n'est pas une carte de référence ld06-slam");
  const gd = d.grid;
  const grid = new GridMap({ resolution: gd.resolution, offI: gd.offI, offJ: gd.offJ, width: gd.width, height: gd.height });
  const values = unRunLength(gd.runs, gd.width * gd.height);
  const { lMin, lMax } = grid.params;
  for (let k = 0; k < values.length; k++) {
    const v = values[k];
    if (v === 128) continue; // inconnu : log-odds 0
    const p = Math.min(0.999, Math.max(0.001, v / 255));
    grid.logOdds[k] = Math.min(lMax, Math.max(lMin, Math.log(p / (1 - p))));
  }
  return makeReferenceMap({
    name: d.name,
    date: d.date,
    clean: d.clean,
    grid,
    segments: d.segments.map(([x1, y1, x2, y2, kind]) => ({ x1, y1, x2, y2, kind })),
    corners: (d.corners ?? []).map(([x, y, theta]) => ({ x, y, theta })),
    scans: (d.scans ?? []).map((s) => ({ name: s.name, pose: { x: s.pose[0], y: s.pose[1], theta: s.pose[2] } })),
  });
}

/** Bornes {minX, minY, maxX, maxY} des cellules connues de la carte de référence. */
export function referenceBounds(ref, margin = 0.3) {
  const g = ref.grid;
  let i0 = Infinity;
  let j0 = Infinity;
  let i1 = -Infinity;
  let j1 = -Infinity;
  for (let j = 0; j < g.height; j++) {
    for (let i = 0; i < g.width; i++) {
      if (g.classify(g.index(i, j)) === 0) continue;
      if (i < i0) i0 = i;
      if (j < j0) j0 = j;
      if (i > i1) i1 = i;
      if (j > j1) j1 = j;
    }
  }
  if (i0 === Infinity) return { minX: g.originX, minY: g.originY, maxX: g.originX + g.width * g.res, maxY: g.originY + g.height * g.res };
  return {
    minX: g.originX + i0 * g.res - margin,
    minY: g.originY + j0 * g.res - margin,
    maxX: g.originX + (i1 + 1) * g.res + margin,
    maxY: g.originY + (j1 + 1) * g.res + margin,
  };
}
