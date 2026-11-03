// Localisation globale (jour 2, étape 3) : retrouver la pose d'un scan SANS pose de départ.
//
// Les méthodes de localization.js cherchent autour d'une pose initiale (±50 cm, ±20°). Ici on
// cherche dans toute la salle, en réduisant intelligemment l'espace des poses à tester :
//   1. positions candidates : les cellules de sol de la grid map de référence, à au moins
//      `clearance` d'un obstacle (le robot a une taille), tous les `step` mètres ;
//   2. orientations candidates : les murs d'une salle sont presque tous parallèles ou
//      perpendiculaires. La direction dominante des segments du scan et celle des murs de la
//      carte donnent l'orientation à 90° près : 4 orientations (± quelques degrés) au lieu de
//      360°. Sans direction nette dans le scan, on teste tous les angles ;
//   3. score de chaque pose sur le champ de distance lissé (σ = 15 cm), avec 150 points du scan ;
//   4. les meilleures poses distinctes sont affinées (multi-résolution locale puis ICP) et
//      classées par la part des points du scan à moins de 5 cm de la carte ;
//   5. confiance : on compare la meilleure à la deuxième, distincte d'au moins 50 cm ou 10°, sur
//      les seuls points qui les départagent (sur la carte pour l'une et pas pour l'autre). Les murs
//      d'une salle presque symétrique s'expliquent aussi bien des deux façons : ce sont les meubles
//      qui tranchent. Si la deuxième a au moins moitié autant de « points à elle » que la première
//      (ou si presque aucun point ne départage), la salle est ambiguë depuis ce point de vue
//      (symétrie, couloir) : on le dit au lieu de donner une pose fausse avec assurance.

import { DEG, normalizeAngleDeg } from './pose.js';
import { subsample, topCandidates } from './matching.js';
import { extractSegments } from './segmentation.js';
import { dominantDirection } from './cleanup.js';
import { length, orientationDeg } from './geometry.js';
import { localize } from './localization.js';

export const DEFAULT_GLOBAL = Object.freeze({
  step: 0.2, // m : pas des positions candidates
  clearance: 0.1, // m : distance minimale du centre du robot à un obstacle
  coarseRes: 0.1, // m : résolution du champ de distance pour le tri des candidates
  sigma: 0.15, // m : tolérance du score de tri (lisse, pour ne pas rater la bonne pose entre deux pas)
  points: 150, // points du scan utilisés pour le tri
  useDominant: true, // orientations tirées des directions des murs (sinon tous les angles)
  dominantSpread: 4, // degrés : ± autour des 4 orientations déduites des murs
  angleStep: 1, // degrés : pas autour de ces orientations
  fullAngleStep: 2, // degrés : pas quand on teste tous les angles
  minSupport: 0.25, // part de la longueur des segments du scan alignée sur sa direction dominante
  candidates: 6, // poses distinctes affinées
  minSeparation: 0.5, // m : deux candidates plus proches sont la même
  minSeparationDeg: 10, // degrés
  ambiguity: 0.5, // ambiguë si la deuxième a au moins moitié autant de points à elle que la première
  minEvidence: 8, // ambiguë aussi si moins de 8 points départagent les deux candidates
});

/** Positions candidates : centres des cellules de sol, tous les `step` m, à au moins `clearance` d'un obstacle. */
export function freePositions(grid, step = DEFAULT_GLOBAL.step, clearance = DEFAULT_GLOBAL.clearance) {
  const every = Math.max(1, Math.round(step / grid.res));
  const R = Math.ceil(clearance / grid.res);
  const out = [];
  for (let j = 0; j < grid.height; j += every) {
    for (let i = 0; i < grid.width; i += every) {
      if (grid.classify(grid.index(i, j)) !== -1) continue;
      let clear = true;
      for (let dj = -R; dj <= R && clear; dj++) {
        for (let di = -R; di <= R && clear; di++) {
          if (grid.inside(i + di, j + dj) && grid.classify(grid.index(i + di, j + dj)) === 1) clear = false;
        }
      }
      if (clear) out.push(grid.cellCenter(i, j));
    }
  }
  return out;
}

/** Direction dominante des segments du scan (degrés dans [0, 90[) et part de la longueur qui la suit. */
export function scanDominant(localPts) {
  const segs = extractSegments(localPts).segments;
  const total = segs.reduce((a, s) => a + length(s), 0);
  if (!segs.length || total < 1) return { direction: 0, support: 0 };
  const direction = dominantDirection(segs);
  let aligned = 0;
  for (const s of segs) {
    let d = Math.abs((orientationDeg(s) % 90) - direction);
    d = Math.min(d, 90 - d);
    if (d <= 5) aligned += length(s);
  }
  return { direction, support: aligned / total };
}

/**
 * Orientations à tester. Avec la direction des murs de la carte (mapDominant) et une direction
 * nette dans le scan : θ = mapDominant − scanDominant + k·90° (k = 0..3), ± dominantSpread.
 * Sinon : tous les angles, tous les fullAngleStep degrés.
 */
export function candidateAngles(localPts, mapDominant, options = {}) {
  const o = { ...DEFAULT_GLOBAL, ...options };
  const dom = scanDominant(localPts);
  if (o.useDominant && Number.isFinite(mapDominant) && dom.support >= o.minSupport) {
    const angles = [];
    for (let k = 0; k < 4; k++) {
      for (let d = -o.dominantSpread; d <= o.dominantSpread + 1e-9; d += o.angleStep) angles.push(normalizeAngleDeg(mapDominant - dom.direction + 90 * k + d));
    }
    return { angles, mode: 'murs', scanDirection: dom.direction, support: dom.support };
  }
  const angles = [];
  for (let a = -180; a < 180; a += o.fullAngleStep) angles.push(a);
  return { angles, mode: 'tous', scanDirection: dom.direction, support: dom.support };
}

/**
 * Localisation globale d'un scan (points dans le repère du robot) sur une référence de
 * localization.js. La grid map donne les positions candidates : ref.grid (référence grid map)
 * ou options.grid. options.mapDominant : direction des murs de la carte (degrés), pour réduire
 * les orientations à tester.
 * Retourne { pose, inliers, candidates: [{ pose, inliers }], ambiguous, ratio, evidence, positions,
 * angles, mode, time } ; candidates est trié (distinctes, la meilleure en premier) ; evidence =
 * { best, second } : points sur la carte pour l'une des deux premières candidates et pas pour
 * l'autre ; ratio = evidence.second / evidence.best.
 */
export function globalLocalize(ref, localPts, options = {}) {
  const o = { ...DEFAULT_GLOBAL, ...options };
  const t0 = performance.now();
  const grid = o.grid ?? ref.grid;
  if (!grid) throw new Error('Localisation globale : il faut une grid map pour les positions candidates');
  const positions = freePositions(grid, o.step, o.clearance);
  const { angles, mode } = candidateAngles(localPts, o.mapDominant, o);

  // 3. tri : score de chaque (position, orientation) sur le champ lissé
  const field = ref.field(o.coarseRes);
  const table = field.scoreTable(o.sigma);
  const { x0, y0, res, nx, ny } = field;
  const pts = subsample(localPts, o.points);
  const n = pts.length;
  const rx = new Float64Array(n);
  const ry = new Float64Array(n);
  const scores = new Float32Array(positions.length * angles.length);
  angles.forEach((th, a) => {
    const c = Math.cos(th * DEG);
    const s = Math.sin(th * DEG);
    for (let k = 0; k < n; k++) {
      rx[k] = c * pts[k].x - s * pts[k].y;
      ry[k] = s * pts[k].x + c * pts[k].y;
    }
    positions.forEach((p, q) => {
      let sum = 0;
      for (let k = 0; k < n; k++) {
        const i = Math.floor((p.x + rx[k] - x0) / res);
        const j = Math.floor((p.y + ry[k] - y0) / res);
        if (i >= 0 && j >= 0 && i < nx && j < ny) sum += table[j * nx + i];
      }
      scores[a * positions.length + q] = sum / n;
    });
  });
  // meilleures poses distinctes (parmi les 3000 premières)
  const order = Array.from(scores.keys()).sort((u, v) => scores[v] - scores[u]).slice(0, 3000);
  const coarse = topCandidates(
    order.map((idx) => {
      const a = Math.floor(idx / positions.length);
      const p = positions[idx % positions.length];
      return { score: scores[idx], pose: { x: p.x, y: p.y, theta: angles[a] } };
    }),
    o.candidates,
    o.minSeparation,
    o.minSeparationDeg,
  );

  // 4. affinage : multi-résolution autour de chaque candidate, puis ICP
  const refined = coarse.map((c) => {
    const m = localize(ref, localPts, c.pose, 'multires', { xyRange: 1.5 * o.step, thRange: mode === 'murs' ? o.dominantSpread + 2 : 2 * o.fullAngleStep });
    const r = localize(ref, localPts, m.pose, 'icp');
    const best = r.inliers >= m.inliers ? r : m;
    return { pose: best.pose, inliers: best.inliers, coarse: c.score };
  });
  // deux candidates peuvent converger vers la même pose : on ne garde que des poses distinctes
  refined.sort((u, v) => v.inliers - u.inliers);
  const distinct = [];
  for (const r of refined) {
    const same = distinct.some(
      (d) => Math.hypot(d.pose.x - r.pose.x, d.pose.y - r.pose.y) < o.minSeparation && Math.abs(normalizeAngleDeg(d.pose.theta - r.pose.theta)) < o.minSeparationDeg,
    );
    if (!same) distinct.push(r);
  }
  // 5. confiance : points qui départagent la meilleure et la deuxième
  const [best, second] = distinct;
  let ratio = 0;
  let ambiguous = false;
  let evidence = null;
  if (second) {
    const fine = ref.field(0.025);
    const onMap = (pose) => {
      const c = Math.cos(pose.theta * DEG);
      const s = Math.sin(pose.theta * DEG);
      return localPts.map((p) => fine.at(pose.x + c * p.x - s * p.y, pose.y + s * p.x + c * p.y) <= 0.05);
    };
    const a = onMap(best.pose);
    const b = onMap(second.pose);
    evidence = { best: 0, second: 0 };
    a.forEach((v, k) => {
      if (v && !b[k]) evidence.best++;
      else if (b[k] && !v) evidence.second++;
    });
    ratio = evidence.second / Math.max(evidence.best, 1);
    ambiguous = evidence.best < o.minEvidence || ratio >= o.ambiguity;
  }
  return {
    pose: best.pose,
    inliers: best.inliers,
    candidates: distinct,
    ratio,
    evidence,
    ambiguous,
    positions: positions.length,
    angles: angles.length,
    mode,
    time: performance.now() - t0,
  };
}
