// Scan matching : trouver la pose (x, y, θ) qui superpose un scan à une référence
// (nuage de points des scans déjà placés).
//
// Trois étages, du grossier au fin :
//  1. recherche exhaustive multi-résolution sur un champ de vraisemblance (distance de
//     chaque cellule au point de référence le plus proche) : robuste à une mauvaise
//     pose initiale (jusqu'à quelques mètres et un demi-tour) ;
//  2. recherche exhaustive fine autour des meilleurs candidats ;
//  3. ICP point à point : précision sub-centimétrique.

import { DEG, normalizeAngleDeg } from './pose.js';

/**
 * Champ de distances (« likelihood field ») : une grille qui donne, pour chaque cellule, la
 * distance (m) au point de référence le plus proche, bornée à maxDist.
 * Pourquoi : pour noter une pose, il faut la distance de chaque point du scan à la référence.
 * La chercher parmi des milliers de points de référence à chaque fois serait trop lent ; on la
 * précalcule une fois sur une grille, puis chaque évaluation n'est plus qu'une lecture de case.
 * Calcul (transformée de distance de chanfrein, deux passes) :
 *   1. cellules contenant un point de référence : 0 ; les autres : maxDist ;
 *   2. passe avant (bas-gauche -> haut-droite) : chaque cellule prend le minimum de sa valeur
 *      et de (voisin déjà vu + distance au voisin : rés pour un voisin droit, rés·√2 en diagonale) ;
 *   3. passe arrière (sens inverse) avec les voisins de l'autre côté.
 * Deux passes suffisent à propager la distance dans toutes les directions (approximation à
 * quelques % près de la distance euclidienne, largement suffisante ici).
 */
export class DistanceField {
  constructor(points, { resolution = 0.05, margin = 1, maxDist = 1 } = {}) {
    let minX = Infinity;
    let minY = Infinity;
    let maxX = -Infinity;
    let maxY = -Infinity;
    for (const p of points) {
      if (p.x < minX) minX = p.x;
      if (p.y < minY) minY = p.y;
      if (p.x > maxX) maxX = p.x;
      if (p.y > maxY) maxY = p.y;
    }
    this.res = resolution;
    this.x0 = minX - margin;
    this.y0 = minY - margin;
    this.nx = Math.ceil((maxX - minX + 2 * margin) / resolution) + 1;
    this.ny = Math.ceil((maxY - minY + 2 * margin) / resolution) + 1;
    this.maxDist = maxDist;
    const d = new Float32Array(this.nx * this.ny).fill(maxDist);
    for (const p of points) d[this.cell(p.x, p.y)] = 0;
    const r = resolution;
    const rd = resolution * Math.SQRT2;
    const { nx, ny } = this;
    for (let j = 0; j < ny; j++) {
      for (let i = 0; i < nx; i++) {
        const k = j * nx + i;
        let v = d[k];
        if (i > 0) v = Math.min(v, d[k - 1] + r);
        if (j > 0) {
          v = Math.min(v, d[k - nx] + r);
          if (i > 0) v = Math.min(v, d[k - nx - 1] + rd);
          if (i < nx - 1) v = Math.min(v, d[k - nx + 1] + rd);
        }
        d[k] = v;
      }
    }
    for (let j = ny - 1; j >= 0; j--) {
      for (let i = nx - 1; i >= 0; i--) {
        const k = j * nx + i;
        let v = d[k];
        if (i < nx - 1) v = Math.min(v, d[k + 1] + r);
        if (j < ny - 1) {
          v = Math.min(v, d[k + nx] + r);
          if (i < nx - 1) v = Math.min(v, d[k + nx + 1] + rd);
          if (i > 0) v = Math.min(v, d[k + nx - 1] + rd);
        }
        d[k] = Math.min(v, maxDist);
      }
    }
    this.dist = d;
  }

  cell(x, y) {
    return Math.floor((y - this.y0) / this.res) * this.nx + Math.floor((x - this.x0) / this.res);
  }

  /** Distance interpolée (bilinéaire) au point (x, y) : continue, donc dérivable (descente de gradient). */
  at(x, y) {
    const fx = (x - this.x0) / this.res - 0.5;
    const fy = (y - this.y0) / this.res - 0.5;
    const i = Math.floor(fx);
    const j = Math.floor(fy);
    if (i < 0 || j < 0 || i >= this.nx - 1 || j >= this.ny - 1) return this.maxDist;
    const a = fx - i;
    const b = fy - j;
    const k = j * this.nx + i;
    const d = this.dist;
    return (1 - b) * ((1 - a) * d[k] + a * d[k + 1]) + b * ((1 - a) * d[k + this.nx] + a * d[k + this.nx + 1]);
  }

  /** Table de score par cellule : exp(-d² / 2σ²), 0 hors du champ. */
  scoreTable(sigma) {
    const t = new Float32Array(this.dist.length);
    const k = 1 / (2 * sigma * sigma);
    for (let i = 0; i < t.length; i++) t[i] = Math.exp(-this.dist[i] * this.dist[i] * k);
    return t;
  }
}

/** Sous-échantillonnage régulier à au plus n points. */
export function subsample(points, n) {
  if (points.length <= n) return points;
  const step = points.length / n;
  const out = [];
  for (let k = 0; k < n; k++) out.push(points[Math.floor(k * step)]);
  return out;
}

/**
 * Recherche exhaustive : teste toutes les poses de la fenêtre et renvoie, pour chaque angle,
 * la meilleure translation. Score = moyenne de exp(-d²/2σ²) sur les points du scan.
 */
export function exhaustive(field, table, pts, center, { xyRange, xyStep, thRange, thStep }) {
  const { x0, y0, res, nx, ny } = field;
  const inv = 1 / res;
  const n = pts.length;
  const rx = new Float64Array(n);
  const ry = new Float64Array(n);
  const steps = Math.round(xyRange / xyStep);
  const perAngle = [];
  for (let dth = -thRange; dth <= thRange + 1e-9; dth += thStep) {
    const th = center.theta + dth;
    const c = Math.cos(th * DEG);
    const s = Math.sin(th * DEG);
    for (let k = 0; k < n; k++) {
      rx[k] = c * pts[k].x - s * pts[k].y;
      ry[k] = s * pts[k].x + c * pts[k].y;
    }
    let best = { score: -1 };
    for (let a = -steps; a <= steps; a++) {
      const x = center.x + a * xyStep;
      for (let b = -steps; b <= steps; b++) {
        const y = center.y + b * xyStep;
        let sc = 0;
        for (let k = 0; k < n; k++) {
          const i = Math.floor((x + rx[k] - x0) * inv);
          const j = Math.floor((y + ry[k] - y0) * inv);
          if (i >= 0 && j >= 0 && i < nx && j < ny) sc += table[j * nx + i];
        }
        if (sc > best.score) best = { score: sc, pose: { x, y, theta: normalizeAngleDeg(th) } };
      }
    }
    best.score /= n;
    perAngle.push(best);
  }
  return perAngle;
}

/** Meilleurs candidats distincts (non-maximum suppression sur la pose). */
export function topCandidates(list, count, minDist = 0.3, minAngle = 8) {
  const sorted = [...list].sort((a, b) => b.score - a.score);
  const out = [];
  for (const c of sorted) {
    const near = out.some(
      (o) =>
        Math.hypot(o.pose.x - c.pose.x, o.pose.y - c.pose.y) < minDist &&
        Math.abs(normalizeAngleDeg(o.pose.theta - c.pose.theta)) < minAngle,
    );
    if (!near) out.push(c);
    if (out.length === count) break;
  }
  return out;
}

/** Index spatial des points de référence pour la recherche du plus proche voisin. */
export class PointHash {
  constructor(points, cell) {
    this.cell = cell;
    this.points = points;
    this.map = new Map();
    points.forEach((p, k) => {
      const key = this.key(Math.floor(p.x / cell), Math.floor(p.y / cell));
      if (!this.map.has(key)) this.map.set(key, []);
      this.map.get(key).push(k);
    });
  }

  key(i, j) {
    return i * 100003 + j;
  }

  nearest(x, y, maxDist) {
    const ci = Math.floor(x / this.cell);
    const cj = Math.floor(y / this.cell);
    let best = null;
    let bd = maxDist * maxDist;
    for (let di = -1; di <= 1; di++) {
      for (let dj = -1; dj <= 1; dj++) {
        for (const k of this.map.get(this.key(ci + di, cj + dj)) ?? []) {
          const p = this.points[k];
          const d = (p.x - x) ** 2 + (p.y - y) ** 2;
          if (d < bd) {
            bd = d;
            best = p;
          }
        }
      }
    }
    return best;
  }
}

/**
 * ICP point à point (Iterative Closest Point).
 * À chaque itération :
 *   1. association : chaque point du scan (transformé avec la pose courante) est apparié au
 *      point de référence le plus proche, s'il est à moins de `gate` ; `gate` diminue de
 *      maxDist à minDist au fil des itérations (on accepte d'abord des couples lointains pour
 *      rattraper l'erreur, puis on ne garde que les couples sûrs) ;
 *   2. transformation optimale en forme close : on cherche la rotation R(dθ) et la translation t
 *      qui minimisent Σ |R·Pk + t − Qk|² (Pk : point du scan, Qk : son correspondant).
 *      Avec les barycentres P̄, Q̄ et les écarts a = Pk − P̄, b = Qk − Q̄ :
 *        dθ = atan2( Σ (ax·by − ay·bx), Σ (ax·bx + ay·by) )   et   t = Q̄ − R(dθ)·P̄
 *      (le numérateur mesure la « torsion » moyenne entre les deux nuages, le dénominateur
 *      leur alignement) ;
 *   3. on compose cette correction avec la pose courante et on recommence jusqu'à ce que la
 *      pose ne bouge plus.
 * Précis et rapide, mais local : il faut partir assez près de la bonne pose.
 */
export function icp(refPoints, localPts, init, { iterations = 40, maxDist = 0.3, minDist = 0.08, hash = null } = {}) {
  const index = hash ?? new PointHash(refPoints, maxDist);
  let pose = { ...init };
  let pairs = 0;
  let rms = Infinity;
  for (let it = 0; it < iterations; it++) {
    const gate = Math.max(minDist, maxDist * (1 - it / iterations));
    const c = Math.cos(pose.theta * DEG);
    const s = Math.sin(pose.theta * DEG);
    const P = [];
    const Q = [];
    for (const p of localPts) {
      const wx = pose.x + c * p.x - s * p.y;
      const wy = pose.y + s * p.x + c * p.y;
      const q = index.nearest(wx, wy, gate);
      if (q) {
        P.push([wx, wy]);
        Q.push([q.x, q.y]);
      }
    }
    pairs = P.length;
    if (pairs < 10) break;
    let px = 0;
    let py = 0;
    let qx = 0;
    let qy = 0;
    for (let k = 0; k < pairs; k++) {
      px += P[k][0];
      py += P[k][1];
      qx += Q[k][0];
      qy += Q[k][1];
    }
    px /= pairs;
    py /= pairs;
    qx /= pairs;
    qy /= pairs;
    let sxx = 0;
    let sxy = 0;
    let syx = 0;
    let syy = 0;
    let sq = 0;
    for (let k = 0; k < pairs; k++) {
      const ax = P[k][0] - px;
      const ay = P[k][1] - py;
      const bx = Q[k][0] - qx;
      const by = Q[k][1] - qy;
      sxx += ax * bx;
      sxy += ax * by;
      syx += ay * bx;
      syy += ay * by;
      sq += (P[k][0] - Q[k][0]) ** 2 + (P[k][1] - Q[k][1]) ** 2;
    }
    rms = Math.sqrt(sq / pairs);
    const dth = Math.atan2(sxy - syx, sxx + syy);
    const cd = Math.cos(dth);
    const sd = Math.sin(dth);
    // incrément : rotation dth autour du barycentre du scan, puis translation vers celui de la référence
    const tx = qx - (cd * px - sd * py);
    const ty = qy - (sd * px + cd * py);
    const next = {
      x: cd * pose.x - sd * pose.y + tx,
      y: sd * pose.x + cd * pose.y + ty,
      theta: normalizeAngleDeg(pose.theta + dth / DEG),
    };
    const moved = Math.hypot(next.x - pose.x, next.y - pose.y) + Math.abs(dth);
    pose = next;
    if (moved < 1e-5 && gate === minDist) break;
  }
  return { pose, pairs, rms };
}

export const DEFAULT_MATCH = Object.freeze({
  xyRange: 2.5, // m : fenêtre de recherche autour de la pose initiale
  thRange: 180, // degrés : ±180 = orientation inconnue
  coarseStep: 0.1, // m
  coarseAngle: 2, // degrés
  coarsePoints: 150, // points du scan utilisés à l'étage grossier
  candidates: 5,
  inlierDist: 0.1, // m : un point est « sur la carte » s'il est à moins de 10 cm d'un point de référence
});

/**
 * Recale un scan (points dans son repère local) sur des points de référence (repère monde).
 * Retourne { pose, score, inliers (proportion de points à moins de inlierDist), rms, time }.
 */
export function matchScan(refPoints, localPts, init, options = {}) {
  const o = { ...DEFAULT_MATCH, ...options };
  const t0 = performance.now();
  // 1. grossier : champ à 10 cm, sigma large, points sous-échantillonnés
  const coarseStep = Math.max(o.coarseStep, o.xyRange / 30);
  const coarseField = new DistanceField(refPoints, { resolution: coarseStep, margin: 1, maxDist: 1 });
  const coarse = exhaustive(coarseField, coarseField.scoreTable(Math.max(0.15, 1.5 * coarseStep)), subsample(localPts, o.coarsePoints), init, {
    xyRange: o.xyRange,
    xyStep: coarseStep,
    thRange: o.thRange,
    thStep: o.coarseAngle,
  });
  // 2. fin : champ à 5 cm autour des meilleurs candidats ; 3. ICP
  const fine = new DistanceField(refPoints, { resolution: 0.05, margin: 0.5, maxDist: 0.5 });
  const fineTable = fine.scoreTable(0.05);
  const hash = new PointHash(refPoints, 0.3);
  let best = null;
  for (const cand of topCandidates(coarse, o.candidates)) {
    const [refined] = topCandidates(
      exhaustive(fine, fineTable, localPts, cand.pose, { xyRange: 1.5 * coarseStep, xyStep: 0.025, thRange: o.coarseAngle, thStep: 0.5 }),
      1,
    );
    const r = icp(refPoints, localPts, refined.pose, { hash });
    const ev = evaluate(hash, localPts, r.pose, o.inlierDist);
    if (!best || ev.inliers > best.inliers) best = { pose: r.pose, rms: r.rms, ...ev };
  }
  best.time = performance.now() - t0;
  return best;
}

/** Proportion de points du scan à moins de d d'un point de référence, et score moyen. */
function evaluate(hash, localPts, pose, d) {
  const c = Math.cos(pose.theta * DEG);
  const s = Math.sin(pose.theta * DEG);
  let n = 0;
  for (const p of localPts) {
    if (hash.nearest(pose.x + c * p.x - s * p.y, pose.y + s * p.x + c * p.y, d)) n++;
  }
  return { inliers: n / localPts.length };
}

const toWorld = (pts, pose) => {
  const c = Math.cos(pose.theta * DEG);
  const s = Math.sin(pose.theta * DEG);
  return pts.map((p) => ({ x: pose.x + c * p.x - s * p.y, y: pose.y + s * p.x + c * p.y }));
};

/**
 * Cohérence d'un ensemble de scans placés : pour chaque scan, proportion de ses points
 * à moins de d d'un point des autres scans. Sert d'indicateur de qualité du recalage.
 */
export function consistency(localSets, poses, d = 0.1) {
  const worlds = localSets.map((pts, k) => toWorld(pts, poses[k]));
  return worlds.map((pts, k) => {
    const others = worlds.flatMap((w, j) => (j === k ? [] : w));
    if (!others.length || !pts.length) return 0;
    const hash = new PointHash(others, d);
    return pts.filter((p) => hash.nearest(p.x, p.y, d)).length / pts.length;
  });
}

/** Réglages de l'affinage local, autour d'une pose déjà approximativement juste. */
export const LOCAL_MATCH = Object.freeze({ xyRange: 0.5, thRange: 15, coarseStep: 0.05, coarseAngle: 1 });

/**
 * Recalage automatique d'un ensemble de scans (le premier reste fixe et définit le repère).
 *  1. Les scans encore en (0, 0, 0) sont placés par recherche globale (fenêtre de 2,5 m,
 *     orientation quelconque) sur les scans déjà placés, en partant de la pose du précédent.
 *  2. Tous les scans sont ensuite affinés localement (±0,5 m, ±15°) face à tous les autres,
 *     en deux passes : une pose posée à la main sert de point de départ.
 * Générateur : produit { phase, k, total } à chaque étape (pour afficher la progression)
 * et retourne les poses recalées.
 */
export function* autoAlign(items, options = {}) {
  const poses = items.map((i) => ({ ...i.pose }));
  const isZero = (p) => p.x === 0 && p.y === 0 && p.theta === 0;
  const n = items.length;
  for (let k = 1; k < n; k++) {
    if (!isZero(poses[k])) continue;
    yield { phase: 'placement', k, total: n };
    const ref = items.slice(0, k).flatMap((it, j) => toWorld(it.points, poses[j]));
    poses[k] = matchScan(ref, items[k].points, poses[k - 1], options).pose;
  }
  for (let pass = 0; pass < 2; pass++) {
    for (let k = 1; k < n; k++) {
      yield { phase: 'affinage', k, total: n, pass };
      const ref = items.flatMap((it, j) => (j === k ? [] : toWorld(it.points, poses[j])));
      poses[k] = matchScan(ref, items[k].points, poses[k], { ...options, ...LOCAL_MATCH }).pose;
    }
  }
  return poses;
}

/**
 * Recale une suite de scans dans l'ordre : le premier reste fixe, chaque suivant est recalé
 * sur l'ensemble des scans déjà placés, en partant de la pose du précédent (le robot a peu bougé).
 * items : [{ points: points locaux, pose: pose initiale }] ; onProgress(k, résultat) facultatif.
 * Retourne les résultats (pose recalée, inliers, ...) dans le même ordre.
 */
export function alignSequence(items, options = {}, onProgress = () => {}) {
  const results = [{ pose: { ...items[0].pose }, inliers: 1, rms: 0, time: 0 }];
  const world = [];
  const addWorld = (pts, pose) => {
    const c = Math.cos(pose.theta * DEG);
    const s = Math.sin(pose.theta * DEG);
    for (const p of pts) world.push({ x: pose.x + c * p.x - s * p.y, y: pose.y + s * p.x + c * p.y });
  };
  addWorld(items[0].points, results[0].pose);
  onProgress(0, results[0]);
  for (let k = 1; k < items.length; k++) {
    const init = options.useOwnPose ? items[k].pose : results[k - 1].pose;
    const r = matchScan(world, items[k].points, init, options);
    results.push(r);
    addWorld(items[k].points, r.pose);
    onProgress(k, r);
  }
  return results;
}
