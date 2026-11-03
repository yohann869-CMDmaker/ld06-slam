// Localisation d'un scan sur une référence (document « Scan matching » du cours) :
//   - trois références : grid map, carte vectorielle, scan de référence ;
//   - plusieurs méthodes d'optimisation : force brute, multi-résolution, descente de
//     gradient, probabiliste (filtre particulaire), ICP, et un calcul direct par moindres
//     carrés point-droite (généralisation de l'algorithme de localisation absolue « Mars
//     Attacks! » : régression sur les points proches des murs, en passes successives à marge
//     décroissante).
// Toutes partent d'une pose initiale (x, y, θ) et renvoient la pose estimée.

import { DEG, normalizeAngleDeg } from './pose.js';
import { DistanceField, exhaustive, icp, subsample, topCandidates } from './matching.js';
import { length } from './geometry.js';
import { gaussian, mulberry32 } from './sim.js';

// ------------------------------------------------------------------ références

/** Référence commune : des points (monde) et des champs de distance mis en cache par résolution. */
function makeReference(kind, points, extra = {}) {
  const fields = new Map();
  return {
    kind,
    points,
    ...extra,
    field(res = 0.025) {
      if (!fields.has(res)) fields.set(res, new DistanceField(points, { resolution: res, margin: 1.5, maxDist: 1 }));
      return fields.get(res);
    },
  };
}

/** Grid map : les centres des cellules occupées. */
export function gridReference(grid) {
  return makeReference('grid', grid.occupiedCells(), { grid });
}

/** Carte vectorielle : les segments, échantillonnés tous les 2 cm pour les méthodes à base de points. */
export function vectorReference(segments, step = 0.02) {
  const points = [];
  for (const s of segments) {
    const n = Math.max(1, Math.ceil(length(s) / step));
    for (let k = 0; k <= n; k++) points.push({ x: s.x1 + ((s.x2 - s.x1) * k) / n, y: s.y1 + ((s.y2 - s.y1) * k) / n });
  }
  return makeReference('vector', points, { segments });
}

/** Scan de référence : ses points dans le repère monde. */
export function scanReference(worldPoints) {
  return makeReference('scan', worldPoints);
}

// ------------------------------------------------------------------ score

const toWorld = (p, pose, c = Math.cos(pose.theta * DEG), s = Math.sin(pose.theta * DEG)) => ({
  x: pose.x + c * p.x - s * p.y,
  y: pose.y + s * p.x + c * p.y,
});

/** Score d'une pose : moyenne de exp(-d²/2σ²), d = distance interpolée à la référence. */
export function scorePose(field, pts, pose, sigma) {
  const c = Math.cos(pose.theta * DEG);
  const s = Math.sin(pose.theta * DEG);
  const k = 1 / (2 * sigma * sigma);
  let sum = 0;
  for (const p of pts) {
    const d = field.at(pose.x + c * p.x - s * p.y, pose.y + s * p.x + c * p.y);
    sum += Math.exp(-d * d * k);
  }
  return sum / pts.length;
}

/** Proportion des points du scan à moins de `tol` de la référence (indicateur de réussite). */
export function inlierRatio(ref, pts, pose, tol = 0.05) {
  const f = ref.field(0.025);
  const c = Math.cos(pose.theta * DEG);
  const s = Math.sin(pose.theta * DEG);
  let n = 0;
  for (const p of pts) if (f.at(pose.x + c * p.x - s * p.y, pose.y + s * p.x + c * p.y) <= tol) n++;
  return n / pts.length;
}

// ------------------------------------------------------------------ méthodes

export const WINDOW = Object.freeze({ xyRange: 0.5, thRange: 20 }); // fenêtre de recherche autour de la pose initiale

/** Force brute : toutes les poses d'une grille (2 cm, 0,5°) dans la fenêtre, une seule résolution. */
function bruteForce(ref, pts, init, o) {
  const field = ref.field(0.025);
  const res = exhaustive(field, field.scoreTable(0.05), subsample(pts, 200), init, {
    xyRange: o.xyRange,
    xyStep: 0.02,
    thRange: o.thRange,
    thStep: 0.5,
  });
  return { pose: topCandidates(res, 1)[0].pose };
}

/** Multi-résolution : grille grossière (10 cm, 2°, champ lissé) puis fine (1 cm, 0,25°) autour des 3 meilleurs. */
function multiResolution(ref, pts, init, o) {
  const coarse = ref.field(0.1);
  const cands = topCandidates(
    exhaustive(coarse, coarse.scoreTable(0.15), subsample(pts, 120), init, { xyRange: o.xyRange, xyStep: 0.1, thRange: o.thRange, thStep: 2 }),
    3,
  );
  const fine = ref.field(0.025);
  const table = fine.scoreTable(0.04);
  let best = null;
  for (const c of cands) {
    const [r] = topCandidates(exhaustive(fine, table, pts, c.pose, { xyRange: 0.12, xyStep: 0.01, thRange: 2.5, thStep: 0.25 }), 1);
    if (!best || r.score > best.score) best = r;
  }
  return { pose: best.pose };
}

/**
 * Descente de gradient (montée sur le score) : gradient numérique par différences centrées
 * sur le champ de distance interpolé, pas adaptatif ; σ décroissant (0,15 puis 0,05 m) pour
 * élargir le bassin de convergence au début puis gagner en précision.
 */
function gradient(ref, pts, init, o) {
  const field = ref.field(0.025);
  const sample = subsample(pts, 250);
  let pose = { ...init };
  let evaluations = 0;
  for (const sigma of [0.15, 0.08, 0.04]) {
    let step = { xy: 0.05, th: 2 };
    let score = scorePose(field, sample, pose, sigma);
    for (let it = 0; it < (o.iterations ?? 60); it++) {
      const h = { xy: 0.002, th: 0.1 };
      const g = {
        x: (scorePose(field, sample, { ...pose, x: pose.x + h.xy }, sigma) - scorePose(field, sample, { ...pose, x: pose.x - h.xy }, sigma)) / (2 * h.xy),
        y: (scorePose(field, sample, { ...pose, y: pose.y + h.xy }, sigma) - scorePose(field, sample, { ...pose, y: pose.y - h.xy }, sigma)) / (2 * h.xy),
        t: (scorePose(field, sample, { ...pose, theta: pose.theta + h.th }, sigma) - scorePose(field, sample, { ...pose, theta: pose.theta - h.th }, sigma)) / (2 * h.th),
      };
      evaluations += 6;
      const gxy = Math.hypot(g.x, g.y) || 1;
      const gt = Math.abs(g.t) || 1;
      const next = { x: pose.x + (step.xy * g.x) / gxy, y: pose.y + (step.xy * g.y) / gxy, theta: pose.theta + (step.th * g.t) / gt };
      const s = scorePose(field, sample, next, sigma);
      evaluations++;
      if (s > score) {
        pose = next;
        score = s;
        step = { xy: step.xy * 1.2, th: step.th * 1.2 };
      } else {
        step = { xy: step.xy * 0.5, th: step.th * 0.5 };
        if (step.xy < 2e-4 && step.th < 0.01) break;
      }
    }
  }
  return { pose: { ...pose, theta: normalizeAngleDeg(pose.theta) }, evaluations };
}

/**
 * Filtre particulaire (approche probabiliste) : des hypothèses de pose tirées autour de la pose
 * initiale sont pondérées par la vraisemblance du scan, rééchantillonnées, puis perturbées par
 * un bruit qui diminue à chaque itération (recuit). Robuste aux minima locaux.
 */
function particles(ref, pts, init, o) {
  const field = ref.field(0.05);
  const sample = subsample(pts, 100);
  const N = o.particles ?? 400;
  const rand = mulberry32(o.seed ?? 12345);
  const gauss = () => gaussian(rand);
  let sx = o.xyRange / 2;
  let st = o.thRange / 2;
  let ps = Array.from({ length: N }, (_, k) =>
    k === 0 ? { ...init } : { x: init.x + gauss() * sx, y: init.y + gauss() * sx, theta: init.theta + gauss() * st },
  );
  let best = null;
  for (let it = 0; it < (o.iterations ?? 15); it++) {
    const sigma = Math.max(0.04, 0.2 * 0.8 ** it);
    const w = ps.map((p) => scorePose(field, sample, p, sigma));
    w.forEach((v, k) => {
      if (!best || v > best.score) best = { score: v, pose: { ...ps[k] } };
    });
    // vraisemblance très piquée : on élève le score à une puissance pour concentrer les tirages
    const weights = w.map((v) => v ** 8);
    const total = weights.reduce((a, b) => a + b, 0) || 1;
    // rééchantillonnage systématique (faible variance)
    const out = [];
    let u = rand() / N;
    let cum = weights[0] / total;
    let k = 0;
    for (let m = 0; m < N; m++) {
      while (u > cum && k < N - 1) cum += weights[++k] / total;
      out.push(ps[k]);
      u += 1 / N;
    }
    sx *= 0.7;
    st *= 0.7;
    ps = out.map((p, m) => (m === 0 ? { ...best.pose } : { x: p.x + gauss() * sx, y: p.y + gauss() * sx, theta: p.theta + gauss() * st }));
  }
  return { pose: { ...best.pose, theta: normalizeAngleDeg(best.pose.theta) }, evaluations: N * (o.iterations ?? 15) };
}

/** ICP point à point sur les points de la référence. */
function icpMethod(ref, pts, init) {
  const r = icp(ref.points, pts, init, { maxDist: 0.3, minDist: 0.05, iterations: 50 });
  return { pose: r.pose };
}

/**
 * Calcul direct sur carte vectorielle (moindres carrés point-droite, résolu en forme close).
 * À chaque passe, chaque point du scan est associé au segment le plus proche s'il en est à
 * moins de la marge ; la correction (dx, dy, dθ) qui annule au mieux les distances
 * perpendiculaires est la solution d'un système linéaire 3 x 3. Les marges décroissent
 * (25, 10, 4 cm) comme le propose l'article « Mars Attacks! » (passes successives à marge
 * décroissante) ; l'article traite le cas d'une table rectangulaire, ici les segments sont quelconques.
 */
function direct(ref, pts, init, o) {
  if (!ref.segments) throw new Error('Le calcul direct demande une carte vectorielle');
  const segs = ref.segments.map((s) => {
    const L = length(s);
    const ux = (s.x2 - s.x1) / L;
    const uy = (s.y2 - s.y1) / L;
    return { ...s, L, ux, uy, nx: -uy, ny: ux };
  });
  let pose = { ...init };
  let used = 0;
  for (const margin of o.margins ?? [0.25, 0.1, 0.04]) {
    for (let it = 0; it < 4; it++) {
      const c = Math.cos(pose.theta * DEG);
      const s = Math.sin(pose.theta * DEG);
      // normale (A) et second membre (b) du système A [dx dy dθ] = b
      const A = [0, 0, 0, 0, 0, 0, 0, 0, 0];
      const b = [0, 0, 0];
      used = 0;
      for (const p of pts) {
        const w = toWorld(p, pose, c, s);
        let bestSeg = null;
        let bestD = margin;
        let bestR = 0;
        for (const g of segs) {
          const t = (w.x - g.x1) * g.ux + (w.y - g.y1) * g.uy;
          if (t < -margin || t > g.L + margin) continue;
          const r = (w.x - g.x1) * g.nx + (w.y - g.y1) * g.ny; // distance signée à la droite
          if (Math.abs(r) < bestD) {
            bestD = Math.abs(r);
            bestSeg = g;
            bestR = r;
          }
        }
        if (!bestSeg) continue;
        // r(dx, dy, dθ) ≈ r + nx dx + ny dy + dθ · n · (−(wy − y0), wx − x0)
        const J = [bestSeg.nx, bestSeg.ny, bestSeg.nx * -(w.y - pose.y) + bestSeg.ny * (w.x - pose.x)];
        for (let i = 0; i < 3; i++) {
          b[i] -= J[i] * bestR;
          for (let j = 0; j < 3; j++) A[3 * i + j] += J[i] * J[j];
        }
        used++;
      }
      if (used < 10) break;
      const d = solve3(A, b);
      if (!d) break;
      pose = { x: pose.x + d[0], y: pose.y + d[1], theta: normalizeAngleDeg(pose.theta + d[2] / DEG) };
      if (Math.hypot(d[0], d[1]) < 1e-5 && Math.abs(d[2]) < 1e-6) break;
    }
  }
  return { pose, used };
}

/** Résolution d'un système 3 x 3 (règle de Cramer), null s'il est singulier (un seul mur vu, par exemple). */
function solve3(A, b) {
  const det = (m) => m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6]) + m[2] * (m[3] * m[7] - m[4] * m[6]);
  const D = det(A);
  if (Math.abs(D) < 1e-12) return null;
  return [0, 1, 2].map((col) => {
    const M = [...A];
    for (let r = 0; r < 3; r++) M[3 * r + col] = b[r];
    return det(M) / D;
  });
}

export const METHODS = Object.freeze({
  brute: { label: 'Force brute', run: bruteForce },
  multires: { label: 'Multi-résolution', run: multiResolution },
  gradient: { label: 'Descente de gradient', run: gradient },
  particles: { label: 'Filtre particulaire (probabiliste)', run: particles },
  icp: { label: 'ICP', run: icpMethod },
  direct: { label: 'Calcul direct (moindres carrés point-droite)', run: direct },
});

/** Méthodes proposées pour chaque type de référence. */
export const METHODS_BY_REFERENCE = Object.freeze({
  grid: ['brute', 'multires', 'gradient', 'particles', 'icp'],
  vector: ['direct', 'brute', 'multires', 'gradient', 'particles', 'icp'],
  scan: ['icp', 'brute', 'multires', 'gradient', 'particles'],
});

/**
 * Localise un scan (points locaux) sur une référence en partant de `init`.
 * Retourne { pose, time (ms), inliers (part des points à moins de 5 cm de la référence) }.
 */
export function localize(ref, localPts, init, method, options = {}) {
  const o = { ...WINDOW, ...options };
  const t0 = performance.now();
  const r = METHODS[method].run(ref, localPts, init, o);
  const time = performance.now() - t0;
  return { ...r, method, time, inliers: inlierRatio(ref, localPts, r.pose) };
}
