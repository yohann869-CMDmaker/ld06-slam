// Transformation d'un nuage de points lidar en segments.
//
// 1. Segmentation (clusters) : on parcourt les points dans l'ordre angulaire et on
//    coupe quand la distance entre deux points successifs dépasse un seuil adaptatif
//    (Borges & Aldon) : D = r * sin(dθ) / sin(λ - dθ) + 3σ.
//    Il grandit avec la distance, car deux points voisins d'un même mur sont plus
//    espacés loin du robot. λ est l'incidence la plus rasante encore acceptée sur un
//    même objet, σ le bruit de distance.
// 2. Split (Douglas-Peucker) : un cluster est coupé récursivement au point le plus
//    éloigné de sa corde tant que cet écart dépasse splitDistance (coins, objets accolés).
// 3. Ajustement : droite des moindres carrés totaux (ACP) sur chaque morceau,
//    extrémités = projections du premier et du dernier point.
// 4. Merge : deux morceaux consécutifs d'un même cluster qui tiennent sur une seule
//    droite (écart max <= splitDistance) sont refusionnés.

import { DEG } from './pose.js';

export const DEFAULT_SEGMENTATION = Object.freeze({
  lambdaDeg: 10, // incidence minimale (degrés) pour rester sur le même objet
  sigma: 0.01, // bruit de distance du lidar (m)
  minBreak: 0.05, // seuil de coupure minimal (m)
  maxGapAngleDeg: 5, // trou angulaire (points manquants) au-delà duquel on coupe toujours
  splitDistance: 0.03, // m
  minPoints: 5, // points minimum pour un segment
  minLength: 0.15, // m
});

/**
 * Ajustement d'une droite par moindres carrés totaux : minimise la somme des
 * distances perpendiculaires (contrairement à y = ax + b, valable aussi pour un mur vertical).
 * Retourne le centre, la direction unitaire et l'écart quadratique moyen.
 */
export function fitLine(points) {
  const n = points.length;
  let cx = 0;
  let cy = 0;
  for (const p of points) {
    cx += p.x;
    cy += p.y;
  }
  cx /= n;
  cy /= n;
  let sxx = 0;
  let syy = 0;
  let sxy = 0;
  for (const p of points) {
    const dx = p.x - cx;
    const dy = p.y - cy;
    sxx += dx * dx;
    syy += dy * dy;
    sxy += dx * dy;
  }
  const phi = 0.5 * Math.atan2(2 * sxy, sxx - syy); // direction de plus grande dispersion
  const ux = Math.cos(phi);
  const uy = Math.sin(phi);
  let sq = 0;
  let maxDev = 0;
  for (const p of points) {
    const d = Math.abs(-(p.x - cx) * uy + (p.y - cy) * ux);
    sq += d * d;
    if (d > maxDev) maxDev = d;
  }
  return { cx, cy, ux, uy, rms: Math.sqrt(sq / n), maxDev };
}

/**
 * Découpe une liste de points ordonnés par angle en clusters (objets distincts).
 * Si un seul cluster fait le tour complet sans coupure (pièce fermée vue en entier),
 * il porte la propriété `closed = true`.
 */
export function clusterize(points, options = {}) {
  const o = { ...DEFAULT_SEGMENTATION, ...options };
  if (points.length === 0) return [];
  const lambda = o.lambdaDeg * DEG;
  const breaks = (prev, p, dThetaDeg) => {
    const dTheta = dThetaDeg * DEG;
    if (dThetaDeg > o.maxGapAngleDeg) return true;
    const dMax = Math.max(o.minBreak, (prev.r * Math.sin(dTheta)) / Math.sin(lambda - dTheta) + 3 * o.sigma);
    return Math.hypot(p.x - prev.x, p.y - prev.y) > dMax;
  };
  const clusters = [[points[0]]];
  for (let i = 1; i < points.length; i++) {
    const prev = points[i - 1];
    const p = points[i];
    if (breaks(prev, p, p.angle - prev.angle)) clusters.push([p]);
    else clusters.at(-1).push(p);
  }
  // Le scan fait le tour complet : le dernier cluster peut continuer dans le premier.
  const last = clusters.at(-1);
  const first = clusters[0];
  const wraps = !breaks(last.at(-1), first[0], first[0].angle + 360 - last.at(-1).angle);
  if (wraps && clusters.length > 1) {
    clusters[0] = last.concat(first);
    clusters.pop();
  } else if (wraps) {
    first.closed = true;
  }
  return clusters;
}

function chordDistance(a, b, p) {
  const l = Math.hypot(b.x - a.x, b.y - a.y);
  if (l === 0) return Math.hypot(p.x - a.x, p.y - a.y);
  return Math.abs((b.x - a.x) * (a.y - p.y) - (a.x - p.x) * (b.y - a.y)) / l;
}

/** Split récursif : retourne des intervalles d'indices [lo, hi] (inclus). */
function split(pts, lo, hi, threshold, out) {
  let kMax = -1;
  let dMax = 0;
  for (let k = lo + 1; k < hi; k++) {
    const d = chordDistance(pts[lo], pts[hi], pts[k]);
    if (d > dMax) {
      dMax = d;
      kMax = k;
    }
  }
  if (dMax > threshold && kMax > 0) {
    split(pts, lo, kMax, threshold, out);
    split(pts, kMax, hi, threshold, out);
  } else {
    out.push([lo, hi]);
  }
}

function segmentFromFit(pts, fit) {
  const proj = (p) => (p.x - fit.cx) * fit.ux + (p.y - fit.cy) * fit.uy;
  const t1 = proj(pts[0]);
  const t2 = proj(pts.at(-1));
  return {
    x1: fit.cx + t1 * fit.ux,
    y1: fit.cy + t1 * fit.uy,
    x2: fit.cx + t2 * fit.ux,
    y2: fit.cy + t2 * fit.uy,
    weight: pts.length,
    rms: fit.rms,
  };
}

/**
 * Nuage de points (ordonné par angle, avec r et angle dans le repère du robot,
 * x et y dans n'importe quel repère) -> segments dans le repère des x, y.
 * Retourne { segments, clusters }.
 */
export function extractSegments(points, options = {}) {
  const o = { ...DEFAULT_SEGMENTATION, ...options };
  const clusters = clusterize(points, o);
  const segments = [];
  for (const cluster of clusters) {
    if (cluster.length < o.minPoints) continue;
    let c = cluster;
    let ranges = [];
    split(c, 0, c.length - 1, o.splitDistance, ranges);
    // Boucle fermée : son début (angle 0°) tombe n'importe où, souvent au milieu d'un mur
    // qui serait alors coupé en deux. On la fait recommencer sur le premier coin trouvé.
    if (cluster.closed && ranges.length > 1) {
      const k = ranges[0][1];
      c = cluster.slice(k).concat(cluster.slice(0, k + 1));
      ranges = [];
      split(c, 0, c.length - 1, o.splitDistance, ranges);
    }
    // merge des morceaux consécutifs qui tiennent sur une même droite
    const merged = [ranges[0]];
    for (let k = 1; k < ranges.length; k++) {
      const prev = merged.at(-1);
      const union = [prev[0], ranges[k][1]];
      if (fitLine(c.slice(union[0], union[1] + 1)).maxDev <= o.splitDistance) merged[merged.length - 1] = union;
      else merged.push(ranges[k]);
    }
    for (const [lo, hi] of merged) {
      const pts = c.slice(lo, hi + 1);
      if (pts.length < o.minPoints) continue;
      const s = segmentFromFit(pts, fitLine(pts));
      if (Math.hypot(s.x2 - s.x1, s.y2 - s.y1) >= o.minLength) segments.push(s);
    }
  }
  return { segments, clusters };
}
