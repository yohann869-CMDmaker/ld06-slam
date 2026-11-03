// Simulateur de scans LD06 dans une scène de segments (même principe que MapTool :
// lancer de rayons contre les murs), avec bruit et défauts réalistes.
// Sert à produire des scans dont la pose vraie est connue (validation, jour 2).

import { composePoses, DEG, invertPose } from './pose.js';

/** Générateur pseudo-aléatoire reproductible (mulberry32). */
export function mulberry32(seed) {
  let a = seed >>> 0;
  return function rng() {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

/** Tirage gaussien centré réduit (Box-Muller). */
export function gaussian(rng) {
  let u = 0;
  while (u === 0) u = rng();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * rng());
}

/** Les 4 côtés d'un rectangle aligné sur les axes. */
export function rectangle(x0, y0, x1, y1) {
  return [
    { x1: x0, y1: y0, x2: x1, y2: y0 },
    { x1: x1, y1: y0, x2: x1, y2: y1 },
    { x1: x1, y1: y1, x2: x0, y2: y1 },
    { x1: x0, y1: y1, x2: x0, y2: y0 },
  ];
}

/**
 * Premier impact du rayon (ox, oy) + t (dx, dy), t > 0, avec une liste de segments.
 * Retourne { t, segment } ou null.
 */
export function rayCast(segments, ox, oy, dx, dy) {
  let best = null;
  for (const s of segments) {
    const ex = s.x2 - s.x1;
    const ey = s.y2 - s.y1;
    const denom = dx * ey - dy * ex;
    if (Math.abs(denom) < 1e-12) continue; // rayon parallèle au segment
    const wx = s.x1 - ox;
    const wy = s.y1 - oy;
    const t = (wx * ey - wy * ex) / denom; // abscisse le long du rayon
    const u = (wx * dy - wy * dx) / denom; // position sur le segment, dans [0, 1]
    if (t > 1e-9 && u >= 0 && u <= 1 && (!best || t < best.t)) best = { t, segment: s };
  }
  return best;
}

export const DEFAULT_SIM = Object.freeze({
  rays: 450, // ~4500 mesures/s à 10 tours/s
  noiseRange: 0.01, // écart-type sur la distance (m), hypothèse
  noiseAngle: 0, // écart-type sur l'angle (degrés)
  maxRange: 12, // au-delà : pas d'impact (r = 0)
  dropRate: 0.01, // proportion de mesures perdues (r = 0)
  seed: 1,
});

/**
 * Simule un scan pris depuis `pose`. Les points sont produits dans l'ordre du LD06
 * (angles antihoraires décroissants), distances arrondies au mm et angles au 0,01°
 * comme dans l'échange binaire RobotControl.
 */
export function simulateScan(segments, pose, options = {}) {
  const o = { ...DEFAULT_SIM, ...options };
  const rng = mulberry32(o.seed);
  const step = 360 / o.rays;
  const offset = rng() * step; // le premier angle d'un tour n'est jamais exactement 0
  const points = [];
  for (let k = 0; k < o.rays; k++) {
    const angle = (360 - (offset + k * step)) % 360;
    const a = (pose.theta + angle + o.noiseAngle * gaussian(rng)) * DEG;
    const hit = rayCast(segments, pose.x, pose.y, Math.cos(a), Math.sin(a));
    let r = 0;
    let intensity = 0;
    if (hit && hit.t <= o.maxRange && rng() >= o.dropRate) {
      r = Math.max(0.001, hit.t + o.noiseRange * gaussian(rng));
      const s = hit.segment;
      const len = Math.hypot(s.x2 - s.x1, s.y2 - s.y1);
      // incidence : 1 = rayon perpendiculaire au mur, 0 = rasant
      const incidence = Math.abs((Math.cos(a) * (s.y2 - s.y1) - Math.sin(a) * (s.x2 - s.x1)) / len);
      intensity = Math.round(Math.min(255, Math.max(0, (225 - 8 * r) * Math.sqrt(incidence) + 6 * gaussian(rng))));
    }
    points.push({ angle: Math.round(angle * 100) / 100, r: Math.round(r * 1000) / 1000, intensity });
  }
  return {
    name: o.name || 'simulé',
    pose: { ...pose },
    poseFound: true,
    points,
    format: 'simulé',
    comments: [`simulated: rays=${o.rays}, noise_r=${o.noiseRange}m, noise_a=${o.noiseAngle}deg, drop=${o.dropRate}, seed=${o.seed}`],
  };
}

/**
 * Trajet simulé (jour 3) : une pose tous les `step` m le long d'une ligne brisée (cap dans le sens
 * de la marche, rotation sur place aux coins), un scan par pose, et l'« odométrie » qu'aurait le
 * robot entre deux poses : le déplacement vrai entaché d'erreurs. Le robot Mecanum n'a pas de
 * codeurs : on ne connaît que la durée des commandes, et les roues glissent (`odoTrans` : erreur en
 * part de la distance, `odoRot` : écart-type de l'angle par pas, `odoBias` : dérive de cap en
 * degrés par mètre parcouru).
 * Retourne { poses (vraies), scans, odom } ; odom[k] = déplacement mesuré de k − 1 à k (repère du robot).
 */
export function simulateTrajectory(scene, waypoints, options = {}) {
  const o = { step: 0.3, odoTrans: 0.1, odoRot: 2, odoBias: 1.5, seed: 7, ...options };
  const poses = [];
  for (let w = 0; w + 1 < waypoints.length; w++) {
    const a = waypoints[w];
    const b = waypoints[w + 1];
    const len = Math.hypot(b.x - a.x, b.y - a.y);
    const heading = (Math.atan2(b.y - a.y, b.x - a.x) * 180) / Math.PI;
    const n = Math.max(1, Math.ceil(len / o.step));
    for (let k = 0; k < n; k++) poses.push({ x: a.x + ((b.x - a.x) * k) / n, y: a.y + ((b.y - a.y) * k) / n, theta: heading });
  }
  const end = waypoints[waypoints.length - 1];
  poses.push({ x: end.x, y: end.y, theta: poses.length ? poses[poses.length - 1].theta : 0 });
  const scans = poses.map((p, k) => ({ ...simulateScan(scene, p, { seed: o.seed * 1000 + k, ...o.scan }), name: `trajet_${String(k).padStart(2, '0')}` }));
  const rng = mulberry32(o.seed);
  const odom = poses.map((p, k) => {
    if (!k) return null;
    const d = composePoses(invertPose(poses[k - 1]), p);
    const dist = Math.hypot(d.x, d.y);
    return {
      x: d.x * (1 + o.odoTrans * gaussian(rng)),
      y: d.y * (1 + o.odoTrans * gaussian(rng)) + o.odoTrans * dist * gaussian(rng),
      theta: d.theta + o.odoRot * gaussian(rng) + o.odoBias * dist,
    };
  });
  return { poses, scans, odom };
}
