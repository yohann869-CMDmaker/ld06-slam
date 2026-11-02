// Opérations géométriques sur les segments (cartes vectorielles).
//
// Un segment est un objet { x1, y1, x2, y2, ... } (mètres). Les propriétés
// supplémentaires (poids, nombre d'observations...) sont conservées par les opérations.
//
// Tolérance numérique EPS = 1 µm : deux points à moins de EPS sont confondus, deux
// longueurs qui diffèrent de moins de EPS sont égales. Elle est très inférieure au
// bruit du lidar (~1 cm) et très supérieure à l'erreur d'arrondi des flottants
// sur des coordonnées de quelques mètres (~1e-15 m), donc elle ne masque aucun cas réel
// et absorbe les erreurs d'arrondi.

import { DEG, localToWorld } from './pose.js';

export const EPS = 1e-6;
/** Deux directions sont parallèles si le sinus de leur angle est inférieur à cette valeur. */
export const PARALLEL_EPS = 1e-9;

const cross = (ax, ay, bx, by) => ax * by - ay * bx;

export function length(s) {
  return Math.hypot(s.x2 - s.x1, s.y2 - s.y1);
}

export function midpoint(s) {
  return { x: (s.x1 + s.x2) / 2, y: (s.y1 + s.y2) / 2 };
}

/** Vecteur directeur unitaire (de P1 vers P2). */
export function direction(s) {
  const l = length(s);
  return l > 0 ? { x: (s.x2 - s.x1) / l, y: (s.y2 - s.y1) / l } : { x: 0, y: 0 };
}

/** Orientation de la droite support, en degrés dans [0, 180[. */
export function orientationDeg(s) {
  const a = Math.atan2(s.y2 - s.y1, s.x2 - s.x1) / DEG;
  return ((a % 180) + 180) % 180;
}

/**
 * Angle entre deux segments, en degrés dans [0, 90].
 * Les segments d'une carte ne sont pas orientés (un mur parcouru dans un sens ou
 * dans l'autre est le même mur), on mesure donc l'angle entre droites supports.
 */
export function angleBetweenDeg(a, b) {
  const d = Math.abs(orientationDeg(a) - orientationDeg(b));
  return Math.min(d, 180 - d);
}

/** Angle orienté de la direction de a vers celle de b, en degrés dans ]-180, 180]. */
export function orientedAngleDeg(a, b) {
  const da = direction(a);
  const db = direction(b);
  return Math.atan2(cross(da.x, da.y, db.x, db.y), da.x * db.x + da.y * db.y) / DEG;
}

/** Abscisse (en mètres, depuis P1) de la projection de (px, py) sur la droite support de s. */
export function projectAlong(s, px, py) {
  const d = direction(s);
  return (px - s.x1) * d.x + (py - s.y1) * d.y;
}

/** Distance du point à la droite support de s. */
export function pointLineDistance(s, px, py) {
  const l = length(s);
  if (l === 0) return Math.hypot(px - s.x1, py - s.y1);
  return Math.abs(cross(s.x2 - s.x1, s.y2 - s.y1, px - s.x1, py - s.y1)) / l;
}

/** Distance du point au segment (et non à la droite). */
export function pointSegmentDistance(s, px, py) {
  const l = length(s);
  if (l === 0) return Math.hypot(px - s.x1, py - s.y1);
  const t = Math.max(0, Math.min(l, projectAlong(s, px, py)));
  const d = direction(s);
  return Math.hypot(px - (s.x1 + t * d.x), py - (s.y1 + t * d.y));
}

/**
 * Mesures de proximité entre deux segments, calculées par rapport au plus long (référence) :
 * - angle    : angle entre droites supports (degrés, [0, 90]) ;
 * - distance : plus grande distance d'une extrémité du plus court à la droite du plus long ;
 * - gap      : écart longitudinal entre les deux segments projetés sur l'axe du plus long
 *              (0 s'ils se recouvrent).
 */
export function closeness(a, b) {
  const [ref, other] = length(a) >= length(b) ? [a, b] : [b, a];
  const lr = length(ref);
  const t1 = projectAlong(ref, other.x1, other.y1);
  const t2 = projectAlong(ref, other.x2, other.y2);
  return {
    angle: angleBetweenDeg(a, b),
    distance: Math.max(pointLineDistance(ref, other.x1, other.y1), pointLineDistance(ref, other.x2, other.y2)),
    gap: Math.max(0, Math.min(t1, t2) - lr, -Math.max(t1, t2)),
  };
}

export const DEFAULT_CLOSE = Object.freeze({
  maxAngleDeg: 10, // proches angulairement
  maxDistance: 0.08, // m, proches en position (écart perpendiculaire)
  maxGap: 0.2, // m, trou maximal entre deux morceaux d'un même mur
});

/** Deux segments sont « proches » s'ils le sont à la fois angulairement et en position. */
export function areClose(a, b, opts = DEFAULT_CLOSE) {
  const o = { ...DEFAULT_CLOSE, ...opts };
  const c = closeness(a, b);
  return c.angle <= o.maxAngleDeg && c.distance <= o.maxDistance && c.gap <= o.maxGap;
}

/**
 * Fusion de deux segments proches.
 * - direction : moyenne pondérée des orientations, calculée sur l'angle double pour
 *   que deux segments de sens opposés (0° et 180°) donnent bien 0° et non 90° ;
 * - position  : la droite passe par le barycentre pondéré des milieux ;
 * - étendue   : de la plus petite à la plus grande projection des 4 extrémités.
 * Poids = propriété `weight` si présente (nombre de points), sinon la longueur.
 * Le sens du résultat est celui de a.
 */
export function mergeSegments(a, b) {
  const wa = a.weight ?? length(a);
  const wb = b.weight ?? length(b);
  const pa = 2 * Math.atan2(a.y2 - a.y1, a.x2 - a.x1);
  const pb = 2 * Math.atan2(b.y2 - b.y1, b.x2 - b.x1);
  const phi = Math.atan2(wa * Math.sin(pa) + wb * Math.sin(pb), wa * Math.cos(pa) + wb * Math.cos(pb)) / 2;
  let ux = Math.cos(phi);
  let uy = Math.sin(phi);
  if (ux * (a.x2 - a.x1) + uy * (a.y2 - a.y1) < 0) {
    ux = -ux;
    uy = -uy;
  }
  const ma = midpoint(a);
  const mb = midpoint(b);
  const cx = (wa * ma.x + wb * mb.x) / (wa + wb);
  const cy = (wa * ma.y + wb * mb.y) / (wa + wb);
  const ts = [
    [a.x1, a.y1],
    [a.x2, a.y2],
    [b.x1, b.y1],
    [b.x2, b.y2],
  ].map(([x, y]) => (x - cx) * ux + (y - cy) * uy);
  const tMin = Math.min(...ts);
  const tMax = Math.max(...ts);
  return {
    ...a,
    x1: cx + tMin * ux,
    y1: cy + tMin * uy,
    x2: cx + tMax * ux,
    y2: cy + tMax * uy,
    weight: wa + wb,
    observations: (a.observations ?? 1) + (b.observations ?? 1),
  };
}

/**
 * Intersection de deux segments a et b.
 * Retour :
 * - { type: 'point', x, y, t, u }   : les deux segments se coupent (t sur a, u sur b, dans [0, 1]) ;
 * - { type: 'outside', x, y, t, u } : les droites se coupent hors d'au moins un des segments ;
 * - { type: 'parallel' }            : droites parallèles distinctes ;
 * - { type: 'colinear', overlap }   : même droite support, recouvrement ou non ;
 * - { type: 'degenerate' }          : un des segments est de longueur nulle (<= eps).
 * Un point à moins de eps d'un segment est considéré comme sur le segment.
 */
export function intersect(a, b, eps = EPS) {
  const rx = a.x2 - a.x1;
  const ry = a.y2 - a.y1;
  const sx = b.x2 - b.x1;
  const sy = b.y2 - b.y1;
  const la = Math.hypot(rx, ry);
  const lb = Math.hypot(sx, sy);
  if (la <= eps || lb <= eps) return { type: 'degenerate' };
  const qx = b.x1 - a.x1;
  const qy = b.y1 - a.y1;
  const denom = cross(rx, ry, sx, sy);
  if (Math.abs(denom) <= PARALLEL_EPS * la * lb) {
    if (Math.abs(cross(qx, qy, rx, ry)) / la > eps) return { type: 'parallel' };
    const t0 = (qx * rx + qy * ry) / (la * la);
    const t1 = t0 + (sx * rx + sy * ry) / (la * la);
    const overlap = Math.max(t0, t1) >= -eps / la && Math.min(t0, t1) <= 1 + eps / la;
    return { type: 'colinear', overlap };
  }
  const t = cross(qx, qy, sx, sy) / denom;
  const u = cross(qx, qy, rx, ry) / denom;
  const onA = t >= -eps / la && t <= 1 + eps / la;
  const onB = u >= -eps / lb && u <= 1 + eps / lb;
  return { type: onA && onB ? 'point' : 'outside', x: a.x1 + t * rx, y: a.y1 + t * ry, t, u };
}

/** Point d'intersection de deux segments, ou null. */
export function intersectionPoint(a, b, eps = EPS) {
  const r = intersect(a, b, eps);
  return r.type === 'point' ? { x: r.x, y: r.y } : null;
}

/**
 * Découpe `target` traversé par `cutter` et conserve le plus long côté.
 * Règles :
 * - l'intersection doit être sur les deux segments (à eps près) ;
 * - elle doit être strictement à l'intérieur de target : à plus de eps de chaque extrémité.
 *   Une intersection à une extrémité ne change rien ;
 * - des segments colinéaires (ou parallèles) ne sont pas découpés ;
 * - si les deux côtés ont la même longueur à eps près, on garde le côté contenant P1.
 * Le côté conservé garde le sens de target : [P1, I] ou [I, P2].
 * Retour : { segment, cut, reason, point?, kept? ('P1' | 'P2') }.
 */
export function cutSegment(target, cutter, eps = EPS) {
  const unchanged = (reason) => ({ segment: target, cut: false, reason });
  const L = length(target);
  if (L <= eps) return unchanged('segment à couper dégénéré');
  const r = intersect(target, cutter, eps);
  if (r.type === 'degenerate') return unchanged('segment coupant dégénéré');
  if (r.type === 'colinear') return unchanged('segments colinéaires');
  if (r.type === 'parallel') return unchanged('segments parallèles');
  if (r.type === 'outside') return unchanged('pas d\'intersection sur les deux segments');
  const d1 = r.t * L; // longueur du côté P1 -> I
  const d2 = L - d1; // longueur du côté I -> P2
  if (d1 <= eps || d2 <= eps) return unchanged('intersection à une extrémité');
  const keepP1 = Math.abs(d1 - d2) <= eps || d1 > d2;
  const segment = keepP1 ? { ...target, x2: r.x, y2: r.y } : { ...target, x1: r.x, y1: r.y };
  return { segment, cut: true, reason: 'découpé', point: { x: r.x, y: r.y }, kept: keepP1 ? 'P1' : 'P2' };
}

/** Segment exprimé dans le repère local d'une pose, ramené dans le repère monde. */
export function transformSegment(s, pose) {
  const p1 = localToWorld(pose, s.x1, s.y1);
  const p2 = localToWorld(pose, s.x2, s.y2);
  return { ...s, x1: p1.x, y1: p1.y, x2: p2.x, y2: p2.y };
}
