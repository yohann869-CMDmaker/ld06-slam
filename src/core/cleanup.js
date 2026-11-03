// Reconstruction « propre » des cartes, pour un rendu lisible comme un plan.
//
// Grid map : on part du classement libre / inconnu / occupé et on retire ce qui n'est pas
// la pièce : poussières (petits groupes de cellules occupées), faisceaux qui sortent par
// les fenêtres ou les portes (ouverture morphologique de l'espace libre), zones libres non
// reliées à une position du robot, murs vus de loin à l'extérieur ; les petits trous dans
// le sol sont bouchés.
//
// Carte vectorielle : les murs d'une salle sont presque tous parallèles ou perpendiculaires.
// On cherche la direction dominante, on redresse les segments proches de ces deux
// directions, on fusionne les morceaux d'un même mur, on supprime les segments trop courts
// et on prolonge ou raccourcit les extrémités pour fermer les coins.
//
// Objets fixes (table, armoire, tableau) : la grid map propre ne garde que les murs, les
// meubles y sont effacés par les faisceaux d'autres scans qui passent dessous ou à côté.
// Les segments, eux, rassemblent les points de tous les scans qui ont vu le meuble : un
// segment vu par plusieurs scans est un objet fixe, il est gardé et redessiné dans la grid.

import { bresenham, GridMap } from './gridmap.js';
import { angleBetweenDeg, areClose, intersect, length, mergeSegments, midpoint, orientationDeg, pointSegmentDistance } from './geometry.js';
import { DEG } from './pose.js';

export const DEFAULT_GRID_CLEAN = Object.freeze({
  minWallCells: 3, // groupe de cellules occupées plus petit : poussière
  openRadius: 2, // cellules : un faisceau libre plus étroit que 2r+1 cellules est retiré
  fillHoles: 40, // cellules : trou inconnu entouré de sol, plus petit : bouché
  wallReach: 2, // cellules : un mur doit toucher le sol gardé à cette distance près
});

/**
 * Grid map propre (nouvel objet, l'original n'est pas modifié). Les cellules valent
 * libre (log-odds lMin), inconnu (0) ou occupé (lMax).
 * poses : positions des robots, pour reconnaître l'intérieur de la pièce.
 */
export function cleanGrid(grid, poses = [], options = {}) {
  const o = { ...DEFAULT_GRID_CLEAN, ...options };
  const { width: w, height: h } = grid;
  const n = w * h;
  const cls = new Int8Array(n);
  for (let k = 0; k < n; k++) cls[k] = grid.classify(k);
  const inside = (i, j) => i >= 0 && j >= 0 && i < w && j < h;

  // composantes connexes (4 ou 8 voisins) d'un ensemble de cellules
  const components = (member, eight) => {
    const label = new Int32Array(n).fill(-1);
    const sizes = [];
    const stack = [];
    const nb = eight ? [[1, 0], [-1, 0], [0, 1], [0, -1], [1, 1], [1, -1], [-1, 1], [-1, -1]] : [[1, 0], [-1, 0], [0, 1], [0, -1]];
    for (let k = 0; k < n; k++) {
      if (!member(k) || label[k] >= 0) continue;
      const id = sizes.length;
      let size = 0;
      label[k] = id;
      stack.push(k);
      while (stack.length) {
        const c = stack.pop();
        size++;
        const i = c % w;
        const j = (c - i) / w;
        for (const [di, dj] of nb) {
          const ni = i + di;
          const nj = j + dj;
          if (!inside(ni, nj)) continue;
          const m = nj * w + ni;
          if (label[m] < 0 && member(m)) {
            label[m] = id;
            stack.push(m);
          }
        }
      }
      sizes.push(size);
    }
    return { label, sizes };
  };

  // 1. poussières
  const occ = components((k) => cls[k] === 1, true);
  for (let k = 0; k < n; k++) if (cls[k] === 1 && occ.sizes[occ.label[k]] < o.minWallCells) cls[k] = 0;

  // 2. ouverture morphologique de l'espace libre (érosion puis dilatation, carré de rayon r)
  const r = o.openRadius;
  const free = (k) => cls[k] === -1;
  const eroded = new Uint8Array(n);
  for (let j = r; j < h - r; j++) {
    for (let i = r; i < w - r; i++) {
      let all = true;
      for (let dj = -r; dj <= r && all; dj++) for (let di = -r; di <= r && all; di++) all = free((j + dj) * w + i + di);
      if (all) eroded[j * w + i] = 1;
    }
  }
  const opened = new Uint8Array(n);
  for (let j = 0; j < h; j++) {
    for (let i = 0; i < w; i++) {
      if (!eroded[j * w + i]) continue;
      for (let dj = -r; dj <= r; dj++) {
        for (let di = -r; di <= r; di++) {
          const m = (j + dj) * w + i + di;
          if (inside(i + di, j + dj) && free(m)) opened[m] = 1;
        }
      }
    }
  }

  // 3. sol de la pièce : composantes libres qui contiennent un robot (sinon la plus grande)
  const floor = components((k) => opened[k] === 1, false);
  const keep = new Set();
  for (const p of poses) {
    const [i, j] = grid.worldToCell(p.x, p.y);
    // le robot peut se trouver sur une cellule retirée par l'érosion : on cherche autour
    for (let rad = 0; rad <= 6 && inside(i, j); rad++) {
      let found = -1;
      for (let dj = -rad; dj <= rad && found < 0; dj++) {
        for (let di = -rad; di <= rad && found < 0; di++) {
          if (inside(i + di, j + dj)) found = floor.label[(j + dj) * w + i + di];
        }
      }
      if (found >= 0) {
        keep.add(found);
        break;
      }
    }
  }
  if (!keep.size && floor.sizes.length) keep.add(floor.sizes.indexOf(Math.max(...floor.sizes)));
  const isFloor = new Uint8Array(n);
  for (let k = 0; k < n; k++) if (opened[k] && keep.has(floor.label[k])) isFloor[k] = 1;

  // 4. petits trous (inconnus, entourés de sol, sans contact avec le bord) : bouchés
  const holes = components((k) => !isFloor[k] && cls[k] !== 1, false);
  const touchesBorder = new Uint8Array(holes.sizes.length);
  for (let i = 0; i < w; i++) {
    for (const j of [0, h - 1]) if (holes.label[j * w + i] >= 0) touchesBorder[holes.label[j * w + i]] = 1;
  }
  for (let j = 0; j < h; j++) {
    for (const i of [0, w - 1]) if (holes.label[j * w + i] >= 0) touchesBorder[holes.label[j * w + i]] = 1;
  }
  for (let k = 0; k < n; k++) {
    const id = holes.label[k];
    if (id >= 0 && !touchesBorder[id] && holes.sizes[id] <= o.fillHoles) isFloor[k] = 1;
  }

  // 5. murs : cellules occupées à moins de wallReach cellules du sol gardé ; puis les trous
  // d'un mur sont bouchés : une cellule non-sol qui touche le sol et a un mur à moins de
  // wallReach cellules devient mur (le contour de la pièce devient continu le long des murs,
  // mais reste ouvert face aux zones d'ombre sans mur proche).
  const out = new GridMap({ ...grid.params, offI: grid.offI, offJ: grid.offJ, width: w, height: h });
  out.scanCount = grid.scanCount;
  const R = o.wallReach;
  const near = (i, j, test) => {
    for (let dj = -R; dj <= R; dj++) {
      for (let di = -R; di <= R; di++) if (inside(i + di, j + dj) && test((j + dj) * w + i + di)) return true;
    }
    return false;
  };
  const wall = new Uint8Array(n);
  for (let j = 0; j < h; j++) {
    for (let i = 0; i < w; i++) {
      const k = j * w + i;
      if (!isFloor[k] && cls[k] === 1 && near(i, j, (m) => isFloor[m] === 1)) wall[k] = 1;
    }
  }
  const touchesFloor = (i, j) =>
    [[1, 0], [-1, 0], [0, 1], [0, -1]].some(([di, dj]) => inside(i + di, j + dj) && isFloor[(j + dj) * w + i + di] === 1);
  for (let j = 0; j < h; j++) {
    for (let i = 0; i < w; i++) {
      const k = j * w + i;
      if (isFloor[k]) out.logOdds[k] = grid.params.lMin;
      else if (wall[k] || (touchesFloor(i, j) && near(i, j, (m) => wall[m] === 1))) out.logOdds[k] = grid.params.lMax;
    }
  }
  return out;
}

/**
 * Ne garde que les segments qui bordent la pièce. Le segment est échantillonné tous les 5 cm :
 * un échantillon est « au bord de la pièce » s'il a une cellule de sol (grid map propre) à
 * moins de maxDist. Moins de la moitié d'échantillons au bord : segment retiré (mur vu à
 * travers une vitre ou une porte) ; sinon il est raccourci à sa partie au bord de la pièce.
 */
export function segmentsNearFloor(segments, cleanedGrid, maxDist = 0.3) {
  const R = Math.ceil(maxDist / cleanedGrid.res);
  const floorNear = (x, y) => {
    const [i, j] = cleanedGrid.worldToCell(x, y);
    for (let dj = -R; dj <= R; dj++) {
      for (let di = -R; di <= R; di++) {
        if (cleanedGrid.inside(i + di, j + dj) && cleanedGrid.classify(cleanedGrid.index(i + di, j + dj)) === -1) return true;
      }
    }
    return false;
  };
  const out = [];
  for (const s of segments) {
    const steps = Math.max(2, Math.ceil(length(s) / 0.05));
    const near = [];
    for (let k = 0; k <= steps; k++) {
      const t = k / steps;
      if (floorNear(s.x1 + t * (s.x2 - s.x1), s.y1 + t * (s.y2 - s.y1))) near.push(t);
    }
    if (near.length < 0.5 * (steps + 1)) continue;
    const [t1, t2] = [near[0], near.at(-1)];
    out.push({
      ...s,
      x1: s.x1 + t1 * (s.x2 - s.x1),
      y1: s.y1 + t1 * (s.y2 - s.y1),
      x2: s.x1 + t2 * (s.x2 - s.x1),
      y2: s.y1 + t2 * (s.y2 - s.y1),
    });
  }
  return out;
}

export const DEFAULT_VECTOR_CLEAN = Object.freeze({
  snapDeg: 7, // segment redressé s'il est à moins de 7° d'une des deux directions principales
  mergeAngle: 4,
  mergeDistance: 0.1,
  mergeGap: 0.35,
  minLength: 0.3, // m : segments plus courts supprimés à la fin
  cornerDist: 0.35, // m : extrémités prolongées ou raccourcies jusqu'au coin à cette distance près
  minObservations: 1, // oubli des segments rarement vus : vus par moins de N scans, ils sont retirés
});

/**
 * Coins (points d'intérêt orientés, document « carte vectorielle ») : intersection de deux
 * segments presque perpendiculaires dont une extrémité de chacun est à moins de maxGap.
 * theta = direction de la bissectrice, vers l'intérieur de l'angle formé par les deux murs.
 */
export function findCorners(segments, { maxGap = 0.06, minAngle = 60 } = {}) {
  const corners = [];
  for (let a = 0; a < segments.length; a++) {
    for (let b = a + 1; b < segments.length; b++) {
      const sa = segments[a];
      const sb = segments[b];
      const ang = Math.abs(orientationDeg(sa) - orientationDeg(sb));
      if (Math.min(ang, 180 - ang) < minAngle) continue;
      const p = lineIntersection(sa, sb);
      if (!p) continue;
      const far = (s) => {
        const d1 = Math.hypot(s.x1 - p.x, s.y1 - p.y);
        const d2 = Math.hypot(s.x2 - p.x, s.y2 - p.y);
        return d1 <= d2 ? { near: d1, x: s.x2, y: s.y2 } : { near: d2, x: s.x1, y: s.y1 };
      };
      const fa = far(sa);
      const fb = far(sb);
      if (fa.near > maxGap || fb.near > maxGap) continue;
      const ua = Math.atan2(fa.y - p.y, fa.x - p.x);
      const ub = Math.atan2(fb.y - p.y, fb.x - p.x);
      const theta = Math.atan2(Math.sin(ua) + Math.sin(ub), Math.cos(ua) + Math.cos(ub)) / DEG;
      corners.push({ x: p.x, y: p.y, theta, angle: Math.min(ang, 180 - ang) });
    }
  }
  return corners;
}

/** Direction principale des murs (degrés dans [0, 90[), histogramme des orientations pondéré par la longueur. */
export function dominantDirection(segments) {
  const hist = new Float64Array(90);
  for (const s of segments) hist[Math.floor(orientationDeg(s) % 90)] += length(s);
  let best = 0;
  let bestScore = -1;
  for (let a = 0; a < 90; a++) {
    let sc = 0;
    for (let d = -3; d <= 3; d++) sc += hist[(a + d + 90) % 90] * (4 - Math.abs(d));
    if (sc > bestScore) {
      bestScore = sc;
      best = a;
    }
  }
  // affinage : moyenne pondérée des orientations à ±4° du pic
  let sum = 0;
  let wsum = 0;
  for (const s of segments) {
    let d = (orientationDeg(s) % 90) - best;
    if (d > 45) d -= 90;
    if (d < -45) d += 90;
    if (Math.abs(d) <= 4) {
      sum += d * length(s);
      wsum += length(s);
    }
  }
  return (((best + (wsum ? sum / wsum : 0)) % 90) + 90) % 90;
}

/** Tourne le segment autour de son milieu pour lui donner l'orientation `deg`, en gardant sa longueur. */
function setOrientation(s, deg) {
  const m = midpoint(s);
  const half = length(s) / 2;
  let ux = Math.cos(deg * DEG);
  let uy = Math.sin(deg * DEG);
  if (ux * (s.x2 - s.x1) + uy * (s.y2 - s.y1) < 0) {
    ux = -ux;
    uy = -uy;
  }
  return { ...s, x1: m.x - half * ux, y1: m.y - half * uy, x2: m.x + half * ux, y2: m.y + half * uy };
}

function snap(s, dom, maxDeg) {
  const a = orientationDeg(s);
  for (const target of [dom, dom + 90, dom + 180]) {
    const d = Math.abs(a - (target % 180));
    if (Math.min(d, 180 - d) <= maxDeg) return setOrientation(s, target % 180);
  }
  return s;
}

/** Intersection des droites supports (null si parallèles). */
function lineIntersection(a, b) {
  const rx = a.x2 - a.x1;
  const ry = a.y2 - a.y1;
  const sx = b.x2 - b.x1;
  const sy = b.y2 - b.y1;
  const den = rx * sy - ry * sx;
  if (Math.abs(den) < 1e-12) return null;
  const t = ((b.x1 - a.x1) * sy - (b.y1 - a.y1) * sx) / den;
  return { x: a.x1 + t * rx, y: a.y1 + t * ry };
}

/** Carte vectorielle propre (nouvelle liste de segments ; la direction dominante est jointe). */
export function cleanVectorMap(segments, options = {}) {
  const o = { ...DEFAULT_VECTOR_CLEAN, ...options };
  if (!segments.length) return { segments: [], dominant: 0 };
  const dom = dominantDirection(segments);
  let segs = segments.map((s) => snap({ ...s }, dom, o.snapDeg));

  // fusion des morceaux d'un même mur, jusqu'à stabilité
  const close = { maxAngleDeg: o.mergeAngle, maxDistance: o.mergeDistance, maxGap: o.mergeGap };
  for (let changed = true; changed; ) {
    changed = false;
    outer: for (let a = 0; a < segs.length; a++) {
      for (let b = a + 1; b < segs.length; b++) {
        if (areClose(segs[a], segs[b], close)) {
          const m = snap(mergeSegments(segs[a], segs[b]), dom, o.snapDeg);
          // observations = scans distincts (deux morceaux vus par le même scan comptent une fois)
          const scans = [...new Set([...(segs[a].scans ?? []), ...(segs[b].scans ?? [])])];
          if (scans.length) Object.assign(m, { scans, observations: scans.length });
          segs = segs.filter((_, k) => k !== a && k !== b).concat(m);
          changed = true;
          break outer;
        }
      }
    }
  }
  segs = segs.filter((s) => length(s) >= o.minLength && (s.observations ?? 1) >= o.minObservations);

  // coins : deux segments presque perpendiculaires dont une extrémité est près de leur intersection
  for (let a = 0; a < segs.length; a++) {
    for (let b = a + 1; b < segs.length; b++) {
      const angle = Math.abs(orientationDeg(segs[a]) - orientationDeg(segs[b]));
      if (Math.min(angle, 180 - angle) < 60) continue;
      const p = lineIntersection(segs[a], segs[b]);
      if (!p) continue;
      const ends = [segs[a], segs[b]].map((s) => {
        const d1 = Math.hypot(s.x1 - p.x, s.y1 - p.y);
        const d2 = Math.hypot(s.x2 - p.x, s.y2 - p.y);
        return d1 <= d2 ? { end: 1, d: d1 } : { end: 2, d: d2 };
      });
      if (ends.every((e) => e.d <= o.cornerDist)) {
        [a, b].forEach((idx, q) => {
          const s = segs[idx];
          segs[idx] = ends[q].end === 1 ? { ...s, x1: p.x, y1: p.y } : { ...s, x2: p.x, y2: p.y };
        });
      }
    }
  }
  return { segments: segs, dominant: dom };
}

export const DEFAULT_OBJECTS = Object.freeze({
  minObservations: 2, // objet fixe : vu par au moins 2 scans (vu une seule fois : chaise, sac, personne...)
  wallDist: 0.25, // m : un segment à moins de 25 cm de l'extérieur de la salle est un mur
  joinDist: 0.1, // m : un segment qui forme un coin avec un segment gardé fait partie du même meuble
  barrierLength: 2, // m : un segment au moins aussi long est un mur qui ferme l'extérieur, même caché
  barriers: [], // segments supplémentaires qui ferment l'extérieur (murs vus par les scans pris en hauteur)
});

/**
 * Murs et objets fixes. Chaque segment gardé reçoit kind = 'mur' ou 'objet' :
 *  - mur : au moins la moitié du segment est à moins de wallDist de l'extérieur de la salle,
 *    c'est-à-dire des cellules inconnues de la grid map propre reliées au bord de la carte.
 *    L'intérieur d'une table (inconnu, entouré de sol) ou d'une armoire contre un mur (fermé par
 *    le mur) n'est pas relié au bord : leurs côtés ne sont pas des murs. Depuis le sol, le mur
 *    derrière l'armoire n'est pas vu : les segments d'au moins barrierLength (les grands murs,
 *    prolongés par la fusion au-delà de ce qu'ils cachent) ferment aussi l'extérieur, de même que
 *    les grands murs de `barriers` (vus d'en haut, par-dessus les meubles, non dessinés) ;
 *  - objet fixe : autre segment vu par au moins minObservations scans (côté de la table, armoire,
 *    tableau), ou qui forme un coin (extrémités à moins de joinDist) avec un segment gardé : face
 *    d'un meuble vue par un seul scan (face de l'armoire accrochée à sa face avant) ;
 *  - les autres segments (vus par un seul scan, isolés) sont retirés : mobilier déplacé, personnes.
 */
export function classifySegments(segments, cleanedGrid, options = {}) {
  const o = { ...DEFAULT_OBJECTS, ...options };
  const g = cleanedGrid;
  const { width: w, height: h } = g;
  // extérieur : remplissage depuis les cellules inconnues du bord, à travers les cellules inconnues
  // (les murs occupés, le sol et les grands murs de la carte vectorielle l'arrêtent)
  const barrier = new Uint8Array(w * h);
  for (const s of [...segments, ...o.barriers]) {
    if (length(s) < o.barrierLength) continue;
    const [i1, j1] = g.worldToCell(s.x1, s.y1);
    const [i2, j2] = g.worldToCell(s.x2, s.y2);
    bresenham(i1, j1, i2, j2, (i, j) => {
      // trait épais d'une cellule de chaque côté : un remplissage en 4 voisins ne le traverse pas
      for (const [di, dj] of [[0, 0], [1, 0], [0, 1]]) if (g.inside(i + di, j + dj)) barrier[g.index(i + di, j + dj)] = 1;
    });
  }
  const outside = new Uint8Array(w * h);
  const stack = [];
  const push = (i, j) => {
    if (!g.inside(i, j)) return;
    const k = g.index(i, j);
    if (outside[k] || barrier[k] || g.classify(k) !== 0) return;
    outside[k] = 1;
    stack.push(k);
  };
  for (let i = 0; i < w; i++) [push(i, 0), push(i, h - 1)];
  for (let j = 0; j < h; j++) [push(0, j), push(w - 1, j)];
  while (stack.length) {
    const k = stack.pop();
    const i = k % w;
    const j = (k - i) / w;
    push(i + 1, j);
    push(i - 1, j);
    push(i, j + 1);
    push(i, j - 1);
  }
  const R = Math.max(1, Math.round(o.wallDist / g.res));
  const outsideNear = (x, y) => {
    const [i, j] = g.worldToCell(x, y);
    for (let dj = -R; dj <= R; dj++) {
      for (let di = -R; di <= R; di++) if (g.inside(i + di, j + dj) && outside[g.index(i + di, j + dj)]) return true;
    }
    return false;
  };
  const onWall = (s) => {
    const steps = Math.max(2, Math.ceil(length(s) / 0.05));
    let n = 0;
    for (let k = 0; k <= steps; k++) if (outsideNear(s.x1 + (k / steps) * (s.x2 - s.x1), s.y1 + (k / steps) * (s.y2 - s.y1))) n++;
    return n >= 0.5 * (steps + 1);
  };
  const out = segments.map((s) => {
    const kind = onWall(s) ? 'mur' : (s.observations ?? 1) >= o.minObservations ? 'objet' : null;
    return { ...s, kind };
  });
  // propagation par les coins : les faces d'un meuble vues par un seul scan le rejoignent
  const ends = (s) => [[s.x1, s.y1], [s.x2, s.y2]];
  const touch = (a, b) => ends(a).some(([x, y]) => ends(b).some(([u, v]) => Math.hypot(x - u, y - v) <= o.joinDist));
  for (let changed = true; changed; ) {
    changed = false;
    for (const s of out) {
      if (!s.kind && out.some((t) => t.kind && touch(s, t))) {
        s.kind = 'objet';
        changed = true;
      }
    }
  }
  // murs vus d'en haut (barriers, scans pris en hauteur) : ils complètent la frontière de la salle
  // là où les meubles la cachent au sol, et remplacent les morceaux du même mur vus du sol. On ne
  // garde que ceux qui bordent l'extérieur et qui ne doublent pas un meuble (dessus d'un bureau).
  const close = { maxAngleDeg: DEFAULT_VECTOR_CLEAN.mergeAngle, maxDistance: DEFAULT_VECTOR_CLEAN.mergeDistance, maxGap: DEFAULT_VECTOR_CLEAN.mergeGap };
  let kept = out.filter((s) => s.kind);
  for (const b of o.barriers) {
    if (length(b) < o.barrierLength || !onWall(b)) continue;
    if (kept.some((s) => s.kind === 'objet' && areClose(s, b, close))) continue;
    kept = kept.filter((s) => !(s.kind === 'mur' && areClose(s, b, close)));
    kept.push({ ...b, kind: 'mur', seenFromAbove: true });
  }
  return kept;
}

/**
 * Probabilité d'occupation donnée aux cellules d'un meuble surélevé : « occupé en partie ». Elle
 * reste au-dessus du seuil d'occupation (c'est un obstacle pour le robot), mais se distingue des
 * murs et des meubles posés au sol (p = 0,98) : les rendus la dessinent en gris.
 */
export const RAISED_P = 0.75;

/** Grid map propre avec les objets fixes dessinés (cellules occupées le long des segments 'objet'). */
export function drawObjects(cleanedGrid, segments) {
  const g = cleanedGrid.clone();
  const raised = Math.log(RAISED_P / (1 - RAISED_P));
  for (const s of segments) {
    if (s.kind !== 'objet' && !s.seenFromAbove) continue; // objets fixes, et murs cachés au sol par les meubles
    const [i1, j1] = g.worldToCell(s.x1, s.y1);
    const [i2, j2] = g.worldToCell(s.x2, s.y2);
    bresenham(i1, j1, i2, j2, (i, j) => {
      if (!g.inside(i, j)) return;
      const k = g.index(i, j);
      // une cellule déjà mur (posé au sol) le reste
      g.logOdds[k] = s.kind === 'objet' && s.raised ? Math.max(g.logOdds[k], raised) : g.params.lMax;
    });
  }
  return g;
}

/** Cellule occupée en partie : meuble surélevé (le lidar passe dessous). */
export const isPartlyOccupied = (grid, k) => grid.classify(k) === 1 && grid.probability(k) < 0.9;

/**
 * Chaîne complète des cartes propres (application, carte de référence) :
 * grid map nettoyée -> carte vectorielle nettoyée -> segments au bord du sol -> murs et objets
 * fixes -> objets redessinés dans la grid map -> coins.
 * vectorSegments : segments SANS découpe par les absences (sinon les meubles, traversés par
 * des faisceaux, sont découpés avant d'avoir pu être reconnus).
 * options.views : [{ pose, points }] des scans au sol, pour reconnaître les meubles surélevés ;
 * options.barriers : murs vus par tous les scans, y compris pris en hauteur (frontière de la salle) ;
 * options.labels : meubles nommés (carte sémantique, voir applyLabels et semanticPlan).
 * Retourne { grid (plan d'affichage), gridLoc (grid map de localisation), segments, dominant, corners }.
 */
export function cleanMaps(grid, vectorSegments, poses, options = {}) {
  const walls = cleanGrid(grid, poses, options.grid);
  const cv = cleanVectorMap(vectorSegments, options.vector);
  const objects = { ...options.objects, barriers: options.barriers ?? [] };
  let segments = classifySegments(segmentsNearFloor(cv.segments, walls), walls, objects);
  if (options.views) segments = markRaised(segments, options.views, options.raised);
  if (options.labels?.length) segments = applyLabels(segments, options.labels);
  // grid map de localisation : murs, meubles et tout ce que le lidar voit depuis le sol (pieds de
  // bureaux, rangées de chaises) : ce sont des repères. Sans eux, des scans glissent de 47 cm le
  // long des murs parallèles (14/18 scans relocalisés au lieu de 18/18).
  const gridLoc = drawObjects(walls, segments);
  // plan pour l'humain : avec la carte sémantique, seuls les murs et les meubles nommés
  const plan = options.labels?.length ? semanticPlan(gridLoc, segments, options.labels) : gridLoc;
  return { grid: plan, gridLoc, segments, dominant: cv.dominant, corners: findCorners(segments) };
}

/** Meubles : groupes de segments 'objet' reliés par leurs extrémités (les 4 côtés d'une table). */
export function objectGroups(segments, joinDist = DEFAULT_OBJECTS.joinDist) {
  const ends = (s) => [[s.x1, s.y1], [s.x2, s.y2]];
  const touch = (a, b) => ends(a).some(([x, y]) => ends(b).some(([u, v]) => Math.hypot(x - u, y - v) <= joinDist));
  const idx = segments.map((s, k) => k).filter((k) => segments[k].kind === 'objet');
  const group = new Map();
  const groups = [];
  for (const k of idx) {
    if (group.has(k)) continue;
    const g = [k];
    group.set(k, g);
    for (let q = 0; q < g.length; q++) {
      for (const m of idx) {
        if (!group.has(m) && touch(segments[g[q]], segments[m])) {
          group.set(m, g);
          g.push(m);
        }
      }
    }
    groups.push(g);
  }
  return groups;
}

export const DEFAULT_RAISED = Object.freeze({
  hitDist: 0.06, // m : impact à moins de 6 cm du segment : le faisceau s'arrête sur le meuble
  behind: 0.15, // m : impact au moins 15 cm derrière le segment : le faisceau est passé dessous
  minCrossAngle: 20, // degrés : faisceau trop rasant ignoré (sensible aux erreurs de pose)
  endMargin: 0.05, // m : faisceau qui passe à moins de 5 cm d'une extrémité ignoré (à côté du meuble)
  // meuble surélevé si au moins 15 % des faisceaux qui l'atteignent passent dessous. Mesuré sur la
  // salle avec les scans au sol : armoire 1 %, table 4 % (l'espace sous la table est plus bas que
  // le lidar), autres meubles posés au sol 2 à 7 %, tableau sur deux pieds 61 %
  minRatio: 0.15,
});

/**
 * Meubles surélevés (table sur roulettes, tableau sur deux pieds) : le plan du lidar passe en
 * partie sous le meuble. Pour chaque meuble (objectGroups), on compte sur ses segments :
 *  - les faisceaux qui s'arrêtent dessus (impact à moins de hitDist) ;
 *  - les faisceaux qui le traversent (le faisceau coupe le segment et son impact est au moins
 *    `behind` plus loin) : ils sont passés dessous, entre les roues ou les pieds.
 * Un meuble posé au sol (armoire) arrête presque tous les faisceaux ; un meuble surélevé en laisse
 * passer une bonne part. Les scans dont le robot est dans l'emprise du meuble sont ignorés :
 * on juge le meuble vu de la salle.
 * views : [{ pose, points }] (points dans le repère monde). Retourne une copie des segments ;
 * les segments 'objet' reçoivent under (part des faisceaux passés dessous) et raised (booléen).
 */
export function markRaised(segments, views, options = {}) {
  const o = { ...DEFAULT_RAISED, ...options };
  const out = segments.map((s) => ({ ...s }));
  for (const g of objectGroups(out)) {
    const segs = g.map((k) => out[k]);
    const box = {
      minX: Math.min(...segs.flatMap((s) => [s.x1, s.x2])),
      maxX: Math.max(...segs.flatMap((s) => [s.x1, s.x2])),
      minY: Math.min(...segs.flatMap((s) => [s.y1, s.y2])),
      maxY: Math.max(...segs.flatMap((s) => [s.y1, s.y2])),
    };
    let stop = 0;
    let under = 0;
    for (const { pose, points } of views) {
      if (pose.x > box.minX && pose.x < box.maxX && pose.y > box.minY && pose.y < box.maxY) continue;
      for (const p of points) {
        const ray = { x1: pose.x, y1: pose.y, x2: p.x, y2: p.y };
        const reach = length(ray);
        for (const s of segs) {
          if (pointSegmentDistance(s, p.x, p.y) <= o.hitDist) {
            stop++;
            break;
          }
          if (angleBetweenDeg(s, ray) < o.minCrossAngle) continue;
          const r = intersect(s, ray);
          const L = length(s);
          if (r.type === 'point' && r.t * L >= o.endMargin && (1 - r.t) * L >= o.endMargin && (1 - r.u) * reach >= o.behind) {
            under++;
            break;
          }
        }
      }
    }
    const ratio = stop + under ? under / (stop + under) : 0;
    for (const s of segs) Object.assign(s, { under: ratio, raised: ratio >= o.minRatio });
  }
  return out;
}

// ------------------------------------------------------------------ carte sémantique

/** Le point (x, y) est-il dans la zone [xmin, ymin, xmax, ymax] ? */
const inZone = (p, [x0, y0, x1, y1]) => p.x >= x0 && p.x <= x1 && p.y >= y0 && p.y <= y1;

/**
 * Carte sémantique : meubles nommés à la main (data/reference/etiquettes.json),
 * labels = [{ nom, detail, zone: [xmin, ymin, xmax, ymax] }] dans le repère de la carte.
 * Un segment d'objet fixe dont le milieu est dans la zone d'un meuble prend son nom (label) ;
 * les autres objets détectés, qui ne correspondent à aucun meuble réel, sont retirés de la
 * carte vectorielle (la grid map garde ce qu'elle voit à leur place). Les murs ne changent pas.
 */
export function applyLabels(segments, labels) {
  const out = [];
  for (const s of segments) {
    if (s.kind !== 'objet') {
      out.push(s);
      continue;
    }
    const l = labels.find((l) => inZone(midpoint(s), l.zone));
    if (l) out.push({ ...s, label: l.nom });
  }
  return out;
}

/**
 * Où écrire chaque étiquette : au centre des segments du meuble (ou de sa zone s'il n'a pas été
 * retrouvé). Retourne [{ nom, detail, zone, x, y, found }].
 */
export function labelPositions(segments, labels) {
  return labels.map((l) => {
    const segs = segments.filter((s) => s.kind === 'objet' && inZone(midpoint(s), l.zone));
    const xs = segs.flatMap((s) => [s.x1, s.x2]);
    const ys = segs.flatMap((s) => [s.y1, s.y2]);
    const [x0, y0, x1, y1] = segs.length ? [Math.min(...xs), Math.min(...ys), Math.max(...xs), Math.max(...ys)] : l.zone;
    return { ...l, x: (x0 + x1) / 2, y: (y0 + y1) / 2, found: segs.length > 0 };
  });
}

/** Relit un fichier d'étiquettes (format ld06-slam/etiquettes). */
export function parseLabels(text) {
  const d = JSON.parse(text);
  if (d.format !== 'ld06-slam/etiquettes') throw new Error("Ce fichier n'est pas un fichier d'étiquettes ld06-slam");
  return d.objets.map(({ nom, detail = '', zone }) => ({ nom, detail, zone: zone.map(Number) }));
}

/**
 * Plan de la salle avec la carte sémantique : dans la salle, seuls les murs et les meubles nommés
 * restent des obstacles. Le reste de l'intérieur devient du sol :
 *  - les cellules occupées loin des murs (chaises, sacs, pieds de bureaux vus d'en bas) ;
 *  - les zones inconnues enfermées dans la salle (dessous des bureaux, que le lidar au sol ne
 *    voit pas). L'extérieur (cellules inconnues reliées au bord de la carte, arrêtées par les
 *    murs) ne change pas. Il ne passe pas par une ouverture de moins de 2 × gapReach (60 cm) entre
 *    deux murs : un petit trou dans le contour ne fait pas « entrer » l'extérieur dans la salle.
 * Les zones des meubles nommés sont laissées telles quelles (intérieur de la table : inconnu).
 * Les murs (segments 'mur', dont les grands murs vus d'en haut) sont tracés en continu.
 */
export function semanticPlan(grid, segments, labels, gapReach = 0.3) {
  const g = grid.clone();
  const { width: w, height: h } = g;
  const { lMin, lMax } = g.params;
  // murs tracés, épaisseur 2 cellules (un remplissage en 4 voisins ne les traverse pas)
  const wall = new Uint8Array(w * h);
  for (const s of segments.filter((s) => s.kind === 'mur')) {
    const [i1, j1] = g.worldToCell(s.x1, s.y1);
    const [i2, j2] = g.worldToCell(s.x2, s.y2);
    bresenham(i1, j1, i2, j2, (i, j) => {
      for (const [di, dj] of [[0, 0], [1, 0], [0, 1]]) if (g.inside(i + di, j + dj)) wall[g.index(i + di, j + dj)] = 1;
    });
  }
  // extérieur : inconnu relié au bord, sans traverser les murs. 1) remplissage par les seules
  // cellules à plus de R cellules d'un mur (une ouverture de moins de 2R ne laisse pas passer) ;
  // 2) extension de R + 1 cellules pour reprendre la bande d'extérieur collée aux murs.
  const R = Math.round(gapReach / g.res);
  const nearLine = new Uint8Array(w * h);
  for (let k = 0; k < w * h; k++) {
    if (!wall[k]) continue;
    const i = k % w;
    const j = (k - i) / w;
    for (let dj = -R; dj <= R; dj++) for (let di = -R; di <= R; di++) if (g.inside(i + di, j + dj)) nearLine[g.index(i + di, j + dj)] = 1;
  }
  const outside = new Uint8Array(w * h);
  const depth = new Int32Array(w * h).fill(-1);
  const queue = [];
  const unknown = (k) => !wall[k] && g.classify(k) === 0;
  for (let k = 0; k < w * h; k++) {
    const i = k % w;
    const j = (k - i) / w;
    if ((i === 0 || j === 0 || i === w - 1 || j === h - 1) && unknown(k) && !nearLine[k]) {
      outside[k] = 1;
      queue.push(k);
    }
  }
  for (let q = 0; q < queue.length; q++) {
    const k = queue[q];
    const i = k % w;
    const j = (k - i) / w;
    for (const [di, dj] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) {
      if (!g.inside(i + di, j + dj)) continue;
      const m = g.index(i + di, j + dj);
      if (outside[m] || !unknown(m) || nearLine[m]) continue;
      outside[m] = 1;
      queue.push(m);
    }
  }
  const ring = queue.slice();
  for (const k of ring) depth[k] = 0;
  for (let q = 0; q < ring.length; q++) {
    const k = ring[q];
    if (depth[k] > R) continue;
    const i = k % w;
    const j = (k - i) / w;
    for (const [di, dj] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) {
      if (!g.inside(i + di, j + dj)) continue;
      const m = g.index(i + di, j + dj);
      if (outside[m] || !unknown(m)) continue;
      outside[m] = 1;
      depth[m] = depth[k] + 1;
      ring.push(m);
    }
  }
  const nearWall = (i, j) => {
    for (let dj = -2; dj <= 2; dj++) for (let di = -2; di <= 2; di++) if (g.inside(i + di, j + dj) && wall[g.index(i + di, j + dj)]) return true;
    return false;
  };
  for (let j = 0; j < h; j++) {
    for (let i = 0; i < w; i++) {
      const k = g.index(i, j);
      if (outside[k]) continue;
      const c = g.cellCenter(i, j);
      if (labels.some((l) => inZone(c, l.zone))) continue;
      if (wall[k]) g.logOdds[k] = lMax;
      else if (g.classify(k) === 0 || (g.classify(k) === 1 && !nearWall(i, j))) g.logOdds[k] = lMin;
    }
  }
  return g;
}
