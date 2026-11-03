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

import { GridMap } from './gridmap.js';
import { areClose, length, mergeSegments, midpoint, orientationDeg } from './geometry.js';
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
