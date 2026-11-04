// Carte finale propre à partir de la carte d'un SLAM : deux étapes de filtrage, puis reconstruction
// du plan de la salle (jour 3, après la session de SLAM).
//
// Filtrage 1 (filterClutter) : retire l'encombrement de la grid map. Les cellules occupées sont
// regroupées en objets (composantes connexes) ; un objet plus petit que `minObject` (pieds de chaises
// et de tables, sacs, jambes) est retiré, ainsi que les cellules vues occupées par moins de `minHits`
// scans. Les murs et les grands meubles restent.
//
// Filtrage 2 + reconstruction (reconstructRoom), d'après les travaux de reconstruction de plans en
// milieu encombré : Okorn et al. (CMU, 2010) pour les droites des murs par histogrammes, Mura et al.
// (Computers & Graphics, 2014) et Ochmann et al. (Computers & Graphics 2016, ISPRS 2019) pour la
// décomposition de l'espace en cellules par ces droites et leur étiquetage intérieur / extérieur ;
// hypothèse « monde Manhattan » (murs parallèles ou perpendiculaires) :
//   1. direction dominante des murs (segments de la carte vectorielle), carte redressée ;
//   2. droites des murs : pics des histogrammes des cellules occupées le long de chaque axe ; plus les
//      limites de l'espace vu libre (un mur que le lidar n'a pas détecté arrête quand même la vue :
//      le bord du libre donne sa position) ; plus les décrochés : un court morceau de mur accroché à
//      un mur extérieur (coin rentrant), le coin derrière lui est hors de la salle ;
//   3. les droites découpent l'espace en rectangles ; un rectangle assez vu libre est INTÉRIEUR,
//      s'il est atteignable depuis le trajet du robot sans traverser de mur observé (diffusion depuis
//      le trajet, comme la diffusion de Mura et al.) : l'espace vu par une porte, derrière un mur,
//      n'appartient pas à la salle ; un meuble isolé (objet compact et épais, loin des murs : bureau)
//      ne compte pas comme un mur, l'espace vu libre derrière lui reste dans la salle ;
//   4. complétion : un rectangle non vu qui touche l'intérieur sans mur entre eux (aucun point
//      occupé sur la frontière commune) est intérieur aussi : zone cachée (coin inaccessible, personnes),
//      pas l'extérieur ; un rectangle entouré d'intérieur (sous un meuble) aussi ;
//   5. le contour de l'intérieur donne les murs, observés (points occupés le long) ou reconstruits ;
//      les grands objets restants à l'intérieur sont les meubles.

import { GridMap } from './gridmap.js';
import { dominantDirection } from './cleanup.js';
import { length, orientationDeg } from './geometry.js';

export const DEFAULT_CLUTTER = Object.freeze({
  minObject: 0.4, // m : taille (diagonale de la boîte englobante) minimale d'un objet gardé
  minHits: 2, // scans minimum ayant vu la cellule occupée
  join: 1, // cellules : dilatation pour relier les morceaux d'un même mur avant de compter les objets
});

/**
 * Filtrage 1. Retourne { grid (copie filtrée), removed (Uint8Array : 1 = cellule retirée), objects
 * (nombre d'objets retirés), kept (objets gardés) }.
 */
export function filterClutter(grid, options = {}) {
  const o = { ...DEFAULT_CLUTTER, ...options };
  const W = grid.width;
  const H = grid.height;
  const occ = new Uint8Array(W * H);
  for (let k = 0; k < occ.length; k++) occ[k] = grid.classify(k) === 1 ? 1 : 0;
  // dilatation : deux morceaux d'un même mur séparés d'une cellule forment un seul objet
  const grown = new Uint8Array(W * H);
  for (let j = 0; j < H; j++) {
    for (let i = 0; i < W; i++) {
      if (!occ[j * W + i]) continue;
      for (let dj = -o.join; dj <= o.join; dj++) {
        for (let di = -o.join; di <= o.join; di++) if (grid.inside(i + di, j + dj)) grown[(j + dj) * W + i + di] = 1;
      }
    }
  }
  const label = new Int32Array(W * H).fill(-1);
  const removed = new Uint8Array(W * H);
  let objects = 0;
  let kept = 0;
  for (let start = 0; start < W * H; start++) {
    if (!grown[start] || label[start] >= 0) continue;
    const comp = [start];
    label[start] = start;
    let minI = Infinity;
    let maxI = -Infinity;
    let minJ = Infinity;
    let maxJ = -Infinity;
    for (let q = 0; q < comp.length; q++) {
      const k = comp[q];
      const i = k % W;
      const j = (k - i) / W;
      minI = Math.min(minI, i);
      maxI = Math.max(maxI, i);
      minJ = Math.min(minJ, j);
      maxJ = Math.max(maxJ, j);
      for (let dj = -1; dj <= 1; dj++) {
        for (let di = -1; di <= 1; di++) {
          if (!grid.inside(i + di, j + dj)) continue;
          const n = (j + dj) * W + i + di;
          if (grown[n] && label[n] < 0) {
            label[n] = start;
            comp.push(n);
          }
        }
      }
    }
    const size = Math.hypot(maxI - minI + 1, maxJ - minJ + 1) * grid.res;
    if (size < o.minObject) {
      objects++;
      for (const k of comp) if (occ[k]) removed[k] = 1;
    } else kept++;
  }
  for (let k = 0; k < occ.length; k++) if (occ[k] && grid.hits[k] < o.minHits) removed[k] = 1;
  const out = grid.clone();
  // une cellule retirée était dans l'espace balayé par les faisceaux : on la note libre
  for (let k = 0; k < removed.length; k++) if (removed[k]) out.logOdds[k] = out.params.lMin / 2;
  return { grid: out, removed, objects, kept };
}

export const DEFAULT_ROOM = Object.freeze({
  bin: 0.05, // m : pas des histogrammes et de la grille redressée
  minWall: 1.5, // m : longueur minimale (cumulée) de cellules occupées pour une droite de mur
  minEdge: 1.2, // m : idem pour une droite tirée du bord de l'espace libre
  frame: 2.5, // m : droites assez longues pour fixer le cadre de la salle (murs extérieurs)
  merge: 0.2, // m : deux droites plus proches sont la même
  insideFree: 0.3, // part de cellules vues libres pour qu'un rectangle soit intérieur
  wallEvidence: 0.25, // part d'une frontière couverte de points occupés pour y voir un mur
  completeMaxArea: 12, // m² : taille maximale d'une zone cachée complétée
  nicheMaxArea: 4, // m² : niche coupée du trajet par un obstacle (armoire, personnes), rattachée à la salle
  shortWall: 0.3, // m : morceau de mur vu, accroché à un mur extérieur (décroché de la salle)
  maxObject: 1.5, // m : objet compact (plus grand côté), loin du cadre : un meuble (bureau), pas un mur
  frameGap: 0.3, // m : un objet plus proche du cadre peut être un morceau de mur
  thinWall: 0.25, // m : objet plus mince (petit côté) : un morceau de mur vu de face, pas un meuble
  joinObject: 0.3, // m : morceaux plus proches regroupés en un seul objet
  aroundFree: 0.5, // part vue libre minimale de chacun des 4 côtés d'un meuble isolé
});

/** Direction dominante des murs (degrés dans [0, 90[), affinée au dixième de degré. */
export function wallDirection(segments) {
  const coarse = dominantDirection(segments);
  let sx = 0;
  let sy = 0;
  for (const s of segments) {
    let d = (orientationDeg(s) % 90) - coarse;
    if (d > 45) d -= 90;
    if (d < -45) d += 90;
    if (Math.abs(d) > 4) continue;
    // moyenne circulaire sur 4 directions : angle × 4
    const a = ((coarse + d) * 4 * Math.PI) / 180;
    sx += length(s) * Math.cos(a);
    sy += length(s) * Math.sin(a);
  }
  const fine = (Math.atan2(sy, sx) * 180) / Math.PI / 4;
  return ((fine % 90) + 90) % 90;
}

/**
 * Filtrage 2 + reconstruction. grid : grid map filtrée ; segments : carte vectorielle (pour la
 * direction des murs) ; options.seeds : positions du robot [{x, y}] (trajet), d'où l'intérieur est
 * diffusé. Retourne { angle, walls [{x1,y1,x2,y2, observed}], furniture [{x1,y1,x2,y2}],
 * rects [{x0,y0,x1,y1, label: 'intérieur' | 'complété' | 'extérieur'}] (repère redressé), lines
 * { xs, ys, kinds }, toWorld (repère redressé -> monde), area (m²), grid (grid map propre) }.
 */
export function reconstructRoom(grid, segments, options = {}) {
  const o = { ...DEFAULT_ROOM, ...options };
  const angle = wallDirection(segments.filter((s) => length(s) >= 0.3));
  const c = Math.cos((angle * Math.PI) / 180);
  const s = Math.sin((angle * Math.PI) / 180);
  const toRot = (x, y) => ({ u: c * x + s * y, v: -s * x + c * y });
  const toWorld = (u, v) => ({ x: c * u - s * v, y: s * u + c * v });

  // grille redressée : cellules libres et occupées de la grid map, re-échantillonnées
  const occPts = [];
  const freePts = [];
  for (let j = 0; j < grid.height; j++) {
    for (let i = 0; i < grid.width; i++) {
      const cl = grid.classify(grid.index(i, j));
      if (!cl) continue;
      const p = grid.cellCenter(i, j);
      (cl === 1 ? occPts : freePts).push(toRot(p.x, p.y));
    }
  }
  let uMin = Infinity;
  let vMin = Infinity;
  let uMax = -Infinity;
  let vMax = -Infinity;
  for (const p of occPts.concat(freePts)) {
    if (p.u < uMin) uMin = p.u;
    if (p.v < vMin) vMin = p.v;
    if (p.u > uMax) uMax = p.u;
    if (p.v > vMax) vMax = p.v;
  }
  const u0 = uMin - 0.5;
  const v0 = vMin - 0.5;
  const nu = Math.ceil((uMax + 0.5 - u0) / o.bin);
  const nv = Math.ceil((vMax + 0.5 - v0) / o.bin);
  const raster = (pts) => {
    const r = new Uint8Array(nu * nv);
    for (const p of pts) r[Math.floor((p.v - v0) / o.bin) * nu + Math.floor((p.u - u0) / o.bin)] = 1;
    return r;
  };
  const occ = raster(occPts);
  const free = raster(freePts);
  // images intégrales : somme sur un rectangle en temps constant
  const integral = (r) => {
    const I = new Float64Array((nu + 1) * (nv + 1));
    for (let j = 0; j < nv; j++) {
      let row = 0;
      for (let i = 0; i < nu; i++) {
        row += r[j * nu + i];
        I[(j + 1) * (nu + 1) + i + 1] = I[j * (nu + 1) + i + 1] + row;
      }
    }
    return I;
  };
  const Iocc = integral(occ);
  const Ifree = integral(free);
  const ci = (u) => Math.max(0, Math.min(nu, Math.round((u - u0) / o.bin)));
  const cj = (v) => Math.max(0, Math.min(nv, Math.round((v - v0) / o.bin)));
  const sum = (I, ua, va, ub, vb) => {
    const a = ci(ua);
    const b = ci(ub);
    const d = cj(va);
    const e = cj(vb);
    return I[e * (nu + 1) + b] - I[d * (nu + 1) + b] - I[e * (nu + 1) + a] + I[d * (nu + 1) + a];
  };

  // 2. droites des murs : pics des histogrammes des cellules occupées le long de chaque axe
  const peaks = (hist, min, origin) => {
    const out = [];
    const smooth = hist.map((_, k) => (hist[k - 1] ?? 0) + hist[k] + (hist[k + 1] ?? 0));
    const order = [...smooth.keys()].sort((a, b) => smooth[b] - smooth[a]);
    for (const k of order) {
      if (smooth[k] * o.bin < min) break;
      const pos = origin + (k + 0.5) * o.bin;
      if (out.some((p) => Math.abs(p.pos - pos) < o.merge)) continue;
      out.push({ pos, weight: smooth[k] * o.bin });
    }
    return out;
  };
  const histU = new Float64Array(nu);
  const histV = new Float64Array(nv);
  for (let j = 0; j < nv; j++) {
    for (let i = 0; i < nu; i++) {
      if (!occ[j * nu + i]) continue;
      histU[i]++;
      histV[j]++;
    }
  }
  const wallU = peaks(histU, o.minWall, u0).map((p) => ({ ...p, kind: 'mur' }));
  const wallV = peaks(histV, o.minWall, v0).map((p) => ({ ...p, kind: 'mur' }));
  // bords de l'espace libre : pour chaque ligne, la dernière cellule libre de chaque côté
  const edgeU = new Float64Array(nu);
  const edgeV = new Float64Array(nv);
  for (let j = 0; j < nv; j++) {
    let a = -1;
    let b = -1;
    for (let i = 0; i < nu; i++) if (free[j * nu + i]) (a < 0 && (a = i), (b = i));
    if (a >= 0) {
      edgeU[a]++;
      edgeU[b]++;
    }
  }
  for (let i = 0; i < nu; i++) {
    let a = -1;
    let b = -1;
    for (let j = 0; j < nv; j++) if (free[j * nu + i]) (a < 0 && (a = j), (b = j));
    if (a >= 0) {
      edgeV[a]++;
      edgeV[b]++;
    }
  }
  const addEdges = (walls, edges, origin) => {
    for (const p of peaks(edges, o.minEdge, origin)) if (!walls.some((w) => Math.abs(w.pos - p.pos) < o.merge)) walls.push({ ...p, kind: 'bord du libre' });
    return walls.sort((a, b) => a.pos - b.pos);
  };
  const linesU = addEdges(wallU, edgeU, u0);
  const linesV = addEdges(wallV, edgeV, v0);
  // cadre de la salle : droites extrêmes parmi les plus longues ; rien d'intérieur au-delà (les
  // faisceaux qui sortent par les fenêtres ou les portes ne font pas partie de la salle)
  const frameOf = (lines) => {
    const strong = lines.filter((l) => l.weight >= o.frame);
    return strong.length >= 2 ? [strong[0].pos, strong.at(-1).pos] : [-Infinity, Infinity];
  };
  const [fu0, fu1] = frameOf(linesU);
  const [fv0, fv1] = frameOf(linesV);
  // décrochés : un court morceau de mur vu (trop court pour un pic d'histogramme), accroché à un mur
  // extérieur du cadre, est un retour du mur (coin rentrant de la salle) ; il donne une droite
  const notches = []; // décrochés : { along, pos, s0, s1, g (mur du cadre où il s'accroche), f (mur parallèle) }
  const shortLines = (along, lines, f0, f1, g0, g1) => {
    // along = 'u' : morceaux horizontaux (droites v = cste), accrochés au cadre en u (g0, g1)
    const nA = along === 'u' ? nu : nv;
    const nB = along === 'u' ? nv : nu;
    const at = (a, b) => (along === 'u' ? occ[b * nu + a] : occ[a * nu + b]);
    const origin = along === 'u' ? v0 : u0;
    const originA = along === 'u' ? u0 : v0;
    const minRun = Math.round(o.shortWall / o.bin);
    for (let b = 1; b < nB - 1; b++) {
      const pos = origin + (b + 0.5) * o.bin;
      if (pos <= f0 + o.merge || pos >= f1 - o.merge || lines.some((l) => Math.abs(l.pos - pos) < o.merge)) continue;
      let start = -1;
      let gap = 0;
      for (let a = 0; a <= nA; a++) {
        const hit = a < nA && (at(a, b - 1) || at(a, b) || at(a, b + 1));
        if (hit) {
          if (start < 0) start = a;
          gap = 0;
          continue;
        }
        if (start >= 0 && ++gap > 1) {
          const end = a - gap;
          const s0 = originA + start * o.bin;
          const s1 = originA + (end + 1) * o.bin;
          const attached = Math.abs(s0 - g0) < 0.15 || Math.abs(s1 - g0) < 0.15 || Math.abs(s0 - g1) < 0.15 || Math.abs(s1 - g1) < 0.15;
          // le coin rentrant est entre le morceau et le mur du cadre parallèle le plus proche (1,5 m au plus)
          const f = Math.abs(pos - f0) < Math.abs(pos - f1) ? f0 : f1;
          if (end - start + 1 >= minRun && attached && Math.abs(pos - f) <= 1.5 && !lines.some((l) => Math.abs(l.pos - pos) < o.merge)) {
            lines.push({ pos, weight: (end - start + 1) * o.bin, kind: 'décroché' });
            const g = [g0, g1].reduce((x, y) => (Math.min(Math.abs(s0 - x), Math.abs(s1 - x)) <= Math.min(Math.abs(s0 - y), Math.abs(s1 - y)) ? x : y));
            notches.push({ along, pos, s0, s1, g, f });
          }
          start = -1;
          gap = 0;
        }
      }
    }
    return lines.sort((p, q) => p.pos - q.pos);
  };
  shortLines('u', linesV, fv0, fv1, fu0, fu1);
  shortLines('v', linesU, fu0, fu1, fv0, fv1);
  const xs = [u0, ...linesU.map((l) => l.pos), u0 + nu * o.bin];
  const ys = [v0, ...linesV.map((l) => l.pos), v0 + nv * o.bin];
  const inFrame = (a, b) => xs[a] >= fu0 - 1e-6 && xs[a + 1] <= fu1 + 1e-6 && ys[b] >= fv0 - 1e-6 && ys[b + 1] <= fv1 + 1e-6;

  // 3. rectangles de l'arrangement, étiquetés par la part de cellules vues libres
  const NX = xs.length - 1;
  const NY = ys.length - 1;
  const lab = new Array(NX * NY).fill('extérieur');
  const freeFrac = new Float64Array(NX * NY);
  for (let b = 0; b < NY; b++) {
    for (let a = 0; a < NX; a++) {
      const area = ((xs[a + 1] - xs[a]) * (ys[b + 1] - ys[b])) / (o.bin * o.bin);
      freeFrac[b * NX + a] = sum(Ifree, xs[a], ys[b], xs[a + 1], ys[b + 1]) / Math.max(area, 1);
      if (freeFrac[b * NX + a] >= o.insideFree && inFrame(a, b)) lab[b * NX + a] = 'intérieur';
    }
  }
  // coins rentrants : les rectangles entre un décroché et le mur parallèle du cadre sont hors de la
  // salle, même vus libres (rayons parasites) ; ni la diffusion ni la complétion n'y entrent
  const blocked = new Uint8Array(NX * NY);
  for (const n of notches) {
    const along = n.along === 'u' ? xs : ys;
    const free = Math.abs(n.s0 - n.g) < Math.abs(n.s1 - n.g) ? n.s1 : n.s0; // bout libre du morceau
    const snap = along.reduce((x, y) => (Math.abs(y - free) < Math.abs(x - free) ? y : x));
    const end = Math.abs(snap - free) <= 0.3 ? snap : free;
    const [a0, a1] = [Math.min(end, n.g), Math.max(end, n.g)];
    const [p0, p1] = [Math.min(n.pos, n.f), Math.max(n.pos, n.f)];
    for (let b = 0; b < NY; b++) {
      for (let a = 0; a < NX; a++) {
        const [r0, r1, q0, q1] = n.along === 'u' ? [xs[a], xs[a + 1], ys[b], ys[b + 1]] : [ys[b], ys[b + 1], xs[a], xs[a + 1]];
        if (r0 >= a0 - 1e-6 && r1 <= a1 + 1e-6 && q0 >= p0 - 1e-6 && q1 <= p1 + 1e-6) {
          blocked[b * NX + a] = 1;
          lab[b * NX + a] = 'extérieur';
        }
      }
    }
  }
  // meubles isolés : un objet compact (plus grand côté au plus maxObject, épais), loin des murs du cadre
  // (bureau du prof au milieu de la salle), n'est pas un mur ; il ne ferme pas l'espace derrière lui
  const wallOcc = occ.slice();
  {
    // morceaux à moins de joinObject l'un de l'autre : un même objet (les bouts d'un mur se recollent)
    const R = Math.round(o.joinObject / o.bin);
    const seen = new Uint8Array(nu * nv);
    for (let start = 0; start < nu * nv; start++) {
      if (!occ[start] || seen[start]) continue;
      const comp = [start];
      seen[start] = 1;
      let i0 = Infinity;
      let i1 = -Infinity;
      let j0 = Infinity;
      let j1 = -Infinity;
      for (let q = 0; q < comp.length; q++) {
        const i = comp[q] % nu;
        const j = (comp[q] - i) / nu;
        i0 = Math.min(i0, i);
        i1 = Math.max(i1, i);
        j0 = Math.min(j0, j);
        j1 = Math.max(j1, j);
        for (let dj = -R; dj <= R; dj++) {
          for (let di = -R; di <= R; di++) {
            const n = (j + dj) * nu + i + di;
            if (i + di >= 0 && i + di < nu && j + dj >= 0 && j + dj < nv && occ[n] && !seen[n]) {
              seen[n] = 1;
              comp.push(n);
            }
          }
        }
      }
      const b = { u0: u0 + i0 * o.bin, u1: u0 + (i1 + 1) * o.bin, v0: v0 + j0 * o.bin, v1: v0 + (j1 + 1) * o.bin };
      // un meuble a de l'épaisseur (deux faces vues) ; un objet mince est un morceau de mur
      const compact = Math.max(b.u1 - b.u0, b.v1 - b.v0) <= o.maxObject && Math.min(b.u1 - b.u0, b.v1 - b.v0) > o.thinWall;
      const farFromFrame = b.u0 > fu0 + o.frameGap && b.u1 < fu1 - o.frameGap && b.v0 > fv0 + o.frameGap && b.v1 < fv1 - o.frameGap;
      // un meuble est posé dans la salle : vu libre tout autour (bandes de 30 cm sur ses 4 côtés) ;
      // un morceau de mur a l'extérieur (jamais vu) d'un côté
      const band = (ua, va, ub, vb) => sum(Ifree, ua, va, ub, vb) / Math.max(((ub - ua) * (vb - va)) / (o.bin * o.bin), 1);
      const m = 0.3;
      const around = [band(b.u0 - m, b.v0, b.u0, b.v1), band(b.u1, b.v0, b.u1 + m, b.v1), band(b.u0, b.v0 - m, b.u1, b.v0), band(b.u0, b.v1, b.u1, b.v1 + m)];
      const standalone = Math.min(...around) >= o.aroundFree;
      if (compact && farFromFrame && standalone) for (const k of comp) wallOcc[k] = 0;
    }
  }
  const Iwall = integral(wallOcc);
  // mur sur la frontière commune de deux rectangles voisins : points occupés le long (± 1 pas)
  const wallOnU = (u, va, vb) => sum(Iwall, u - o.bin * 1.5, va, u + o.bin * 1.5, vb) / Math.max((vb - va) / o.bin, 1);
  const wallOnV = (v, ua, ub) => sum(Iwall, ua, v - o.bin * 1.5, ub, v + o.bin * 1.5) / Math.max((ub - ua) / o.bin, 1);
  const isIn = (a, b) => a >= 0 && b >= 0 && a < NX && b < NY && lab[b * NX + a] !== 'extérieur';
  // frontière commune de deux rectangles voisins (a, b) -> (a2, b2) : part couverte de points occupés
  const between = (a, b, a2, b2) =>
    a2 !== a ? wallOnU(xs[Math.max(a, a2)], ys[b], ys[b + 1]) : wallOnV(ys[Math.max(b, b2)], xs[a], xs[a + 1]);

  // diffusion depuis le trajet : seuls les rectangles intérieurs atteignables sans traverser de mur
  const cut = new Uint8Array(NX * NY); // intérieurs coupés du trajet
  if (o.seeds?.length) {
    const rectAt = (u, v) => {
      const a = xs.findIndex((x, k) => k < NX && u >= x && u < xs[k + 1]);
      const b = ys.findIndex((y, k) => k < NY && v >= y && v < ys[k + 1]);
      return a >= 0 && b >= 0 ? [a, b] : null;
    };
    const reached = new Uint8Array(NX * NY);
    const queue = [];
    for (const p of o.seeds) {
      const r = toRot(p.x, p.y);
      const ab = rectAt(r.u, r.v);
      if (ab && !reached[ab[1] * NX + ab[0]] && !blocked[ab[1] * NX + ab[0]]) {
        reached[ab[1] * NX + ab[0]] = 1;
        queue.push(ab);
        if (lab[ab[1] * NX + ab[0]] === 'extérieur') lab[ab[1] * NX + ab[0]] = 'intérieur';
      }
    }
    for (let q = 0; q < queue.length; q++) {
      const [a, b] = queue[q];
      for (const [a2, b2] of [[a - 1, b], [a + 1, b], [a, b - 1], [a, b + 1]]) {
        if (!isIn(a2, b2) || reached[b2 * NX + a2] || blocked[b2 * NX + a2]) continue;
        if (between(a, b, a2, b2) >= o.wallEvidence) continue; // un mur sépare
        reached[b2 * NX + a2] = 1;
        queue.push([a2, b2]);
      }
    }
    for (let k = 0; k < NX * NY; k++) {
      if (lab[k] === 'intérieur' && !reached[k]) {
        lab[k] = 'extérieur';
        cut[k] = 1;
      }
    }
  }

  // niches : petite zone (un ou plusieurs rectangles voisins, sans mur entre eux) vue libre mais coupée
  // du trajet par un obstacle, ou jamais vue, au bord de la salle et bordée par la salle sur deux côtés
  // au moins (coin derrière un bureau, une armoire ou des personnes). L'obstacle n'est pas un mur de la
  // salle : la niche lui est rattachée (occlusions, Mura et al. 2014). Décidé d'un coup sur
  // l'étiquetage courant : une grande zone derrière un mur reste dehors.
  const candidate = (a, b) =>
    a >= 0 && b >= 0 && a < NX && b < NY && !blocked[b * NX + a] && lab[b * NX + a] === 'extérieur' && inFrame(a, b) &&
    (cut[b * NX + a] || freeFrac[b * NX + a] < o.insideFree);
  const grouped = new Uint8Array(NX * NY);
  const niches = [];
  for (let b = 0; b < NY; b++) {
    for (let a = 0; a < NX; a++) {
      if (grouped[b * NX + a] || !candidate(a, b)) continue;
      const group = [[a, b]];
      grouped[b * NX + a] = 1;
      let area = 0;
      let inside = 0;
      let atEdge = false;
      for (let g = 0; g < group.length; g++) {
        const [p, q] = group[g];
        area += (xs[p + 1] - xs[p]) * (ys[q + 1] - ys[q]);
        for (const [p2, q2] of [[p - 1, q], [p + 1, q], [p, q - 1], [p, q + 1]]) {
          if (p2 < 0 || q2 < 0 || p2 >= NX || q2 >= NY || !inFrame(p2, q2)) atEdge = true;
          else if (isIn(p2, q2)) inside++;
          else if (!grouped[q2 * NX + p2] && candidate(p2, q2) && between(p, q, p2, q2) < o.wallEvidence) {
            grouped[q2 * NX + p2] = 1;
            group.push([p2, q2]);
          }
        }
      }
      if (area <= o.nicheMaxArea && inside >= 2 && atEdge) niches.push(...group.map(([p, q]) => q * NX + p));
    }
  }
  for (const k of niches) lab[k] = 'complété';

  // 4. complétion : rectangle non vu, collé à l'intérieur sans mur entre eux (zone cachée)
  for (let pass = 0; pass < 3; pass++) {
    let changed = false;
    for (let b = 0; b < NY; b++) {
      for (let a = 0; a < NX; a++) {
        if (lab[b * NX + a] !== 'extérieur' || !inFrame(a, b) || blocked[b * NX + a]) continue;
        const area = (xs[a + 1] - xs[a]) * (ys[b + 1] - ys[b]);
        if (area > o.completeMaxArea) continue;
        const sides = [
          [a - 1, b, () => wallOnU(xs[a], ys[b], ys[b + 1])],
          [a + 1, b, () => wallOnU(xs[a + 1], ys[b], ys[b + 1])],
          [a, b - 1, () => wallOnV(ys[b], xs[a], xs[a + 1])],
          [a, b + 1, () => wallOnV(ys[b + 1], xs[a], xs[a + 1])],
        ];
        const inside = sides.filter(([p, q]) => isIn(p, q));
        if (!inside.length) continue;
        const open = inside.filter(([, , w]) => w() < o.wallEvidence).length;
        const enclosed = inside.length === 4;
        // au moins deux côtés ouverts sur l'intérieur (coin caché), ou entièrement entouré (meuble)
        if (enclosed || (open >= 2 && freeFrac[b * NX + a] < o.insideFree)) {
          lab[b * NX + a] = 'complété';
          changed = true;
        }
      }
    }
    if (!changed) break;
  }

  // 5. contour : frontières entre intérieur (ou complété) et extérieur ; murs observés ou reconstruits
  const raw = [];
  for (let b = 0; b < NY; b++) {
    for (let a = 0; a < NX; a++) {
      if (!isIn(a, b)) continue;
      if (!isIn(a - 1, b)) raw.push({ u1: xs[a], v1: ys[b], u2: xs[a], v2: ys[b + 1], observed: wallOnU(xs[a], ys[b], ys[b + 1]) >= o.wallEvidence });
      if (!isIn(a + 1, b)) raw.push({ u1: xs[a + 1], v1: ys[b], u2: xs[a + 1], v2: ys[b + 1], observed: wallOnU(xs[a + 1], ys[b], ys[b + 1]) >= o.wallEvidence });
      if (!isIn(a, b - 1)) raw.push({ u1: xs[a], v1: ys[b], u2: xs[a + 1], v2: ys[b], observed: wallOnV(ys[b], xs[a], xs[a + 1]) >= o.wallEvidence });
      if (!isIn(a, b + 1)) raw.push({ u1: xs[a], v1: ys[b + 1], u2: xs[a + 1], v2: ys[b + 1], observed: wallOnV(ys[b + 1], xs[a], xs[a + 1]) >= o.wallEvidence });
    }
  }
  // fusion des morceaux alignés et contigus de même nature
  const merged = [];
  for (const w of raw.sort((p, q) => (p.u1 === p.u2 ? 0 : 1) - (q.u1 === q.u2 ? 0 : 1) || p.u1 - q.u1 || p.v1 - q.v1)) {
    const last = merged.at(-1);
    const vertical = w.u1 === w.u2;
    if (last && last.observed === w.observed && vertical && last.u1 === last.u2 && last.u1 === w.u1 && Math.abs(last.v2 - w.v1) < 1e-9) last.v2 = w.v2;
    else if (last && last.observed === w.observed && !vertical && last.v1 === last.v2 && last.v1 === w.v1 && Math.abs(last.u2 - w.u1) < 1e-9) last.u2 = w.u2;
    else merged.push({ ...w });
  }
  const walls = merged.map((w) => {
    const p = toWorld(w.u1, w.v1);
    const q = toWorld(w.u2, w.v2);
    return { x1: p.x, y1: p.y, x2: q.x, y2: q.y, observed: w.observed };
  });

  // meubles : groupes de cellules occupées à l'intérieur, loin du contour (boîte redressée)
  const furniture = [];
  const seen = new Uint8Array(nu * nv);
  const rectOf = (u, v) => {
    const a = xs.findIndex((x, k) => k < NX && u >= x && u < xs[k + 1]);
    const b = ys.findIndex((y, k) => k < NY && v >= y && v < ys[k + 1]);
    return isIn(a, b);
  };
  for (let start = 0; start < nu * nv; start++) {
    if (!occ[start] || seen[start]) continue;
    const comp = [start];
    seen[start] = 1;
    for (let q = 0; q < comp.length; q++) {
      const i = comp[q] % nu;
      const j = (comp[q] - i) / nu;
      for (let dj = -2; dj <= 2; dj++) {
        for (let di = -2; di <= 2; di++) {
          const n = (j + dj) * nu + i + di;
          if (i + di >= 0 && i + di < nu && j + dj >= 0 && j + dj < nv && occ[n] && !seen[n]) {
            seen[n] = 1;
            comp.push(n);
          }
        }
      }
    }
    const us = comp.map((k) => u0 + ((k % nu) + 0.5) * o.bin);
    const vs = comp.map((k) => v0 + (Math.floor(k / nu) + 0.5) * o.bin);
    const box = { u1: Math.min(...us), v1: Math.min(...vs), u2: Math.max(...us), v2: Math.max(...vs) };
    if (Math.hypot(box.u2 - box.u1, box.v2 - box.v1) < 0.4) continue;
    const cu = (box.u1 + box.u2) / 2;
    const cv = (box.v1 + box.v2) / 2;
    const nearWall = linesU.some((l) => l.kind === 'mur' && Math.abs(l.pos - box.u1) < 0.15 && Math.abs(l.pos - box.u2) < 0.15) || linesV.some((l) => l.kind === 'mur' && Math.abs(l.pos - box.v1) < 0.15 && Math.abs(l.pos - box.v2) < 0.15);
    if (nearWall || !rectOf(cu, cv)) continue;
    // meuble = boîte englobante, s'il n'est pas collé au contour
    const margin = 0.15;
    // collé à un morceau du contour : sur la même droite ET en face (les intervalles se recouvrent)
    const overlaps = (a0, a1, b0, b1) => Math.min(a1, b1) > Math.max(a0, b0) - margin;
    const touches = merged.some((w) =>
      w.u1 === w.u2
        ? (Math.abs(w.u1 - box.u1) < margin || Math.abs(w.u1 - box.u2) < margin) && overlaps(Math.min(w.v1, w.v2), Math.max(w.v1, w.v2), box.v1, box.v2)
        : (Math.abs(w.v1 - box.v1) < margin || Math.abs(w.v1 - box.v2) < margin) && overlaps(Math.min(w.u1, w.u2), Math.max(w.u1, w.u2), box.u1, box.u2));
    if (touches) continue;
    const corners = [toWorld(box.u1, box.v1), toWorld(box.u2, box.v1), toWorld(box.u2, box.v2), toWorld(box.u1, box.v2)];
    for (let k = 0; k < 4; k++) furniture.push({ x1: corners[k].x, y1: corners[k].y, x2: corners[(k + 1) % 4].x, y2: corners[(k + 1) % 4].y });
  }

  // grid map propre : intérieur libre, murs et meubles occupés, extérieur inconnu
  const res = grid.res;
  const clean = new GridMap({ resolution: res, offI: grid.offI, offJ: grid.offJ, width: grid.width, height: grid.height });
  for (let j = 0; j < clean.height; j++) {
    for (let i = 0; i < clean.width; i++) {
      const p = clean.cellCenter(i, j);
      const r = toRot(p.x, p.y);
      if (rectOf(r.u, r.v)) clean.logOdds[clean.index(i, j)] = clean.params.lMin;
    }
  }
  const stamp = (segs) => {
    for (const w of segs) {
      const n = Math.max(1, Math.ceil(Math.hypot(w.x2 - w.x1, w.y2 - w.y1) / (res / 2)));
      for (let k = 0; k <= n; k++) {
        const [i, j] = clean.worldToCell(w.x1 + ((w.x2 - w.x1) * k) / n, w.y1 + ((w.y2 - w.y1) * k) / n);
        if (clean.inside(i, j)) clean.logOdds[clean.index(i, j)] = clean.params.lMax;
      }
    }
  };
  stamp(walls);
  stamp(furniture);

  const rects = [];
  let area = 0;
  for (let b = 0; b < NY; b++) {
    for (let a = 0; a < NX; a++) {
      const l = lab[b * NX + a];
      rects.push({ x0: xs[a], y0: ys[b], x1: xs[a + 1], y1: ys[b + 1], label: l });
      if (l !== 'extérieur') area += (xs[a + 1] - xs[a]) * (ys[b + 1] - ys[b]);
    }
  }
  return { angle, walls, furniture, rects, lines: { u: linesU, v: linesV }, toWorld, area, grid: clean };
}
