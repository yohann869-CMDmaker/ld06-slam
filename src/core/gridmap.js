// Grid map (carte d'occupation) 2D probabiliste.
//
// Chaque cellule stocke le log-odds de sa probabilité d'occupation :
//   l = ln(p / (1 - p)), l = 0 <=> p = 0,5 (inconnu).
// La mise à jour bayésienne d'une cellule devient une simple addition
// (+lOcc pour un impact, +lFree pour une cellule traversée), bornée dans
// [lMin, lMax] pour que la carte puisse encore changer d'avis (objets déplacés).
// À l'export, p est ramenée sur un octet : 0 = libre, 128 = inconnu, 255 = occupé.
//
// Indices : i = colonne (axe X), j = ligne (axe Y, vers le haut).
// La grille est alignée sur des multiples de la résolution : la cellule (i, j)
// couvre [(offI + i) * res, (offI + i + 1) * res[ en X, idem en Y.

import { scanToWorldXY, DEFAULT_FILTER } from './scan.js';

export const DEFAULT_GRID = Object.freeze({
  resolution: 0.05, // m par cellule
  lOcc: 0.85, // impact : p passe de 0,5 à 0,70
  lFree: -0.4, // traversée : p passe de 0,5 à 0,40
  lMin: -4, // p >= 0,018
  lMax: 4, // p <= 0,982
  // Seuils de classement : une seule observation suffit à classer une cellule
  // (impact -> 0,70 >= pOcc, traversée -> 0,40 <= pFree), sinon un scan isolé
  // ne donnerait qu'une carte « inconnue ».
  pOcc: 0.6,
  pFree: 0.45,
  freeMargin: 0.05, // m : pas de marquage libre dans les 5 derniers cm avant l'impact (~5σ du bruit)
  fillGaps: true, // faisceaux intermédiaires (libres) entre deux impacts voisins
  fillMaxAngleDeg: 2, // au-delà, les deux impacts ne sont pas consécutifs
});

export const UNKNOWN_RGB = Object.freeze([214, 219, 224]);
const FREE_RGB = [252, 252, 250];
const OCC_RGB = [29, 35, 39];

/**
 * Couleur d'affichage d'une cellule : blanc = libre, gris-bleu = inconnu, encre = occupé,
 * plus ou moins marqué selon la confiance (|log-odds|). Un affichage linéaire de p rendrait
 * gris moyen une cellule libre vue une seule fois (p = 0,40), difficile à distinguer de l'inconnu.
 */
export function occupancyRGB(grid, k) {
  const c = grid.classify(k);
  if (c === 0) return UNKNOWN_RGB;
  const t = 0.55 + 0.45 * Math.min(1, Math.abs(grid.logOdds[k]) / 2);
  const target = c < 0 ? FREE_RGB : OCC_RGB;
  return UNKNOWN_RGB.map((u, i) => Math.round(u + (target[i] - u) * t));
}

/**
 * Tracé de segment de Bresenham entre deux cellules, extrémités comprises.
 * Appelle visit(i, j) pour chaque cellule, dans l'ordre du départ vers l'arrivée.
 */
export function bresenham(i0, j0, i1, j1, visit) {
  const dx = Math.abs(i1 - i0);
  const dy = -Math.abs(j1 - j0);
  const sx = i0 < i1 ? 1 : -1;
  const sy = j0 < j1 ? 1 : -1;
  let err = dx + dy;
  let i = i0;
  let j = j0;
  for (;;) {
    visit(i, j);
    if (i === i1 && j === j1) return;
    const e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      i += sx;
    }
    if (e2 <= dx) {
      err += dx;
      j += sy;
    }
  }
}

const logit = (p) => Math.log(p / (1 - p));

export class GridMap {
  /**
   * @param {object} o  { resolution, offI, offJ, width, height, ...paramètres DEFAULT_GRID }
   */
  constructor(o = {}) {
    this.params = { ...DEFAULT_GRID, ...o };
    this.res = this.params.resolution;
    this.offI = o.offI ?? 0;
    this.offJ = o.offJ ?? 0;
    this.width = o.width ?? 1;
    this.height = o.height ?? 1;
    this.scanCount = 0;
    this.#allocate();
  }

  #allocate() {
    const n = this.width * this.height;
    this.logOdds = new Float32Array(n);
    this.hits = new Uint16Array(n); // nombre d'impacts reçus (compteur de confiance)
    this.lastSeen = new Int32Array(n).fill(-1); // numéro du dernier scan ayant observé la cellule
    this.occStamp = new Int32Array(n); // dédoublonnage : une seule mise à jour par cellule et par scan
    this.freeStamp = new Int32Array(n);
  }

  /** Grille couvrant une boîte {minX, minY, maxX, maxY} plus une marge (m). */
  static fromBounds(b, params = {}, margin = 1) {
    const res = params.resolution ?? DEFAULT_GRID.resolution;
    const i0 = Math.floor((b.minX - margin) / res);
    const j0 = Math.floor((b.minY - margin) / res);
    const i1 = Math.floor((b.maxX + margin) / res);
    const j1 = Math.floor((b.maxY + margin) / res);
    return new GridMap({ ...params, offI: i0, offJ: j0, width: i1 - i0 + 1, height: j1 - j0 + 1 });
  }

  get originX() {
    return this.offI * this.res;
  }

  get originY() {
    return this.offJ * this.res;
  }

  /** Cellule contenant le point monde (x, y). Peut être hors de la grille. */
  worldToCell(x, y) {
    return [Math.floor(x / this.res) - this.offI, Math.floor(y / this.res) - this.offJ];
  }

  /** Centre de la cellule (i, j) dans le repère monde. */
  cellCenter(i, j) {
    return { x: (this.offI + i + 0.5) * this.res, y: (this.offJ + j + 0.5) * this.res };
  }

  inside(i, j) {
    return i >= 0 && j >= 0 && i < this.width && j < this.height;
  }

  index(i, j) {
    return j * this.width + i;
  }

  /**
   * Agrandit la grille si la boîte b n'est pas couverte (avec une marge), en
   * conservant le contenu. Retourne true si la grille a été agrandie.
   */
  ensureContains(b, margin = 1) {
    const [ia, ja] = this.worldToCell(b.minX, b.minY);
    const [ib, jb] = this.worldToCell(b.maxX, b.maxY);
    if (ia >= 0 && ja >= 0 && ib < this.width && jb < this.height) return false;
    const m = Math.ceil(margin / this.res);
    const left = ia < 0 ? -ia + m : 0;
    const bottom = ja < 0 ? -ja + m : 0;
    const right = ib >= this.width ? ib - this.width + 1 + m : 0;
    const top = jb >= this.height ? jb - this.height + 1 + m : 0;
    const old = { w: this.width, logOdds: this.logOdds, hits: this.hits, lastSeen: this.lastSeen };
    const oldH = this.height;
    this.width += left + right;
    this.height += bottom + top;
    this.offI -= left;
    this.offJ -= bottom;
    this.#allocate();
    for (let j = 0; j < oldH; j++) {
      const src = j * old.w;
      const dst = (j + bottom) * this.width + left;
      this.logOdds.set(old.logOdds.subarray(src, src + old.w), dst);
      this.hits.set(old.hits.subarray(src, src + old.w), dst);
      this.lastSeen.set(old.lastSeen.subarray(src, src + old.w), dst);
    }
    return true;
  }

  /**
   * Intègre un scan pris depuis `pose` :
   *  1. points lidar -> repère monde -> cellules d'impact ;
   *  2. cellules traversées par chaque faisceau (Bresenham, robot -> impact, impact exclu) : lFree,
   *     plus des faisceaux intermédiaires entre impacts voisins (option fillGaps) ;
   *  3. cellules d'impact : lOcc.
   * Une cellule n'est mise à jour qu'une fois par scan, et si un faisceau s'y arrête,
   * l'occupation l'emporte sur les faisceaux qui la traversent (évite d'éroder les murs
   * vus en incidence rasante).
   */
  integrateScan(scan, pose = scan.pose, filter = DEFAULT_FILTER, options = {}) {
    const pts = scanToWorldXY(scan, pose, filter, options);
    if (options.grow !== false && pts.length) {
      const xs = pts.map((p) => p.x).concat(pose.x);
      const ys = pts.map((p) => p.y).concat(pose.y);
      this.ensureContains({ minX: Math.min(...xs), minY: Math.min(...ys), maxX: Math.max(...xs), maxY: Math.max(...ys) });
    }
    const { lOcc, lFree, lMin, lMax } = this.params;
    const stamp = ++this.scanCount;
    const [ri, rj] = this.worldToCell(pose.x, pose.y);

    const hitCells = [];
    for (const p of pts) {
      const [i, j] = this.worldToCell(p.x, p.y);
      if (!this.inside(i, j)) continue;
      const k = this.index(i, j);
      if (this.occStamp[k] !== stamp) {
        this.occStamp[k] = stamp;
        hitCells.push(k);
      }
    }

    let freeCells = 0;
    // Le faisceau vers (x, y) libère les cellules jusqu'à freeMargin avant l'impact, cellule
    // de ce point exclue : un impact bruité juste derrière un mur ne libère pas la cellule du mur.
    const m = this.params.freeMargin;
    const traceFree = (x, y) => {
      const d = Math.hypot(x - pose.x, y - pose.y);
      if (d <= m) return;
      const [ei, ej] = this.worldToCell(pose.x + ((x - pose.x) * (d - m)) / d, pose.y + ((y - pose.y) * (d - m)) / d);
      bresenham(ri, rj, ei, ej, (i, j) => {
        if ((i === ei && j === ej) || !this.inside(i, j)) return;
        const k = this.index(i, j);
        if (this.occStamp[k] === stamp || this.freeStamp[k] === stamp) return;
        this.freeStamp[k] = stamp;
        this.logOdds[k] = Math.max(lMin, this.logOdds[k] + lFree);
        this.lastSeen[k] = stamp;
        freeCells++;
      });
    };
    for (const p of pts) traceFree(p.x, p.y);

    // Faisceaux intermédiaires : deux faisceaux réels voisins sont espacés de r·dθ
    // (5,6 cm à 4 m pour dθ = 0,8°). Au-delà d'une cellule d'écart, des cellules libres
    // ne seraient jamais traversées (hachures). Entre deux impacts consécutifs sur une même
    // surface, on trace des faisceaux virtuels qui ne marquent que du libre.
    if (this.params.fillGaps) {
      for (let k = 1; k < pts.length; k++) {
        const p = pts[k - 1];
        const q = pts[k];
        if (q.angle - p.angle > this.params.fillMaxAngleDeg) continue; // mesures manquantes entre les deux
        if (Math.abs(q.r - p.r) > 0.1 * Math.min(p.r, q.r) + 0.05) continue; // pas la même surface
        const n = Math.floor(Math.hypot(q.x - p.x, q.y - p.y) / this.res);
        for (let s = 1; s <= n; s++) {
          const t = s / (n + 1);
          traceFree(p.x + t * (q.x - p.x), p.y + t * (q.y - p.y));
        }
      }
    }

    for (const k of hitCells) {
      this.logOdds[k] = Math.min(lMax, this.logOdds[k] + lOcc);
      if (this.hits[k] < 65535) this.hits[k]++;
      this.lastSeen[k] = stamp;
    }
    return { points: pts.length, freeCells, occupiedCells: hitCells.length };
  }

  probability(k) {
    return 1 / (1 + Math.exp(-this.logOdds[k]));
  }

  /** Valeur sur un octet : 0 = libre, 128 = inconnu, 255 = occupé. */
  value(k) {
    return Math.round(255 * this.probability(k));
  }

  /** -1 = libre, 0 = inconnu, 1 = occupé (seuils pFree / pOcc). */
  classify(k) {
    const p = this.probability(k);
    if (p >= this.params.pOcc) return 1;
    if (p <= this.params.pFree) return -1;
    return 0;
  }

  /** Centres des cellules occupées (utile pour l'affichage et le scan matching). */
  occupiedCells() {
    const out = [];
    for (let j = 0; j < this.height; j++) {
      for (let i = 0; i < this.width; i++) {
        if (this.classify(this.index(i, j)) === 1) out.push(this.cellCenter(i, j));
      }
    }
    return out;
  }

  /** Comptage des cellules libres / inconnues / occupées. */
  stats() {
    const s = { free: 0, unknown: 0, occupied: 0 };
    for (let k = 0; k < this.logOdds.length; k++) {
      const c = this.classify(k);
      if (c === 1) s.occupied++;
      else if (c === -1) s.free++;
      else s.unknown++;
    }
    return s;
  }

  /**
   * Décroissance temporelle : les cellules non observées depuis au moins `minAge`
   * scans voient leur log-odds multiplié par `factor` (retour progressif vers inconnu).
   */
  decay(factor = 0.9, minAge = 1) {
    let n = 0;
    for (let k = 0; k < this.logOdds.length; k++) {
      if (this.logOdds[k] !== 0 && this.scanCount - this.lastSeen[k] >= minAge) {
        this.logOdds[k] *= factor;
        n++;
      }
    }
    return n;
  }

  /** Filtrage : une cellule occupée sans aucune voisine occupée (8-voisinage) repasse à inconnu. */
  removeIsolated() {
    const occ = new Uint8Array(this.logOdds.length);
    for (let k = 0; k < occ.length; k++) occ[k] = this.classify(k) === 1 ? 1 : 0;
    let removed = 0;
    for (let j = 0; j < this.height; j++) {
      for (let i = 0; i < this.width; i++) {
        const k = this.index(i, j);
        if (!occ[k]) continue;
        let neighbours = 0;
        for (let dj = -1; dj <= 1; dj++) {
          for (let di = -1; di <= 1; di++) {
            if ((di || dj) && this.inside(i + di, j + dj)) neighbours += occ[this.index(i + di, j + dj)];
          }
        }
        if (neighbours === 0) {
          this.logOdds[k] = 0;
          removed++;
        }
      }
    }
    return removed;
  }

  /** Niveaux de gris ligne par ligne, du haut vers le bas (blanc = libre, noir = occupé). */
  toGray() {
    const out = new Uint8Array(this.width * this.height);
    for (let j = 0; j < this.height; j++) {
      const row = (this.height - 1 - j) * this.width;
      for (let i = 0; i < this.width; i++) out[row + i] = 255 - this.value(this.index(i, j));
    }
    return out;
  }

  clone() {
    const g = new GridMap({ ...this.params, offI: this.offI, offJ: this.offJ, width: this.width, height: this.height });
    g.logOdds.set(this.logOdds);
    g.hits.set(this.hits);
    g.lastSeen.set(this.lastSeen);
    g.scanCount = this.scanCount;
    return g;
  }

  /** Export CSV : en-tête de métadonnées puis une ligne de valeurs 0..255 par rangée, du haut vers le bas. */
  toCSV() {
    const lines = [
      '# gridmap ld06-slam',
      `# resolution=${this.res} originX=${this.originX.toFixed(4)} originY=${this.originY.toFixed(4)} width=${this.width} height=${this.height}`,
      '# probabilité d\'occupation sur un octet : 0 = libre, 128 = inconnu, 255 = occupé ; lignes du haut (Y max) vers le bas',
    ];
    for (let j = this.height - 1; j >= 0; j--) {
      const row = new Array(this.width);
      for (let i = 0; i < this.width; i++) row[i] = this.value(this.index(i, j));
      lines.push(row.join(','));
    }
    return lines.join('\n') + '\n';
  }

  /**
   * Import d'une grille exportée par toCSV(), ou par MapTool
   * ("# Grid map NxM, cell=0.1 m ; 1=occupied,0=free", origine en (0, 0)).
   */
  static fromCSV(text, params = {}) {
    const lines = text.split(/\r?\n/).map((l) => l.trim()).filter(Boolean);
    const header = lines.filter((l) => l.startsWith('#')).join(' ');
    const rows = lines.filter((l) => !l.startsWith('#')).map((l) => l.split(',').map(Number));
    const height = rows.length;
    const width = rows[0]?.length ?? 0;
    const maptool = /cell\s*=\s*([\d.]+)/i.exec(header);
    const ours = /resolution=([\d.]+)\s+originX=(-?[\d.]+)\s+originY=(-?[\d.]+)/.exec(header);
    let res;
    let originX = 0;
    let originY = 0;
    if (ours) {
      res = Number(ours[1]);
      originX = Number(ours[2]);
      originY = Number(ours[3]);
    } else if (maptool) {
      res = Number(maptool[1]);
    } else {
      throw new Error('En-tête de grid map non reconnu');
    }
    const g = new GridMap({
      ...params,
      resolution: res,
      offI: Math.round(originX / res),
      offJ: Math.round(originY / res),
      width,
      height,
    });
    const { lMin, lMax } = g.params;
    for (let r = 0; r < height; r++) {
      const j = height - 1 - r;
      for (let i = 0; i < width; i++) {
        const v = rows[r][i];
        let l;
        if (!ours) l = v >= 1 ? lMax : lMin; // MapTool : binaire, pas d'inconnu
        else if (v === 128) l = 0;
        else l = Math.min(lMax, Math.max(lMin, logit(Math.min(0.999, Math.max(0.001, v / 255)))));
        g.logOdds[g.index(i, j)] = l;
      }
    }
    return g;
  }
}

/** Outil 1 : scan + pose -> nouvelle carte d'occupation. */
export function gridFromScan(scan, pose = scan.pose, params = {}, filter = DEFAULT_FILTER, options = {}) {
  const pts = scanToWorldXY(scan, pose, filter, options);
  const xs = pts.map((p) => p.x).concat(pose.x);
  const ys = pts.map((p) => p.y).concat(pose.y);
  const grid = GridMap.fromBounds(
    { minX: Math.min(...xs), minY: Math.min(...ys), maxX: Math.max(...xs), maxY: Math.max(...ys) },
    params,
  );
  grid.integrateScan(scan, pose, filter, options);
  return grid;
}

/** Outil 2 : carte existante + scan + pose -> carte mise à jour (copie, l'original n'est pas modifié). */
export function updateGrid(grid, scan, pose = scan.pose, filter = DEFAULT_FILTER, options = {}) {
  const g = grid.clone();
  g.integrateScan(scan, pose, filter, options);
  return g;
}
