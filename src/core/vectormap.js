// Carte vectorielle : liste de segments (murs, faces de meubles).
//
// Intégration d'un scan + pose (document « Construction d'une carte vectorielle ») :
//  1. points lidar filtrés -> repère monde ;
//  2. gestion des absences : chaque faisceau (robot -> impact, raccourci d'une marge)
//     qui traverse un segment de la carte prouve que ce segment n'existe plus à cet
//     endroit : le segment est découpé (cutSegment, on garde le plus long côté) ;
//  3. extraction des segments du scan (clusters + split-and-merge) ;
//  4. chaque nouveau segment est fusionné avec les segments « proches » de la carte
//     (areClose + mergeSegments, en cascade), sinon ajouté ;
//  5. nettoyage : suppression des segments devenus trop courts.

import { angleBetweenDeg, areClose, cutSegment, DEFAULT_CLOSE, length, mergeSegments } from './geometry.js';
import { DEFAULT_FILTER, scanToWorldXY } from './scan.js';
import { DEFAULT_SEGMENTATION, extractSegments } from './segmentation.js';

export const DEFAULT_VECTORMAP = Object.freeze({
  close: DEFAULT_CLOSE,
  segmentation: DEFAULT_SEGMENTATION,
  minLength: 0.15, // m : en dessous, un segment est supprimé
  absence: true, // gestion des absences (découpage par les faisceaux)
  rayMargin: 0.15, // m : le faisceau est raccourci avant l'impact pour ne pas couper l'objet touché
  minCrossAngleDeg: 20, // un faisceau trop rasant ne découpe pas (sensible aux erreurs de pose)
});

export class VectorMap {
  constructor(segments = [], params = {}) {
    this.params = { ...DEFAULT_VECTORMAP, ...params };
    this.segments = segments.map((s) => ({ observations: 1, ...s }));
    this.scanCount = 0;
  }

  clone() {
    const m = new VectorMap([], this.params);
    m.segments = this.segments.map((s) => ({ ...s }));
    m.scanCount = this.scanCount;
    return m;
  }

  /**
   * Fusionne un segment avec ses voisins proches (en cascade) ou l'ajoute. Retourne le nombre de fusions.
   * `scans` liste les numéros des scans qui ont vu le segment : deux morceaux d'un même
   * mur extraits du même scan ne comptent que pour une observation.
   */
  addSegment(segment) {
    let s = { observations: 1, ...segment };
    let merges = 0;
    for (;;) {
      const k = this.segments.findIndex((m) => areClose(m, s, this.params.close));
      if (k < 0) break;
      const m = this.segments[k];
      const scans = [...new Set([...(m.scans ?? []), ...(s.scans ?? [])])];
      s = mergeSegments(m, s);
      s.scans = scans;
      if (scans.length) s.observations = scans.length;
      this.segments.splice(k, 1);
      merges++;
    }
    this.segments.push(s);
    return merges;
  }

  /**
   * Découpe les segments traversés par les faisceaux du scan.
   * rays : [{x1, y1, x2, y2}] du robot vers l'impact, déjà raccourcis.
   */
  applyAbsences(rays) {
    let cuts = 0;
    for (const ray of rays) {
      for (let k = 0; k < this.segments.length; k++) {
        const s = this.segments[k];
        if (angleBetweenDeg(s, ray) < this.params.minCrossAngleDeg) continue;
        const r = cutSegment(s, ray);
        if (r.cut) {
          this.segments[k] = r.segment;
          cuts++;
        }
      }
    }
    return cuts;
  }

  /** Intègre un scan pris depuis `pose`. Retourne des statistiques. */
  integrateScan(scan, pose = scan.pose, filter = DEFAULT_FILTER, options = {}) {
    const p = this.params;
    const pts = scanToWorldXY(scan, pose, filter, options);
    this.scanCount++;

    let cuts = 0;
    if (p.absence && this.segments.length) {
      const rays = [];
      for (const q of pts) {
        if (q.r <= p.rayMargin) continue;
        const k = (q.r - p.rayMargin) / q.r;
        rays.push({ x1: pose.x, y1: pose.y, x2: pose.x + k * (q.x - pose.x), y2: pose.y + k * (q.y - pose.y) });
      }
      cuts = this.applyAbsences(rays);
    }

    const { segments } = extractSegments(pts, p.segmentation);
    let merged = 0;
    let added = 0;
    for (const s of segments) {
      const m = this.addSegment({ ...s, scans: [this.scanCount] });
      if (m) merged += m;
      else added++;
    }

    const before = this.segments.length;
    this.segments = this.segments.filter((s) => length(s) >= p.minLength);
    return { extracted: segments.length, added, merged, cuts, removed: before - this.segments.length };
  }

  /** Export au format des outils du module : une ligne x1,y1,x2,y2 par segment. */
  toTXT() {
    const lines = ['# Carte vectorielle ld06-slam', '# x1,y1,x2,y2 (m)'];
    for (const s of this.segments) lines.push([s.x1, s.y1, s.x2, s.y2].map((v) => v.toFixed(4)).join(','));
    return lines.join('\n') + '\n';
  }

  static fromTXT(text, params = {}) {
    const segs = [];
    for (const line of text.split(/\r?\n/)) {
      const l = line.trim();
      if (!l || l.startsWith('#')) continue;
      const v = l.split(',').map(Number);
      if (v.length >= 4 && v.slice(0, 4).every(Number.isFinite)) segs.push({ x1: v[0], y1: v[1], x2: v[2], y2: v[3] });
    }
    return new VectorMap(segs, params);
  }
}

/** Outil 1 : scan + pose -> nouvelle carte vectorielle. */
export function vectorMapFromScan(scan, pose = scan.pose, params = {}, filter = DEFAULT_FILTER, options = {}) {
  const m = new VectorMap([], params);
  m.integrateScan(scan, pose, filter, options);
  return m;
}

/** Outil 2 : carte vectorielle + scan + pose -> carte mise à jour (copie). */
export function updateVectorMap(map, scan, pose = scan.pose, filter = DEFAULT_FILTER, options = {}) {
  const m = map.clone();
  m.integrateScan(scan, pose, filter, options);
  return m;
}
