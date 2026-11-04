// SLAM en direct, côté page : le calcul tourne dans un Web Worker (slam-worker.js) ; cette classe en
// garde une copie légère (dernière pose, scans clés, boucles, grid map et carte vectorielle) pour
// l'affichage, avec la même interface que Slam pour ce dont l'appli a besoin. addScan renvoie une
// promesse : la page reste fluide pendant le calcul.
import { GridMap } from '../src/core/gridmap.js';
import { VectorMap } from '../src/core/vectormap.js';

export class LiveSlam {
  constructor(options = {}) {
    this.worker = new Worker(new URL('./slam-worker.js', import.meta.url), { type: 'module' });
    this.worker.postMessage({ type: 'init', options });
    this.worker.onmessage = ({ data }) => this.receive(data);
    this.worker.onerror = (ev) => {
      for (const p of this.pending.values()) p.reject(new Error(`SLAM : ${ev.message}`));
      this.pending.clear();
    };
    this.pending = new Map();
    this.seq = 0;
    this.keyframes = []; // [{ index, pose }]
    this.poses = new Map(); // index de scan clé -> pose
    this.loops = [];
    this.last = null;
    this.grid = null;
    this.vector = new VectorMap([], { absence: false });
  }

  /** Comme Slam.addScan, mais asynchrone : la promesse donne le résultat du scan. */
  addScan(scan, odom = null, first = null) {
    const id = ++this.seq;
    const plain = { name: scan.name, pose: scan.pose, points: scan.points, format: scan.format, comments: scan.comments, elevated: scan.elevated };
    return new Promise((resolve, reject) => {
      this.pending.set(id, { resolve, reject });
      this.worker.postMessage({ type: 'scan', id, scan: plain, odom, first, time: performance.now() });
    });
  }

  receive(data) {
    const p = this.pending.get(data.id);
    this.pending.delete(data.id);
    if (!p) return;
    if (data.type === 'error') return p.reject(new Error(`SLAM : ${data.message}`));
    this.last = data.last;
    if (data.keyframes) {
      this.keyframes = data.keyframes;
      this.poses = new Map(data.keyframes.map((k) => [k.index, k.pose]));
      const g = data.grid;
      this.grid = new GridMap({ resolution: g.resolution, offI: g.offI, offJ: g.offJ, width: g.width, height: g.height });
      this.grid.logOdds.set(g.logOdds);
      this.grid.scanCount = g.scanCount;
      this.vector = new VectorMap(data.segments, { absence: false });
    }
    if (data.r.loop) this.loops.push(data.r.loop);
    p.resolve(data.r);
  }

  /** Pose actuelle du scan clé pris au scan n° index (corrigée par les fermetures de boucle). */
  poseOf(index) {
    return this.poses.get(index);
  }

  terminate() {
    this.worker.terminate();
    for (const p of this.pending.values()) p.reject(new Error('SLAM réinitialisé'));
    this.pending.clear();
  }
}
