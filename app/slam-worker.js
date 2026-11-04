// Fil de calcul (Web Worker) du SLAM en direct : un pas de SLAM coûte 100 à 300 ms dans la salle
// (2 s pour une relocalisation dans toute la salle). Dans la page, ce calcul la bloquait (touches,
// dessin, boutons) ; ici il tourne à côté. La page lui envoie les scans et reçoit, pour chacun, le
// résultat (pose, mode) et, quand la carte change (scan clé, boucle), les poses des scans clés, la
// grid map et les segments de la carte vectorielle.
import { Slam } from '../src/core/slam.js';

let slam = null;

self.onmessage = ({ data }) => {
  if (data.type === 'init') {
    slam = new Slam(data.options);
    return;
  }
  if (data.type !== 'scan') return;
  try {
    const r = slam.addScan(data.scan, data.odom, data.first, data.time);
    const msg = {
      type: 'result',
      id: data.id,
      r: { index: r.index, pose: r.pose, mode: r.mode, inliers: r.inliers, keyframe: r.keyframe, loop: r.loop, time: r.time },
      last: slam.last,
    };
    if (r.keyframe || r.loop) {
      const g = slam.grid;
      msg.keyframes = slam.keyframes.map((kf) => ({ index: kf.index, pose: kf.pose }));
      msg.grid = { resolution: g.res, offI: g.offI, offJ: g.offJ, width: g.width, height: g.height, scanCount: g.scanCount, logOdds: g.logOdds.slice() };
      msg.segments = slam.vector.segments;
    }
    self.postMessage(msg);
  } catch (err) {
    self.postMessage({ type: 'error', id: data.id, message: err.message });
  }
};
