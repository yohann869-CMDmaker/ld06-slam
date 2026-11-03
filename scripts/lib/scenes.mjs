// Scènes simulées du jour 3 (SLAM), partagées par scripts/benchmark-slam.mjs et scripts/gen-trajet.mjs.
import { rectangle } from '../../src/core/sim.js';

/** Salle de classe simulée 10 m × 8 m : table au centre, armoire, bureaux, pilier, tableau en diagonale. */
export const CLASSROOM = [
  ...rectangle(0, 0, 10, 8),
  ...rectangle(3.5, 3, 6, 4.4), // table
  ...rectangle(9.4, 1, 10, 2.6), // armoire
  ...rectangle(1, 7.3, 4, 8), // bureaux
  ...rectangle(2, 1.2, 2.3, 1.5), // pilier
  { x1: 0.3, y1: 1.3, x2: 1.3, y2: 0.3 }, // tableau
];

/** Boucle de 21 m autour de la table. */
export const CLASSROOM_LOOP = [{ x: 1.5, y: 2.2 }, { x: 8.2, y: 2.2 }, { x: 8.2, y: 6.2 }, { x: 1.5, y: 6.2 }, { x: 1.5, y: 2.2 }];

/** Couloir en boucle de 3 m de large autour d'un bloc central (24 m × 10 m), quelques piliers et renfoncements. */
export const CORRIDOR = [...rectangle(0, 0, 24, 10), ...rectangle(3, 3, 21, 7)];
for (const x of [6, 11, 15, 19]) CORRIDOR.push(...rectangle(x, 0, x + 0.3, 0.4));
for (const x of [4, 9, 13.5, 18]) CORRIDOR.push(...rectangle(x, 9.6, x + 0.5, 10));
CORRIDOR.push(...rectangle(0, 4, 0.5, 5.2), ...rectangle(23.4, 6, 24, 6.8));

/** Boucle de 54 m dans le couloir. */
export const CORRIDOR_LOOP = [{ x: 1.5, y: 1.5 }, { x: 22.5, y: 1.5 }, { x: 22.5, y: 8.5 }, { x: 1.5, y: 8.5 }, { x: 1.5, y: 1.5 }];
