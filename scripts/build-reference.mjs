// Construit la carte de référence de la salle (jour 2) à partir des scans réels :
//   1. lecture des scans et de leurs poses (posées à la main dans l'application) ;
//   2. recalage automatique (multi-résolution + ICP, chaque scan face aux autres) ;
//   3. grid map et carte vectorielle (scans pris au sol seulement), puis versions propres,
//      murs et meubles (posés au sol ou surélevés) et coins ;
//   4. enregistrement : data/reference/carte_reference.json (chargeable dans l'application),
//      data/reference/poses_recalees.csv et un aperçu docs/img/j2_carte_reference.png.
// Usage : node scripts/build-reference.mjs [dossier des scans, défaut data/real]
import { existsSync } from 'node:fs';
import { mkdir, readdir, readFile, writeFile } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { looksElevated, parseScanCSV, scanToLocalXY, scanToWorldXY } from '../src/core/scan.js';
import { gridFromScan } from '../src/core/gridmap.js';
import { VectorMap } from '../src/core/vectormap.js';
import { autoAlign, consistency } from '../src/core/matching.js';
import { cleanMaps, cleanVectorMap, isPartlyOccupied, labelPositions, objectGroups, parseLabels } from '../src/core/cleanup.js';
import { makeReferenceMap, referenceBounds, serializeReferenceMap } from '../src/core/refmap.js';
import { hex, Raster } from './lib/raster.mjs';

const ROOT = fileURLToPath(new URL('../', import.meta.url));
const dir = process.argv[2] ?? ROOT + 'data/real';
const files = (await readdir(dir)).filter((f) => f.startsWith('scan_') && /\.(csv|txt)$/.test(f)).sort();
if (files.length < 2) {
  console.error(`Il faut au moins 2 scans dans ${dir}`);
  process.exit(1);
}
const scans = [];
for (const f of files) scans.push(parseScanCSV(await readFile(`${dir}/${f}`, 'utf8'), f));

// 2. recalage
const items = scans.map((s) => ({ points: scanToLocalXY(s), pose: s.pose }));
const steps = autoAlign(items);
let r;
while (!(r = steps.next()).done);
const poses = r.value;
const mean = (a) => Math.round((100 * a.reduce((x, y) => x + y, 0)) / a.length);
const before = mean(consistency(items.map((i) => i.points), items.map((i) => i.pose)));
const after = mean(consistency(items.map((i) => i.points), poses));

// 3. cartes brutes puis propres, avec les scans pris au sol seulement : un scan pris en hauteur
//    (robot posé sur l'armoire ou sur la table) passe au-dessus des meubles et les effacerait.
//    Il a servi au recalage ci-dessus (il voit bien les murs). Pas de découpe par les faisceaux :
//    sur données réelles, une petite erreur de pose suffirait à découper des murs.
for (const s of scans) {
  if (!s.elevated && looksElevated(s)) console.warn(`Attention : ${s.name} semble pris en hauteur (rien à moins de 1 m) ; sinon ajouter « # pris en hauteur : ... » dans le fichier`);
}
const floor = scans.map((s, k) => k).filter((k) => !scans[k].elevated);
const grid = gridFromScan(scans[floor[0]], poses[floor[0]]);
floor.slice(1).forEach((k) => grid.integrateScan(scans[k], poses[k]));
const vm = new VectorMap([], { absence: false });
floor.forEach((k) => vm.integrateScan(scans[k], poses[k]));
const views = floor.map((k) => ({ pose: poses[k], points: scanToWorldXY(scans[k], poses[k]) }));
// frontière de la salle : murs vus par tous les scans (d'en haut, le mur derrière l'armoire est visible)
const vmAll = new VectorMap([], { absence: false });
scans.forEach((s, k) => vmAll.integrateScan(s, poses[k]));
const barriers = cleanVectorMap(vmAll.segments).segments;
// carte sémantique : meubles nommés à la main (table, armoire, tableau) ; les autres objets
// détectés sont retirés de la carte vectorielle
const labelsFile = ROOT + 'data/reference/etiquettes.json';
const labels = existsSync(labelsFile) ? parseLabels(await readFile(labelsFile, 'utf8')) : [];
const vc = cleanMaps(grid, vm.segments, floor.map((k) => poses[k]), { views, barriers, labels });
const gridClean = vc.grid; // plan (affichage)
const corners = vc.corners;
const objects = vc.segments.filter((s) => s.kind === 'objet');

// 4. enregistrement
const ref = makeReferenceMap({
  name: `Salle de classe (${scans.length} scans)`,
  grid: vc.gridLoc, // localisation : tout ce que voit le lidar au sol
  plan: gridClean, // affichage : murs et meubles nommés
  segments: vc.segments,
  corners,
  labels,
  scans: scans.map((s, k) => ({ name: s.name, pose: poses[k], elevated: s.elevated })),
  clean: true,
});
await mkdir(ROOT + 'data/reference', { recursive: true });
await writeFile(ROOT + 'data/reference/carte_reference.json', serializeReferenceMap(ref));
await writeFile(
  ROOT + 'data/reference/poses_recalees.csv',
  ['scan,x_m,y_m,theta_deg', ...scans.map((s, k) => `${s.name},${poses[k].x.toFixed(4)},${poses[k].y.toFixed(4)},${poses[k].theta.toFixed(2)}`)].join('\n') + '\n',
);

const b = referenceBounds(ref, 0.4);
const img = new Raster(b, 70, '#d6dbe0');
for (let j = 0; j < gridClean.height; j++) {
  for (let i = 0; i < gridClean.width; i++) {
    const c = gridClean.classify(gridClean.index(i, j));
    if (c === 0) continue;
    const col = c === -1 ? [255, 255, 255] : isPartlyOccupied(gridClean, gridClean.index(i, j)) ? [193, 198, 204] : [134, 142, 150];
    for (let y = Math.round(img.Y(gridClean.originY + (j + 1) * gridClean.res)); y < Math.round(img.Y(gridClean.originY + j * gridClean.res)); y++) {
      for (let x = Math.round(img.X(gridClean.originX + i * gridClean.res)); x < Math.round(img.X(gridClean.originX + (i + 1) * gridClean.res)); x++) img.blend(x, y, col);
    }
  }
}
img.metric(1);
// murs en noir, meubles en violet : trait plein posés au sol, pointillés surélevés
for (const s of vc.segments) img.line(s.x1, s.y1, s.x2, s.y2, hex(s.kind === 'objet' ? '#7048e8' : '#1d2327'), 4, s.raised ? [10, 7] : null);
for (const c of corners) img.dot(c.x, c.y, 5, hex('#f59e0b'));
// robots au sol en bleu, robots posés en hauteur (recalage seulement) en orange
poses.forEach((p, k) => img.robot(p, hex(scans[k].elevated ? '#e8590c' : '#1c7ed6'), 5));
await mkdir(ROOT + 'docs/img', { recursive: true });
const png = img.png();
await writeFile(ROOT + 'docs/img/j2_carte_reference.png', png);
// même image avec les étiquettes des meubles (SVG : texte net, l'image PNG est incluse)
const esc = (t) => t.replace(/&/g, '&amp;').replace(/</g, '&lt;');
const tags = labelPositions(vc.segments, labels).map((p) => {
  const w = Math.max(p.nom.length * 10, p.detail.length * 7.4) + 18;
  const h = p.detail ? 40 : 24;
  const x = img.X(p.x);
  const y = img.Y(p.y);
  return `<g><rect x="${(x - w / 2).toFixed(1)}" y="${(y - h / 2).toFixed(1)}" width="${w.toFixed(1)}" height="${h}" rx="6" fill="#fff" fill-opacity="0.93" stroke="#7048e8" stroke-width="2"/>` +
    `<text x="${x.toFixed(1)}" y="${(y - h / 2 + 17).toFixed(1)}" font-size="16" font-weight="600" fill="#3b2a8f">${esc(p.nom)}</text>` +
    (p.detail ? `<text x="${x.toFixed(1)}" y="${(y - h / 2 + 33).toFixed(1)}" font-size="12.5" fill="#495057">${esc(p.detail)}</text>` : '') + '</g>';
});
await writeFile(
  ROOT + 'docs/img/j2_carte_reference.svg',
  `<svg xmlns="http://www.w3.org/2000/svg" width="${img.width}" height="${img.height}" viewBox="0 0 ${img.width} ${img.height}" font-family="Segoe UI, system-ui, sans-serif" text-anchor="middle">` +
    `<image width="${img.width}" height="${img.height}" href="data:image/png;base64,${png.toString('base64')}"/>${tags.join('')}</svg>
`,
);

const st = gridClean.stats();
console.log(`${scans.length} scans recalés : ${before} % -> ${after} % de points cohérents`);
console.log(`Cartes construites avec les ${floor.length} scans pris au sol ; en hauteur (recalage seulement) : ${scans.filter((s) => s.elevated).map((s) => s.name).join(', ') || 'aucun'}`);
console.log(`Grid map : ${gridClean.width} x ${gridClean.height} cellules de ${gridClean.res} m (${st.free} libres, ${st.occupied} occupées)`);
console.log(`Carte vectorielle : ${vc.segments.length} segments dont ${objects.length} d'objets fixes (${objects.filter((s) => s.raised).length} surélevés), ${corners.length} coins, direction des murs ${vc.dominant.toFixed(1)}°`);
for (const g of objectGroups(vc.segments)) {
  const segs = g.map((k) => vc.segments[k]);
  const cx = segs.reduce((a, s) => a + s.x1 + s.x2, 0) / (2 * segs.length);
  const cy = segs.reduce((a, s) => a + s.y1 + s.y2, 0) / (2 * segs.length);
  console.log(`  meuble en (${cx.toFixed(2)}, ${cy.toFixed(2)}) : ${segs.length} segment(s), ${Math.round(100 * segs[0].under)} % des faisceaux passent dessous -> ${segs[0].raised ? 'surélevé' : 'posé au sol'}`);
}
console.log(`Emprise : ${(b.maxX - b.minX).toFixed(1)} m x ${(b.maxY - b.minY).toFixed(1)} m`);
console.log(`Étiquettes : ${labels.map((l) => l.nom).join(', ') || 'aucune'}`);
console.log('Écrit : data/reference/carte_reference.json, data/reference/poses_recalees.csv, docs/img/j2_carte_reference.png et .svg');
