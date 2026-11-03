// Figures et fichiers du jour 1 : versions brutes (telles que calculées depuis les scans)
// et versions propres (reconstruction nettoyée, rendu inspiré de MapTool), plus des
// fichiers directement lisibles par MapTool.
import { DEG } from '../src/core/pose.js';
import { occupancyRGB } from '../src/core/gridmap.js';
import { isPartlyOccupied, labelPositions } from '../src/core/cleanup.js';

export const MAPTOOL = Object.freeze({
  background: '#1e293b', // fond ardoise de MapTool
  floor: '#ffffff',
  wall: '#000000',
  segment: '#ef4444',
  object: '#7048e8', // objets fixes (table, armoire, tableau) : violet, distinct des murs
  raised: '#868e96', // grid map : cellules d'un meuble surélevé, occupées en partie (le lidar passe dessous)
  gridLine: '#d1d5db',
});

/** Grid map -> canvas d'une cellule par pixel, couleur donnée par cellule (null = transparent). */
function gridBitmap(grid, color) {
  const c = document.createElement('canvas');
  c.width = grid.width;
  c.height = grid.height;
  const g = c.getContext('2d');
  const img = g.createImageData(grid.width, grid.height);
  for (let j = 0; j < grid.height; j++) {
    for (let i = 0; i < grid.width; i++) {
      const rgb = color(grid.index(i, j));
      if (!rgb) continue;
      const o = 4 * ((grid.height - 1 - j) * grid.width + i);
      img.data.set(rgb, o);
      img.data[o + 3] = 255;
    }
  }
  g.putImageData(img, 0, 0);
  return c;
}

const hexRGB = (h) => [parseInt(h.slice(1, 3), 16), parseInt(h.slice(3, 5), 16), parseInt(h.slice(5, 7), 16)];

/** Couleurs MapTool d'une grid map propre : sol blanc, mur noir, inconnu transparent. */
export function maptoolColor(grid) {
  const floor = hexRGB(MAPTOOL.floor);
  const wall = hexRGB(MAPTOOL.wall);
  const raised = hexRGB(MAPTOOL.raised);
  return (k) => {
    const c = grid.classify(k);
    if (c === 1) return isPartlyOccupied(grid, k) ? raised : wall;
    return c === -1 ? floor : null;
  };
}

/** Bornes {minX, minY, maxX, maxY} d'une grid map, éventuellement réduites aux cellules connues. */
export function gridBounds(grid, knownOnly = false, margin = 0.4) {
  if (!knownOnly) {
    return { minX: grid.originX, minY: grid.originY, maxX: grid.originX + grid.width * grid.res, maxY: grid.originY + grid.height * grid.res };
  }
  let i0 = Infinity;
  let j0 = Infinity;
  let i1 = -Infinity;
  let j1 = -Infinity;
  for (let j = 0; j < grid.height; j++) {
    for (let i = 0; i < grid.width; i++) {
      if (grid.classify(grid.index(i, j)) === 0) continue;
      i0 = Math.min(i0, i);
      j0 = Math.min(j0, j);
      i1 = Math.max(i1, i);
      j1 = Math.max(j1, j);
    }
  }
  if (i0 === Infinity) return gridBounds(grid);
  return {
    minX: grid.originX + i0 * grid.res - margin,
    minY: grid.originY + j0 * grid.res - margin,
    maxX: grid.originX + (i1 + 1) * grid.res + margin,
    maxY: grid.originY + (j1 + 1) * grid.res + margin,
  };
}

/**
 * Figure avec titre, sous-titre, barre d'échelle et légende. draw(ctx, X, Y, ppm) dessine le
 * contenu en coordonnées monde converties par X() et Y().
 */
function figure(bounds, { title, subtitle, legend = [], dark = false, background }, draw) {
  const w = bounds.maxX - bounds.minX;
  const h = bounds.maxY - bounds.minY;
  const ppm = Math.max(40, Math.min(160, 1800 / Math.max(w, h)));
  const pad = 28;
  const top = 74;
  const bottom = 64;
  const c = document.createElement('canvas');
  c.width = Math.round(w * ppm + 2 * pad);
  c.height = Math.round(h * ppm + top + bottom);
  const ctx = c.getContext('2d');
  const ink = dark ? '#f1f5f9' : '#1d2327';
  ctx.fillStyle = dark ? MAPTOOL.background : '#ffffff';
  ctx.fillRect(0, 0, c.width, c.height);
  if (background) {
    ctx.fillStyle = background;
    ctx.fillRect(pad, top, w * ppm, h * ppm);
  }
  const X = (x) => pad + (x - bounds.minX) * ppm;
  const Y = (y) => top + (bounds.maxY - y) * ppm;
  ctx.save();
  ctx.beginPath();
  ctx.rect(pad, top, w * ppm, h * ppm);
  ctx.clip();
  draw(ctx, X, Y, ppm);
  ctx.restore();

  ctx.fillStyle = ink;
  ctx.font = '600 22px system-ui, Segoe UI, sans-serif';
  ctx.fillText(title, pad, 34);
  ctx.font = '14px system-ui, Segoe UI, sans-serif';
  ctx.globalAlpha = 0.75;
  ctx.fillText(subtitle, pad, 56);
  ctx.globalAlpha = 1;

  // barre d'échelle de 1 m
  const yb = c.height - 28;
  ctx.strokeStyle = ink;
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.moveTo(pad, yb);
  ctx.lineTo(pad + ppm, yb);
  ctx.moveTo(pad, yb - 6);
  ctx.lineTo(pad, yb + 6);
  ctx.moveTo(pad + ppm, yb - 6);
  ctx.lineTo(pad + ppm, yb + 6);
  ctx.stroke();
  ctx.font = '14px system-ui, Segoe UI, sans-serif';
  ctx.fillText('1 m', pad + ppm + 10, yb + 5);

  // légende, de droite à gauche
  let x = c.width - pad;
  ctx.textAlign = 'right';
  for (const [color, label] of [...legend].reverse()) {
    ctx.fillText(label, x, yb + 5);
    x -= ctx.measureText(label).width + 8;
    ctx.fillStyle = color;
    ctx.fillRect(x - 14, yb - 7, 14, 14);
    ctx.strokeStyle = dark ? '#94a3b8' : '#adb5bd';
    ctx.lineWidth = 1;
    ctx.strokeRect(x - 14, yb - 7, 14, 14);
    ctx.fillStyle = ink;
    x -= 34;
  }
  ctx.textAlign = 'left';
  return c;
}

function metricLines(ctx, X, Y, bounds, color, step = 1) {
  ctx.strokeStyle = color;
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let x = Math.ceil(bounds.minX / step) * step; x <= bounds.maxX; x += step) {
    ctx.moveTo(Math.round(X(x)) + 0.5, Y(bounds.minY));
    ctx.lineTo(Math.round(X(x)) + 0.5, Y(bounds.maxY));
  }
  for (let y = Math.ceil(bounds.minY / step) * step; y <= bounds.maxY; y += step) {
    ctx.moveTo(X(bounds.minX), Math.round(Y(y)) + 0.5);
    ctx.lineTo(X(bounds.maxX), Math.round(Y(y)) + 0.5);
  }
  ctx.stroke();
}

function drawGrid(ctx, X, Y, grid, color) {
  ctx.imageSmoothingEnabled = false;
  const x0 = X(grid.originX);
  const y0 = Y(grid.originY + grid.height * grid.res);
  ctx.drawImage(gridBitmap(grid, color), x0, y0, X(grid.originX + grid.width * grid.res) - x0, Y(grid.originY) - y0);
}

function drawSegments(ctx, X, Y, segs, color, width, dash = null) {
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.lineCap = dash ? 'butt' : 'round';
  ctx.setLineDash(dash ?? []);
  ctx.beginPath();
  for (const s of segs) {
    ctx.moveTo(X(s.x1), Y(s.y1));
    ctx.lineTo(X(s.x2), Y(s.y2));
  }
  ctx.stroke();
  ctx.setLineDash([]);
}

/**
 * Étiquettes de la carte sémantique : nom du meuble et détail dans un cartouche blanc bordé de
 * violet, centré sur le meuble. positions : labelPositions(...) ; X, Y : monde -> pixels.
 */
export function drawLabels(ctx, X, Y, positions, scale = 1) {
  ctx.save();
  ctx.textAlign = 'center';
  ctx.textBaseline = 'top';
  const big = `600 ${12 * scale}px system-ui, Segoe UI, sans-serif`;
  const small = `${10.5 * scale}px system-ui, Segoe UI, sans-serif`;
  for (const p of positions) {
    ctx.font = big;
    let w = ctx.measureText(p.nom).width;
    ctx.font = small;
    if (p.detail) w = Math.max(w, ctx.measureText(p.detail).width);
    w += 14 * scale;
    const h = (p.detail ? 31 : 19) * scale;
    const cx = X(p.x);
    const top = Y(p.y) - h / 2;
    ctx.fillStyle = 'rgba(255, 255, 255, 0.93)';
    ctx.strokeStyle = MAPTOOL.object;
    ctx.lineWidth = 1.5 * scale;
    ctx.beginPath();
    ctx.roundRect(cx - w / 2, top, w, h, 5 * scale);
    ctx.fill();
    ctx.stroke();
    ctx.font = big;
    ctx.fillStyle = '#3b2a8f';
    ctx.fillText(p.nom, cx, top + 3.5 * scale);
    if (p.detail) {
      ctx.font = small;
      ctx.fillStyle = '#495057';
      ctx.fillText(p.detail, cx, top + 17 * scale);
    }
  }
  ctx.restore();
}

/** Objets fixes : posés au sol en trait plein, surélevés (le lidar passe dessous) en pointillés. */
function drawObjectSegments(ctx, X, Y, segs, width) {
  drawSegments(ctx, X, Y, segs.filter((s) => !s.raised), MAPTOOL.object, width);
  drawSegments(ctx, X, Y, segs.filter((s) => s.raised), MAPTOOL.object, width, [3 * width, 2 * width]);
}

function drawRobots(ctx, X, Y, scans, numbered = true) {
  scans.forEach((e, k) => {
    const { x, y, theta } = e.pose;
    ctx.save();
    ctx.translate(X(x), Y(y));
    ctx.rotate(-theta * DEG);
    ctx.beginPath();
    ctx.moveTo(11, 0);
    ctx.lineTo(-7, 7);
    ctx.lineTo(-7, -7);
    ctx.closePath();
    ctx.fillStyle = e.color;
    ctx.fill();
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 1.5;
    ctx.stroke();
    ctx.restore();
    if (numbered) {
      ctx.font = '600 12px system-ui, sans-serif';
      ctx.fillStyle = '#1d2327';
      ctx.fillText(String(k + 1), X(x) + 9, Y(y) - 9);
    }
  });
}

function drawCorners(ctx, X, Y, corners) {
  ctx.fillStyle = '#f59e0b';
  ctx.strokeStyle = '#1e293b';
  ctx.lineWidth = 1.5;
  for (const c of corners) {
    const x = X(c.x);
    const y = Y(c.y);
    ctx.beginPath();
    ctx.moveTo(x, y - 7);
    ctx.lineTo(x + 7, y);
    ctx.lineTo(x, y + 7);
    ctx.lineTo(x - 7, y);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
  }
}

function drawPoints(ctx, X, Y, pts, color, size) {
  ctx.fillStyle = color;
  for (const p of pts) ctx.fillRect(X(p.x) - size / 2, Y(p.y) - size / 2, size, size);
}

const toBlob = (canvas) => new Promise((r) => canvas.toBlob(r, 'image/png'));

/** Fichiers au format MapTool : coordonnées décalées dans [0, W] x [0, H] (MapTool n'accepte pas de négatifs). */
function maptoolFiles(gridClean, segments, prefix) {
  const b = gridBounds(gridClean, true, 0.2);
  const dx = -b.minX;
  const dy = -b.minY;
  const W = Math.ceil((b.maxX - b.minX) * 10) / 10;
  const H = Math.ceil((b.maxY - b.minY) * 10) / 10;
  const txt = [
    `# Carte vectorielle propre pour MapTool : régler Largeur = ${W.toFixed(1)} m et Hauteur = ${H.toFixed(1)} m, puis importer ce fichier`,
    `# coordonnées décalées de (${dx.toFixed(3)}, ${dy.toFixed(3)}) m par rapport au repère du premier scan`,
    '# x1,y1,x2,y2 (m)',
    ...segments.map((s) => [s.x1 + dx, s.y1 + dy, s.x2 + dx, s.y2 + dy].map((v) => v.toFixed(4)).join(',')),
  ].join('\n');
  // grid map binaire à 10 cm, origine (0, 0), lignes du haut vers le bas, comme l'export MapTool
  const cell = 0.1;
  const nx = Math.round(W / cell);
  const ny = Math.round(H / cell);
  const rows = [`# Grid map ${nx}x${ny}, cell=${cell} m ; 1=occupied,0=free (décalage ${dx.toFixed(3)}, ${dy.toFixed(3)} m)`];
  for (let j = ny - 1; j >= 0; j--) {
    const row = [];
    for (let i = 0; i < nx; i++) {
      let occ = 0;
      for (let sy = 0.25; sy < 1 && !occ; sy += 0.5) {
        for (let sx = 0.25; sx < 1 && !occ; sx += 0.5) {
          const [gi, gj] = gridClean.worldToCell((i + sx) * cell - dx, (j + sy) * cell - dy);
          if (gridClean.inside(gi, gj) && gridClean.classify(gridClean.index(gi, gj)) === 1) occ = 1;
        }
      }
      row.push(occ);
    }
    rows.push(row.join(','));
  }
  return [
    { name: `${prefix}maptool_carte_vectorielle.txt`, text: txt + '\n' },
    { name: `${prefix}maptool_grid_map.csv`, text: rows.join('\n') + '\n' },
  ];
}

const segmentsTXT = (segs, title) =>
  [`# ${title}`, '# x1,y1,x2,y2 (m), repère du premier scan', ...segs.map((s) => [s.x1, s.y1, s.x2, s.y2].map((v) => v.toFixed(4)).join(','))].join('\n') + '\n';

/**
 * Toutes les figures et tous les fichiers du jour 1.
 * d = { scans: [{name, color, pose, points, segments}], grid, gridClean, vector, vectorClean, corners }
 * Retourne [{ name, blob | text }].
 */
export async function dayOneExports(d) {
  const n = d.scans.length;
  const sub = (extra) => `${n} scans · ${new Date().toLocaleString('fr-FR')} · ${extra}`;
  const raw = gridBounds(d.grid, true, 0.3);
  const clean = gridBounds(d.gridClean, true, 0.4);
  const scanLegend = d.scans.slice(0, 8).map((e, k) => [e.color, `${k + 1}`]);
  const files = [];
  const add = async (name, canvas) => files.push({ name, blob: await toBlob(canvas) });

  await add('01_nuages_de_points.png', figure(raw, { title: 'Nuages de points', subtitle: sub('une couleur par scan, triangle = pose du robot'), legend: scanLegend }, (ctx, X, Y) => {
    metricLines(ctx, X, Y, raw, '#e9ecef');
    d.scans.forEach((e) => drawPoints(ctx, X, Y, e.points, e.color, 3));
    drawRobots(ctx, X, Y, d.scans);
  }));

  await add('02_grid_map_brute.png', figure(raw, {
    title: 'Grid map brute',
    subtitle: sub(`cellules de ${Math.round(d.grid.res * 100)} cm, points des scans superposés`),
    legend: [['#fcfcfa', 'libre'], ['#d6dbe0', 'inconnu'], ['#1d2327', 'occupé']],
    background: '#d6dbe0',
  }, (ctx, X, Y) => {
    drawGrid(ctx, X, Y, d.grid, (k) => occupancyRGB(d.grid, k));
    metricLines(ctx, X, Y, raw, 'rgba(29,35,39,0.08)');
    d.scans.forEach((e) => drawPoints(ctx, X, Y, e.points, e.color, 2));
    drawRobots(ctx, X, Y, d.scans);
  }));

  await add('03_grid_map_propre.png', figure(clean, {
    title: 'Grid map propre',
    subtitle: sub('poussières, faisceaux sortis par les fenêtres et petits trous retirés'),
    legend: [[MAPTOOL.floor, 'libre'], [MAPTOOL.background, 'inconnu'], [MAPTOOL.wall, 'occupé'], [MAPTOOL.raised, 'occupé en partie (meuble surélevé)']],
    dark: true,
  }, (ctx, X, Y) => {
    drawGrid(ctx, X, Y, d.gridClean, maptoolColor(d.gridClean));
    metricLines(ctx, X, Y, clean, 'rgba(209,213,219,0.35)');
    drawRobots(ctx, X, Y, d.scans);
  }));

  await add('04_segments_extraits.png', figure(raw, { title: 'Segments extraits de chaque scan', subtitle: sub('clusters + split-and-merge, une couleur par scan'), legend: scanLegend }, (ctx, X, Y) => {
    metricLines(ctx, X, Y, raw, '#e9ecef');
    d.scans.forEach((e) => drawPoints(ctx, X, Y, e.points, '#ced4da', 2));
    d.scans.forEach((e) => drawSegments(ctx, X, Y, e.segments, e.color, 3));
    drawRobots(ctx, X, Y, d.scans);
  }));

  await add('05_carte_vectorielle_brute.png', figure(raw, {
    title: 'Carte vectorielle brute',
    subtitle: sub(`${d.vector.segments.length} segments après fusion des scans`),
    legend: [['#d6336c', 'segment'], ['#ced4da', 'points des scans']],
  }, (ctx, X, Y) => {
    metricLines(ctx, X, Y, raw, '#e9ecef');
    d.scans.forEach((e) => drawPoints(ctx, X, Y, e.points, '#ced4da', 2));
    drawSegments(ctx, X, Y, d.vector.segments, '#d6336c', 3);
    drawRobots(ctx, X, Y, d.scans);
  }));

  const walls = d.vectorClean.segments.filter((s) => s.kind !== 'objet');
  const objects = d.vectorClean.segments.filter((s) => s.kind === 'objet');
  await add('06_carte_vectorielle_propre.png', figure(clean, {
    title: 'Carte vectorielle propre',
    subtitle: sub(`${walls.length} segments de murs redressés (direction ${d.vectorClean.dominant.toFixed(1)}°) et ${objects.length} d'objets fixes`),
    legend: [[MAPTOOL.segment, 'mur'], [MAPTOOL.object, 'meuble : trait plein posé au sol, pointillés surélevé'], ['#f59e0b', `coin (${d.corners.length})`], [MAPTOOL.floor, 'sol']],
    dark: true,
  }, (ctx, X, Y) => {
    drawGrid(ctx, X, Y, d.gridClean, (k) => (d.gridClean.classify(k) === -1 ? [255, 255, 255] : null));
    metricLines(ctx, X, Y, clean, 'rgba(209,213,219,0.35)');
    drawSegments(ctx, X, Y, walls, MAPTOOL.segment, 4);
    drawObjectSegments(ctx, X, Y, objects, 4);
    drawCorners(ctx, X, Y, d.corners);
    drawLabels(ctx, X, Y, labelPositions(d.vectorClean.segments, d.labels ?? []), 1.3);
  }));

  await add('07_plan_de_la_salle.png', figure(clean, {
    title: 'Plan de la salle',
    subtitle: sub('grid map propre + carte vectorielle propre'),
    legend: [[MAPTOOL.floor, 'sol'], ['#1d2327', 'murs'], [MAPTOOL.object, 'meubles : trait plein posé au sol, pointillés surélevé'], ['#f59e0b', 'coins']],
    background: '#d6dbe0',
  }, (ctx, X, Y) => {
    drawGrid(ctx, X, Y, d.gridClean, (k) => {
      const c = d.gridClean.classify(k);
      if (c === 1) return isPartlyOccupied(d.gridClean, k) ? [193, 198, 204] : [134, 142, 150];
      return c === -1 ? [255, 255, 255] : null;
    });
    metricLines(ctx, X, Y, clean, 'rgba(29,35,39,0.07)');
    drawSegments(ctx, X, Y, walls, '#1d2327', 5);
    drawObjectSegments(ctx, X, Y, objects, 5);
    drawCorners(ctx, X, Y, d.corners);
    drawLabels(ctx, X, Y, labelPositions(d.vectorClean.segments, d.labels ?? []), 1.3);
    drawRobots(ctx, X, Y, d.scans);
  }));

  files.push(
    { name: 'grid_map_brute.csv', text: d.grid.toCSV() },
    { name: 'grid_map_propre.csv', text: d.gridClean.toCSV() },
    { name: 'carte_vectorielle_brute.txt', text: segmentsTXT(d.vector.segments, 'Carte vectorielle brute ld06-slam') },
    { name: 'carte_vectorielle_propre.txt', text: segmentsTXT(d.vectorClean.segments, 'Carte vectorielle propre ld06-slam') },
    ...maptoolFiles(d.gridClean, d.vectorClean.segments, ''),
    {
      name: 'coins.txt',
      text: [
        "# Coins (points d'intérêt orientés) : x,y (m), theta = bissectrice vers l'intérieur (degrés), angle entre les murs (degrés)",
        ...d.corners.map((c) => [c.x.toFixed(4), c.y.toFixed(4), c.theta.toFixed(1), c.angle.toFixed(1)].join(',')),
      ].join('\n') + '\n',
    },
    {
      name: 'poses.csv',
      text: ['scan,x_m,y_m,theta_deg', ...d.scans.map((e) => `${e.name},${e.pose.x.toFixed(4)},${e.pose.y.toFixed(4)},${e.pose.theta.toFixed(2)}`)].join('\n') + '\n',
    },
  );
  return files;
}
