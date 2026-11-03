// Application web : affichage de plusieurs scans, poses modifiables,
// construction et mise à jour d'une grid map et d'une carte vectorielle.
import { DEG, normalizeAngleDeg, poseError, transformPoints } from '../src/core/pose.js';
import { parseScanCSV, scanToLocalXY, serializeScanCSV, validPoints } from '../src/core/scan.js';
import { GridMap, gridFromScan, occupancyRGB, UNKNOWN_RGB } from '../src/core/gridmap.js';
import { VectorMap } from '../src/core/vectormap.js';
import { autoAlign, consistency, matchScan } from '../src/core/matching.js';
import { cleanMaps, isPartlyOccupied } from '../src/core/cleanup.js';
import { gridReference, localize, METHODS, METHODS_BY_REFERENCE, scanReference, vectorReference } from '../src/core/localization.js';
import { dayOneExports, MAPTOOL, maptoolColor } from './figures.js';
import { makeReferenceMap, parseReferenceMap, referenceBounds, serializeReferenceMap } from '../src/core/refmap.js';
import { extractSegments } from '../src/core/segmentation.js';
import { transformSegment } from '../src/core/geometry.js';

const PALETTE = ['#e8590c', '#1c7ed6', '#2f9e44', '#ae3ec9', '#f08c00', '#0c8599', '#c2255c', '#5c940d', '#5f3dc4', '#795548'];
const $ = (id) => document.getElementById(id);
const canvas = $('view');
const ctx = canvas.getContext('2d');

const state = {
  scans: [],
  selected: null,
  nextId: 1,
  grid: null,
  gridImage: null,
  vector: null,
  reference: null,
  view: { x: 4, y: 3, scale: 70 }, // centre de la vue (m) et pixels par mètre
  mouse: null,
};

// ------------------------------------------------------------------ utilitaires

function filterOpts() {
  return { minRange: Number($('fMin').value), maxRange: Number($('fMax').value), minIntensity: Number($('fInt').value) };
}

function toast(msg) {
  const t = $('toast');
  t.textContent = msg;
  t.hidden = false;
  clearTimeout(toast.timer);
  toast.timer = setTimeout(() => (t.hidden = true), 2600);
}

function download(name, content, type = 'text/plain') {
  const blob = content instanceof Blob ? content : new Blob([content], { type });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = name;
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
}

const readFile = (file) => file.text();

// ------------------------------------------------------------------ scans

function addScan(scan) {
  const id = state.nextId++;
  const entry = {
    id,
    scan,
    name: scan.name,
    pose: { ...scan.pose },
    filePose: { ...scan.pose },
    color: PALETTE[(id - 1) % PALETTE.length],
    visible: true,
    flip: false,
    cache: null,
  };
  state.scans.push(entry);
  return entry;
}

/** Points et segments dans le repère du robot (indépendants de la pose, donc mis en cache). */
function local(entry) {
  const f = filterOpts();
  const key = `${f.minRange}|${f.maxRange}|${f.minIntensity}|${entry.flip}`;
  if (!entry.cache || entry.cache.key !== key) {
    const pts = scanToLocalXY(entry.scan, f, { flipAngles: entry.flip });
    entry.cache = { key, pts, segs: extractSegments(pts).segments };
  }
  return entry.cache;
}

const worldPoints = (e) => transformPoints(local(e).pts, e.pose);
const worldSegments = (e) => local(e).segs.map((s) => transformSegment(s, e.pose));
const integrateOpts = (e) => [e.scan, e.pose, filterOpts(), { flipAngles: e.flip }];

function select(entry) {
  state.selected = entry;
  renderScanList();
  renderPoseEditor();
  draw();
}

function removeScan(entry) {
  state.scans = state.scans.filter((e) => e !== entry);
  if (state.selected === entry) state.selected = null;
  renderScanList();
  renderPoseEditor();
  poseChanged();
}

function renderScanList() {
  const ul = $('scanList');
  ul.replaceChildren();
  $('scanEmpty').hidden = state.scans.length > 0;
  for (const e of state.scans) {
    const li = document.createElement('li');
    if (e === state.selected) li.className = 'selected';
    const vis = Object.assign(document.createElement('input'), { type: 'checkbox', checked: e.visible, title: 'Afficher' });
    vis.onclick = (ev) => ev.stopPropagation();
    vis.onchange = () => {
      e.visible = vis.checked;
      poseChanged();
    };
    const col = Object.assign(document.createElement('input'), { type: 'color', value: e.color, title: 'Couleur' });
    col.onclick = (ev) => ev.stopPropagation();
    col.oninput = () => {
      e.color = col.value;
      draw();
    };
    const label = document.createElement('div');
    const c = local(e);
    label.innerHTML = `<div class="name"></div><div class="meta">${c.pts.length} pts · ${c.segs.length} segments · ${e.scan.format}</div>`;
    label.firstChild.textContent = e.name;
    const rm = Object.assign(document.createElement('button'), { className: 'remove', textContent: '×', title: 'Retirer' });
    rm.onclick = (ev) => {
      ev.stopPropagation();
      removeScan(e);
    };
    li.append(vis, col, label, rm);
    li.onclick = () => select(e);
    ul.append(li);
  }
}

function renderPoseEditor() {
  const e = state.selected;
  $('poseEditor').hidden = !e;
  if (!e) return;
  $('poseName').textContent = e.name;
  if (document.activeElement !== $('poseX')) $('poseX').value = e.pose.x.toFixed(3);
  if (document.activeElement !== $('poseY')) $('poseY').value = e.pose.y.toFixed(3);
  if (document.activeElement !== $('poseT')) $('poseT').value = e.pose.theta.toFixed(2);
  $('poseFlip').checked = e.flip;
}

let rebuildTimer = null;
/**
 * Une pose, la visibilité ou la liste des scans a changé : redessin, et reconstruction
 * automatique de la grid map et de la carte vectorielle (case « Cartes automatiques »).
 */
function poseChanged({ final = true } = {}) {
  renderPoseEditor();
  draw();
  if ($('autoRebuild').checked && final) {
    clearTimeout(rebuildTimer);
    rebuildTimer = setTimeout(() => {
      if (!state.scans.some((e) => e.visible)) return;
      buildGridAll(true);
      buildVectorAll(true);
    }, 150);
  }
}

const tick = () => new Promise((r) => setTimeout(r, 0));
const visibleItems = () => state.scans.filter((e) => e.visible).map((e) => ({ entry: e, points: local(e).pts, pose: e.pose }));

/** Bouton « Recaler automatiquement » : place puis affine tous les scans visibles (le premier est la référence). */
async function runAutoAlign() {
  const items = visibleItems();
  if (items.length < 2) return toast('Il faut au moins deux scans visibles');
  $('btnAlign').disabled = true;
  const before = consistency(items.map((i) => i.points), items.map((i) => i.pose));
  const t0 = performance.now();
  const steps = autoAlign(items);
  let r;
  while (!(r = steps.next()).done) {
    const { phase, k, total } = r.value;
    toast(`Recalage automatique : ${phase} du scan ${k + 1}/${total}…`);
    await tick();
  }
  r.value.forEach((pose, k) => (items[k].entry.pose = pose));
  const after = consistency(items.map((i) => i.points), r.value);
  const mean = (a) => Math.round((100 * a.reduce((x, y) => x + y, 0)) / a.length);
  $('btnAlign').disabled = false;
  poseChanged();
  if (!$('autoRebuild').checked) {
    buildGridAll(true);
    buildVectorAll(true);
  }
  toast(`Recalage terminé en ${((performance.now() - t0) / 1000).toFixed(1)} s : ${mean(before)} % → ${mean(after)} % des points collent aux autres scans`);
}

/** Recalage d'un scan qui vient d'être capturé, sur les scans déjà présents (fenêtre large). */
function alignNewScan(entry) {
  const others = state.scans.filter((e) => e !== entry && e.visible);
  if (!others.length) return;
  const ref = others.flatMap((e) => worldPoints(e));
  const r = matchScan(ref, local(entry).pts, entry.pose);
  entry.pose = r.pose;
  return r;
}

async function loadFiles(files) {
  let n = 0;
  for (const file of files) {
    const text = await readFile(file);
    const kind = classify(text);
    if (kind === 'grid') {
      setGrid(GridMap.fromCSV(text, { resolution: Number($('gridRes').value) }));
      toast(`Grid map importée : ${file.name}`);
    } else if (kind === 'vector') {
      state.reference = VectorMap.fromTXT(text);
      $('showRef').checked = true;
      toast(`Carte de référence : ${file.name} (${state.reference.segments.length} segments)`);
    } else {
      const scan = parseScanCSV(text, file.name);
      if (!scan.points.length) {
        toast(`${file.name} : aucun point lu`);
        continue;
      }
      addScan(scan);
      n++;
    }
  }
  if (n) {
    renderScanList();
    fitView();
    poseChanged();
  }
  draw();
}

/** Devine le type d'un fichier déposé : grid map, carte vectorielle ou scan. */
function classify(text) {
  if (/^#\s*(gridmap|grid map)/im.test(text)) return 'grid';
  const data = text.split(/\r?\n/).map((l) => l.trim()).filter((l) => l && !l.startsWith('#'));
  if (data.length && data.every((l) => /^-?[\d.]+(\s*,\s*-?[\d.]+){3}$/.test(l))) return 'vector';
  return 'scan';
}

async function loadSimulated() {
  try {
    const index = await (await fetch('../data/sim/index.json')).json();
    for (const s of index.scans) {
      const text = await (await fetch(`../data/sim/${s.file}`)).text();
      addScan(parseScanCSV(text, s.file));
    }
    state.reference = VectorMap.fromTXT(await (await fetch('../data/sim/scene_A.txt')).text());
    $('showRef').checked = false;
    renderScanList();
    fitView();
    poseChanged();
    toast(`${index.scans.length} scans simulés chargés (référence : scene_A.txt, masquée)`);
  } catch (err) {
    toast(`Chargement impossible : ${err.message}`);
  }
}

// ------------------------------------------------------------------ robot en temps réel
// Le relais scripts/live.py lit l'ESP32 sur le port USB et diffuse chaque nouveau scan
// sur /live (Server-Sent Events). Le scan courant est dessiné en continu à la pose du scan
// sélectionné ; « Ajouter ce scan » (ou Espace) le fige dans la liste.

const live = { source: null, text: null, scan: null, time: 0, status: null };

function liveStatus(msg) {
  $('liveStat').textContent = msg;
}

async function startLive() {
  try {
    const res = await fetch('/live/status', { cache: 'no-store' });
    if (!res.ok) throw new Error();
  } catch {
    liveStatus('Pas de robot : lancer « python scripts/live.py » (au lieu de node serve.mjs) puis recharger la page.');
    return;
  }
  live.source = new EventSource('/live');
  live.source.addEventListener('status', (ev) => {
    live.status = JSON.parse(ev.data);
    renderLiveStatus();
  });
  live.source.addEventListener('scan', (ev) => {
    const d = JSON.parse(ev.data);
    live.text = d.text;
    live.scan = parseScanCSV(d.text, 'temps réel');
    const firstScan = live.time === 0;
    live.time = Date.now();
    $('liveAdd').disabled = false;
    renderLiveStatus();
    if (firstScan && !state.scans.length) fitView();
    else draw();
  });
  live.source.onerror = () => liveStatus('Relais live.py injoignable (arrêté ?), nouvel essai…');
}

function renderLiveStatus() {
  const s = live.status;
  let msg = s ? s.message : 'Connexion au relais…';
  if (live.scan && s?.connected) {
    const n = validPoints(live.scan, filterOpts()).length;
    const tours = live.scan.mergedLines > 1 ? `, ${live.scan.mergedLines} tours fusionnés` : '';
    msg += ` · ${n} points${tours}`;
  }
  liveStatus(msg);
}

/** Pose à laquelle le scan en temps réel est dessiné : celle du scan sélectionné (point de départ du recalage). */
function livePose() {
  return state.selected ? state.selected.pose : { x: 0, y: 0, theta: 0 };
}

/** Fige le scan en temps réel : il rejoint la liste avec sa propre couleur. */
function liveCapture() {
  if (!live.text) return toast('Aucun scan reçu du robot pour l\'instant');
  const scan = parseScanCSV(live.text, captureName());
  const entry = addLiveScan(scan);
  const n = validPoints(scan, filterOpts()).length;
  const r = $('liveAlign').checked ? alignNewScan(entry) : null;
  poseChanged();
  toast(r ? `${scan.name} ajouté et recalé : ${n} points, ${Math.round(r.inliers * 100)} % collent aux scans précédents` : `${scan.name} ajouté : ${n} points`);
}

/**
 * Ajoute un scan acquis à l'instant (temps réel ou collage). Le robot a peu bougé
 * depuis le scan précédent : le nouveau part de la pose du scan sélectionné, il ne
 * reste qu'à le recaler. Le premier scan reste en (0, 0, 0) et définit le repère.
 */
function addLiveScan(scan) {
  const first = state.scans.length === 0;
  const ref = state.selected;
  const entry = addScan(scan);
  if (ref) entry.pose = { ...ref.pose };
  select(entry);
  if (first) fitView();
  return entry;
}

/** Texte collé (moniteur série, Teleplot ou CSV) -> nouveau scan. */
function addPastedText(text) {
  const scan = parseScanCSV(text, captureName());
  const n = validPoints(scan, filterOpts()).length;
  if (n < 10) {
    toast('Aucun scan reconnu dans le texte collé (lignes >lidar:…|xy ou CSV attendues)');
    return;
  }
  addLiveScan(scan);
  const tours = scan.mergedLines > 1 ? ` (${scan.mergedLines} tours fusionnés)` : '';
  toast(`${scan.name} ajouté : ${n} points valides${tours}`);
}

/** Nom de fichier horodaté : pas d'écrasement d'une séance de TP à l'autre. */
function captureName() {
  const d = new Date();
  const p = (n) => String(n).padStart(2, '0');
  return `scan_${p(d.getMonth() + 1)}${p(d.getDate())}_${p(d.getHours())}${p(d.getMinutes())}${p(d.getSeconds())}.csv`;
}


/** Enregistre chaque scan avec sa pose actuelle, plus un récapitulatif poses.csv. */
async function saveAll() {
  if (!state.scans.length) return toast('Aucun scan à enregistrer');
  const files = state.scans.map((e) => ({ name: e.name.replace(/\.(csv|txt)$/i, '') + '.csv', text: serializeScanCSV(e.scan, e.pose) }));
  files.push({
    name: 'poses.csv',
    text: ['scan,x_m,y_m,theta_deg', ...state.scans.map((e) => `${e.name},${e.pose.x.toFixed(4)},${e.pose.y.toFixed(4)},${e.pose.theta.toFixed(2)}`)].join('\n') + '\n',
  });
  await saveFiles(files);
}

/**
 * Écrit des fichiers { name, text | blob } : dans un dossier choisi (Chrome, Edge) ou en
 * téléchargements successifs (Firefox).
 */
async function saveFiles(files) {
  if (!window.showDirectoryPicker) {
    for (const f of files) {
      download(f.name, f.blob ?? f.text, 'text/plain');
      await new Promise((r) => setTimeout(r, 150)); // Firefox ignore des téléchargements trop rapprochés
    }
    toast(`${files.length} fichiers téléchargés`);
    return;
  }
  try {
    const dir = await window.showDirectoryPicker({ mode: 'readwrite' });
    for (const f of files) {
      const w = await (await dir.getFileHandle(f.name, { create: true })).createWritable();
      await w.write(f.blob ?? f.text);
      await w.close();
    }
    toast(`${files.length} fichiers enregistrés dans « ${dir.name} »`);
  } catch (err) {
    if (err.name !== 'AbortError') toast(`Enregistrement impossible : ${err.message}`);
  }
}

// ------------------------------------------------------------------ grid map

function gridParams() {
  return { resolution: Number($('gridRes').value) };
}

function setGrid(g) {
  state.grid = g;
  state.gridImage = null;
  if (g) {
    const s = g.stats();
    $('gridStat').textContent = `${g.width} × ${g.height} cellules de ${g.res * 100} cm · ${g.scanCount} scan(s) · occupées ${s.occupied}, libres ${s.free}, inconnues ${s.unknown}`;
  } else {
    $('gridStat').textContent = '';
  }
  refreshClean();
  draw();
}

function gridImage() {
  if (state.gridImage || !state.grid) return state.gridImage;
  const g = state.grid;
  const off = document.createElement('canvas');
  off.width = g.width;
  off.height = g.height;
  const octx = off.getContext('2d');
  const img = octx.createImageData(g.width, g.height);
  for (let j = 0; j < g.height; j++) {
    for (let i = 0; i < g.width; i++) {
      const o = 4 * ((g.height - 1 - j) * g.width + i); // ligne 0 de l'image = haut de la carte
      img.data.set(occupancyRGB(g, g.index(i, j)), o);
      img.data[o + 3] = 255;
    }
  }
  octx.putImageData(img, 0, 0);
  state.gridImage = off;
  return off;
}

function requireSelection() {
  if (!state.selected) toast('Sélectionner un scan dans la liste');
  return state.selected;
}

function buildGridAll(silent = false) {
  const vis = state.scans.filter((e) => e.visible);
  if (!vis.length) return silent || toast('Aucun scan visible');
  const t0 = performance.now();
  const [first, ...rest] = vis;
  const g = gridFromScan(first.scan, first.pose, gridParams(), filterOpts(), { flipAngles: first.flip });
  for (const e of rest) g.integrateScan(...integrateOpts(e));
  setGrid(g);
  if (!silent) toast(`Grid map : ${vis.length} scans intégrés en ${(performance.now() - t0).toFixed(0)} ms`);
}

// ------------------------------------------------------------------ carte vectorielle

function vecParams() {
  return { absence: $('vecAbsence').checked };
}

function setVector(m, info = '') {
  state.vector = m;
  $('vecStat').textContent = m ? `${m.segments.length} segments · ${m.scanCount} scan(s)${info ? ' · ' + info : ''}` : '';
  refreshClean();
  draw();
}

function buildVectorAll(silent = false) {
  const vis = state.scans.filter((e) => e.visible);
  if (!vis.length) return silent || toast('Aucun scan visible');
  const m = new VectorMap([], vecParams());
  let cuts = 0;
  for (const e of vis) cuts += m.integrateScan(...integrateOpts(e)).cuts;
  setVector(m, `${cuts} découpes`);
  if (!silent) toast(`Carte vectorielle : ${m.segments.length} segments`);
}

// ------------------------------------------------------------------ vue

function resize() {
  const dpr = window.devicePixelRatio || 1;
  const r = canvas.getBoundingClientRect();
  canvas.width = Math.round(r.width * dpr);
  canvas.height = Math.round(r.height * dpr);
  draw();
}

const W = () => canvas.width / (window.devicePixelRatio || 1);
const H = () => canvas.height / (window.devicePixelRatio || 1);
const sx = (x) => W() / 2 + (x - state.view.x) * state.view.scale;
const sy = (y) => H() / 2 - (y - state.view.y) * state.view.scale;
const wx = (px) => state.view.x + (px - W() / 2) / state.view.scale;
const wy = (py) => state.view.y - (py - H() / 2) / state.view.scale;

function fitView() {
  const xs = [];
  const ys = [];
  const pointSets = state.scans.filter((s) => s.visible).map(worldPoints);
  if (!pointSets.length && live.scan) pointSets.push(livePoints());
  if (!pointSets.length && refmap.map) {
    const b = referenceBounds(refmap.map);
    pointSets.push([{ x: b.minX, y: b.minY }, { x: b.maxX, y: b.maxY }]);
  }
  for (const pts of pointSets) {
    for (const p of pts) {
      xs.push(p.x);
      ys.push(p.y);
    }
  }
  for (const e of state.scans.filter((s) => s.visible)) {
    xs.push(e.pose.x);
    ys.push(e.pose.y);
  }
  if (!xs.length) return draw();
  const minX = Math.min(...xs);
  const maxX = Math.max(...xs);
  const minY = Math.min(...ys);
  const maxY = Math.max(...ys);
  state.view.x = (minX + maxX) / 2;
  state.view.y = (minY + maxY) / 2;
  state.view.scale = 0.9 * Math.min(W() / Math.max(0.5, maxX - minX), H() / Math.max(0.5, maxY - minY));
  draw();
}

let frame = 0;
function draw() {
  if (!frame) frame = requestAnimationFrame(render);
}

function render() {
  frame = 0;
  const dpr = window.devicePixelRatio || 1;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  // Avec une grid map affichée, le fond prend la couleur « inconnu » : les zones libres
  // (blanches) ressortent au lieu de se confondre avec un fond blanc.
  ctx.fillStyle = $('showGrid').checked && state.grid ? `rgb(${UNKNOWN_RGB.join(',')})` : '#fbfbf9';
  ctx.fillRect(0, 0, W(), H());
  const s = state.view.scale;

  const showRef = $('showRefMap').checked && refmap.map;
  // Vue propre (style MapTool) : fond ardoise, sol blanc, murs noirs, segments rouges.
  const clean = $('showClean').checked && (state.gridClean || showRef);
  if (clean) {
    ctx.fillStyle = MAPTOOL.background;
    ctx.fillRect(0, 0, W(), H());
    ctx.imageSmoothingEnabled = false;
    if (showRef) {
      const g = refmap.map.grid;
      refmap.cleanImage ??= cleanGridImage(g);
      ctx.globalAlpha = state.gridClean ? 0.55 : 1;
      ctx.drawImage(refmap.cleanImage, sx(g.originX), sy(g.originY + g.height * g.res), g.width * g.res * s, g.height * g.res * s);
      ctx.globalAlpha = 1;
    }
    if (state.gridClean) {
      const g = state.gridClean;
      state.gridCleanImage ??= cleanGridImage(g);
      ctx.drawImage(state.gridCleanImage, sx(g.originX), sy(g.originY + g.height * g.res), g.width * g.res * s, g.height * g.res * s);
    }
    if ($('showMetric').checked) drawMetricGrid();
    if (showRef && !state.vectorClean) strokeCleanSegments(refmap.map.segments);
    if (state.vectorClean) strokeCleanSegments(state.vectorClean.segments);
    drawCorners();
    for (const e of state.scans.filter((v) => v.visible)) drawRobot(e, e === state.selected);
    drawLive(s);
    renderStatus();
    return;
  }

  if (showRef) {
    // carte de référence en fond : sol blanc, murs gris, segments gris foncé
    const g = refmap.map.grid;
    refmap.image ??= referenceImage(g);
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(refmap.image, sx(g.originX), sy(g.originY + g.height * g.res), g.width * g.res * s, g.height * g.res * s);
    ctx.strokeStyle = '#495057';
    ctx.lineWidth = 2;
    ctx.lineCap = 'round';
    strokeSegments(refmap.map.segments);
  }

  if ($('showGrid').checked && state.grid) {
    const g = state.grid;
    ctx.imageSmoothingEnabled = false;
    ctx.globalAlpha = showRef ? 0.6 : 0.9;
    ctx.drawImage(gridImage(), sx(g.originX), sy(g.originY + g.height * g.res), g.width * g.res * s, g.height * g.res * s);
    ctx.globalAlpha = 1;
  }

  if ($('showMetric').checked) drawMetricGrid();

  if ($('showRef').checked && state.reference) {
    ctx.save();
    ctx.setLineDash([6, 4]);
    ctx.strokeStyle = '#8a949c';
    ctx.lineWidth = 1.2;
    strokeSegments(state.reference.segments);
    ctx.restore();
  }

  if ($('showVector').checked && state.vector) {
    ctx.strokeStyle = '#d6336c';
    ctx.lineWidth = 3;
    ctx.lineCap = 'round';
    strokeSegments(state.vector.segments);
    ctx.fillStyle = '#d6336c';
    for (const seg of state.vector.segments) {
      dot(seg.x1, seg.y1, 3);
      dot(seg.x2, seg.y2, 3);
    }
  }

  const visible = state.scans.filter((e) => e.visible);
  const sel = state.selected;

  if ($('showRays').checked && sel && sel.visible) {
    ctx.strokeStyle = hexAlpha(sel.color, 0.18);
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (const p of worldPoints(sel)) {
      ctx.moveTo(sx(sel.pose.x), sy(sel.pose.y));
      ctx.lineTo(sx(p.x), sy(p.y));
    }
    ctx.stroke();
  }

  if ($('showSegments').checked) {
    for (const e of visible) {
      ctx.strokeStyle = e.color;
      ctx.lineWidth = 2;
      strokeSegments(worldSegments(e));
    }
  }

  if ($('showPoints').checked) {
    const size = Math.max(2, Math.min(4, s / 40));
    for (const e of visible) {
      ctx.fillStyle = e.color;
      for (const p of worldPoints(e)) ctx.fillRect(sx(p.x) - size / 2, sy(p.y) - size / 2, size, size);
    }
  }

  for (const e of visible) drawRobot(e, e === sel);
  drawLive(s);
  renderStatus();
}

/** Scan en temps réel : en noir, à la pose du scan sélectionné (cyan en vue propre, comme le scan de MapTool). */
function drawLive(s) {
  if (!$('showLive').checked || !live.scan) return;
  const stale = Date.now() - live.time > 3000; // plus de nouveau scan depuis 3 s
  const clean = $('showClean').checked && state.gridClean;
  ctx.fillStyle = clean ? (stale ? 'rgba(6,182,212,0.4)' : '#06b6d4') : stale ? 'rgba(29,35,39,0.3)' : '#1d2327';
  const size = Math.max(2.5, Math.min(4.5, s / 35));
  for (const p of livePoints()) ctx.fillRect(sx(p.x) - size / 2, sy(p.y) - size / 2, size, size);
  drawRobot({ pose: livePose(), color: '#ffffff' }, true);
}

/** Image de la grid map propre aux couleurs MapTool (inconnu transparent). */
function cleanGridImage(g) {
  const off = document.createElement('canvas');
  off.width = g.width;
  off.height = g.height;
  const octx = off.getContext('2d');
  const img = octx.createImageData(g.width, g.height);
  const color = maptoolColor(g);
  for (let j = 0; j < g.height; j++) {
    for (let i = 0; i < g.width; i++) {
      const rgb = color(g.index(i, j));
      if (!rgb) continue;
      const o = 4 * ((g.height - 1 - j) * g.width + i);
      img.data.set(rgb, o);
      img.data[o + 3] = 255;
    }
  }
  octx.putImageData(img, 0, 0);
  return off;
}

/**
 * Versions propres des cartes (recalculées à chaque reconstruction) : murs et objets fixes
 * (table, armoire...), voir cleanMaps. La carte vectorielle propre part de segments SANS
 * découpe par les absences : la découpe efface les meubles que des faisceaux traversent
 * (sous une table, entre ses pieds) avant qu'ils aient pu être reconnus.
 */
function refreshClean() {
  state.gridCleanImage = null;
  state.gridClean = null;
  state.vectorClean = null;
  state.corners = null;
  if (!state.grid) return;
  const vis = state.scans.filter((e) => e.visible);
  let segments = [];
  if (state.vector && !state.vector.params.absence) segments = state.vector.segments;
  else if (state.vector) {
    const m = new VectorMap([], { ...vecParams(), absence: false });
    for (const e of vis) m.integrateScan(...integrateOpts(e));
    segments = m.segments;
  }
  const c = cleanMaps(state.grid, segments, vis.map((e) => e.pose), {
    vector: { minObservations: Math.max(1, Number($('vecMinObs').value) || 1) },
    views: vis.map((e) => ({ pose: e.pose, points: worldPoints(e) })), // meubles surélevés
  });
  state.gridClean = c.grid;
  if (state.vector) {
    state.vectorClean = { segments: c.segments, dominant: c.dominant };
    state.corners = c.corners;
  }
}

/** Segments propres : murs en rouge (comme MapTool), objets fixes (meubles) en violet. */
function strokeCleanSegments(segs) {
  ctx.lineWidth = 3;
  ctx.lineCap = 'round';
  ctx.strokeStyle = MAPTOOL.segment;
  strokeSegments(segs.filter((s) => s.kind !== 'objet'));
  ctx.strokeStyle = MAPTOOL.object;
  strokeSegments(segs.filter((s) => s.kind === 'objet' && !s.raised));
  // meubles surélevés (le lidar passe dessous) : pointillés, comme sur un plan d'architecte
  ctx.setLineDash([9, 6]);
  ctx.lineCap = 'butt';
  strokeSegments(segs.filter((s) => s.kind === 'objet' && s.raised));
  ctx.setLineDash([]);
}

/** Coins (points d'intérêt) de la carte vectorielle propre : losanges orange, comme le tracé en cours de MapTool. */
function drawCorners() {
  if (!state.corners) return;
  ctx.fillStyle = '#f59e0b';
  ctx.strokeStyle = '#1e293b';
  ctx.lineWidth = 1.5;
  for (const c of state.corners) {
    const x = sx(c.x);
    const y = sy(c.y);
    ctx.beginPath();
    ctx.moveTo(x, y - 6);
    ctx.lineTo(x + 6, y);
    ctx.lineTo(x, y + 6);
    ctx.lineTo(x - 6, y);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
  }
}

// ------------------------------------------------------------------ carte de référence (jour 2)
// La carte de la salle construite au jour 1, figée : on y localise les nouveaux scans.

const refmap = { map: null, image: null, cleanImage: null, gridRef: null, vectorRef: null };

function setReferenceMap(map) {
  Object.assign(refmap, { map, image: null, cleanImage: null, gridRef: null, vectorRef: null });
  const d = new Date(map.date);
  const when = `${d.toLocaleDateString('fr-FR')} ${d.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit' })}`;
  $('refStat').textContent = `${map.name} · ${map.segments.length} segments, ${map.corners.length} coins, grid ${map.grid.width} × ${map.grid.height} (${Math.round(map.grid.res * 100)} cm) · ${map.clean ? 'version propre' : 'version brute'} · ${when}`;
  $('showRefMap').checked = true;
  $('locRef').value = 'refvector';
  fillMethods();
  // multi-résolution : 17 scans réels sur 18 relocalisés à moins de 5 cm et 2° depuis un départ
  // à 29 cm et 8° ; le calcul direct, local, en rate 2 (au-delà de sa marge de 25 cm)
  $('locMethod').value = 'multires';
  if (!state.scans.length) fitView();
  draw();
}

/** Image de la carte de référence pour la vue normale : sol blanc, occupé gris, inconnu transparent. */
function referenceImage(g) {
  const off = document.createElement('canvas');
  off.width = g.width;
  off.height = g.height;
  const octx = off.getContext('2d');
  const img = octx.createImageData(g.width, g.height);
  for (let j = 0; j < g.height; j++) {
    for (let i = 0; i < g.width; i++) {
      const c = g.classify(g.index(i, j));
      if (c === 0) continue;
      const o = 4 * ((g.height - 1 - j) * g.width + i);
      img.data.set(c === -1 ? [255, 255, 255, 255] : isPartlyOccupied(g, g.index(i, j)) ? [193, 198, 204, 255] : [134, 142, 150, 255], o);
    }
  }
  octx.putImageData(img, 0, 0);
  return off;
}

async function loadProjectReference() {
  try {
    const res = await fetch('../data/reference/carte_reference.json', { cache: 'no-store' });
    if (!res.ok) throw new Error('fichier data/reference/carte_reference.json introuvable (node scripts/build-reference.mjs)');
    setReferenceMap(parseReferenceMap(await res.text()));
    toast('Carte de la salle chargée comme référence');
  } catch (err) {
    toast(`Chargement impossible : ${err.message}`);
  }
}

/** Les cartes actuelles (scans visibles) deviennent la référence, en version propre ou brute. */
function freezeReference() {
  const vis = state.scans.filter((e) => e.visible);
  if (!vis.length) return toast('Aucun scan visible pour construire la carte');
  if (!state.grid) buildGridAll(true);
  if (!state.vector) buildVectorAll(true);
  refreshClean();
  const clean = $('refClean').checked;
  setReferenceMap(
    makeReferenceMap({
      name: `Carte figée (${vis.length} scans)`,
      grid: clean ? state.gridClean : state.grid,
      segments: clean ? state.vectorClean.segments : state.vector.segments,
      corners: clean ? state.corners ?? [] : [],
      scans: vis.map((e) => ({ name: e.name, pose: e.pose })),
      clean,
    }),
  );
  toast('Cartes figées comme référence : pense à « Enregistrer… »');
}

function saveReference() {
  if (!refmap.map) return toast('Aucune carte de référence à enregistrer');
  download('carte_reference.json', serializeReferenceMap(refmap.map), 'application/json');
}

// ------------------------------------------------------------------ localisation (jour 2)

/** Type de référence (grid, vector, scan) correspondant au choix de la liste. */
function locKind() {
  const v = $('locRef').value;
  return { imported: 'vector', refvector: 'vector', refgrid: 'grid' }[v] ?? v;
}

/** Méthodes proposées selon la référence choisie. */
function fillMethods() {
  const kind = locKind();
  const sel = $('locMethod');
  const prev = sel.value;
  sel.replaceChildren(...METHODS_BY_REFERENCE[kind].map((m) => new Option(METHODS[m].label, m)));
  if (METHODS_BY_REFERENCE[kind].includes(prev)) sel.value = prev;
}

/**
 * Référence de localisation pour le scan `target` : grid map ou carte vectorielle construites
 * avec les autres scans visibles (ou avec tous, case décochée), carte importée, ou scan précédent.
 */
function locReference(target) {
  const kind = $('locRef').value;
  if (kind === 'refgrid' || kind === 'refvector') {
    if (!refmap.map) throw new Error('Aucune carte de référence : « Charger la carte de la salle » ou « Figer les cartes actuelles »');
    if (kind === 'refgrid') return (refmap.gridRef ??= gridReference(refmap.map.grid));
    return (refmap.vectorRef ??= vectorReference(refmap.map.segments));
  }
  if (kind === 'imported') {
    if (!state.reference) throw new Error('Aucune carte importée : bouton « Référence… » de la carte vectorielle');
    return vectorReference(state.reference.segments);
  }
  const visible = state.scans.filter((e) => e.visible);
  if (kind === 'scan') {
    const k = visible.indexOf(target);
    const ref = k > 0 ? visible[k - 1] : visible.find((e) => e !== target);
    if (!ref) throw new Error('Il faut un autre scan visible comme référence');
    return scanReference(worldPoints(ref));
  }
  const used = $('locExclude').checked ? visible.filter((e) => e !== target) : visible;
  if (!used.length) throw new Error("Il faut d'autres scans visibles pour construire la carte");
  if (kind === 'grid') {
    const [first, ...rest] = used;
    const g = gridFromScan(first.scan, first.pose, gridParams(), filterOpts(), { flipAngles: first.flip });
    for (const e of rest) g.integrateScan(...integrateOpts(e));
    return gridReference(g);
  }
  const m = new VectorMap([], { ...vecParams(), absence: false });
  for (const e of used) m.integrateScan(...integrateOpts(e));
  return vectorReference(m.segments);
}

/** Tableau des résultats : déplacement depuis la pose initiale, écart à la pose du fichier, temps. */
function showLocResults(results, init, entry) {
  const ref = entry.scan.poseFound ? entry.filePose : null;
  const best = results.reduce((a, b) => (b.inliers > a.inliers ? b : a));
  const rows = results.map((r) => {
    const moved = poseError(r.pose, init);
    const err = ref ? poseError(r.pose, ref) : null;
    return `<tr class="${r === best && results.length > 1 ? 'best' : ''}"><td>${METHODS[r.method].label}</td>
      <td>${(moved.dist * 100).toFixed(1)} cm / ${moved.angle.toFixed(1)}°</td>
      ${ref ? `<td>${(err.dist * 100).toFixed(1)} cm / ${err.angle.toFixed(2)}°</td>` : ''}
      <td>${Math.round(r.inliers * 100)} %</td><td>${r.time.toFixed(0)} ms</td></tr>`;
  });
  $('locResult').innerHTML = `<table class="loc-table"><tr><th>Méthode</th><th>Correction</th>${ref ? '<th>Écart pose fichier</th>' : ''}<th>Points sur la carte</th><th>Temps</th></tr>${rows.join('')}</table>`;
}

function runLocalize() {
  const e = requireSelection();
  if (!e) return;
  let ref;
  try {
    ref = locReference(e);
  } catch (err) {
    return toast(err.message);
  }
  const init = { ...e.pose };
  const r = localize(ref, local(e).pts, init, $('locMethod').value);
  e.pose = r.pose;
  poseChanged();
  showLocResults([r], init, e);
  toast(`${e.name} localisé (${METHODS[r.method].label}) : ${Math.round(r.inliers * 100)} % des points sur la carte`);
}

/** Toutes les méthodes depuis la même pose initiale ; la pose du scan n'est pas modifiée. */
async function runCompare() {
  const e = requireSelection();
  if (!e) return;
  let ref;
  try {
    ref = locReference(e);
  } catch (err) {
    return toast(err.message);
  }
  const kind = locKind();
  const init = { ...e.pose };
  const results = [];
  for (const m of METHODS_BY_REFERENCE[kind]) {
    toast(`Comparaison : ${METHODS[m].label}…`);
    await tick();
    results.push(localize(ref, local(e).pts, init, m));
  }
  showLocResults(results, init, e);
  toast('Comparaison terminée (pose du scan inchangée)');
}

/** Bouton « Exporter tout le jour 1 » : figures brutes et propres, cartes, fichiers MapTool, poses. */
async function exportDayOne() {
  const vis = state.scans.filter((e) => e.visible);
  if (!vis.length) return toast('Aucun scan visible à exporter');
  if (!state.grid) buildGridAll(true);
  if (!state.vector) buildVectorAll(true);
  refreshClean();
  toast('Préparation des figures…');
  await tick();
  const files = await dayOneExports({
    scans: vis.map((e) => ({ name: e.name, color: e.color, pose: e.pose, points: worldPoints(e), segments: worldSegments(e) })),
    grid: state.grid,
    gridClean: state.gridClean,
    vector: state.vector,
    vectorClean: state.vectorClean,
    corners: state.corners ?? [],
  });
  await saveFiles(files);
}

/** Points du scan en temps réel, dans le repère monde (à la pose du scan sélectionné). */
function livePoints() {
  return live.scan ? transformPoints(scanToLocalXY(live.scan, filterOpts()), livePose()) : [];
}

function drawMetricGrid() {
  const s = state.view.scale;
  const step = s > 25 ? 1 : 5;
  const x0 = Math.floor(wx(0) / step) * step;
  const x1 = wx(W());
  const y0 = Math.floor(wy(H()) / step) * step;
  const y1 = wy(0);
  ctx.lineWidth = 1;
  for (let x = x0; x <= x1; x += step) {
    ctx.strokeStyle = x === 0 ? 'rgba(29,35,39,0.35)' : 'rgba(29,35,39,0.07)';
    ctx.beginPath();
    ctx.moveTo(Math.round(sx(x)) + 0.5, 0);
    ctx.lineTo(Math.round(sx(x)) + 0.5, H());
    ctx.stroke();
  }
  for (let y = y0; y <= y1; y += step) {
    ctx.strokeStyle = y === 0 ? 'rgba(29,35,39,0.35)' : 'rgba(29,35,39,0.07)';
    ctx.beginPath();
    ctx.moveTo(0, Math.round(sy(y)) + 0.5);
    ctx.lineTo(W(), Math.round(sy(y)) + 0.5);
    ctx.stroke();
  }
}

function strokeSegments(segs) {
  ctx.beginPath();
  for (const g of segs) {
    ctx.moveTo(sx(g.x1), sy(g.y1));
    ctx.lineTo(sx(g.x2), sy(g.y2));
  }
  ctx.stroke();
}

function dot(x, y, r) {
  ctx.beginPath();
  ctx.arc(sx(x), sy(y), r, 0, 2 * Math.PI);
  ctx.fill();
}

function drawRobot(e, selected) {
  const { x, y, theta } = e.pose;
  const a = theta * DEG;
  const size = selected ? 13 : 10;
  ctx.save();
  ctx.translate(sx(x), sy(y));
  ctx.rotate(-a);
  ctx.beginPath();
  ctx.moveTo(size, 0);
  ctx.lineTo(-size * 0.6, size * 0.6);
  ctx.lineTo(-size * 0.6, -size * 0.6);
  ctx.closePath();
  ctx.fillStyle = e.color;
  ctx.fill();
  ctx.lineWidth = selected ? 2.5 : 1;
  ctx.strokeStyle = selected ? '#1d2327' : '#ffffff';
  ctx.stroke();
  ctx.restore();
}

function hexAlpha(hex, a) {
  const n = parseInt(hex.slice(1), 16);
  return `rgba(${n >> 16}, ${(n >> 8) & 255}, ${n & 255}, ${a})`;
}

function renderStatus() {
  const m = state.mouse;
  let txt = `échelle ${state.view.scale.toFixed(0)} px/m`;
  if (m) {
    txt = `x = ${m.x.toFixed(3)} m · y = ${m.y.toFixed(3)} m · ` + txt;
    const g = state.grid;
    if (g) {
      const [i, j] = g.worldToCell(m.x, m.y);
      if (g.inside(i, j)) txt += ` · cellule (${i}, ${j}) p = ${g.probability(g.index(i, j)).toFixed(2)}`;
    }
  }
  $('status').textContent = txt;
}

// ------------------------------------------------------------------ souris et clavier

let drag = null;

function eventWorld(ev) {
  const r = canvas.getBoundingClientRect();
  const px = ev.clientX - r.left;
  const py = ev.clientY - r.top;
  return { px, py, x: wx(px), y: wy(py) };
}

/** Scan visible dont un point (ou le robot) est à moins de `tol` pixels. */
function hitScan(px, py, tol = 7) {
  const order = [state.selected, ...state.scans.filter((e) => e !== state.selected)].filter((e) => e && e.visible);
  for (const e of order) {
    if (Math.hypot(sx(e.pose.x) - px, sy(e.pose.y) - py) < 14) return e;
    for (const p of worldPoints(e)) if (Math.hypot(sx(p.x) - px, sy(p.y) - py) < tol) return e;
  }
  return null;
}

canvas.addEventListener('mousedown', (ev) => {
  const m = eventWorld(ev);
  const sel = state.selected;
  if (ev.shiftKey && sel) {
    drag = { mode: 'rotate', entry: sel, a0: Math.atan2(m.y - sel.pose.y, m.x - sel.pose.x), t0: sel.pose.theta };
  } else {
    const hit = hitScan(m.px, m.py);
    if (hit) {
      if (hit !== sel) select(hit);
      drag = { mode: 'move', entry: hit, dx: hit.pose.x - m.x, dy: hit.pose.y - m.y };
    } else {
      drag = { mode: 'pan', px: m.px, py: m.py, vx: state.view.x, vy: state.view.y };
    }
  }
  canvas.classList.add('dragging');
});

window.addEventListener('mousemove', (ev) => {
  const m = eventWorld(ev);
  state.mouse = ev.target === canvas ? m : null;
  if (!drag) return renderStatus();
  if (drag.mode === 'pan') {
    state.view.x = drag.vx - (m.px - drag.px) / state.view.scale;
    state.view.y = drag.vy + (m.py - drag.py) / state.view.scale;
    draw();
  } else if (drag.mode === 'move') {
    drag.entry.pose.x = m.x + drag.dx;
    drag.entry.pose.y = m.y + drag.dy;
    poseChanged({ final: false });
  } else {
    const a = Math.atan2(m.y - drag.entry.pose.y, m.x - drag.entry.pose.x);
    drag.entry.pose.theta = normalizeAngleDeg(drag.t0 + (a - drag.a0) / DEG);
    poseChanged({ final: false });
  }
});

window.addEventListener('mouseup', () => {
  if (drag && drag.mode !== 'pan') poseChanged();
  drag = null;
  canvas.classList.remove('dragging');
});

canvas.addEventListener('wheel', (ev) => {
  ev.preventDefault();
  const m = eventWorld(ev);
  const f = Math.exp(-ev.deltaY * 0.0015);
  state.view.scale = Math.min(2000, Math.max(5, state.view.scale * f));
  // garder le point sous le curseur fixe
  state.view.x = m.x - (m.px - W() / 2) / state.view.scale;
  state.view.y = m.y + (m.py - H() / 2) / state.view.scale;
  draw();
}, { passive: false });

window.addEventListener('keydown', (ev) => {
  if (['INPUT', 'SELECT', 'TEXTAREA'].includes(document.activeElement?.tagName) || $('pasteDialog').open) return;
  if (ev.key === ' ' && live.scan) {
    ev.preventDefault();
    liveCapture();
    return;
  }
  const e = state.selected;
  if (!e) return;
  const d = ev.shiftKey ? 0.1 : 0.01;
  const r = ev.shiftKey ? 5 : 0.5;
  const k = ev.key.toLowerCase();
  if (ev.key === 'ArrowLeft') e.pose.x -= d;
  else if (ev.key === 'ArrowRight') e.pose.x += d;
  else if (ev.key === 'ArrowUp') e.pose.y += d;
  else if (ev.key === 'ArrowDown') e.pose.y -= d;
  else if (k === 'q' || k === 'a') e.pose.theta = normalizeAngleDeg(e.pose.theta + r);
  else if (k === 'e') e.pose.theta = normalizeAngleDeg(e.pose.theta - r);
  else return;
  ev.preventDefault();
  poseChanged();
});

// ------------------------------------------------------------------ glisser-déposer

let dragDepth = 0;
window.addEventListener('dragenter', (ev) => {
  ev.preventDefault();
  dragDepth++;
  $('drop').hidden = false;
});
window.addEventListener('dragleave', () => {
  if (--dragDepth <= 0) $('drop').hidden = true;
});
window.addEventListener('dragover', (ev) => ev.preventDefault());
window.addEventListener('drop', (ev) => {
  ev.preventDefault();
  dragDepth = 0;
  $('drop').hidden = true;
  loadFiles([...ev.dataTransfer.files]);
});

// ------------------------------------------------------------------ boutons

$('fileScans').onchange = (ev) => {
  loadFiles([...ev.target.files]);
  ev.target.value = '';
};
$('btnSim').onclick = loadSimulated;
$('liveAdd').onclick = liveCapture;
$('btnAlign').onclick = runAutoAlign;
$('btnExportDay1').onclick = exportDayOne;
$('btnRefProject').onclick = loadProjectReference;
$('btnRefFreeze').onclick = freezeReference;
$('btnRefSave').onclick = saveReference;
$('refLoad').onchange = async (ev) => {
  const f = ev.target.files[0];
  ev.target.value = '';
  if (!f) return;
  try {
    setReferenceMap(parseReferenceMap(await readFile(f)));
    toast(`Carte de référence chargée : ${f.name}`);
  } catch (err) {
    toast(err.message);
  }
};
$('showRefMap').addEventListener('change', draw);
$('locRef').onchange = fillMethods;
$('btnLocalize').onclick = runLocalize;
$('btnCompare').onclick = runCompare;
$('vecMinObs').onchange = () => {
  refreshClean();
  draw();
};
$('btnSaveAll').onclick = saveAll;
$('btnPaste').onclick = () => {
  $('pasteText').value = '';
  $('pasteDialog').showModal();
  $('pasteText').focus();
};
$('pasteOk').onclick = (ev) => {
  ev.preventDefault();
  $('pasteDialog').close();
  if ($('pasteText').value.trim()) addPastedText($('pasteText').value);
};
// Ctrl+V n'importe où hors des champs de saisie : ajoute directement le scan collé.
document.addEventListener('paste', (ev) => {
  if (['INPUT', 'SELECT', 'TEXTAREA'].includes(document.activeElement?.tagName) || $('pasteDialog').open) return;
  const text = ev.clipboardData?.getData('text');
  if (!text) return;
  ev.preventDefault();
  addPastedText(text);
});
$('btnFit').onclick = fitView;
$('btnShot').onclick = () => canvas.toBlob((b) => download('capture.png', b));

for (const id of ['poseX', 'poseY', 'poseT']) {
  $(id).addEventListener('input', () => {
    const e = state.selected;
    if (!e) return;
    const v = Number($(id).value);
    if (!Number.isFinite(v)) return;
    if (id === 'poseX') e.pose.x = v;
    else if (id === 'poseY') e.pose.y = v;
    else e.pose.theta = v;
    poseChanged();
  });
}
$('poseFlip').onchange = () => {
  if (!state.selected) return;
  state.selected.flip = $('poseFlip').checked;
  renderScanList();
  poseChanged();
};
$('btnPoseReset').onclick = () => {
  if (!state.selected) return;
  state.selected.pose = { ...state.selected.filePose };
  poseChanged();
};
$('btnScanExport').onclick = () => {
  const e = requireSelection();
  if (e) download(e.name.replace(/\.(csv|txt)$/i, '') + '_pose.csv', serializeScanCSV(e.scan, e.pose), 'text/csv');
};

for (const id of ['fMin', 'fMax', 'fInt']) {
  $(id).addEventListener('change', () => {
    renderScanList();
    poseChanged();
  });
}

$('gridFromSel').onclick = () => {
  const e = requireSelection();
  if (!e) return;
  setGrid(gridFromScan(e.scan, e.pose, gridParams(), filterOpts(), { flipAngles: e.flip }));
  toast(`Grid map créée depuis ${e.name}`);
};
$('gridUpdateSel').onclick = () => {
  const e = requireSelection();
  if (!e) return;
  if (!state.grid) return $('gridFromSel').onclick();
  const st = state.grid.integrateScan(...integrateOpts(e));
  setGrid(state.grid);
  toast(`${e.name} intégré : ${st.freeCells} cellules libérées, ${st.occupiedCells} cellules d'impact`);
};
$('gridAll').onclick = () => buildGridAll();
$('gridClear').onclick = () => setGrid(null);
$('gridDecay').onclick = () => {
  if (!state.grid) return;
  const n = state.grid.decay(0.7, 1);
  setGrid(state.grid);
  toast(`Décroissance appliquée à ${n} cellules`);
};
$('gridIsolated').onclick = () => {
  if (!state.grid) return;
  const n = state.grid.removeIsolated();
  setGrid(state.grid);
  toast(`${n} cellules isolées retirées`);
};
$('gridExport').onclick = () => state.grid && download('grid_map.csv', state.grid.toCSV(), 'text/csv');
$('gridExportPng').onclick = () => state.grid && gridImage().toBlob((b) => download('grid_map.png', b));
$('gridImport').onchange = async (ev) => {
  const f = ev.target.files[0];
  ev.target.value = '';
  if (!f) return;
  try {
    setGrid(GridMap.fromCSV(await readFile(f), gridParams()));
  } catch (err) {
    toast(err.message);
  }
};
$('gridRes').onchange = () => state.grid && buildGridAll();

$('vecFromSel').onclick = () => {
  const e = requireSelection();
  if (!e) return;
  const m = new VectorMap([], vecParams());
  m.integrateScan(...integrateOpts(e));
  setVector(m);
};
$('vecUpdateSel').onclick = () => {
  const e = requireSelection();
  if (!e) return;
  if (!state.vector) return $('vecFromSel').onclick();
  state.vector.params.absence = $('vecAbsence').checked;
  const st = state.vector.integrateScan(...integrateOpts(e));
  setVector(state.vector, `dernier scan : ${st.extracted} extraits, ${st.merged} fusions, ${st.added} ajouts, ${st.cuts} découpes`);
};
$('vecAll').onclick = () => buildVectorAll();
$('vecClear').onclick = () => setVector(null);
$('vecExport').onclick = () => state.vector && download('vector_map.txt', state.vector.toTXT());
$('vecImport').onchange = async (ev) => {
  const f = ev.target.files[0];
  ev.target.value = '';
  if (f) setVector(VectorMap.fromTXT(await readFile(f), vecParams()), 'importée');
};
$('refImport').onchange = async (ev) => {
  const f = ev.target.files[0];
  ev.target.value = '';
  if (!f) return;
  state.reference = VectorMap.fromTXT(await readFile(f));
  $('showRef').checked = true;
  draw();
};

for (const id of ['showPoints', 'showRays', 'showSegments', 'showGrid', 'showVector', 'showRef', 'showMetric', 'showLive', 'showClean']) {
  $(id).addEventListener('change', draw);
}

window.addEventListener('resize', resize);
resize();
renderScanList();
fillMethods();
startLive();
