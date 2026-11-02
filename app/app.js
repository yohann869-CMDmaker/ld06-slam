// Application web : affichage de plusieurs scans, poses modifiables,
// construction et mise à jour d'une grid map et d'une carte vectorielle.
import { DEG, normalizeAngleDeg, transformPoints } from '../src/core/pose.js';
import { parseScanCSV, scanToLocalXY, serializeScanCSV, validPoints } from '../src/core/scan.js';
import { GridMap, gridFromScan, occupancyRGB } from '../src/core/gridmap.js';
import { VectorMap } from '../src/core/vectormap.js';
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
  draw();
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
/** Une pose ou la visibilité a changé : redessin, et reconstruction des cartes si demandé. */
function poseChanged({ final = true } = {}) {
  renderPoseEditor();
  draw();
  if ($('autoRebuild').checked && final) {
    clearTimeout(rebuildTimer);
    rebuildTimer = setTimeout(() => {
      if (state.grid) buildGridAll(true);
      if (state.vector) buildVectorAll(true);
    }, 120);
  }
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
    toast(`${index.scans.length} scans simulés chargés (référence : scene_A.txt, masquée)`);
  } catch (err) {
    toast(`Chargement impossible : ${err.message}`);
  }
}

// ------------------------------------------------------------------ acquisition USB (Web Serial)
// Dialogue avec firmware/ESP32_LD06_Capture : 's' demande un scan, renvoyé entre
// "# LD06 scan ..." et "# end" ; 'i' renvoie une ligne "# info ...".

const usb = { port: null, reader: null, writer: null, loop: null, buffer: '', block: null, waiting: null };

function usbStatus(msg) {
  $('usbStat').textContent = msg;
}

function usbSend(text) {
  return usb.writer.write(new TextEncoder().encode(text));
}

async function usbConnect() {
  if (usb.port) return usbDisconnect();
  if (!('serial' in navigator)) return toast('Acquisition USB : ouvrir l\'application dans Chrome ou Edge (Web Serial)');
  try {
    usb.port = await navigator.serial.requestPort();
    await usb.port.open({ baudRate: 115200 });
  } catch (err) {
    usb.port = null;
    if (err.name !== 'NotFoundError') toast(`Connexion impossible : ${err.message}`);
    return;
  }
  usb.writer = usb.port.writable.getWriter();
  usb.loop = usbReadLoop();
  $('usbConnect').textContent = 'Déconnecter';
  $('usbCapture').disabled = false;
  usbStatus('Connecté, attente du robot…');
  // L'ESP32 redémarre souvent à l'ouverture du port : on demande l'état après son démarrage.
  setTimeout(() => usb.writer && usbSend('i'), 2000);
}

async function usbDisconnect() {
  try {
    await usb.reader?.cancel();
    await usb.loop;
    usb.writer?.releaseLock();
    await usb.port?.close();
  } catch {
    // port déjà fermé (câble débranché)
  }
  Object.assign(usb, { port: null, reader: null, writer: null, loop: null, buffer: '', block: null });
  $('usbConnect').textContent = 'Connecter le robot (USB)';
  $('usbCapture').disabled = true;
  usbStatus('Déconnecté');
}

async function usbReadLoop() {
  const decoder = new TextDecoder();
  usb.reader = usb.port.readable.getReader();
  try {
    for (;;) {
      const { value, done } = await usb.reader.read();
      if (done) break;
      usb.buffer += decoder.decode(value, { stream: true });
      let nl;
      while ((nl = usb.buffer.indexOf('\n')) >= 0) {
        const line = usb.buffer.slice(0, nl).replace(/\r$/, '');
        usb.buffer = usb.buffer.slice(nl + 1);
        onUsbLine(line);
      }
    }
  } catch (err) {
    toast(`Liaison USB interrompue : ${err.message}`);
  } finally {
    usb.reader.releaseLock();
  }
}

function onUsbLine(line) {
  if (line.startsWith('# LD06 scan')) {
    usb.block = [line];
  } else if (usb.block) {
    usb.block.push(line);
    if (line.startsWith('# end')) {
      const text = usb.block.join('\n');
      usb.block = null;
      usb.waiting?.resolve(text);
    }
  } else if (line.startsWith('# error')) {
    usb.waiting?.reject(new Error(line.slice(8)));
  } else if (line.startsWith('# LD06 ready')) {
    usbStatus('Robot prêt');
  } else if (line.startsWith('# info')) {
    usbStatus(line.includes('scans=0') ? `Le lidar n'envoie aucun scan : ${line.slice(7)}` : `Robot prêt : ${line.slice(7)}`);
  } else if (line.startsWith('# raw')) {
    usbStatus(`Diagnostic : ${line.slice(6)}`);
  }
  // autres lignes (messages de démarrage de l'ESP32) : ignorées
}

/** Nom de fichier horodaté : pas d'écrasement d'une séance de TP à l'autre. */
function captureName() {
  const d = new Date();
  const p = (n) => String(n).padStart(2, '0');
  return `scan_${p(d.getMonth() + 1)}${p(d.getDate())}_${p(d.getHours())}${p(d.getMinutes())}${p(d.getSeconds())}.csv`;
}

async function usbCapture() {
  if (!usb.writer || usb.waiting) return;
  $('usbCapture').disabled = true;
  usbStatus('Capture en cours…');
  let timer;
  try {
    const text = await new Promise((resolve, reject) => {
      usb.waiting = { resolve, reject };
      timer = setTimeout(() => reject(new Error('pas de réponse du robot (programme ESP32_LD06_Capture téléversé ?)')), 5000);
      usbSend('s').catch(reject);
    });
    const scan = parseScanCSV(text, captureName());
    const first = state.scans.length === 0;
    // Le robot a peu bougé depuis la capture précédente : on part de la pose du scan
    // sélectionné, il ne reste qu'à recaler.
    const ref = state.selected;
    const entry = addScan(scan);
    if (ref) entry.pose = { ...ref.pose };
    select(entry);
    if (first) fitView();
    usbStatus(`${scan.name} : ${validPoints(scan, filterOpts()).length} points valides · ${scan.comments[0].replace('LD06 scan ', '')}`);
  } catch (err) {
    usbStatus(`Échec de la capture : ${err.message}`);
  } finally {
    clearTimeout(timer);
    usb.waiting = null;
    $('usbCapture').disabled = !usb.writer;
  }
}

/** Enregistre chaque scan avec sa pose actuelle, plus un récapitulatif poses.csv. */
async function saveAll() {
  if (!state.scans.length) return toast('Aucun scan à enregistrer');
  const files = state.scans.map((e) => ({ name: e.name.replace(/\.(csv|txt)$/i, '') + '.csv', text: serializeScanCSV(e.scan, e.pose) }));
  files.push({
    name: 'poses.csv',
    text: ['scan,x_m,y_m,theta_deg', ...state.scans.map((e) => `${e.name},${e.pose.x.toFixed(4)},${e.pose.y.toFixed(4)},${e.pose.theta.toFixed(2)}`)].join('\n') + '\n',
  });
  if (!window.showDirectoryPicker) {
    for (const f of files) download(f.name, f.text, 'text/csv');
    return;
  }
  try {
    const dir = await window.showDirectoryPicker({ mode: 'readwrite' });
    for (const f of files) {
      const w = await (await dir.getFileHandle(f.name, { create: true })).createWritable();
      await w.write(f.text);
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
  for (const e of state.scans.filter((s) => s.visible)) {
    for (const p of worldPoints(e)) {
      xs.push(p.x);
      ys.push(p.y);
    }
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
  ctx.fillStyle = '#fbfbf9';
  ctx.fillRect(0, 0, W(), H());
  const s = state.view.scale;

  if ($('showGrid').checked && state.grid) {
    const g = state.grid;
    ctx.imageSmoothingEnabled = false;
    ctx.globalAlpha = 0.9;
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
  renderStatus();
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
  const e = state.selected;
  if (!e || ['INPUT', 'SELECT', 'TEXTAREA'].includes(document.activeElement?.tagName)) return;
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
$('usbConnect').onclick = usbConnect;
$('usbCapture').onclick = usbCapture;
$('btnSaveAll').onclick = saveAll;
navigator.serial?.addEventListener('disconnect', (ev) => {
  if (ev.target === usb.port) usbDisconnect();
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

for (const id of ['showPoints', 'showRays', 'showSegments', 'showGrid', 'showVector', 'showRef', 'showMetric']) {
  $(id).addEventListener('change', draw);
}

window.addEventListener('resize', resize);
resize();
renderScanList();
