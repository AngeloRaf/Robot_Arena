'use strict';

/* ── Références DOM ──────────────────────────────────────────────────── */
const $ = id => document.getElementById(id);

const connDot      = $('conn-dot');
const connLabel    = $('conn-label');
const clock        = $('clock');
const stateDisplay = $('state-display');
const stateText    = $('state-text');
const targetPill   = $('target-pill');
const pillText     = $('pill-text');
const videoOverlay = $('video-overlay');
const overlayMsg   = $('overlay-msg');
const reticle      = $('reticle');
const proxFill     = $('prox-fill');
const telX         = $('tel-x');
const telArea      = $('tel-area');
const telMode      = $('tel-mode');
const telPing      = $('tel-ping');
const toast        = $('toast');
const colorHeroName  = $('color-hero-name');
const colorHeroSwatch = $('color-hero-swatch');

const colorButtons = document.querySelectorAll('.btn-color');
const root = document.documentElement;

/* ── Palettes par couleur de détection ───────────────────────────────── */
const PALETTES = {
  red: {
    main:  '#e53935',
    light: '#fff0f0',
    mid:   '#ffc4c4',
    label: 'ROUGE',
  },
  blue: {
    main:  '#1e6fdb',
    light: '#f0f5ff',
    mid:   '#b3cff7',
    label: 'BLEU',
  },
  yellow: {
    main:  '#e6a800',
    light: '#fffbe0',
    mid:   '#ffe082',
    label: 'JAUNE',
  },
  green: {
    main:  '#28a745',
    light: '#f0fff4',
    mid:   '#a8e6b8',
    label: 'VERT',
  },
  none: {
    main:  '#8890aa',
    light: '#f5f6f8',
    mid:   '#d0d4de',
    label: 'AUCUNE',
  },
};

/* ── Application de la charte dynamique ─────────────────────────────── */
let lastAppliedColor = '';

function applyPalette(colorName) {
  if (colorName === lastAppliedColor) return;
  lastAppliedColor = colorName;

  const p = PALETTES[colorName] || PALETTES.none;

  /* Met à jour les 3 variables CSS utilisées partout dans style.css */
  root.style.setProperty('--active-color',       p.main);
  root.style.setProperty('--active-color-light', p.light);
  root.style.setProperty('--active-color-mid',   p.mid);

  /* Bandeau hero */
  colorHeroName.textContent  = p.label;
  colorHeroSwatch.style.background = p.main;
}

/* ── Horloge ─────────────────────────────────────────────────────────── */
function updateClock() {
  const n = new Date();
  clock.textContent = [n.getHours(), n.getMinutes(), n.getSeconds()]
    .map(v => String(v).padStart(2, '0')).join(':');
}
setInterval(updateClock, 1000);
updateClock();

/* ── Toast ───────────────────────────────────────────────────────────── */
let toastTimer = null;
function showToast(msg, duration = 2000) {
  toast.textContent = msg;
  toast.classList.add('show');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => toast.classList.remove('show'), duration);
}

/* ── Mise à jour état robot ──────────────────────────────────────────── */
let lastState = '';

function applyState(data) {
  const state = (data.state || 'IDLE').toUpperCase();
  const col   = (data.target_color || 'none').toLowerCase();

  /* Charte graphique dynamique */
  applyPalette(col);

  /* Badge état FSM */
  stateDisplay.className = `state-display state-${state}`;
  stateText.textContent  = state;

  if (state === 'ERROR' && lastState !== 'ERROR') {
    showToast('⚠ ROBOT EN ERREUR', 5000);
  }
  lastState = state;

  /* Cible visible */
  const visible = data.target_visible === true;
  targetPill.classList.toggle('visible', visible);
  pillText.textContent = visible ? 'CIBLE EN VUE' : 'RECHERCHE EN COURS';

  /* Réticule */
  reticle.classList.toggle('active', visible);
  if (visible && typeof data.target_x === 'number') {
    reticle.style.left = `${((data.target_x + 1) / 2 * 100).toFixed(1)}%`;
  }

  /* Barre de proximité */
  const area = typeof data.target_area === 'number' ? data.target_area : 0;
  proxFill.style.width = `${Math.min(area * 100, 100).toFixed(0)}%`;
  proxFill.classList.toggle('danger', area > 0.6);

  /* Highlight bouton actif */
  colorButtons.forEach(btn =>
    btn.classList.toggle('active', btn.dataset.color === col && col !== 'none')
  );

  /* Télémétrie */
  telX.textContent    = typeof data.target_x    === 'number' ? data.target_x.toFixed(2)   : '—';
  telArea.textContent = typeof data.target_area === 'number' ? data.target_area.toFixed(3) : '—';
  telMode.textContent = col !== 'none' ? col.toUpperCase() : 'AUCUN';
}

/* ── Connexion ───────────────────────────────────────────────────────── */
let connectionOk = true;
function setConnected(online) {
  if (online === connectionOk) return;
  connectionOk = online;
  connDot.className     = `conn-indicator ${online ? 'online' : 'offline'}`;
  connLabel.textContent = online ? 'CONNECTÉ' : 'ROBOT DÉCONNECTÉ';
  videoOverlay.classList.toggle('show', !online);
  overlayMsg.textContent = 'ROBOT DÉCONNECTÉ';
  if (!online) showToast('Connexion perdue', 4000);
}

/* ── Polling /api/state ──────────────────────────────────────────────── */
async function pollState() {
  const t0 = performance.now();
  try {
    const res  = await fetch('/api/state');
    const ping = (performance.now() - t0).toFixed(0);
    telPing.textContent = `${ping} ms`;
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    const data = await res.json();
    setConnected(true);
    videoOverlay.classList.remove('show');
    applyState(data);
  } catch (err) {
    setConnected(false);
    telPing.textContent = 'ERR';
  }
}
setInterval(pollState, 500);
pollState();

/* ── Envoi couleur ───────────────────────────────────────────────────── */
async function sendColor(color) {
  /* Feedback visuel immédiat */
  colorButtons.forEach(btn =>
    btn.classList.toggle('active', btn.dataset.color === color)
  );
  applyPalette(color);

  try {
    const res  = await fetch('/api/color', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ color }),
    });
    const data = await res.json();
    if (data.ok) showToast(`Couleur → ${color.toUpperCase()}`);
    else showToast(`Erreur : ${data.error}`, 3000);
  } catch (err) {
    showToast('Envoi échoué — vérifier connexion', 3000);
  }
}

/* ── Bouton STOP ─────────────────────────────────────────────────────── */
async function sendStop() {
  colorButtons.forEach(btn => btn.classList.remove('active'));
  applyPalette('none');
  try {
    await fetch('/api/stop', { method: 'POST' });
    showToast('ARRÊT — robot stoppé');
  } catch (err) {
    showToast('Envoi STOP échoué', 3000);
  }
}

/* ── Handlers ────────────────────────────────────────────────────────── */
colorButtons.forEach(btn =>
  btn.addEventListener('click', () => sendColor(btn.dataset.color))
);
$('btn-stop').addEventListener('click', sendStop);