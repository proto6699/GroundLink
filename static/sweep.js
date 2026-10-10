/* Sweep ◈ roomba mode.
 *
 * Self-contained on purpose: everything lives in this closure. The only hooks into app.js are
 * globals it already exposes (map, logEvent, previous, droneMarker, ...) and the single
 * window.sweepOnMessage function that app.js calls for "sweep*" WebSocket frames.
 *
 * The server does the geometry and the battery maths (src/sweep/). This file draws the area, shows
 * the pre-flight, and draws the lanes as red pellets that Neco eats while the sweep flies.
 */
(() => {
  'use strict';
  const $ = (id) => document.getElementById(id);
  const drawer = $('sweep-drawer');
  if (!drawer || typeof L === 'undefined') return;

  const RED = '#ff4a3d';
  const MAX_CORNERS = 64;
  const RESERVE_KEY = 'groundlink-sweep-reserve';
  const LEARNED_KEY = 'groundlink-sweep-learned';
  const FIELDS = ['capacity', 'remaining', 'amps', 'reserve'];

  const ui = {
    badge: $('sweep-badge'), draw: $('sweep-draw'), undo: $('sweep-undo'), fit: $('sweep-fit'), clear: $('sweep-clear'),
    hint: $('sweep-hint'), spacing: $('sweep-spacing'), angle: $('sweep-angle'), alt: $('sweep-alt'), speed: $('sweep-speed'),
    lanes: $('sweep-lanes'), warn: $('sweep-warn'), read: $('sweep-read'), autoRtl: $('sweep-autortl'),
    calc: $('sweep-calc'), notes: $('sweep-notes'), gate: $('sweep-gate'), forget: $('sweep-forget'), upload: $('sweep-upload'),
    progress: $('sweep-progress'), barFill: $('sweep-bar-fill'), progressText: $('sweep-progress-text'), result: $('sweep-result'),
    capacity: $('sweep-capacity'), remaining: $('sweep-remaining'), amps: $('sweep-amps'), reserve: $('sweep-reserve'),
  };
  const fieldBox = (name) => drawer.querySelector(`.sweep-field[data-field="${name}"]`);

  // ---------- state ----------
  let ready = false;
  let everOpened = false;
  let config = {};
  let vertices = [];            // L.LatLng corners of the area being drawn
  let vertexMarkers = [];
  let drawing = false;
  let preview = null;           // last good /api/sweep/preview response for the draft
  let loaded = null;            // the sweep the server is watching
  let progress = null;          // latest sweep_progress for `loaded`
  let view = 'draft';           // 'draft' shows the preview, 'loaded' shows the uploaded sweep
  let uploading = false;
  let vehicle = null;           // last /api/sweep/vehicle response
  let wasConnected = false;
  let geom = null;              // geometry of the route currently on the map
  let pellets = [];
  let nextPellet = 0;
  let lastLane = 0;
  let previewTimer = null;
  let previewSeq = 0;
  const auto = { capacity: null, remaining: null, amps: null, reserve: null };
  const overridden = { capacity: false, remaining: false, amps: false, reserve: false };
  let flight = { sum: 0, n: 0 };
  let learned = null;
  const pending = [];

  // ---------- map layers ----------
  const areaLayer = L.polygon([], { color: RED, weight: 2, opacity: .9, dashArray: '4 6', fillColor: RED, fillOpacity: .05, interactive: false });
  const trailLine = L.polyline([], { color: '#7f6a66', weight: 2, opacity: .55, dashArray: '2 6', interactive: false });
  const laneLine = L.polyline([], { color: RED, weight: 3, opacity: .95, interactive: false });
  const pelletLayer = L.layerGroup();
  function ensureOnMap() {
    for (const layer of [areaLayer, trailLine, laneLine, pelletLayer]) if (!map.hasLayer(layer)) layer.addTo(map);
  }

  // ---------- small helpers ----------
  const esc = (value) => escapeHtml(value);
  const num = (input) => (input.value.trim() === '' ? NaN : Number(input.value));
  const clamp01 = (x) => Math.min(1, Math.max(0, x));
  const round1 = (x) => Math.round(x * 10) / 10;
  const fmtDist = (m) => (m >= 1000 ? `${(m / 1000).toFixed(2)} km` : `${Math.round(m)} m`);
  const fmtMin = (s) => `${(s / 60).toFixed(1)} min`;
  const fmtMah = (v) => `${Math.round(v).toLocaleString('en-US')} mAh`;
  const post = (url, body) => fetch(url, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });

  function setResult(text, kind = '') {
    ui.result.className = `mission-result ${kind}`.trim();
    ui.result.textContent = text;
  }

  // ---------- route geometry (metres along the route, like the server's) ----------
  function buildGeom(path) {
    const pts = path.map((p) => L.latLng(p.lat, p.lon));
    const cum = [0];
    for (let i = 1; i < pts.length; i++) cum.push(cum[i - 1] + map.distance(pts[i - 1], pts[i]));
    return { pts, cum, total: cum[cum.length - 1] };
  }
  function pointAt(g, s) {
    const at = Math.min(g.total, Math.max(0, s));
    for (let i = 1; i < g.pts.length; i++) {
      if (at <= g.cum[i]) {
        const len = g.cum[i] - g.cum[i - 1];
        const t = len > 0 ? (at - g.cum[i - 1]) / len : 0;
        const a = g.pts[i - 1], b = g.pts[i];
        return L.latLng(a.lat + (b.lat - a.lat) * t, a.lng + (b.lng - a.lng) * t);
      }
    }
    return g.pts[g.pts.length - 1];
  }
  function slice(g, s0, s1) {
    if (s1 <= s0) return [];
    const out = [pointAt(g, s0)];
    for (let i = 0; i < g.pts.length; i++) if (g.cum[i] > s0 && g.cum[i] < s1) out.push(g.pts[i]);
    out.push(pointAt(g, s1));
    return out;
  }

  // ---------- pellets: the red lanes Neco eats ----------
  function clearPellets() { pelletLayer.clearLayers(); pellets = []; nextPellet = 0; }
  function layPellets(g) {
    clearPellets();
    if (!g || g.total <= 0) return;
    const step = Math.max(4, g.total / 400);
    for (let s = step / 2; s < g.total; s += step) {
      const marker = L.circleMarker(pointAt(g, s), { radius: 2.6, color: '#2a0b08', weight: 1, fillColor: RED, fillOpacity: 1, interactive: false });
      pelletLayer.addLayer(marker);
      pellets.push({ frac: s / g.total, marker });
    }
  }
  function showRoute(path) {
    geom = path && path.length > 1 ? buildGeom(path) : null;
    if (!geom) { laneLine.setLatLngs([]); trailLine.setLatLngs([]); clearPellets(); return; }
    ensureOnMap();
    laneLine.setLatLngs(geom.pts);
    trailLine.setLatLngs([]);
    layPellets(geom);
  }
  /** Eat everything up to `frac` of the route. Returns how many pellets went. */
  function eatTo(frac) {
    if (!geom) return 0;
    const s = frac * geom.total;
    trailLine.setLatLngs(slice(geom, 0, s));
    laneLine.setLatLngs(slice(geom, s, geom.total));
    let ate = 0;
    while (nextPellet < pellets.length && pellets[nextPellet].frac <= frac) {
      pelletLayer.removeLayer(pellets[nextPellet].marker);
      nextPellet++; ate++;
    }
    return ate;
  }
  function chomp() {
    const shell = droneMarker.getElement()?.querySelector('.neco-marker-shell');
    if (!shell) return;
    shell.classList.remove('chomp'); void shell.offsetWidth; shell.classList.add('chomp');
    setTimeout(() => shell.classList.remove('chomp'), 450);
  }
  function renderView() {
    if (view === 'loaded' && loaded) {
      showRoute(loaded.path);
      if (progress && progress.plan_id === loaded.id && progress.total_m > 0) eatTo(clamp01(progress.s_m / progress.total_m));
    } else {
      showRoute(preview ? preview.lanes.path : null);
    }
  }

  // ---------- drawing the area ----------
  const vertexIcon = (i) => L.divIcon({ className: '', html: `<div class="sweep-vertex">${i + 1}</div>`, iconSize: [22, 22], iconAnchor: [11, 11] });
  function rebuildMarkers() {
    vertexMarkers.forEach((marker) => map.removeLayer(marker));
    vertexMarkers = [];
    vertices.forEach((latlng, i) => {
      const marker = L.marker(latlng, { icon: vertexIcon(i), draggable: true, zIndexOffset: 700, keyboard: false }).addTo(map);
      marker.on('drag', () => { vertices[i] = marker.getLatLng(); areaLayer.setLatLngs(vertices); });
      marker.on('dragend', draftChanged);
      const remove = (event) => { if (event.originalEvent) L.DomEvent.stop(event.originalEvent); removeVertex(i); };
      marker.on('contextmenu', remove);
      marker.on('dblclick', remove);
      vertexMarkers.push(marker);
    });
    areaLayer.setLatLngs(vertices);
    ensureOnMap();
  }
  function removeVertex(i) { vertices.splice(i, 1); rebuildMarkers(); draftChanged(); }
  function setDrawing(on) {
    drawing = on;
    ui.draw.classList.toggle('active', on);
    ui.draw.textContent = on ? '✓ DRAWING ◈ CLICK TO STOP' : (vertices.length ? '＋ ADD CORNERS' : '＋ DRAW AREA');
    map.getContainer().style.cursor = on || plannerActive ? 'crosshair' : '';
    if (on && plannerActive) els.planToggle.click();
  }
  map.on('click', (event) => {
    if (!drawing) return;
    if (vertices.length >= MAX_CORNERS) { logEvent(`sweep areas are capped at ${MAX_CORNERS} corners`, 'warning'); return; }
    vertices.push(event.latlng);
    rebuildMarkers();
    draftChanged();
  });
  els.planToggle.addEventListener('click', () => { if (plannerActive && drawing) setDrawing(false); });
  ui.draw.addEventListener('click', () => setDrawing(!drawing));
  ui.undo.addEventListener('click', () => { if (vertices.length) { vertices.pop(); rebuildMarkers(); draftChanged(); } });
  ui.clear.addEventListener('click', () => { vertices = []; rebuildMarkers(); setDrawing(drawing); draftChanged(); logEvent('sweep area cleared'); });
  ui.fit.addEventListener('click', () => {
    const points = vertices.length ? vertices : (geom ? geom.pts : []);
    if (points.length) map.fitBounds(L.latLngBounds(points).pad(.2), { maxZoom: 19 });
  });

  // ---------- preview + pre-flight maths (done on the server) ----------
  function readSettings() {
    const spacing = num(ui.spacing), alt = num(ui.alt), speed = num(ui.speed);
    const angleText = ui.angle.value.trim();
    const angle = angleText === '' ? null : Number(angleText);
    if (![spacing, alt, speed].every(Number.isFinite) || (angle !== null && !Number.isFinite(angle))) {
      return { error: 'check lane spacing, angle, altitude and speed' };
    }
    const values = FIELDS.map((name) => num(ui[name]));
    const battery = values.every(Number.isFinite)
      ? { capacity_mah: values[0], remaining_pct: values[1], avg_current_a: values[2], reserve_pct: values[3] }
      : null;
    return {
      body: {
        polygon: vertices.map((v) => ({ lat: v.lat, lon: v.lng })),
        spacing_m: spacing, angle_deg: angle, alt_m: alt, speed_m_s: speed, battery, auto_rtl: ui.autoRtl.checked,
      },
    };
  }
  /** The area or lane settings changed: the draft replaces whatever is on the map. */
  function draftChanged() {
    view = 'draft';
    clearTimeout(previewTimer);
    previewTimer = setTimeout(runPreview, 280);
    updateBadge(); refreshGate();
  }
  /** Only the battery numbers changed: re-run the maths but leave the map alone. */
  function batteryChanged() {
    clearTimeout(previewTimer);
    previewTimer = setTimeout(runPreview, 280);
  }
  function setLaneText(text, bad = false) {
    ui.lanes.textContent = text;
    ui.lanes.style.color = bad ? 'var(--danger)' : '';
  }
  function showWarnings(list) {
    ui.warn.hidden = !list.length;
    ui.warn.textContent = list.map((w) => `◈ ${w}`).join('  ');
  }
  async function runPreview() {
    if (vertices.length < 3) {
      preview = null;
      setLaneText('no area yet ◈ draw at least 3 corners'); showWarnings([]);
      if (view === 'draft') renderView();
      renderCalc(); updateBadge(); refreshGate();
      return;
    }
    const settings = readSettings();
    if (settings.error) {
      preview = null; setLaneText(`✕ ${settings.error}`, true); showWarnings([]);
      if (view === 'draft') renderView();
      renderCalc(); updateBadge(); refreshGate();
      return;
    }
    const seq = ++previewSeq;
    try {
      const response = await post('/api/sweep/preview', settings.body);
      const data = await response.json();
      if (seq !== previewSeq) return;
      if (!response.ok || !data.ok) {
        preview = null; setLaneText(`✕ ${data.error || `HTTP ${response.status}`}`, true); showWarnings([]);
      } else {
        preview = data;
        const l = data.lanes;
        setLaneText(`${l.lane_count} lanes ◈ route ${fmtDist(l.path_m)} ◈ bearing ${String(Math.round(l.angle_deg) % 360).padStart(3, '0')}°${l.auto_angle ? ' (auto: fewest turns)' : ''} ◈ spacing ${l.effective_spacing_m.toFixed(1)} m ◈ ${(l.area_m2 / 10000).toFixed(2)} ha`);
        const warnings = [...l.warnings];
        if (!data.home_known) warnings.push('vehicle home is not known yet, so the trip to and from home is not in the battery estimate');
        showWarnings(warnings);
      }
    } catch (error) {
      if (seq !== previewSeq) return;
      preview = null; setLaneText(`✕ could not reach GroundLink: ${error.message}`, true);
    }
    if (view === 'draft') renderView();
    renderCalc(); updateBadge(); refreshGate();
  }
  function renderCalc() {
    const box = ui.calc;
    if (!preview) { box.innerHTML = ''; return; }
    const e = preview.estimate;
    if (!e) {
      box.innerHTML = `<div class="pending">${preview.battery_error ? `✕ ${esc(preview.battery_error)}` : 'fill in the battery boxes above to get a verdict'}</div>`;
      return;
    }
    const verdictText = { ok: 'OK ◈ IT FITS', tight: 'TIGHT ◈ IT FITS, BARELY', too_long: 'TOO LONG ◈ NOT WITH THIS BATTERY' }[e.verdict];
    const sign = e.margin_mah >= 0 ? '+' : '−';
    let advice = '';
    if (e.verdict === 'too_long') {
      advice = e.sorties == null
        ? 'the pack is already below your reserve: charge or swap it first.'
        : `this battery covers about ${Math.floor(e.coverage_pct)}% of the route (${e.sorties} batteries for all of it). Widen the lane spacing, shrink the area, or split it into two areas.`;
    } else if (e.verdict === 'tight') {
      advice = `that uses ${Math.round(e.used_pct_of_usable)}% of what is usable. A bit of wind or a cold pack and it will not make it.`;
    }
    box.innerHTML = `
      <div class="row"><span>route</span><strong>${fmtDist(e.total_m)} (lanes ${fmtDist(e.sweep_m)} + ${fmtDist(e.transit_m)} to and from home${preview.home_known ? '' : ', home unknown, not counted'})</strong></div>
      <div class="row"><span>time</span><strong>~${fmtMin(e.flight_s)} (${preview.lanes.turns} turns, climb and landing included)</strong></div>
      <div class="row"><span>battery needed</span><strong>${fmtMah(e.need_mah)} at ${esc(ui.amps.value)} A average</strong></div>
      <div class="row"><span>battery usable</span><strong>${fmtMah(Math.max(0, e.usable_mah))} (${fmtMah(e.available_mah)} left − ${fmtMah(e.reserve_mah)} reserve)</strong></div>
      <div class="row"><span>margin</span><strong>${sign}${fmtMah(Math.abs(e.margin_mah))}</strong></div>
      <div class="row verdict ${e.verdict}"><span>verdict</span><strong>${verdictText}</strong></div>
      ${advice ? `<span class="advice ${e.verdict === 'tight' ? 'tight' : ''}">◈ ${esc(advice)}</span>` : ''}`;
  }

  // ---------- upload gate ----------
  function gateReason() {
    if (uploading) return 'uploading…';
    if (!preview) return vertices.length < 3 ? 'draw an area (3+ corners)' : 'checking the area…';
    if (!vehicleConnected) return 'vehicle not connected';
    if (!homePosition) return 'waiting for vehicle home';
    if (currentSource !== 'demo' && previous?.armed) return 'disarm first';
    if (!preview.estimate) return 'fill in the battery numbers';
    if (preview.estimate.verdict === 'too_long') return 'too long for this battery';
    return '';
  }
  function refreshGate() {
    const reason = gateReason();
    ui.upload.disabled = !!reason;
    ui.gate.textContent = reason ? `◈ ${reason}` : '◈ ready ◈ upload does not arm';
  }

  async function launch() {
    if (gateReason()) return;
    const settings = readSettings();
    if (settings.error) { setResult(`✕ ${settings.error}`, 'error'); return; }
    uploading = true; refreshGate();
    setResult(currentSource === 'demo' ? 'feeding the sweep to the demo goblin...' : 'negotiating the sweep with ArduPilot...');
    try {
      const response = await post('/api/sweep/upload', settings.body);
      const data = await response.json();
      if (!response.ok || !data.ok) throw new Error(data.error || `HTTP ${response.status}`);
      loaded = data.plan; progress = null; lastLane = 0; view = 'loaded';
      renderView(); renderProgress(); updateBadge();
      const e = loaded.estimate;
      logEvent(`roomba time ◈ ${loaded.lane_count} lanes of snacks ◈ ${fmtDist(e.total_m)} ◈ ~${fmtMin(e.flight_s)} ◈ ${fmtMah(e.need_mah)} of ${fmtMah(Math.max(0, e.usable_mah))} usable`, 'nominal');
      setResult(`✓ ${data.message}`, 'success');
      els.necoLine.textContent = currentSource === 'demo' ? 'sweep acquired. neco is hungry.' : 'sweep loaded. arm and select AUTO when ready.';
    } catch (error) {
      setResult(`✕ ${error.message}`, 'error');
      logEvent(`sweep upload failed ◈ ${error.message}`, 'danger');
    } finally {
      uploading = false; refreshGate();
    }
  }
  ui.upload.addEventListener('click', launch);
  ui.forget.addEventListener('click', async () => {
    try { await post('/api/sweep/clear', {}); } catch (error) { logEvent(`could not clear the sweep ◈ ${error.message}`, 'danger'); return; }
    logEvent('sweep forgotten by GroundLink ◈ the vehicle keeps its mission until the next upload');
    setResult('sweep forgotten ◈ the vehicle keeps its mission until the next upload');
  });
  ui.autoRtl.addEventListener('change', batteryChanged);

  // ---------- progress ----------
  function renderProgress() {
    ui.forget.hidden = !loaded;
    if (!loaded) { ui.progress.hidden = true; return; }
    ui.progress.hidden = false;
    const p = progress;
    if (!p) {
      ui.barFill.style.width = '0%';
      ui.progressText.textContent = 'scout loaded ◈ waiting for you to arm and select AUTO';
      return;
    }
    const pct = p.total_m > 0 ? Math.round(clamp01(p.s_m / p.total_m) * 100) : 0;
    ui.barFill.style.width = `${pct}%`;
    const battery = p.need_mah != null && p.usable_mah != null
      ? ` ◈ finishing and getting home needs ${fmtMah(p.need_mah)} (x1.2) of ${fmtMah(Math.max(0, p.usable_mah))} usable` : '';
    ui.progressText.textContent = {
      transit: 'flying to the first lane ◈ neco starts eating when it gets there',
      sweeping: `neco is eating ◈ lane ${Math.min(p.lane + 1, p.lanes)}/${p.lanes} ◈ ${pct}%${battery}`,
      done: `sweep complete ◈ ${p.lane}/${p.lanes} lanes eaten ◈ neco is full`,
      rtl: `interrupted ◈ heading home with ${p.lane}/${p.lanes} lanes eaten${p.rtl_commanded ? ' ◈ GroundLink commanded RTL because the battery could not finish' : ''}`,
    }[p.phase] || '';
  }
  function updateBadge() {
    let text = 'NO AREA', active = false;
    if (loaded && view === 'loaded') {
      active = true;
      const p = progress;
      const pct = p && p.total_m > 0 ? Math.round(clamp01(p.s_m / p.total_m) * 100) : 0;
      text = !p ? 'LOADED' : { transit: 'LOADED', sweeping: `EATING ${pct}%`, done: 'DONE', rtl: 'RTL' }[p.phase] || 'LOADED';
    } else if (preview) {
      text = `${preview.lanes.lane_count} LANES${preview.estimate ? ` ◈ ${preview.estimate.verdict.replace('_', ' ').toUpperCase()}` : ''}`;
    } else if (vertices.length) {
      text = `${vertices.length} CORNER${vertices.length === 1 ? '' : 'S'}`;
    }
    ui.badge.textContent = text;
    ui.badge.classList.toggle('active', active);
  }

  window.sweepOnMessage = (message) => {
    if (!ready) { pending.push(message); return; }
    if (message.type === 'sweep') {
      if (message.plan) {
        if (!loaded || loaded.id !== message.plan.id) {
          loaded = message.plan; progress = null; lastLane = 0;
          if (view === 'loaded' || !vertices.length) { view = 'loaded'; renderView(); }
        }
      } else {
        loaded = null; progress = null; lastLane = 0;
        if (view === 'loaded') { view = 'draft'; renderView(); }
      }
      renderProgress(); updateBadge(); refreshGate();
    } else if (message.type === 'sweep_progress') {
      const p = message.progress;
      if (!loaded || p.plan_id !== loaded.id) return;
      progress = p;
      if (view === 'loaded' && p.total_m > 0 && eatTo(clamp01(p.s_m / p.total_m)) > 0) chomp();
      if (p.lane > lastLane) {
        lastLane = p.lane;
        logEvent(`yum ◈ lane ${p.lane}/${p.lanes} eaten`, 'nominal');
      }
      renderProgress(); updateBadge();
    } else if (message.type === 'sweep_note') {
      logEvent(message.message, ['warning', 'danger', 'nominal'].includes(message.level) ? message.level : '');
    }
  };

  // ---------- pre-flight fields: fill what we can read, let the pilot override ----------
  function sameAuto(a, b) { return (!a && !b) || (a && b && a.value === b.value && a.source === b.source); }
  function writeField(name) { ui[name].value = auto[name] ? String(auto[name].value) : ''; }
  function paintField(name) {
    const box = fieldBox(name), tag = box.querySelector('.sweep-source');
    const mine = overridden[name], have = !!auto[name];
    box.classList.toggle('override', mine);
    box.classList.toggle('missing', !mine && !have);
    tag.textContent = mine ? 'YOURS ↺' : have ? auto[name].source : 'ENTER IT';
    tag.title = mine ? 'Click to use the automatic value again' : have ? `Filled in automatically: ${auto[name].source.toLowerCase()}` : 'GroundLink could not read this: type it in';
  }
  function setAuto(name, value, source) {
    const next = value == null || !Number.isFinite(value) ? null : { value, source };
    if (sameAuto(auto[name], next)) return;
    auto[name] = next;
    if (!overridden[name]) { writeField(name); batteryChanged(); }
    paintField(name);
  }
  for (const name of FIELDS) {
    ui[name].addEventListener('input', () => {
      overridden[name] = ui[name].value.trim() !== '';
      if (name === 'reserve' && overridden[name]) { try { localStorage.setItem(RESERVE_KEY, ui[name].value.trim()); } catch { /* storage is optional */ } }
      paintField(name); batteryChanged(); refreshGate();
    });
    ui[name].addEventListener('change', () => {
      // Leaving a box empty means "go back to automatic".
      if (ui[name].value.trim() === '') { overridden[name] = false; writeField(name); paintField(name); batteryChanged(); }
    });
    fieldBox(name).querySelector('.sweep-source').addEventListener('click', () => {
      if (!overridden[name]) return;
      overridden[name] = false; writeField(name); paintField(name); batteryChanged();
    });
  }
  for (const input of [ui.spacing, ui.angle, ui.alt, ui.speed]) input.addEventListener('input', draftChanged);

  /** Average battery current while airborne, remembered so the next pre-flight can use it. */
  function trackFlight() {
    const t = previous;
    const airborne = !!(t && t.armed && t.alt_m > 3 && t.current_a > 2 && currentSource !== 'demo');
    if (airborne) {
      flight.sum += t.current_a; flight.n += 1;
      if (flight.n % 10 === 0) persistLearned();
    } else if (flight.n > 0 && !(t && t.armed)) {
      persistLearned(); flight = { sum: 0, n: 0 };
    }
  }
  function persistLearned() {
    if (flight.n < 20) return;
    learned = { amps: flight.sum / flight.n, seconds: flight.n, at: Date.now() };
    try { localStorage.setItem(LEARNED_KEY, JSON.stringify(learned)); } catch { /* storage is optional */ }
  }
  function tickAuto() {
    const t = previous;
    setAuto('remaining', t && t.battery_pct != null ? t.battery_pct : null, 'LIVE');
    let amps = null, source = '';
    if (currentSource === 'demo' && vehicle && vehicle.typical_current_a) { amps = vehicle.typical_current_a; source = 'DEMO'; }
    else if (flight.n >= 10) { amps = round1(flight.sum / flight.n); source = 'THIS FLIGHT'; }
    else if (learned && learned.amps > 1) { amps = round1(learned.amps); source = 'LAST FLIGHT'; }
    setAuto('amps', amps, source);
  }
  function renderNotes(notes) {
    ui.notes.innerHTML = (notes || []).map((note) => `<li>${esc(note)}</li>`).join('');
  }
  async function readVehicle(announce = false) {
    ui.read.disabled = true; ui.read.textContent = 'READING…';
    try {
      const response = await fetch('/api/sweep/vehicle');
      vehicle = await response.json();
      setAuto('capacity', vehicle.capacity_mah, vehicle.capacity_source === 'demo' ? 'DEMO' : 'VEHICLE');
      renderNotes(vehicle.notes);
      tickAuto();
      if (announce) logEvent(vehicle.capacity_mah ? `sweep read the battery ◈ ${fmtMah(vehicle.capacity_mah)} (${vehicle.capacity_source})` : 'sweep could not read a battery capacity ◈ enter it by hand', vehicle.capacity_mah ? 'nominal' : 'warning');
    } catch (error) {
      renderNotes([`could not read the vehicle: ${error.message}`]);
    } finally {
      ui.read.disabled = false; ui.read.textContent = 'READ VEHICLE';
    }
  }
  ui.read.addEventListener('click', () => readVehicle(true));

  // ---------- start-up ----------
  function start() {
    try { learned = JSON.parse(localStorage.getItem(LEARNED_KEY) || 'null'); } catch { learned = null; }
    let savedReserve = null;
    try { savedReserve = Number(localStorage.getItem(RESERVE_KEY)); } catch { /* storage is optional */ }
    const reserve = Number.isFinite(savedReserve) && savedReserve > 0 ? savedReserve : (config.default_reserve_pct ?? 30);
    auto.reserve = { value: reserve, source: 'DEFAULT' };
    writeField('reserve');
    for (const name of FIELDS) paintField(name);
    tickAuto();
    drawer.addEventListener('toggle', () => {
      if (!drawer.open) return;
      if (!everOpened) { everOpened = true; readVehicle(); }
      refreshGate();
    });
    setInterval(() => {
      trackFlight(); tickAuto(); refreshGate();
      if (vehicleConnected !== wasConnected) { wasConnected = vehicleConnected; if (vehicleConnected && everOpened) readVehicle(); }
    }, 1000);
    ready = true;
    drawer.hidden = false;
    pending.splice(0).forEach(window.sweepOnMessage);
    updateBadge(); renderProgress(); refreshGate();
  }

  fetch('/api/sweep/config')
    .then((response) => response.json())
    .then((data) => { config = data; if (data.enabled) start(); })
    .catch(() => { /* no Sweep on this server: the drawer stays hidden */ });
})();
