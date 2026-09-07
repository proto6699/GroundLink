const $ = (id) => document.getElementById(id);
const els = {
  themeButton: $('theme-button'), luckButton: $('luck-button'), statusDot: $('status-dot'), topStatus: $('top-status'),
  footerDot: $('footer-dot'), footerStatus: $('footer-status'), packetTime: $('packet-time'), altitude: $('altitude'),
  battery: $('battery'), voltage: $('voltage'), gps: $('gps'), armed: $('armed'), mode: $('mode'), coords: $('coords'),
  mapComment: $('map-comment'), eventLog: $('event-log'), clearLog: $('clear-log'), necoLine: $('neco-line'),
  petButton: $('pet-button'), panicButton: $('panic-button'), petCount: $('pet-count'), necoCard: $('neco-card'),
  horizonWorld: $('horizon-world'), rollValue: $('roll-value'), pitchValue: $('pitch-value'), yawValue: $('yaw-value'), headingTag: $('heading-tag'),
  planToggle: $('plan-toggle'), sampleRoute: $('sample-route'), fitRoute: $('fit-route'), plannerHint: $('planner-hint'),
  waypointList: $('waypoint-list'), missionCount: $('mission-count'), missionStamp: $('mission-stamp'), defaultAlt: $('default-alt'),
  clearMission: $('clear-mission'), uploadMission: $('upload-mission'), missionResult: $('mission-result'),
  luckModal: $('luck-modal'), luckImage: $('luck-image'), luckTitle: $('luck-title'), luckMessage: $('luck-message'), luckRoll: $('luck-roll'),
};

let previous = null;
let hasCentered = false;
let reconnectTimer = null;
let pets = 0;
let plannerActive = false;
let waypoints = [];
let missionMarkers = [];
let currentSource = 'sitl';
let latestPosition = null;

const map = L.map('map', { zoomControl: true }).setView([24.7136, 46.6753], 15);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
  maxZoom: 20,
  attribution: '&copy; OpenStreetMap contributors',
}).addTo(map);

const necoIcon = L.divIcon({
  className: 'neco-marker-wrap',
  html: '<div class="neco-marker-shell"><img src="/neco-drone.png" alt=""></div>',
  iconSize: [62, 72], iconAnchor: [31, 66],
});
const droneMarker = L.marker([24.7136, 46.6753], { icon: necoIcon, zIndexOffset: 1000 }).addTo(map);
const missionLine = L.polyline([], { color: '#d8e86c', weight: 3, opacity: .9, dashArray: '8 6' }).addTo(map);

const necoQuips = [
  'burunyuu... packets acquired.', 'GPS says the drone exists. promising.', 'rust thread status: emotionally asynchronous.',
  'MAVLink has entered the cat dimension.', 'battery still contains electricity. incredible.', 'telemetry delicious. no notes.',
  'all systems nominal-ish.', 'waypoints are just GPS coordinates with ambition.',
];
const mapQuips = ['the little guy has been located', 'yes that cat is allegedly your aircraft', 'coordinates successfully bullied out of GPS', 'satellites are gossiping again'];
const randomFrom = (items) => items[Math.floor(Math.random() * items.length)];

function escapeHtml(value) {
  return String(value).replaceAll('&','&amp;').replaceAll('<','&lt;').replaceAll('>','&gt;').replaceAll('"','&quot;').replaceAll("'",'&#039;');
}
function clamp(value, min, max) { return Math.min(max, Math.max(min, value)); }
function formatCoord(value) { return Number(value).toFixed(6); }

function setTheme(theme) {
  document.documentElement.dataset.theme = theme;
  localStorage.setItem('groundlink-theme', theme);
  els.themeButton.textContent = theme === 'dark' ? '☼ LIGHT' : '☾ DARK';
  document.querySelector('meta[name="theme-color"]').setAttribute('content', theme === 'dark' ? '#17130f' : '#f0e5d2');
}
const savedTheme = localStorage.getItem('groundlink-theme');
setTheme(savedTheme || (matchMedia('(prefers-color-scheme: light)').matches ? 'light' : 'dark'));
els.themeButton.addEventListener('click', () => setTheme(document.documentElement.dataset.theme === 'dark' ? 'light' : 'dark'));

function setLinkStatus(connected, source = 'sitl') {
  currentSource = source;
  for (const dot of [els.statusDot, els.footerDot]) {
    dot.classList.toggle('online', connected); dot.classList.toggle('offline', !connected);
  }
  const label = source === 'demo' ? 'DEMO' : 'SITL';
  els.topStatus.textContent = connected ? `${label} LIVE` : `${label} OFFLINE`;
  els.footerStatus.textContent = connected ? `${label} source connected ◈ we have gossip` : `${label} source disconnected ◈ suspicious silence`;
  if (connected) els.necoLine.textContent = source === 'demo' ? 'demo drone detected. draw a route and make me chase it.' : 'real MAVLink detected. mission desk is pretending to be professional.';
}

function logEvent(message, level = '') {
  const row = document.createElement('div');
  row.className = `log-row ${level}`.trim();
  row.innerHTML = `<span class="time">${new Date().toLocaleTimeString([], {hour12:false})}</span><span class="msg">◈ ${escapeHtml(message)}</span>`;
  els.eventLog.prepend(row);
  while (els.eventLog.children.length > 90) els.eventLog.lastElementChild.remove();
}

function renderAttitude(t) {
  const roll = Number.isFinite(t.roll_deg) ? clamp(t.roll_deg, -89, 89) : 0;
  const pitch = Number.isFinite(t.pitch_deg) ? clamp(t.pitch_deg, -30, 30) : 0;
  const yaw = Number.isFinite(t.yaw_deg) ? ((t.yaw_deg % 360) + 360) % 360 : 0;
  // Nose-up => horizon moves down. Right roll => horizon rotates left.
  els.horizonWorld.style.transform = `translateY(${pitch * 4}px) rotate(${-roll}deg)`;
  els.rollValue.textContent = `${roll.toFixed(1)}°`;
  els.pitchValue.textContent = `${pitch.toFixed(1)}°`;
  els.yawValue.textContent = `${yaw.toFixed(1)}°`;
  els.headingTag.textContent = `HDG ${String(Math.round(yaw)).padStart(3,'0')}°`;
}

function renderTelemetry(t) {
  els.altitude.textContent = Number(t.alt_m).toFixed(1);
  els.battery.textContent = t.battery_pct == null ? '--%' : `${t.battery_pct}%`;
  els.voltage.textContent = t.voltage_mv == null ? '-- V / spicy electrons' : `${(t.voltage_mv/1000).toFixed(2)} V / spicy electrons`;
  els.gps.textContent = t.gps_sats ?? '--';
  els.armed.textContent = t.armed ? 'ARMED' : 'DISARMED';
  els.mode.textContent = `MODE ${t.flight_mode}`;
  els.packetTime.textContent = `FRAME ${new Date(t.timestamp_ms).toLocaleTimeString([], {hour12:false})}`;
  renderAttitude(t);

  const batteryCard = els.battery.closest('.stat-card');
  batteryCard.classList.toggle('warning', t.battery_pct != null && t.battery_pct <= 25);
  batteryCard.classList.toggle('nominal', t.battery_pct != null && t.battery_pct > 25);
  els.armed.closest('.stat-card').classList.toggle('nominal', t.armed);

  if (Number.isFinite(t.lat) && Number.isFinite(t.lon) && !(t.lat === 0 && t.lon === 0)) {
    latestPosition = [t.lat, t.lon];
    droneMarker.setLatLng(latestPosition);
    els.coords.textContent = `${t.lat.toFixed(5)}, ${t.lon.toFixed(5)}`;
    if (!hasCentered) { map.setView(latestPosition, 18); els.mapComment.textContent = randomFrom(mapQuips); hasCentered = true; }
  }

  if (previous) {
    if (previous.armed !== t.armed) logEvent(t.armed ? 'vehicle armed ◈ tiny violence enabled' : 'vehicle disarmed ◈ paws off', t.armed ? 'nominal' : '');
    if (previous.flight_mode !== t.flight_mode) logEvent(`mode ${previous.flight_mode} → ${t.flight_mode}`, 'nominal');
    if ((previous.battery_pct ?? 101) > 25 && (t.battery_pct ?? 101) <= 25) logEvent(`low battery ${t.battery_pct}% ◈ electrons escaping`, 'warning');
    if ((previous.gps_sats ?? 99) >= 6 && (t.gps_sats ?? 99) < 6) logEvent(`GPS degraded: ${t.gps_sats ?? 0} satellites ◈ sky friends left`, 'warning');
  } else {
    logEvent('first telemetry frame received ◈ it lives', 'nominal');
  }
  if (Math.random() < .012) els.necoLine.textContent = randomFrom(necoQuips);
  previous = t;
}

function waypointIcon(index) {
  return L.divIcon({ className:'', html:`<div class="waypoint-map-marker">${index+1}</div>`, iconSize:[29,29], iconAnchor:[14,14] });
}

function renderMission() {
  missionMarkers.forEach((marker) => map.removeLayer(marker));
  missionMarkers = [];
  const latlngs = [];
  waypoints.forEach((wp, index) => {
    const marker = L.marker([wp.lat, wp.lon], {icon: waypointIcon(index), zIndexOffset: 500}).addTo(map);
    marker.bindTooltip(`WP ${index+1} ◈ ${wp.alt_m.toFixed(0)} m`, {direction:'top'});
    missionMarkers.push(marker); latlngs.push([wp.lat, wp.lon]);
  });
  missionLine.setLatLngs(latlngs);
  els.missionCount.textContent = `${waypoints.length} WAYPOINT${waypoints.length === 1 ? '' : 'S'}`;
  els.missionStamp.textContent = waypoints.length ? `${waypoints.length} LITTLE DESTINATION${waypoints.length === 1 ? '' : 'S'}` : 'NO CRIMES PLANNED';
  els.uploadMission.disabled = waypoints.length === 0;

  if (!waypoints.length) {
    els.waypointList.innerHTML = '<div class="empty-mission">no waypoints yet. activate DROP WAYPOINTS and click the map.</div>';
    return;
  }
  els.waypointList.innerHTML = waypoints.map((wp,index) => `
    <div class="waypoint-row" data-index="${index}">
      <span class="wp-number">${index+1}</span>
      <div class="wp-coords"><strong>${formatCoord(wp.lat)}, ${formatCoord(wp.lon)}</strong><span>WP ${String(index+1).padStart(2,'0')} / NAV_WAYPOINT</span></div>
      <label class="wp-alt"><input type="number" min="1" max="500" step="1" value="${Number(wp.alt_m).toFixed(0)}" data-alt="${index}"> m</label>
      <button class="wp-delete" type="button" data-delete="${index}" aria-label="Delete waypoint ${index+1}">×</button>
    </div>`).join('');

  els.waypointList.querySelectorAll('[data-alt]').forEach((input) => input.addEventListener('change', () => {
    const i = Number(input.dataset.alt); const value = clamp(Number(input.value) || 25, 1, 500); waypoints[i].alt_m = value; input.value = Math.round(value); renderMission();
  }));
  els.waypointList.querySelectorAll('[data-delete]').forEach((button) => button.addEventListener('click', () => {
    const i = Number(button.dataset.delete); waypoints.splice(i,1); renderMission(); logEvent(`waypoint ${i+1} deleted ◈ erased from destiny`);
  }));
}

function addWaypoint(lat, lon) {
  const alt = clamp(Number(els.defaultAlt.value) || 25, 1, 500);
  waypoints.push({lat, lon, alt_m:alt}); renderMission();
  logEvent(`waypoint ${waypoints.length} dropped at ${lat.toFixed(5)}, ${lon.toFixed(5)} ◈ ${alt} m`, 'nominal');
}

map.on('click', (event) => {
  if (!plannerActive) return;
  addWaypoint(event.latlng.lat, event.latlng.lng);
});

els.planToggle.addEventListener('click', () => {
  plannerActive = !plannerActive; els.planToggle.classList.toggle('active', plannerActive);
  els.planToggle.textContent = plannerActive ? '✓ DROPPING WAYPOINTS' : '＋ DROP WAYPOINTS';
  els.plannerHint.textContent = plannerActive ? 'planner awake ◈ click map to add points' : 'planner asleep ◈ map clicks just map';
  map.getContainer().style.cursor = plannerActive ? 'crosshair' : '';
});

els.sampleRoute.addEventListener('click', () => {
  const [lat, lon] = latestPosition || [24.7136,46.6753];
  waypoints = [
    {lat:lat+0.00055, lon:lon+0.00015, alt_m:25},
    {lat:lat+0.00010, lon:lon+0.00070, alt_m:32},
    {lat:lat-0.00045, lon:lon+0.00005, alt_m:22},
    {lat:lat+0.00055, lon:lon+0.00015, alt_m:25},
  ];
  renderMission(); fitMission(); logEvent('silly triangle loaded ◈ geometry has entered the chat', 'nominal');
});
function fitMission() {
  const points = waypoints.map(wp => [wp.lat,wp.lon]);
  if (latestPosition) points.push(latestPosition);
  if (points.length) map.fitBounds(L.latLngBounds(points).pad(.18), {maxZoom:18});
}
els.fitRoute.addEventListener('click', fitMission);
els.clearMission.addEventListener('click', () => { waypoints=[]; renderMission(); els.missionResult.className='mission-result'; els.missionResult.textContent='mission cleared locally ◈ vehicle mission unchanged until next upload'; logEvent('local waypoint plan cleared'); });

els.uploadMission.addEventListener('click', async () => {
  if (!waypoints.length) return;
  els.uploadMission.disabled = true; els.uploadMission.textContent = 'UPLOADING...';
  els.missionResult.className = 'mission-result'; els.missionResult.textContent = currentSource === 'demo' ? 'feeding route to demo goblin...' : 'negotiating MAVLink mission protocol with ArduPilot...';
  try {
    const response = await fetch('/api/mission', { method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({waypoints}) });
    const result = await response.json();
    if (!response.ok || !result.ok) throw new Error(result.error || `HTTP ${response.status}`);
    els.missionResult.className='mission-result success'; els.missionResult.textContent=`✓ ${result.message}`;
    logEvent(`mission upload accepted ◈ ${result.count} waypoint(s)`, 'nominal');
    els.necoLine.textContent = currentSource === 'demo' ? 'route acquired. deploying cat toward coordinates.' : 'ArduPilot accepted the mission. do not forget: upload is not arm.';
  } catch (error) {
    els.missionResult.className='mission-result error'; els.missionResult.textContent=`✕ ${error.message}`;
    logEvent(`mission upload failed ◈ ${error.message}`, 'danger');
  } finally {
    els.uploadMission.disabled = waypoints.length === 0; els.uploadMission.textContent='UPLOAD TO DRONE ◈';
  }
});

function connectWebSocket() {
  const protocol = location.protocol === 'https:' ? 'wss' : 'ws';
  const ws = new WebSocket(`${protocol}://${location.host}/ws`);
  ws.addEventListener('open', () => logEvent('browser telemetry socket connected ◈ hello websocket','nominal'));
  ws.addEventListener('message', (event) => {
    try {
      const message = JSON.parse(event.data);
      if (message.type === 'status') {
        setLinkStatus(message.connected, message.source);
        logEvent(message.connected ? `${message.source} source acquired ◈ tasty packets` : `${message.source} source lost ◈ where drone`, message.connected ? 'nominal' : 'warning');
      } else if (message.type === 'telemetry') renderTelemetry(message.data);
      else if (message.type === 'mission') { waypoints = message.waypoints.map(wp => ({...wp})); renderMission(); }
    } catch (error) { console.error(error); logEvent('frame parse error ◈ JSON committed a crime','danger'); }
  });
  ws.addEventListener('close', () => { setLinkStatus(false,currentSource); logEvent('websocket disconnected; retrying because giving up is cringe','warning'); clearTimeout(reconnectTimer); reconnectTimer=setTimeout(connectWebSocket,1500); });
  ws.addEventListener('error', () => ws.close());
}

els.clearLog.addEventListener('click', () => { els.eventLog.innerHTML=''; logEvent('event log yeeted into the void'); });

const luckImages=['/neco-drone.png','https://www.pngall.com/wp-content/uploads/14/Neco-Arc-PNG-File.png','https://i.kym-cdn.com/photos/images/newsfeed/002/633/798/9a0.png'];
const luckyLines=['congrats. absolutely nothing exploded. rare GroundLink W.','the cat inspected your MAVLink packets and found them crunchy.','you won +3 imaginary GPS satellites. redeem nowhere.','flight controller spared. for now.','burunyuu says your async runtime has good aura.','you have been granted one (1) legally questionable loiter.'];
function openLuckModal(){els.luckModal.classList.add('open');els.luckModal.setAttribute('aria-hidden','false');}
function closeLuckModal(){els.luckModal.classList.remove('open');els.luckModal.setAttribute('aria-hidden','true');els.luckModal.querySelector('.luck-window').classList.remove('shutdown');}
document.querySelectorAll('[data-close-luck]').forEach(el=>el.addEventListener('click',closeLuckModal));
els.luckButton.addEventListener('click', async()=>{
  els.luckButton.disabled=true; els.luckTitle.textContent='consulting the cat...'; els.luckMessage.textContent='rolling a completely trustworthy 1-in-10 backend dice.'; els.luckImage.src=randomFrom(luckImages); els.luckRoll.textContent='ROLLING...'; openLuckModal();
  try { const response=await fetch('/luck',{method:'POST'}); const result=await response.json(); els.luckRoll.textContent=`ROLL ${result.roll} / 10`; if(result.shutdown){els.luckImage.src='/neco-drone.png';els.luckTitle.textContent='opsi...';els.luckMessage.textContent='i touched cargo. GroundLink is going bye-bye now. ◈';els.luckModal.querySelector('.luck-window').classList.add('shutdown');logEvent('TRY YOUR LUCK rolled cursed 1/10 ◈ shutdown incoming','danger');} else {els.luckTitle.textContent='burunyuu! you live.';els.luckMessage.textContent=randomFrom(luckyLines);logEvent(`luck roll ${result.roll}/10 ◈ cargo survives`,'nominal');}}
  catch {els.luckTitle.textContent='the cat ate the request';els.luckMessage.textContent='backend did not answer. maybe you already won the cursed roll.';els.luckRoll.textContent='ROLL ???';}
  finally {els.luckButton.disabled=false;}
});

els.panicButton.addEventListener('click',()=>{document.body.classList.remove('panik');void document.body.offsetWidth;document.body.classList.add('panik');els.necoCard.classList.add('bonk');logEvent('PANIK manually requested ◈ panik deployed','danger');els.necoLine.textContent='AAAAAAAAAAAAAAAA MAVLINK';setTimeout(()=>{document.body.classList.remove('panik');els.necoCard.classList.remove('bonk');els.necoLine.textContent='okay false alarm. probably.';},1100);});
els.petButton.addEventListener('click',()=>{pets++;els.petCount.textContent=`pets: ${pets}`;els.necoCard.classList.remove('bonk');void els.necoCard.offsetWidth;els.necoCard.classList.add('bonk');els.necoLine.textContent=randomFrom(['burunyuu +1','avionics morale increased by 0.3%','crew member appeased','pet registered in volatile memory','flight safety unchanged, vibes improved']);setTimeout(()=>els.necoCard.classList.remove('bonk'),750);});

setLinkStatus(false,'sitl'); renderMission(); logEvent('GroundLink v0.2 initialized ◈ waypoint crimes enabled'); connectWebSocket();
