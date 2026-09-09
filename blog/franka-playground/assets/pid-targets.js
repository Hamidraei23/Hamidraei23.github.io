// Plate coordinates are meters. Parameterize drawings by distance, independent
// of mouse speed, and ease the target to rest at both ends of the timed path.
const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
export function makePath(points) {
  const clean = [], distances = [];
  let length = 0;
  for (const p of points) {
    const point = { x: clamp(p.x, -.1, .1), y: clamp(p.y, -.1, .1) };
    const previous = clean.at(-1);
    const distance = previous ? Math.hypot(point.x - previous.x, point.y - previous.y) : 0;
    if (previous && distance < 1e-6) continue;
    length += distance; clean.push(point); distances.push(length);
  }
  return { points: clean, distances, length };
}
export function samplePath(path, elapsed, duration) {
  if (!path.points.length) return { x: 0, y: 0, vx: 0, vy: 0 };
  if (path.points.length === 1 || elapsed <= 0) return { ...path.points[0], vx: 0, vy: 0 };
  if (elapsed >= duration) return { ...path.points.at(-1), vx: 0, vy: 0 };
  const t = elapsed / duration;
  const distance = path.length * t**3 * (10 + t * (-15 + 6*t));
  const speed = path.length * 30*t*t*(1-t)**2 / duration;
  let lo = 1, hi = path.points.length - 1;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (path.distances[mid] < distance) lo = mid + 1; else hi = mid;
  }
  const a = path.points[lo - 1], b = path.points[lo];
  const segment = path.distances[lo] - path.distances[lo - 1];
  const fraction = (distance - path.distances[lo - 1]) / segment;
  return { x: a.x + (b.x-a.x)*fraction, y: a.y + (b.y-a.y)*fraction,
    vx: (b.x-a.x)/segment*speed, vy: (b.y-a.y)/segment*speed };
}

export function setupPIDTargets({ onChange, onPathChange }) {
  const el = id => document.getElementById(id), map = el('pid-map');
  let point = { x: 0, y: 0 }, path = makePath([]), duration = 10, drawing = null;
  const mode = () => el('pid-target-mode').value;
  function help(text, error = false) {
    el('pid-target-help').textContent = text;
    el('pid-target-help').dataset.error = String(error);
  }
  function drawPath(points = path.points) {
    el('pid-map-path').setAttribute('points', points.map(p => `${p.x*1000},${-p.y*1000}`).join(' '));
  }
  function syncPoint() {
    el('pid-target-x').value = (point.x*1000).toFixed(1);
    el('pid-target-y').value = (point.y*1000).toFixed(1);
    for (const axis of ['x', 'y']) el(`pid-target-${axis}`).setCustomValidity('');
  }
  function setPoint(next) {
    point = next; syncPoint();
    help('Click the plate or enter X/Y. Arrow keys move the target by 1 mm; Shift moves by 10 mm.');
    onChange({ pause: false });
  }
  function validate() {
    if (mode() === 'point') {
      for (const axis of ['x', 'y']) {
        const input = el(`pid-target-${axis}`), value = input.valueAsNumber;
        if (!Number.isFinite(value) || value < -100 || value > 100) {
          input.setCustomValidity('Enter a coordinate from −100 to 100 mm.');
          help('Enter a coordinate from −100 to 100 mm for both axes.', true); return false;
        }
        input.setCustomValidity('');
      }
      return true;
    }
    const value = el('pid-path-duration').valueAsNumber;
    if (!Number.isFinite(value) || value < .5 || value > 120) {
      help('Enter a trajectory duration from 0.5 to 120 seconds.', true); return false;
    }
    if (drawing || path.points.length < 2) {
      help('Draw a path on the plate before starting the trajectory.', true); return false;
    }
    duration = value; return true;
  }
  function readPoint() {
    if (!validate()) { onChange({ pause: true }); return; }
    setPoint({ x: el('pid-target-x').valueAsNumber/1000, y: el('pid-target-y').valueAsNumber/1000 });
  }
  for (const axis of ['x', 'y']) {
    el(`pid-target-${axis}`).addEventListener('change', readPoint);
    el(`pid-target-${axis}`).addEventListener('keydown', event => {
      if (event.key === 'Enter') { event.preventDefault(); readPoint(); }
    });
  }
  el('pid-target-center').addEventListener('click', () => setPoint({ x: 0, y: 0 }));
  function cancelDrawing() {
    if (!drawing) return;
    const id = drawing.id; drawing = null;
    if (map.hasPointerCapture(id)) map.releasePointerCapture(id);
    drawPath();
  }
  el('pid-target-mode').addEventListener('change', () => {
    cancelDrawing();
    const isPath = mode() === 'path';
    el('pid-point-controls').hidden = isPath;
    el('pid-path-controls').hidden = !isPath;
    el('pid-map-path').hidden = !isPath;
    // SVG does not consistently implement the HTML hidden property.
    el('pid-map-path').style.display = isPath ? '' : 'none';
    el('pid-target-time').hidden = !isPath;
    map.setAttribute('aria-label', isPath ? 'Draw the desired ball trajectory on the plate' : 'Select the desired ball position on the plate');
    help(isPath ? 'Drag to draw a path. Set its duration, then start. The target holds at the end.' : 'Click the plate or enter X/Y. Choose Center to return to the middle.');
    onPathChange(isPath ? path.points : []); onChange({ pause: true });
  });
  el('pid-path-duration').addEventListener('change', () => {
    const value = el('pid-path-duration').valueAsNumber;
    if (Number.isFinite(value) && value >= .5 && value <= 120) {
      duration = value; help('Duration updated. Start to follow the path from its beginning.');
    } else help('Enter a trajectory duration from 0.5 to 120 seconds.', true);
    onChange({ pause: true });
  });
  el('pid-path-clear').addEventListener('click', () => {
    cancelDrawing(); path = makePath([]); drawPath(); onPathChange([]); onChange({ pause: true });
    help('Drag on the plate to draw a new trajectory.');
  });
  el('pid-path-restart').addEventListener('click', () => {
    onChange({ pause: true }); help('Trajectory rewound. Start to follow it again.');
  });
  function coordinates(event) {
    const p = map.createSVGPoint(); p.x = event.clientX; p.y = event.clientY;
    const local = p.matrixTransform(map.getScreenCTM().inverse());
    return { x: clamp(local.x / 1000, -.1, .1), y: clamp(-local.y / 1000, -.1, .1) };
  }
  map.addEventListener('pointerdown', event => {
    if (event.button !== 0 || !event.isPrimary) return;
    event.preventDefault(); map.focus({ preventScroll: true });
    if (mode() === 'point') { setPoint(coordinates(event)); return; }
    cancelDrawing(); drawing = { id: event.pointerId, points: [coordinates(event)] };
    map.setPointerCapture(event.pointerId); onChange({ pause: true }); drawPath(drawing.points);
    help('Drawing… release to finish.');
  });
  function appendPoint(event, force = false) {
    if (!drawing || event.pointerId !== drawing.id) return;
    const next = coordinates(event), last = drawing.points.at(-1);
    if (Math.hypot(next.x-last.x, next.y-last.y) >= (force ? 1e-6 : .001)) {
      // Bound both path storage and the corresponding 3D geometry for long drags.
      if (drawing.points.length >= 512) drawing.points = drawing.points.filter((_, i) => i%2 === 0);
      drawing.points.push(next); drawPath(drawing.points);
    }
  }
  map.addEventListener('pointermove', event => appendPoint(event));
  map.addEventListener('pointerup', event => {
    if (!drawing || event.pointerId !== drawing.id) return;
    appendPoint(event, true);
    const next = makePath(drawing.points); cancelDrawing();
    if (next.length < .002) { help('Drag at least 2 mm to create a trajectory.', true); return; }
    path = next; drawPath(); onPathChange(path.points); onChange({ pause: true });
    help('Path ready. Start the challenge to track it; the target will hold at the end.');
  });
  for (const event of ['pointercancel', 'lostpointercapture']) map.addEventListener(event, () => {
    if (drawing) { cancelDrawing(); help('Drawing canceled. The previous path is unchanged.'); }
  });
  map.addEventListener('keydown', event => {
    if (event.key === 'Escape' && drawing) { cancelDrawing(); help('Drawing canceled.'); return; }
    if (mode() !== 'point' || !['ArrowLeft','ArrowRight','ArrowUp','ArrowDown','Home'].includes(event.key)) return;
    event.preventDefault();
    if (event.key === 'Home') { setPoint({ x: 0, y: 0 }); return; }
    const step = event.shiftKey ? .01 : .001, next = { ...point };
    if (event.key === 'ArrowLeft') next.x -= step;
    if (event.key === 'ArrowRight') next.x += step;
    if (event.key === 'ArrowUp') next.y += step;
    if (event.key === 'ArrowDown') next.y -= step;
    setPoint({ x: clamp(next.x, -.1, .1), y: clamp(next.y, -.1, .1) });
  });
  function sample(elapsed) { return mode() === 'point' ? { ...point, vx: 0, vy: 0 } : samplePath(path, elapsed, duration); }
  function render(elapsed) {
    const desired = sample(elapsed);
    el('pid-map-target').setAttribute('cx', desired.x*1000);
    el('pid-map-target').setAttribute('cy', -desired.y*1000);
    el('pid-target-position').value = `${(desired.x*1000).toFixed(1)} / ${(desired.y*1000).toFixed(1)} mm`;
    el('pid-target-time').value = `${Math.min(elapsed,duration).toFixed(1)} / ${duration.toFixed(1)} s${elapsed >= duration ? ' · holding end' : ''}`;
    return desired;
  }
  syncPoint();
  return { sample, render, validate, cancelDrawing };
}
