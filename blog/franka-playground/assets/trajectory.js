// Pose storage and kinematic playback. Angles are always stored in radians.
const STORAGE_KEY = 'franka-playground.poses.v1';
const MAX_POSES = 100;
export const ease = t => t * t * t * (10 + t * (-15 + 6 * t));

// Monotone cubic interpolation keeps each joint within its sampled limits while
// giving the sampled Cartesian path continuous joint velocity.
export function sampleJoints(samples, t) {
  const x = Math.max(0, Math.min(1, t)) * (samples.length - 1);
  const i = Math.min(Math.floor(x), samples.length - 2);
  const u = x - i;
  const slope = (a, b) => a * b <= 0 ? 0 : 2 * a * b / (a + b);
  return samples[i].map((a, j) => {
    const b = samples[i + 1][j], delta = b - a;
    const left = i ? slope(a - samples[i - 1][j], delta) : delta;
    const right = i + 2 < samples.length ? slope(delta, samples[i + 2][j] - b) : delta;
    return (2*u**3 - 3*u**2 + 1)*a + (u**3 - 2*u**2 + u)*left
      + (-2*u**3 + 3*u**2)*b + (u**3 - u**2)*right;
  });
}

export function setupTrajectories({ robot, tool, Vector3, Quaternion, solveIK,
  stopDemo, onPoseChanged, cartesian }) {
  const el = id => document.getElementById(id);
  const joints = Array.from({ length: 7 }, (_, i) => robot.joints[`panda_joint${i + 1}`]);
  let poses = [], motion = null, planning = false, generation = 0;
  const manual = [...document.querySelectorAll('#joint-panel input, #cartesian-panel input, #cartesian-panel select, #cartesian-panel button, #home, #demo, #gripper, #joint-tab, #cartesian-tab')];
  const disabledBefore = new Map();
  const status = (message, error = false) => {
    el('trajectory-status').textContent = message;
    el('trajectory-status').dataset.error = String(error);
  };
  const selected = () => poses.find(p => p.id === el('saved-poses').value);
  const values = () => joints.map(joint => joint.angle);
  function apply(q, gripper) {
    joints.forEach((joint, i) => joint.setJointValue(q[i]));
    robot.setJointValue('panda_finger_joint1', gripper / 2);
    robot.updateMatrixWorld(true);
  }
  function capture(name = '') {
    robot.updateMatrixWorld(true);
    return { name, joints: values(), gripper: robot.joints.panda_finger_joint1.angle * 2,
      position: tool.getWorldPosition(new Vector3()).toArray(),
      quaternion: tool.getWorldQuaternion(new Quaternion()).toArray() };
  }
  function refreshRobot() {
    el('gripper').value = robot.joints.panda_finger_joint1.angle * 2000;
    el('gripper-value').value = `${Number(el('gripper').value).toFixed(1)} mm`;
    onPoseChanged();
    cartesian.sync();
  }
  function lockManual(lock) {
    if (lock) {
      for (const control of manual) {
        disabledBefore.set(control, control.disabled);
        control.disabled = true;
      }
    } else {
      for (const [control, disabled] of disabledBefore) control.disabled = disabled;
      disabledBefore.clear();
    }
    cartesian.setEnabled(!lock);
  }
  function updateButtons() {
    const busy = planning || !!motion, index = poses.indexOf(selected());
    for (const id of ['save-pose', 'pose-name', 'saved-poses', 'trajectory-mode', 'trajectory-duration']) el(id).disabled = busy;
    el('save-pose').disabled = busy || poses.length >= MAX_POSES;
    for (const id of ['rename-pose', 'update-pose', 'delete-pose', 'move-to-pose']) el(id).disabled = busy || index < 0;
    el('pose-up').disabled = busy || index <= 0;
    el('pose-down').disabled = busy || index < 0 || index === poses.length - 1;
    el('play-poses').disabled = busy || !poses.length;
    el('pause-trajectory').disabled = !motion;
    el('pause-trajectory').textContent = motion?.paused ? 'Resume' : 'Pause';
    el('stop-trajectory').disabled = !busy;
  }
  function render(selection = el('saved-poses').value) {
    el('saved-poses').replaceChildren(...poses.map((pose, i) => {
      const option = document.createElement('option');
      option.value = pose.id;
      option.textContent = `${i + 1}. ${pose.name}`;
      return option;
    }));
    el('saved-poses').value = poses.some(p => p.id === selection) ? selection : poses[0]?.id || '';
    updateButtons();
  }
  function persist() {
    try {
      localStorage.setItem(STORAGE_KEY, JSON.stringify({ version: 1, poses }));
      el('pose-storage-status').textContent = 'Saved in this browser.';
    } catch {
      el('pose-storage-status').textContent = 'Browser storage unavailable or full. Poses are kept only for this visit.';
    }
  }
  const finiteArray = (value, count) => Array.isArray(value) && value.length === count && value.every(Number.isFinite);
  function validPose(pose) {
    return pose && typeof pose.id === 'string' && typeof pose.name === 'string'
      && pose.name.trim().length > 0 && pose.name.length <= 60
      && finiteArray(pose.joints, 7) && pose.joints.every((q, i) => q >= joints[i].limit.lower && q <= joints[i].limit.upper)
      && Number.isFinite(pose.gripper) && pose.gripper >= 0 && pose.gripper <= .08
      && finiteArray(pose.position, 3) && finiteArray(pose.quaternion, 4)
      && Math.abs(Math.hypot(...pose.quaternion) - 1) < .001;
  }
  try {
    const raw = localStorage.getItem(STORAGE_KEY);
    if (raw) {
      const saved = JSON.parse(raw);
      if (saved.version !== 1 || !Array.isArray(saved.poses) || saved.poses.length > MAX_POSES
        || !saved.poses.every(validPose) || new Set(saved.poses.map(p => p.id)).size !== saved.poses.length) throw new Error('Invalid pose data');
      // Derive tool transforms from the saved joints so stale transforms cannot
      // disagree with the model's actual fingertip reference point.
      const initial = capture();
      try {
        poses = saved.poses.map(pose => {
          apply(pose.joints, pose.gripper);
          return { ...capture(pose.name), id: pose.id };
        });
      } finally { apply(initial.joints, initial.gripper); }
    }
  } catch {
    el('pose-storage-status').textContent = 'Saved poses could not be read. You can save new poses for this visit.';
  }
  el('save-pose').addEventListener('click', () => {
    if (poses.length >= MAX_POSES) return;
    stopDemo();
    const pose = { ...capture(el('pose-name').value.trim() || `Pose ${poses.length + 1}`), id: crypto.randomUUID() };
    poses.push(pose);
    persist(); render(pose.id);
    el('pose-name').value = '';
    status(`Saved “${pose.name}”.`);
  });
  el('saved-poses').addEventListener('change', () => {
    el('pose-name').value = selected()?.name || '';
    updateButtons();
  });
  el('rename-pose').addEventListener('click', () => {
    const pose = selected(), name = el('pose-name').value.trim();
    if (!pose) return;
    if (!name) { status('Enter a name to rename the selected pose.', true); return; }
    pose.name = name;
    persist(); render(); status(`Renamed to “${name}”.`);
  });
  el('update-pose').addEventListener('click', () => {
    const pose = selected();
    if (!pose) return;
    stopDemo();
    Object.assign(pose, capture(pose.name));
    persist(); status(`Updated “${pose.name}” from the current robot pose.`);
  });
  el('delete-pose').addEventListener('click', () => {
    const pose = selected();
    if (!pose) return;
    poses.splice(poses.indexOf(pose), 1);
    persist(); render(); status(`Deleted “${pose.name}”.`);
  });
  for (const [id, direction] of [['pose-up', -1], ['pose-down', 1]]) {
    el(id).addEventListener('click', () => {
      const i = poses.indexOf(selected()), j = i + direction;
      if (i < 0 || j < 0 || j >= poses.length) return;
      [poses[i], poses[j]] = [poses[j], poses[i]];
      persist(); render();
    });
  }

  function cartesianSamples(start, end) {
    const from = new Vector3().fromArray(start.position), to = new Vector3().fromArray(end.position);
    const fromQ = new Quaternion().fromArray(start.quaternion), toQ = new Quaternion().fromArray(end.quaternion);
    const count = Math.max(100, Math.ceil(from.distanceTo(to) / .002), Math.ceil(fromQ.angleTo(toQ) / .01));
    const samples = [start.joints];
    apply(start.joints, start.gripper);
    for (let i = 1; i <= count; i++) {
      const fraction = i / count;
      const position = from.clone().lerp(to, fraction), quaternion = fromQ.clone().slerp(toQ, fraction);
      const result = solveIK(position, quaternion);
      if (!result.success) throw new Error(`Cannot reach the straight path to “${end.name}” (${Math.round(fraction * 100)}%). Try smooth joints or add an intermediate pose.`);
      const q = values();
      if (q.some((value, j) => !Number.isFinite(value) || Math.abs(value - samples.at(-1)[j]) > .12)) {
        throw new Error(`The straight path to “${end.name}” needs an abrupt joint change. Try smooth joints or an intermediate pose.`);
      }
      samples.push(q);
    }
    return samples;
  }
  async function startPlayback(destinations) {
    if (planning || motion || !destinations.length) return;
    const seconds = el('trajectory-duration').valueAsNumber;
    if (!Number.isFinite(seconds) || seconds < .5 || seconds > 60) {
      status('Enter a move duration between 0.5 and 60 seconds.', true); return;
    }
    stopDemo();
    planning = true;
    const token = ++generation, initial = capture(), segments = [];
    const mode = el('trajectory-mode').value;
    lockManual(true); updateButtons();
    el('trajectory-progress').value = 0;
    let start = initial;
    try {
      for (const end of destinations) {
        status(`Checking move ${segments.length + 1}/${destinations.length}: “${end.name}”…`);
        // Yield with the real robot restored so Stop and rendering stay usable.
        await new Promise(resolve => setTimeout(resolve, 0));
        if (token !== generation) return;
        let samples;
        try {
          samples = mode === 'cartesian' ? cartesianSamples(start, end) : [start.joints, end.joints];
        } finally { apply(initial.joints, initial.gripper); }
        segments.push({ samples, start, end, seconds });
        start = { ...end, joints: samples.at(-1) };
      }
      if (token !== generation) return;
      motion = { segments, index: 0, elapsed: 0, paused: false };
      el('motion-state').textContent = '● Saved pose trajectory';
      status(`Moving to “${segments[0].end.name}” · 1/${segments.length}`);
    } catch (error) {
      status(error.message, true);
    } finally {
      if (token === generation) {
        planning = false;
        if (!motion) lockManual(false);
        refreshRobot(); updateButtons();
      }
    }
  }
  el('move-to-pose').addEventListener('click', () => startPlayback(selected() ? [selected()] : []));
  el('play-poses').addEventListener('click', () => startPlayback([...poses]));
  el('pause-trajectory').addEventListener('click', () => {
    if (!motion) return;
    motion.paused = !motion.paused;
    el('motion-state').textContent = motion.paused ? '● Trajectory paused' : '● Saved pose trajectory';
    status(`${motion.paused ? 'Paused on the way' : 'Moving'} to “${motion.segments[motion.index].end.name}”.`);
    updateButtons();
  });
  function stop() {
    if (!motion && !planning) return;
    generation++;
    motion = null; planning = false;
    lockManual(false); refreshRobot(); updateButtons();
    el('motion-state').textContent = '● Ready to explore';
    status('Stopped at the current pose.');
  }
  el('stop-trajectory').addEventListener('click', stop);
  function tick(dt) {
    if (!motion || motion.paused) return;
    const segment = motion.segments[motion.index];
    motion.elapsed = Math.min(segment.seconds, motion.elapsed + Math.max(0, dt));
    const t = motion.elapsed / segment.seconds, blend = ease(t);
    apply(sampleJoints(segment.samples, blend), segment.start.gripper + (segment.end.gripper - segment.start.gripper) * blend);
    refreshRobot();
    el('trajectory-progress').value = (motion.index + t) / motion.segments.length;
    if (t === 1) {
      motion.index++; motion.elapsed = 0;
      if (motion.index === motion.segments.length) {
        motion = null;
        lockManual(false); updateButtons();
        el('motion-state').textContent = '● Ready to explore';
        status(`Reached “${segment.end.name}”.`);
      } else status(`Moving to “${motion.segments[motion.index].end.name}” · ${motion.index + 1}/${motion.segments.length}`);
    }
  }
  render();
  if (poses.length) status('Select a saved pose or play the full sequence.');
  return { tick, stop };
}
