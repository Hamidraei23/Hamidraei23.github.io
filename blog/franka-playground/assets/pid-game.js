import { BALL_RADIUS, PLATE_SIZE, FIXED_DT, DEFAULT_GAINS, initialState, pushBall, stepPhysics, rollingAcceleration } from './pid-physics.js';

// Joint 3 is 0.10 rad below the screenshot to provide tilt clearance.
export const CHALLENGE_JOINTS = [-.437, -1.002, .583, -2.362, -.958, 3.009, .764];

export function setupPIDGame({ robot, tool, scene, three, createIK, cartesian, trajectory, stopDemo, onPoseChanged }) {
  const { Group, Mesh, BoxGeometry, SphereGeometry, TorusGeometry, MeshStandardMaterial, Vector3, Quaternion, Euler } = three;
  const el = id => document.getElementById(id);
  const joints = CHALLENGE_JOINTS.map((_, i) => robot.joints[`panda_joint${i + 1}`]);
  const solve = createIK(robot, tool, { positionTolerance: .00001, orientationTolerance: .0001 });
  const board = new Group(); board.name = 'PID plate · fixed center'; board.visible = false; scene.add(board);
  const material = color => new MeshStandardMaterial({ color, roughness: .55, metalness: .15 });
  const plateMaterial = material('#246574'), markings = material('#9cebe5'), fixtureMaterial = material('#adb8c4');
  function box(x, y, z, position, mat = plateMaterial) {
    const mesh = new Mesh(new BoxGeometry(x, y, z), mat);
    mesh.position.fromArray(position); mesh.castShadow = mat !== markings; mesh.receiveShadow = mat !== markings; board.add(mesh); return mesh;
  }
  box(PLATE_SIZE, PLATE_SIZE, .004, [0, 0, -.002]);
  // An underside edge fixture between the fingertips and the plate. Its rigid
  // transform is preserved while the gripper moves around the plate center.
  box(.035, .014, .02, [-.0925, 0, -.014], fixtureMaterial);
  for (let i = -2; i <= 2; i++) {
    box(.0004, .198, .0002, [i * .04, 0, .00015], markings);
    box(.198, .0004, .0002, [0, i * .04, .00015], markings);
  }
  const target = new Mesh(new TorusGeometry(.01, .0006, 6, 40), markings);
  target.position.z = .0006; board.add(target);
  const ball = new Mesh(new SphereGeometry(BALL_RADIUS, 32, 24), material('#ffc568'));
  ball.castShadow = true; board.add(ball);
  const stripeMaterial = material('#62411d');
  for (const tilt of [0, Math.PI / 2]) {
    const stripe = new Mesh(new TorusGeometry(BALL_RADIUS, .0003, 6, 40), stripeMaterial);
    stripe.rotation.x = tilt; ball.add(stripe);
  }
  let active = false, running = false, accumulator = 0, falls = 0, beforeGame = null;
  let state = initialState(), gains = structuredClone(DEFAULT_GAINS), currentTab = 'joint';
  const pivot = new Vector3(), graspOffset = new Vector3(), graspRotation = new Quaternion();
  let yaw = 0;
  const disabledBefore = new Map();
  const locked = [...document.querySelectorAll('#home, #demo, #gripper, #joint-panel input, #cartesian-panel input, #cartesian-panel select, #cartesian-panel button, #shared-pose-controls input, #shared-pose-controls select, #shared-pose-controls button')];
  function status(text, error = false) {
    el('pid-status').textContent = text;
    el('pid-status').dataset.error = String(error);
  }
  const plateQuaternion = (roll, pitch) => new Quaternion().setFromEuler(new Euler(roll, pitch, yaw, 'ZYX'));
  function applyJoints(q, opening = .00809) {
    joints.forEach((joint, i) => joint.setJointValue(q[i]));
    robot.setJointValue('panda_finger_joint1', opening);
    robot.updateMatrixWorld(true);
  }
  function syncRobot() {
    onPoseChanged(); cartesian.sync();
    const mm = robot.joints.panda_finger_joint1.angle * 2000;
    el('gripper').value = mm; el('gripper-value').value = `${mm.toFixed(1)} mm`;
  }
  function acceptTilt(roll, pitch) {
    const q = plateQuaternion(roll, pitch);
    const position = graspOffset.clone().applyQuaternion(q).add(pivot);
    const orientation = q.clone().multiply(graspRotation);
    return solve(position, orientation).success;
  }
  function updateView() {
    board.position.copy(pivot); board.quaternion.copy(plateQuaternion(state.roll, state.pitch));
    ball.position.set(state.x, state.y, BALL_RADIUS);
    el('pid-map-ball').setAttribute('cx', state.x * 1000);
    el('pid-map-ball').setAttribute('cy', -state.y * 1000);
    el('pid-tilt').value = `${(state.roll * 180 / Math.PI).toFixed(1)}° / ${(state.pitch * 180 / Math.PI).toFixed(1)}°`;
    el('pid-position').value = `${(state.x * 1000).toFixed(1)} / ${(state.y * 1000).toFixed(1)} mm`;
    el('pid-speed').value = `${Math.hypot(state.vx, state.vy).toFixed(3)} m/s`;
    el('pid-score').value = `${state.time.toFixed(1)} s / ${falls}`;
    el('pid-start').textContent = running ? 'Pause challenge' : 'Start challenge';
    el('motion-state').textContent = running ? (state.saturated ? '● PID game · tilt limit' : '● PID game running') : '● PID game paused';
  }
  function reset(message = 'Challenge reset. Your PID gains are unchanged.') {
    state = initialState(); accumulator = 0; ball.quaternion.identity();
    applyJoints(CHALLENGE_JOINTS);
    updateView(); syncRobot(); status(message);
  }
  function enter() {
    stopDemo(); trajectory.stop();
    beforeGame = { joints: joints.map(j => j.angle), opening: robot.joints.panda_finger_joint1.angle };
    for (const control of locked) { disabledBefore.set(control, control.disabled); control.disabled = true; }
    active = true; running = false; falls = 0;
    el('position-label').textContent = 'PLATE CENTER · BASE FRAME';
    applyJoints(CHALLENGE_JOINTS);
    const toolPosition = tool.getWorldPosition(new Vector3());
    const toolRotation = tool.getWorldQuaternion(new Quaternion());
    const direction = new Vector3(0, 0, 1).applyQuaternion(toolRotation);
    direction.z = 0;
    if (direction.length() < .1) direction.set(1, 0, 0);
    direction.normalize(); yaw = Math.atan2(direction.y, direction.x);
    pivot.copy(toolPosition).addScaledVector(direction, .1); pivot.z += .018;
    const inverse = plateQuaternion(0, 0).invert();
    graspOffset.copy(toolPosition).sub(pivot).applyQuaternion(inverse);
    graspRotation.copy(inverse).multiply(toolRotation);
    board.visible = true;
    reset('Start the challenge or give the ball a push.');
  }
  function leave() {
    running = false; active = false; accumulator = 0; board.visible = false;
    el('position-label').textContent = 'FINGERTIP POINT · BASE FRAME';
    applyJoints(beforeGame.joints, beforeGame.opening);
    for (const [control, disabled] of disabledBefore) control.disabled = disabled;
    disabledBefore.clear(); cartesian.setEnabled(true); syncRobot();
    el('motion-state').textContent = '● Ready to explore';
  }
  function selectTab(name) {
    if (name === currentTab) return;
    if (active) leave();
    if (name === 'pid') enter();
    currentTab = name;
    cartesian.setMode(name === 'cartesian');
    cartesian.setEnabled(name !== 'pid');
    for (const tab of ['joint', 'cartesian', 'pid']) {
      const selected = tab === name;
      el(`${tab}-tab`).setAttribute('aria-selected', String(selected));
      el(`${tab}-tab`).tabIndex = selected ? 0 : -1;
      el(`${tab}-panel`).hidden = !selected;
    }
    el('shared-pose-controls').hidden = name === 'pid';
    el('home').parentElement.hidden = name === 'pid';
    document.querySelector('.sidebar').scrollTop = 0;
  }
  const tabs = ['joint', 'cartesian', 'pid'];
  for (const name of tabs) {
    el(`${name}-tab`).addEventListener('click', () => selectTab(name));
    el(`${name}-tab`).addEventListener('keydown', event => {
      if (!['ArrowLeft', 'ArrowRight', 'Home', 'End'].includes(event.key)) return;
      event.preventDefault();
      const available = tabs.filter(tab => !el(`${tab}-tab`).disabled);
      const index = available.indexOf(name);
      const next = event.key === 'Home' ? available[0] : event.key === 'End' ? available.at(-1)
        : available[(index + (event.key === 'ArrowRight' ? 1 : -1) + available.length) % available.length];
      selectTab(next); el(`${next}-tab`).focus();
    });
  }
  function readGains() {
    const next = { roll: {}, pitch: {} };
    for (const axis of ['roll', 'pitch']) for (const term of ['kp', 'ki', 'kd']) {
      const input = el(`pid-${axis}-${term}`), value = input.valueAsNumber;
      if (!Number.isFinite(value) || value < 0 || value > 100) {
        running = false; status('Enter a gain from 0 to 100 in all six fields.', true); updateView(); return false;
      }
      next[axis][term] = value;
    }
    gains = next;
    return true;
  }
  for (const input of document.querySelectorAll('.pid-gains input')) input.addEventListener('change', () => {
    if (readGains()) { state.rollIntegral = 0; state.pitchIntegral = 0; status('PID gains updated.'); }
  });
  el('pid-start').addEventListener('click', () => {
    if (!active || !readGains()) return;
    running = !running; accumulator = 0; updateView(); status(running ? 'Balance the ball at the center.' : 'Challenge paused.');
  });
  el('pid-reset').addEventListener('click', () => { if (active) reset(); });
  el('pid-push-speed').addEventListener('input', () => {
    const speed = el('pid-push-speed').valueAsNumber.toFixed(2);
    el('pid-push-speed-value').value = `${speed} m/s`;
    el('pid-push').textContent = `Random push · ${speed} m/s`;
  });
  el('pid-push').addEventListener('click', () => {
    if (!active || !readGains()) return;
    const speed = el('pid-push-speed').valueAsNumber;
    pushBall(state, Math.random() * Math.PI * 2, speed);
    running = true; updateView(); status(`Added a random ${speed.toFixed(2)} m/s velocity push.`);
  });
  function tick(dt) {
    if (!active || !running) return;
    accumulator += Math.min(Math.max(dt, 0), .05);
    while (accumulator >= FIXED_DT) {
      if (!stepPhysics(state, gains, FIXED_DT, acceptTilt)) {
        running = false; accumulator = 0;
        status('Robot tilt could not be reached. Reset the challenge to continue.', true); break;
      }
      accumulator -= FIXED_DT;
      const { ballOmega, omega } = rollingAcceleration(state);
      const rotation = new Vector3(...ballOmega.map((value, i) => value - omega[i]));
      const speed = rotation.length();
      if (speed > 1e-10) ball.quaternion.premultiply(new Quaternion().setFromAxisAngle(rotation.divideScalar(speed), speed * FIXED_DT)).normalize();
      if (state.fallen) { falls++; reset('Ball fell — robot, plate and ball reset. Try adjusting the gains.'); break; }
    }
    updateView(); onPoseChanged();
  }
  return { tick, get active() { return active; }, get center() { return pivot; } };
}
