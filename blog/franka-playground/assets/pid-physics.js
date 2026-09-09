// Solid sphere, no slip on a rotating plane. SI units throughout.
// See ../PID-GAME.md for the constraint equations and assumptions.
export const GRAVITY = 9.81;
export const BALL_RADIUS = .01;
export const PLATE_SIZE = .2;
export const MAX_TILT = 25 * Math.PI / 180;
export const FIXED_DT = 1 / 240;
export const PUSH_SPEED = .1;
export const DEFAULT_GAINS = { roll: { kp: 2, ki: .1, kd: .6 }, pitch: { kp: 2, ki: .1, kd: .6 } };
const clamp = (x, min, max) => Math.max(min, Math.min(max, x));
const cross = (a, b) => [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];

export function initialState() {
  return { x: .025, y: -.02, vx: 0, vy: 0, spin: 0,
    roll: 0, pitch: 0, rollRate: 0, pitchRate: 0, rollIntegral: 0, pitchIntegral: 0,
    time: 0, fallen: false, saturated: false };
}

export function pushBall(state, angle) {
  // A velocity impulse of exactly 0.1 m/s in a random direction on the plate.
  // The rolling constraint supplies the corresponding change in angular velocity.
  state.vx += PUSH_SPEED * Math.cos(angle);
  state.vy += PUSH_SPEED * Math.sin(angle);
}

function pid(position, velocity, integral, gains, dt) {
  const next = gains.ki > 0 ? clamp(integral + position * dt, -MAX_TILT / gains.ki, MAX_TILT / gains.ki) : 0;
  const raw = gains.kp * position + gains.ki * next + gains.kd * velocity;
  // Conditional integration: no windup while the error pushes into saturation.
  const accepted = Math.abs(raw) <= MAX_TILT || Math.sign(position) !== Math.sign(raw);
  const value = gains.kp * position + gains.ki * (accepted ? next : integral) + gains.kd * velocity;
  return { integral: accepted ? next : integral, command: clamp(value, -MAX_TILT, MAX_TILT), saturated: Math.abs(value) >= MAX_TILT };
}

function actuator(angle, rate, target, dt) {
  // Critically damped angular servo with finite speed/acceleration. Avoids an
  // instantaneous tilt injecting an unbounded acceleration into the ball.
  const acceleration = clamp(18**2 * (target - angle) - 36 * rate, -4, 4);
  let nextRate = clamp(rate + acceleration * dt, -.8, .8);
  const nextAngle = clamp(angle + nextRate * dt, -MAX_TILT, MAX_TILT);
  if (Math.abs(nextAngle) === MAX_TILT && nextAngle * nextRate > 0) nextRate = 0;
  return [nextAngle, nextRate, (nextRate - rate) / dt];
}

export function rollingAcceleration(state, rollAcceleration = 0, pitchAcceleration = 0) {
  const { x, y, vx, vy, roll, pitch, rollRate, pitchRate, spin } = state;
  const sr = Math.sin(roll), cr = Math.cos(roll);
  const omega = [rollRate, pitchRate * cr, -pitchRate * sr];
  const alpha = [rollAcceleration, pitchAcceleration * cr - pitchRate * sr * rollRate,
    -pitchAcceleration * sr - pitchRate * cr * rollRate];
  const radius = [x, y, BALL_RADIUS], velocity = [vx, vy, 0];
  const gravity = [GRAVITY * Math.sin(pitch), -GRAVITY * sr * Math.cos(pitch), -GRAVITY * cr * Math.cos(pitch)];
  const coriolis = cross(omega, velocity), euler = cross(alpha, radius), centrifugal = cross(omega, cross(omega, radius));
  const transport = coriolis.map((v, i) => 2*v + euler[i] + centrifugal[i]);
  const ballOmega = [omega[0] - vy / BALL_RADIUS, omega[1] + vx / BALL_RADIUS, spin];
  const gyroscopic = cross(omega, ballOmega);
  const rotational = cross([0, 0, 1], alpha.map((v, i) => v + gyroscopic[i]));
  const acceleration = [0, 1].map(i => (gravity[i] - transport[i] + .4 * BALL_RADIUS * rotational[i]) / 1.4);
  return { ax: acceleration[0], ay: acceleration[1], spinRate: -gyroscopic[2],
    normalForcePerMass: transport[2] - gravity[2], ballOmega, omega };
}

export function stepPhysics(state, gains, dt = FIXED_DT, acceptTilt = () => true) {
  if (state.fallen) return false;
  const r = pid(state.y, state.vy, state.rollIntegral, gains.roll, dt);
  const p = pid(state.x, state.vx, state.pitchIntegral, gains.pitch, dt);
  const [roll, rollRate, rollAcceleration] = actuator(state.roll, state.rollRate, r.command, dt);
  const [pitch, pitchRate, pitchAcceleration] = actuator(state.pitch, state.pitchRate, -p.command, dt);
  if (!acceptTilt(roll, pitch)) return false;
  Object.assign(state, { roll, pitch, rollRate, pitchRate, rollIntegral: r.integral, pitchIntegral: p.integral,
    saturated: r.saturated || p.saturated });
  const { ax, ay, spinRate, normalForcePerMass } = rollingAcceleration(state, rollAcceleration, pitchAcceleration);
  state.vx += ax * dt; state.vy += ay * dt;
  state.x += state.vx * dt; state.y += state.vy * dt;
  state.spin += spinRate * dt;
  state.time += dt;
  // Contact is lost when the contact point passes an edge, or the plane would
  // have to pull the ball down to keep it attached (negative normal force).
  state.fallen = Math.abs(state.x) > PLATE_SIZE / 2 || Math.abs(state.y) > PLATE_SIZE / 2 || normalForcePerMass <= 0;
  return true;
}
