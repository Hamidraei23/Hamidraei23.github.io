import test from 'node:test';
import assert from 'node:assert/strict';
import { initialState, rollingAcceleration, stepPhysics, pushBall, GRAVITY, BALL_RADIUS, MAX_TILT, FIXED_DT, DEFAULT_GAINS } from '../assets/pid-physics.js';
const zero = { roll: { kp: 0, ki: 0, kd: 0 }, pitch: { kp: 0, ki: 0, kd: 0 } };
const close = (a,b,tol=1e-10) => assert.ok(Math.abs(a-b)<tol, `${a} != ${b}`);
function simulate(state, gains, seconds, dt=FIXED_DT) {
  for(let i=0;i<Math.round(seconds/dt);i++) { stepPhysics(state,gains,dt); if(state.fallen) break; }
  return state;
}

test('level stationary plate has no horizontal acceleration or artificial friction',()=>{
  const s=initialState();s.vx=.01;s.vy=-.005;
  const a=rollingAcceleration(s);
  close(a.ax,0);close(a.ay,0);close(a.normalForcePerMass,GRAVITY);
  simulate(s,zero,1);
  close(s.x,.035);close(s.y,-.025);close(s.vx,.01);close(s.vy,-.005);
});

test('static incline matches a solid sphere: acceleration is 5/7 of tangential gravity',()=>{
  const s=initialState();s.pitch=.2;
  let a=rollingAcceleration(s);close(a.ax,5/7*GRAVITY*Math.sin(.2));close(a.ay,0);
  s.roll=-.15;a=rollingAcceleration(s);
  close(a.ax,5/7*GRAVITY*Math.sin(.2));close(a.ay,5/7*GRAVITY*Math.sin(.15)*Math.cos(.2));
});

test('rolling angular velocity cancels relative contact velocity',()=>{
  const s={...initialState(),roll:.2,pitch:.1,rollRate:.3,pitchRate:-.2,vx:.04,vy:-.06};
  const {ballOmega:w,omega}=rollingAcceleration(s);
  close(s.vx + BALL_RADIUS*(omega[1]-w[1]),0);
  close(s.vy - BALL_RADIUS*(omega[0]-w[0]),0);
});

test('plate angular acceleration affects the ball even at the center',()=>{
  const s={...initialState(),x:0,y:0};
  const a=rollingAcceleration(s,1,2);
  close(a.ax,-2*BALL_RADIUS);close(a.ay,BALL_RADIUS);
});

test('push adds the selected speed for every direction and preserves existing velocity',()=>{
  for(const speed of [.1,.2,.3,.4]) for(let i=0;i<32;i++){
    const s={...initialState(),vx:.03,vy:-.04};pushBall(s,i*Math.PI/16,speed);
    close(Math.hypot(s.vx-.03,s.vy+.04),speed);
  }
});

test('default gains settle the initial offset and reject pushes in all directions',()=>{
  for(let i=0;i<16;i++){
    const s=simulate(initialState(),DEFAULT_GAINS,10);
    assert(!s.fallen);assert(Math.hypot(s.x,s.y)<.002);
    pushBall(s,i*Math.PI/8);simulate(s,DEFAULT_GAINS,8);
    assert(!s.fallen);assert(Math.hypot(s.x,s.y)<.002);
  }
});

test('ball falls past the edge; failed state cannot continue integrating',()=>{
  const s={...initialState(),x:.099,vx:.1};simulate(s,zero,.1);
  assert(s.fallen);const before={...s};assert.equal(stepPhysics(s,zero),false);assert.deepEqual(s,before);
});

test('loss of normal contact is detected',()=>{
  const s={...initialState(),x:.099,pitchRate:0};
  assert(rollingAcceleration(s,0,200).normalForcePerMass<0);
});

test('commands saturate at ±25 degrees without integrator windup',()=>{
  const s=initialState();const gains={roll:{kp:100,ki:100,kd:0},pitch:{kp:100,ki:100,kd:0}};
  for(let i=0;i<1000;i++){
    // Hold the error fixed to exercise the actuator limit for a full interval.
    Object.assign(s,{x:.09,y:-.09,vx:0,vy:0,fallen:false});stepPhysics(s,gains);
    assert(Math.abs(s.roll)<=MAX_TILT);assert(Math.abs(s.pitch)<=MAX_TILT);
    assert(Math.abs(s.rollIntegral)<.001);assert(Math.abs(s.pitchIntegral)<.001);
  }
  close(s.roll,-MAX_TILT);close(s.pitch,-MAX_TILT);
});

test('unreachable actuator pose leaves dynamics unchanged',()=>{
  const s=initialState(), before={...s};
  assert.equal(stepPhysics(s,DEFAULT_GAINS,FIXED_DT,()=>false),false);
  assert.deepEqual(s,before);
});

test('fixed timestep converges when the physics interval is halved',()=>{
  const a=simulate(initialState(),DEFAULT_GAINS,2),b=simulate(initialState(),DEFAULT_GAINS,2,FIXED_DT/2);
  close(a.x,b.x,.0001);close(a.y,b.y,.0001);close(a.roll,b.roll,.0005);close(a.pitch,b.pitch,.0005);
});
