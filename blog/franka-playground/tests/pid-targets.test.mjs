import test from 'node:test';
import assert from 'node:assert/strict';
import { makePath, samplePath } from '../assets/pid-targets.js';
import { initialState, stepPhysics, DEFAULT_GAINS, FIXED_DT } from '../assets/pid-physics.js';
const close = (a,b,tol=1e-9) => assert.ok(Math.abs(a-b)<tol, `${a} != ${b}`);

test('drawing stays within the plate and repeated pointer coordinates are removed',()=>{
  const path=makePath([{x:-.2,y:.2},{x:-.1,y:.1},{x:.2,y:-.2}]);
  assert.deepEqual(path.points,[{x:-.1,y:.1},{x:.1,y:-.1}]);
  close(path.length,Math.hypot(.2,.2));
});

test('trajectory timing uses distance, independent of pointer sample density',()=>{
  const a=makePath([{x:0,y:0},{x:.08,y:0}]);
  const b=makePath([{x:0,y:0},{x:.001,y:0},{x:.002,y:0},{x:.08,y:0}]);
  for(let t=0;t<=10;t+=.25){close(samplePath(a,t,10).x,samplePath(b,t,10).x);close(samplePath(a,t,10).vx,samplePath(b,t,10).vx);}
  close(samplePath(a,5,10).x,.04);
});

test('timed trajectory starts at rest and holds its last point after completion',()=>{
  const points=[{x:-.02,y:.01},{x:.03,y:.04}],path=makePath(points);
  assert.deepEqual(samplePath(path,0,4),{...points[0],vx:0,vy:0});
  for(const time of [4,5,100])assert.deepEqual(samplePath(path,time,4),{...points[1],vx:0,vy:0});
  assert.deepEqual(samplePath(makePath([]),1,4),{x:0,y:0,vx:0,vy:0});
});

test('moving reference velocity matches its time derivative',()=>{
  const path=makePath([{x:-.03,y:-.02},{x:.04,y:.06}]),h=1e-5;
  for(const t of [.5,1,2,3]){
    const a=samplePath(path,t-h,4),b=samplePath(path,t+h,4),s=samplePath(path,t,4);
    close((b.x-a.x)/(2*h),s.vx,1e-7);close((b.y-a.y)/(2*h),s.vy,1e-7);
  }
});

test('PID controls an arbitrary point, rather than always returning to center',()=>{
  const desired={x:-.02,y:.025,vx:0,vy:0},state=initialState();
  for(let i=0;i<15/FIXED_DT;i++){assert(stepPhysics(state,DEFAULT_GAINS,FIXED_DT,()=>true,desired));assert(!state.fallen);}
  close(state.x,desired.x,.002);close(state.y,desired.y,.002);
});

test('PID uses relative velocity for a moving target',()=>{
  const state={...initialState(),vx:.02,vy:-.01};
  const desired={x:state.x,y:state.y,vx:state.vx,vy:state.vy};
  stepPhysics(state,DEFAULT_GAINS,FIXED_DT,()=>true,desired);
  close(state.roll,0);close(state.pitch,0);close(state.vx,.02);close(state.vy,-.01);
});

test('ball follows a slow drawn trajectory and holds its endpoint',()=>{
  const state=initialState(), path=makePath([{x:.025,y:-.02},{x:.04,y:.005},{x:.01,y:.025}]);
  for(let i=0;i<16/FIXED_DT;i++){
    assert(stepPhysics(state,DEFAULT_GAINS,FIXED_DT,()=>true,samplePath(path,i*FIXED_DT,10)));
    assert(!state.fallen);
  }
  close(state.x,.01,.002);close(state.y,.025,.002);
});
