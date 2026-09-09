const {chromium}=require(process.env.PLAYWRIGHT_MODULE || 'playwright');
const assert=require('node:assert/strict');
const {join}=require('node:path');
const {tmpdir}=require('node:os');
const url=process.env.PID_GAME_URL || 'http://127.0.0.1:8765/blog/franka-playground/';
(async()=>{
const browser=await chromium.launch({headless:true, executablePath:process.env.PLAYWRIGHT_CHROMIUM_EXECUTABLE || undefined,args:['--no-sandbox','--enable-unsafe-swiftshader']});
try{
const page=await browser.newPage({viewport:{width:1440,height:1080}});
const errors=[];page.on('pageerror',e=>errors.push(e.message));
await page.route('**/pid-game.js',async route=>{
 const response=await route.fetch();let body=await response.text();
 body=body.replace('return { tick, get active() { return active; }, get center() { return pivot; } };', `window.__pidTest={tick, reset, reference:q=>{CHALLENGE_JOINTS.splice(0,7,...q);selectTab('joint');selectTab('pid')}, tilt:acceptTilt, read:()=>({state:{...state},targetTime,desired:targets.sample(targetTime),segments:pathGroup.children.length,meters:[...document.querySelectorAll('#pid-joint-bars meter')].map(e=>e.value),active,running,falls,pivot:pivot.toArray(),board:board.position.toArray(),quaternion:board.quaternion.toArray(),q:joints.map(j=>j.angle),actualCenter:tool.getWorldPosition(new Vector3()).sub(graspOffset.clone().applyQuaternion(tool.getWorldQuaternion(new Quaternion()).multiply(graspRotation.clone().invert()))).toArray()})}; return { tick, get active() { return active; }, get center() { return pivot; } };`);
 await route.fulfill({response,body});
});
await page.goto(url);
await page.waitForSelector('body[data-ready="true"]',{timeout:60000});
const q=()=>page.locator('#joints input[type=number]').evaluateAll(es=>es.map(e=>+e.value));
const initial=await q();
await page.click('#pid-tab');
assert.equal(await page.locator('#pid-panel').isVisible(),true);
assert.equal(await page.locator('#shared-pose-controls').isVisible(),false);
assert.equal(await page.locator('#home').isDisabled(),true);
let s=await page.evaluate(()=>window.__pidTest.read());
assert.deepEqual(s.q,[-.437,-1.002,.583,-2.362,-.958,3.009,.764]);
console.log('entry',s.pivot);

const read=()=>page.evaluate(()=>window.__pidTest.read());
const advance=seconds=>page.evaluate(seconds=>{for(let i=0;i<Math.round(seconds/.05);i++)window.__pidTest.tick(.05);return window.__pidTest.read()},seconds);
const screenPoint=async(x,y)=>page.locator('#pid-map').evaluate((map,{x,y})=>{
  const p=map.createSVGPoint();p.x=x*1000;p.y=-y*1000;const screen=p.matrixTransform(map.getScreenCTM());return {x:screen.x,y:screen.y};
},{x,y});
assert.equal(await page.locator('#pid-joint-bars meter').count(),7);
await page.selectOption('#angle-unit','radians');
assert.equal(await page.locator('#pid-joint-bars output').first().textContent(),'-0.437 rad');
await page.locator('#pid-map').scrollIntoViewIfNeeded();
let pos=await screenPoint(-.02,.025);await page.mouse.click(pos.x,pos.y);
s=await read();assert(Math.abs(s.desired.x+.02)<.001);assert(Math.abs(s.desired.y-.025)<.001);
await page.click('#pid-start');s=await advance(.3);
assert.deepEqual(s.meters,s.q);
assert(s.q.some((v,i)=>Math.abs(v-[-.437,-1.002,.583,-2.362,-.958,3.009,.764][i])>.001));
s=await advance(12);
assert.equal(s.falls,0);assert(Math.abs(s.state.x+.02)<.002);assert(Math.abs(s.state.y-.025)<.002);
assert.deepEqual(s.meters,s.q);
await page.click('#pid-start');
await page.click('#pid-target-center');assert.equal((await read()).desired.x,0);assert.equal((await read()).desired.y,0);
await page.locator('#pid-map').focus();await page.keyboard.press('ArrowRight');await page.keyboard.press('Shift+ArrowUp');
s=await read();assert.equal(s.desired.x,.001);assert.equal(s.desired.y,.01);
await page.keyboard.press('Home');assert.equal((await read()).desired.x,0);
await page.fill('#pid-target-x','100');await page.locator('#pid-target-x').press('Enter');assert.equal((await read()).desired.x,.1);
await page.fill('#pid-target-x','101');await page.locator('#pid-target-x').press('Enter');
assert.equal(await page.locator('#pid-target-help').getAttribute('data-error'),'true');assert.equal((await read()).running,false);
await page.click('#pid-target-center');await page.click('#pid-reset');
await page.selectOption('#pid-target-mode','path');
await page.click('#pid-start');assert.equal((await read()).running,false);
assert((await page.locator('#pid-target-help').textContent()).includes('Draw a path'));
await page.locator('#pid-map').scrollIntoViewIfNeeded();
const drawn=[{x:.025,y:-.02},{x:.04,y:.005},{x:.01,y:.025}];
pos=await screenPoint(drawn[0].x,drawn[0].y);await page.mouse.move(pos.x,pos.y);await page.mouse.down();
for(const p of drawn.slice(1)){pos=await screenPoint(p.x,p.y);await page.mouse.move(pos.x,pos.y,{steps:15});}
await page.mouse.up();
s=await read();assert(s.segments>10);assert(Math.abs(s.desired.x-.025)<.001);assert.equal(s.running,false);
const pathPoints=await page.locator('#pid-map-path').getAttribute('points');assert(pathPoints.length>20);
await page.fill('#pid-path-duration','.1');await page.locator('#pid-path-duration').dispatchEvent('change');
await page.click('#pid-start');assert.equal((await read()).running,false);assert.equal(await page.locator('#pid-target-help').getAttribute('data-error'),'true');
await page.fill('#pid-path-duration','8');await page.locator('#pid-path-duration').dispatchEvent('change');
await page.click('#pid-start');s=await advance(3);assert(s.targetTime>=3 && s.targetTime<4);assert(s.desired.vx!==0 || s.desired.vy!==0);
await page.click('#pid-start');const paused=await read();await advance(1);await page.waitForTimeout(100);
assert.deepEqual((await read()).state,paused.state);assert.equal((await read()).targetTime,paused.targetTime);
await page.click('#pid-start');s=await advance(10);assert.equal(s.falls,0);
assert(Math.abs(s.desired.x-.01)<.001);assert(Math.abs(s.desired.y-.025)<.001);assert.equal(s.desired.vx,0);assert.equal(s.desired.vy,0);
assert(Math.abs(s.state.x-.01)<.003);assert(Math.abs(s.state.y-.025)<.003);
assert((await page.locator('#pid-target-time').textContent()).includes('holding end'));
await page.click('#pid-start');await page.click('#pid-path-restart');assert.equal((await read()).targetTime,0);
await page.click('#pid-reset');assert.equal((await read()).targetTime,0);assert.equal(await page.locator('#pid-map-path').getAttribute('points'),pathPoints);
// A fall rewinds the trajectory while retaining its drawing and duration.
for(const input of await page.locator('.pid-gains input').all()){await input.fill('0');await input.dispatchEvent('change');}
await page.fill('#pid-push-speed','0.4');await page.locator('#pid-push-speed').dispatchEvent('input');
await page.evaluate(()=>Math.random=()=>0);await page.click('#pid-push');
s=await page.evaluate(()=>{for(let i=0;i<60;i++){window.__pidTest.tick(.05);if(window.__pidTest.read().falls)break;}return window.__pidTest.read()});
assert.equal(s.falls,1);assert.equal(s.targetTime,0);assert.equal(await page.locator('#pid-map-path').getAttribute('points'),pathPoints);
await page.click('#pid-start');
await page.selectOption('#angle-unit','degrees');assert((await page.locator('#pid-joint-bars output').first().textContent()).endsWith('°'));
await page.locator('#pid-map').scrollIntoViewIfNeeded();await page.screenshot({path:join(tmpdir(),'pid-targets-desktop.png'),fullPage:true});
await page.click('#pid-path-clear');assert.equal((await read()).segments,0);assert.equal(await page.locator('#pid-map-path').getAttribute('points'),'');
await page.click('#pid-start');assert.equal((await read()).running,false);
await page.selectOption('#pid-target-mode','point');assert.equal(await page.locator('#pid-point-controls').isVisible(),true);
await page.setViewportSize({width:390,height:844});
await page.locator('#pid-map').scrollIntoViewIfNeeded();pos=await screenPoint(.015,-.015);await page.mouse.click(pos.x,pos.y);
s=await read();assert(Math.abs(s.desired.x-.015)<.001);assert(Math.abs(s.desired.y+.015)<.001);
assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));
await page.screenshot({path:join(tmpdir(),'pid-targets-mobile.png'),fullPage:true});
assert.deepEqual(errors,[]);
console.log('PASS: point click/numeric/keyboard control, timed drawing, 3D path, live joint bars and units, target settling, pause/resume, final hold, reset/fall rewind, input validation, clear/mode switch, mobile map');
}finally{await browser.close()}
})().catch(e=>{console.error(e);process.exit(1)});
