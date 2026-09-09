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
 body=body.replace('return { tick, get active() { return active; }, get center() { return pivot; } };', `window.__pidTest={tick, reset, reference:q=>{CHALLENGE_JOINTS.splice(0,7,...q);selectTab('joint');selectTab('pid')}, tilt:acceptTilt, read:()=>({state:{...state},active,running,falls,pivot:pivot.toArray(),board:board.position.toArray(),quaternion:board.quaternion.toArray(),q:joints.map(j=>j.angle),actualCenter:tool.getWorldPosition(new Vector3()).sub(graspOffset.clone().applyQuaternion(tool.getWorldQuaternion(new Quaternion()).multiply(graspRotation.clone().invert()))).toArray()})}; return { tick, get active() { return active; }, get center() { return pivot; } };`);
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
const corners=await page.evaluate(()=>{
 let result=[];
 for(const r of [-25,0,25])for(const p of [-25,0,25]){
 window.__pidTest.reset();
 let ok=true; for(let k=1;k<=25;k++) if(!window.__pidTest.tilt(r*k/25*Math.PI/180,p*k/25*Math.PI/180)){ok=false;break;}
 const s=window.__pidTest.read();
 result.push({r,p,ok,error:Math.hypot(...s.actualCenter.map((v,i)=>v-s.pivot[i]))});
 }
 window.__pidTest.reset(); return result;
});
console.log('tilt range',corners);
const sweep=await page.evaluate(()=>{
  window.__pidTest.reset();let previous=[0,0],worst=0;
  for(let row=0;row<=10;row++)for(let col=0;col<=10;col++){
    const r=-25+row*5,p=-25+(row%2?10-col:col)*5;
    const steps=Math.max(1,Math.abs(r-previous[0]),Math.abs(p-previous[1]));
    for(let k=1;k<=steps;k++){
      if(!window.__pidTest.tilt((previous[0]+(r-previous[0])*k/steps)*Math.PI/180,(previous[1]+(p-previous[1])*k/steps)*Math.PI/180))return {ok:false,r,p};
      const s=window.__pidTest.read();worst=Math.max(worst,Math.hypot(...s.actualCenter.map((v,i)=>v-s.pivot[i])));
    }
    previous=[r,p];
  }
  window.__pidTest.reset();return {ok:true,worst};
});
console.log('continuous tilt sweep',sweep);assert(sweep.ok);assert(sweep.worst<.0001);
assert(corners.every(c=>c.ok && c.error<.0001));
await page.click('#pid-start');
s=await page.evaluate(()=>{for(let i=0;i<200;i++)window.__pidTest.tick(.05);return window.__pidTest.read()});
console.log('after 10 seconds',s.state,s.falls);
assert.equal(s.falls,0);assert(s.running);assert(Math.hypot(s.state.x,s.state.y)<.003);
assert(Math.hypot(...s.actualCenter.map((v,i)=>v-s.pivot[i]))<.0001);
await page.click('#pid-push');
s=await page.evaluate(()=>{for(let i=0;i<120;i++)window.__pidTest.tick(.05);return window.__pidTest.read()});
assert.equal(s.falls,0);assert(Math.hypot(s.state.x,s.state.y)<.003);
await page.click('#pid-start');
const paused=await page.evaluate(()=>window.__pidTest.read().state);
await page.waitForTimeout(200);
assert.deepEqual(await page.evaluate(()=>window.__pidTest.read().state),paused);
// Zero gains must permit the ball to roll off. The reset retains these gains.
for(const input of await page.locator('.pid-gains input').all()){await input.fill('0');await input.dispatchEvent('change');}
await page.click('#pid-reset');await page.click('#pid-push');
s=await page.evaluate(()=>{for(let i=0;i<60;i++){window.__pidTest.tick(.05);if(window.__pidTest.read().falls)break;}return window.__pidTest.read()});
assert.equal(s.falls,1);assert.deepEqual(s.q,[-.437,-1.002,.583,-2.362,-.958,3.009,.764]);
assert.equal(s.state.x,.025);assert.equal(s.state.y,-.02);assert.equal(s.state.roll,0);assert.equal(s.state.pitch,0);
assert((await page.locator('#pid-status').textContent()).includes('Ball fell'));
await page.locator('#pid-roll-kp').fill('-1');await page.locator('#pid-roll-kp').dispatchEvent('change');
assert.equal(await page.locator('#pid-status').getAttribute('data-error'),'true');
assert.equal((await page.evaluate(()=>window.__pidTest.read())).running,false);
await page.locator('#pid-roll-kp').fill('2');await page.locator('#pid-roll-kp').dispatchEvent('change');
await page.locator('.sidebar').evaluate(e=>e.scrollTop=0);
await page.screenshot({path:join(tmpdir(),'pid-game-desktop.png'),fullPage:true});
await page.click('#cartesian-tab');
assert.deepEqual(await q(),initial);
assert.equal(await page.locator('#home').isDisabled(),false);
assert.equal(await page.locator('#cartesian-panel').isVisible(),true);
await page.locator('#cartesian-tab').press('ArrowRight');
assert.equal(await page.locator('#pid-tab').getAttribute('aria-selected'),'true');
await page.locator('#pid-tab').press('ArrowRight');
assert.equal(await page.locator('#joint-tab').getAttribute('aria-selected'),'true');
await page.click('#pid-tab');
await page.setViewportSize({width:390,height:844});
await page.locator('#pid-push').scrollIntoViewIfNeeded();
await page.click('#pid-push');
const mobileTime=await page.evaluate(()=>window.__pidTest.read().state.time);
await page.waitForTimeout(250);
assert((await page.evaluate(()=>window.__pidTest.read().state.time))>mobileTime);
await page.click('#pid-start');
await page.locator('.sidebar').evaluate(e=>e.scrollTop=0);
await page.screenshot({path:join(tmpdir(),'pid-game-mobile.png'),fullPage:true});
assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));
assert.deepEqual(errors,[]);
console.log('PASS: PID tab, screenshot joints, all ±25° corner tilts, fixed center, settling, push, pause, fall reset, gain validation, restoring prior pose, keyboard tabs, mobile layout');
}finally{await browser.close()}
})().catch(e=>{console.error(e);process.exit(1)});
