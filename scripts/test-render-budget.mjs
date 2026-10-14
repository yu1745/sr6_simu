import assert from 'node:assert/strict';
import {createRenderBudget} from '../lib/sr6/render-budget.ts';

for(const refreshRate of [60,120,144,240]){
  const budget=createRenderBudget(120);let draws=0,updates=0;
  for(let i=0;i<refreshRate*10;i++){
    budget.invalidate(); // Continuous playback/camera changes, including skipped ticks.
    if(budget.begin(i*1000/refreshRate,true)){updates++;if(budget.takeDraw())draws++;}
  }
  const expected=Math.min(refreshRate,120)*10;
  assert.ok(draws>=expected-1&&draws<=expected+1,`${refreshRate} Hz: ${draws} GPU frames`);
  assert.equal(updates,draws);
}
const idle=createRenderBudget();let draws=0;
for(let i=0;i<600;i++)if(idle.begin(i*1000/60,true)&&idle.takeDraw())draws++;
assert.equal(draws,1,'Stationary scene must not redraw');
idle.invalidate();
assert.equal(idle.begin(10001,false),false,'Hidden page must not update');
assert.equal(idle.begin(10050,true),true);
assert.equal(idle.takeDraw(),true,'Pending resize/worker/selection survives hidden page');
assert.equal(idle.takeDraw(),false);
idle.invalidate();
assert.equal(idle.begin(10051,true),false);
assert.equal(idle.begin(10100,true),true);
assert.equal(idle.takeDraw(),true,'Async worker result must survive rate limiting');
console.log('Render budget: up to 120 fps across 60–240 Hz, one idle draw, hidden-page suppression, pending invalidation passed.');
