import type * as T from 'three';
import type { TorqueAnalysis } from './statics.ts';
import type { PlaybackState, ScriptSample } from './buffered-trajectory.ts';
import { createReactionHistory } from './reaction-history.ts';

/** Six synchronized script-time plots. No wall-clock animation while paused. */
export function createBaseReactionLabel(canvas:HTMLCanvasElement){
  const layer=document.createElement('section');layer.className='base-reaction-overlay';
  layer.setAttribute('aria-label','固定底座反力曲线');
  const heading=document.createElement('div');heading.className='base-reaction-heading';
  const title=document.createElement('strong');title.textContent='底座反力 / 反力矩';
  const status=document.createElement('small');heading.appendChild(title);heading.appendChild(status);layer.appendChild(heading);
  const grid=document.createElement('div');grid.className='base-reaction-grid';layer.appendChild(grid);
  const ns='http://www.w3.org/2000/svg';
  const charts=['Fx','Fy','Fz','Mx','My','Mz'].map((axis,i)=>{
    const cell=document.createElement('div');cell.className='base-reaction-chart';
    cell.setAttribute('aria-label',`${axis} ${i<3?'N':'N·m'} 曲线`);
    const label=document.createElement('div');label.className='base-reaction-chart-label';
    const name=document.createElement('strong');name.textContent=axis;
    const value=document.createElement('span');value.textContent='—';
    const unit=document.createElement('small');unit.textContent=i<3?'N':'N·m';[name,value,unit].forEach(e=>label.appendChild(e));
    const svg=document.createElementNS(ns,'svg');svg.setAttribute('viewBox','0 0 180 90');svg.setAttribute('preserveAspectRatio','none');
    const zero=document.createElementNS(ns,'line');zero.classList.add('reaction-zero');zero.setAttribute('x1','32');zero.setAttribute('x2','178');
    const path=document.createElementNS(ns,'path');path.classList.add('reaction-trace');
    const top=document.createElementNS(ns,'text');top.setAttribute('x','1');top.setAttribute('y','12');
    const bottom=document.createElementNS(ns,'text');bottom.setAttribute('x','1');bottom.setAttribute('y','70');
    const start=document.createElementNS(ns,'text');start.setAttribute('x','32');start.setAttribute('y','87');
    const end=document.createElementNS(ns,'text');end.setAttribute('x','178');end.setAttribute('y','87');end.setAttribute('text-anchor','end');
    [zero,path,top,bottom,start,end].forEach(e=>svg.appendChild(e));cell.appendChild(label);cell.appendChild(svg);grid.appendChild(cell);
    return {value,path,top,bottom,start,end,zero};
  });
  canvas.parentElement?.appendChild(layer);
  const history=createReactionHistory();let playback:PlaybackState={playing:false,timeMs:0,epoch:0};
  function draw(){
    const points=history.points,end=points.at(-1)?.timeMs??0,start=Math.max(0,end-history.windowMs),span=history.windowMs;
    layer.dataset.samples=String(points.length);layer.dataset.time=String(end);
    for(const [i,c] of charts.entries()){
      const values=points.map(p=>p.values[i]);let min=values.length?Math.min(...values):0,max=values.length?Math.max(...values):0;
      const pad=Math.max((max-min)*.12,i<3?.01:.001);min-=pad;max+=pad;
      const y=(v:number)=>70-(v-min)/(max-min)*64;
      const gridValue=min<=0&&max>=0?0:(min+max)/2;
      c.zero.setAttribute('y1',String(y(gridValue)));c.zero.setAttribute('y2',String(y(gridValue)));
      const digits=max-min<.1?3:max-min<1?2:1;
      c.top.textContent=max.toFixed(digits);c.bottom.textContent=min.toFixed(digits);
      c.start.textContent=`${(start/1000).toFixed(1)}s`;c.end.textContent=`${((start+span)/1000).toFixed(1)}s`;
      c.path.setAttribute('d',points.map((p,j)=>`${j?'L':'M'}${(32+(p.timeMs-start)/span*146).toFixed(2)},${y(p.values[i]).toFixed(2)}`).join(' '));
      const last=values.at(-1);c.value.textContent=last===undefined?'—':last.toFixed(2);
    }
  }
  draw();
  return {
    playback(state:PlaybackState){
      playback=state;if(history.sync(state))draw();
      const text=`${state.playing?'记录中':'已暂停'} · 最近 10 s · 各轴自适应 · 空格暂停`;
      if(status.textContent!==text)status.textContent=text;
      layer.dataset.playing=String(state.playing);
    },
    values(result:TorqueAnalysis,sample?:ScriptSample){
      const values=result.valid?result.baseReaction?.total:undefined;
      layer.classList.toggle('invalid',!values);
      if(values&&history.append(sample,values,playback))draw();
    },
    update(_anchor:T.Vector3,_camera:T.Camera,visible:boolean){layer.hidden=!visible;},
    dispose(){layer.remove();},
  };
}
