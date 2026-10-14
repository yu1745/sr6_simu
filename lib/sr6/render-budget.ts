/** Poll external motion at a bounded rate; submit GPU work only when invalidated. */
export function createRenderBudget(fps=120){
  const interval=1000/fps;
  let last=-Infinity,dirty=true;
  return {
    invalidate(){dirty=true;},
    begin(now:number,visible:boolean){
      if(!visible)return false;
      const elapsed=now-last;
      if(elapsed<interval-0.01)return false;
      last=Number.isFinite(last)?last+Math.max(1,Math.floor((elapsed+0.01)/interval))*interval:now;
      return true;
    },
    takeDraw(){const draw=dirty;dirty=false;return draw;},
  };
}
