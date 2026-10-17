import type { PlaybackState, ScriptSample } from './buffered-trajectory.ts';
export type ReactionPoint={timeMs:number;values:readonly number[]};
export function createReactionHistory(windowMs=10000){
  let epoch=-1,points:ReactionPoint[]=[];
  return {
    sync(state:PlaybackState){if(state.epoch!==epoch){epoch=state.epoch;points=[];return true;}return false;},
    append(sample:ScriptSample|undefined,values:readonly number[],state:PlaybackState){
      if(!sample||!state.playing||sample.epoch!==state.epoch||sample.epoch!==epoch||!values.every(Number.isFinite))return false;
      const last=points.at(-1);
      if(last&&sample.timeMs<=last.timeMs)return false;
      points.push({timeMs:sample.timeMs,values:[...values]});
      const cutoff=sample.timeMs-windowMs;
      while(points.length>1&&points[1].timeMs<cutoff)points.shift();
      if(points.length>1500)points.splice(0,points.length-1500);
      return true;
    },
    get points(){return points;},windowMs,
  };
}
