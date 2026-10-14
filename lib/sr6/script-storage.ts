import type { ScriptGroup } from './funscript.ts';

export const SCRIPT_STORAGE_KEY='sr6.scripts.v1';
export const SCRIPT_POSITION_KEY='sr6.scripts.position.v1';

// Reject damaged or incompatible cache data before passing it to playback.
export function validScriptGroups(value:unknown):value is ScriptGroup[]{
  if(!Array.isArray(value)||!value.length)return false;
  return value.every(group=>{
    const b=group?.bundle;
    if(typeof group?.name!=='string'||!b||!Number.isFinite(b.duration)||b.duration<0||
      !Array.isArray(b.warnings)||!b.warnings.every((w:unknown)=>typeof w==='string')||
      !Array.isArray(b.tracks)||b.tracks.length!==6)return false;
    let duration=0,hasTrack=false;
    for(const track of b.tracks){
      if(track===null)continue;
      if(!track||typeof track.source!=='string'||!Array.isArray(track.actions)||!track.actions.length)return false;
      hasTrack=true;let previous=-1;
      for(const p of track.actions){
        if(!p||!Number.isFinite(p.at)||p.at<0||p.at<=previous||!Number.isFinite(p.pos)||p.pos<0||p.pos>100)return false;
        previous=p.at;
      }
      duration=Math.max(duration,previous);
    }
    return hasTrack&&duration===b.duration;
  });
}
