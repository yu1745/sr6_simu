import { ZERO, type Pose } from './kinematics.ts';

// Format and aliases follow xtp-web/src/funscript/funscript.ts.
// Playback is local to the simulator and has no media or transport dependency.
export const SCRIPT_AXES = [
  {id:'L0',alias:'heave',key:'y',label:'升降',limit:60,unit:'mm'},
  {id:'L1',alias:'surge',key:'x',label:'前后',limit:30,unit:'mm'},
  {id:'L2',alias:'sway',key:'z',label:'侧移',limit:30,unit:'mm'},
  {id:'R0',alias:'twist',key:'twist',label:'Twist',limit:45,unit:'°'},
  {id:'R1',alias:'roll',key:'roll',label:'Roll',limit:25,unit:'°'},
  {id:'R2',alias:'pitch',key:'pitch',label:'Pitch',limit:25,unit:'°'},
] as const;
export type Action = {at:number;pos:number};
export type ScriptTrack = {actions:Action[];source:string};
export type ScriptBundle = {tracks:(ScriptTrack|null)[];duration:number;warnings:string[]};
export type ScriptGroup = {name:string;bundle:ScriptBundle};
// Total travel (mm) or total angular span (degrees) from normalized 0 to 1.
export type AxisSetting = {range:number;reverse:boolean};
const axisIndex = (name:string)=>SCRIPT_AXES.findIndex(a=>a.id.toLowerCase()===name.toLowerCase()||a.alias===name.toLowerCase());
function object(value:unknown): value is Record<string,unknown> {return !!value&&typeof value==='object'&&!Array.isArray(value);}

function actions(value:unknown, context:string, warnings:string[]): Action[] {
  if(!Array.isArray(value)) throw Error(`${context}：actions 必须是数组`);
  const result:Action[]=[];
  let clipped=0;
  for(const [i,point] of value.entries()) {
    if(!object(point)||typeof point.at!=='number'||!Number.isFinite(point.at)||point.at<0||
      typeof point.pos!=='number'||!Number.isFinite(point.pos)) throw Error(`${context}：第 ${i+1} 个动作无效`);
    if(point.pos<0||point.pos>100)clipped++;
    result.push({at:point.at,pos:Math.max(0,Math.min(100,point.pos))});
  }
  result.sort((a,b)=>a.at-b.at);
  const unique=result.filter((p,i)=>i===0||p.at!==result[i-1].at);
  if(unique.length<result.length)warnings.push(`${context}：重复时间保留第一个动作`);
  if(clipped)warnings.push(`${context}：${clipped} 个位置已限制到 0–100`);
  return unique;
}

export function parseScript(text:string, filename:string): ScriptBundle {
  let data:unknown;
  try{data=JSON.parse(text.replace(/^\uFEFF/,''));}catch{throw Error(`${filename}：JSON 格式错误`);}
  if(!object(data))throw Error(`${filename}：不是有效的 funscript`);
  const tracks:(ScriptTrack|null)[]=Array(6).fill(null), warnings:string[]=[];
  function assign(index:number,value:unknown){
    const points=actions(value,`${filename} / ${SCRIPT_AXES[index].id}`,warnings);
    if(points.length)tracks[index]={actions:points,source:filename};
  }
  if(Array.isArray(data.axes)){
    if(data.actions!==undefined)assign(0,data.actions);
    for(const axis of data.axes){
      if(!object(axis)||typeof axis.id!=='string')throw Error(`${filename}：轴定义无效`);
      const index=axisIndex(axis.id);
      if(index<0){warnings.push(`${filename}：忽略不支持的轴 ${axis.id}`);continue;}
      assign(index,axis.actions);
    }
  }else if(Array.isArray(data.actions)){
    const stem=filename.replace(/\.json$/i,'').replace(/\.funscript$/i,'');
    const suffix=stem.slice(stem.lastIndexOf('.')+1);
    const index=axisIndex(suffix);
    assign(index<0?0:index,data.actions);
  }else throw Error(`${filename}：缺少 actions 或 axes`);
  if(!tracks.some(Boolean))throw Error(`${filename}：没有可播放的六轴动作`);
  return {tracks,warnings,duration:Math.max(...tracks.map(t=>t?.actions.at(-1)?.at??0))};
}

// Only strip format extensions and recognized axis suffixes. All other text
// belongs to the group name; no version-number or title-specific heuristics.
function groupPrefix(filename:string):string{
  const stem=filename.replace(/\.json$/i,'').replace(/\.funscript$/i,'');
  const dot=stem.lastIndexOf('.');
  return dot>=0&&axisIndex(stem.slice(dot+1))>=0?stem.slice(0,dot):stem;
}
export async function loadScriptGroups(files:File[]):Promise<ScriptGroup[]>{
  const groups=new Map<string,File[]>();
  for(const file of files){
    const prefix=groupPrefix(file.name);
    const group=groups.get(prefix)??[];group.push(file);groups.set(prefix,group);
  }
  // Lexicographic prefix order puts a shorter shared prefix before its extension.
  const names=[...groups.keys()].sort((a,b)=>a<b?-1:a>b?1:0);
  return Promise.all(names.map(async name=>({name,bundle:await mergeGroup(groups.get(name)!)})));
}
export async function loadScripts(files:File[]):Promise<ScriptBundle>{
  const groups=await loadScriptGroups(files);
  if(!groups.length)throw Error('未选择脚本');
  return groups[0].bundle;
}
async function mergeGroup(files:File[]):Promise<ScriptBundle>{
  const parsed=await Promise.all(files.map(async f=>parseScript(await f.text(),f.name)));
  const tracks:(ScriptTrack|null)[]=Array(6).fill(null),warnings:string[]=[];
  for(const bundle of parsed){
    warnings.push(...bundle.warnings);
    bundle.tracks.forEach((track,i)=>{
      if(!track)return;
      if(tracks[i]){
        warnings.push(`${SCRIPT_AXES[i].id} 已去重：保留 ${tracks[i]!.source}，忽略 ${track.source}`);
        return;
      }
      tracks[i]=track;
    });
  }
  return {tracks,warnings,duration:Math.max(0,...tracks.map(t=>t?.actions.at(-1)?.at??0))};
}

// Binary search allows seeking backwards and avoids rescanning long scripts.
// Before a delayed first action, hold its position; after the last, hold the end.
export function sampleTrack(points:Action[],time:number):number{
  if(!points.length)return 50;
  if(time<=points[0].at)return points[0].pos;
  if(time>=points[points.length-1].at)return points[points.length-1].pos;
  let lo=0,hi=points.length-1;
  while(hi-lo>1){const mid=(lo+hi)>>1;if(points[mid].at<=time)lo=mid;else hi=mid;}
  const a=points[lo],b=points[hi];
  return a.pos+(b.pos-a.pos)*(time-a.at)/(b.at-a.at);
}
export function scriptPose(bundle:ScriptBundle,time:number,settings:AxisSetting[]):Pose{
  const pose={...ZERO};
  SCRIPT_AXES.forEach((axis,i)=>{
    const track=bundle.tracks[i];
    if(!track)return;
    const unit=axis.unit==='°'?Math.PI/180:1;
    pose[axis.key]=(sampleTrack(track.actions,time)/100-.5)*settings[i].range*unit*(settings[i].reverse?-1:1);
  });
  return pose;
}
