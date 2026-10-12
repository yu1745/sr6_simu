"use client";
import { useEffect, useRef, useState } from 'react';
import { SCRIPT_AXES, loadScriptGroups, scriptPose, type ScriptBundle, type ScriptGroup, type AxisSetting } from '@/lib/sr6/funscript';
import type { StickSettings } from '@/lib/sr6/virtual-stick';
import type { Pose } from '@/lib/sr6/kinematics';

const timeLabel=(ms:number)=>`${Math.floor(ms/60000)}:${(ms/1000%60).toFixed(1).padStart(4,'0')}`;
export function FunscriptControls({onPose,ready,stopToken,stick,onStick}:{stick:StickSettings;onStick:(s:StickSettings)=>void;onPose:(p:Pose)=>boolean;ready:boolean;stopToken:number}){
  const [bundle,setBundle]=useState<ScriptBundle|null>(null);
  const [groups,setGroups]=useState<ScriptGroup[]>([]),[groupIndex,setGroupIndex]=useState(0);
  const [playing,setPlaying]=useState(false),[time,setTime]=useState(0),[rate,setRate]=useState(1);
  const [loop,setLoop]=useState(false),[loading,setLoading]=useState(false),[error,setError]=useState('');
  const [settings,setSettings]=useState<AxisSetting[]>(()=>SCRIPT_AXES.map(a=>({range:a.limit*2,reverse:false})));
  const cursor=useRef(0),active=useRef(false),request=useRef(0),callback=useRef(onPose);
  callback.current=onPose;
  function pause(){active.current=false;setPlaying(false);}
  useEffect(()=>{active.current=false;setPlaying(false);},[stopToken]);
  useEffect(()=>()=>{request.current++;active.current=false;},[]);
  function output(next:number,source=bundle,config=settings){
    cursor.current=next;setTime(next);
    if(!source)return false;
    const valid=callback.current(scriptPose(source,next,config));
    if(!valid){pause();setError('当前脚本位置不可达，已暂停。请减小轴行程或转角，或调整进度。');}
    else setError('');
    return valid;
  }
  async function load(files:File[]){
    if(!files.length)return;
    pause();setLoading(true);setError('');const id=++request.current;
    try{
      const nextGroups=await loadScriptGroups(files);
      if(id!==request.current)return;
      setGroups(nextGroups);setGroupIndex(0);setBundle(nextGroups[0]?.bundle??null);cursor.current=0;setTime(0);
      // Loading does not move the assembly; play or seek explicitly applies it.
    }catch(e){if(id===request.current)setError(e instanceof Error?e.message:String(e));}
    finally{if(id===request.current)setLoading(false);}
  }
  function start(){
    if(!bundle||!ready||loading)return;
    const next=cursor.current>=bundle.duration?0:cursor.current;
    if(output(next)&&bundle.duration>0){active.current=true;setPlaying(true);}
  }
  const togglePlayback=useRef(()=>{});
  togglePlayback.current=()=>{if(active.current)pause();else start();};
  useEffect(()=>{
    const keydown=(event:KeyboardEvent)=>{
      if(event.code!=='Space'||event.defaultPrevented||event.altKey||event.ctrlKey||event.metaKey||event.shiftKey||event.isComposing)return;
      const target=event.target;
      if(target instanceof HTMLElement&&(target.isContentEditable||target.closest('input,textarea,select,button,a,[role="button"],[role="slider"],[role="tab"],summary')))return;
      event.preventDefault();
      if(!event.repeat)togglePlayback.current();
    };
    window.addEventListener('keydown',keydown);
    return ()=>window.removeEventListener('keydown',keydown);
  },[]);
  useEffect(()=>{
    if(!playing||!bundle)return;
    let frame=0,last=performance.now();
    function tick(now:number){
      if(!active.current)return;
      const next=cursor.current+(now-last)*rate;last=now;
      const end=next>=bundle!.duration;
      const position=end?(loop?next%bundle!.duration:bundle!.duration):next;
      cursor.current=position;setTime(position);
      if(!callback.current(scriptPose(bundle!,position,settings))){
        active.current=false;setPlaying(false);setError('当前脚本位置不可达，已暂停。请减小轴行程或转角，或调整进度。');return;
      }
      if(end&&!loop){active.current=false;setPlaying(false);return;}
      frame=requestAnimationFrame(tick);
    }
    frame=requestAnimationFrame(tick);
    const visibility=()=>{if(document.hidden){active.current=false;setPlaying(false);}};
    document.addEventListener('visibilitychange',visibility);
    return ()=>{cancelAnimationFrame(frame);document.removeEventListener('visibilitychange',visibility);};
  },[playing,bundle,rate,loop,settings]);
  function changeSetting(index:number,patch:Partial<AxisSetting>){
    pause();const next=settings.map((s,i)=>i===index?{...s,...patch}:s);setSettings(next);
    if(bundle&&ready)output(cursor.current,bundle,next);
  }
  return <section className="script-controls" aria-label="Funscript 脚本控制">
    <div className="section-label"><span>Funscript 脚本</span><small>独立时间轴 · 线性插值</small></div>
    <label className="script-file">{loading?'正在解析…':'选择脚本（可多选）'}<input type="file" accept=".funscript,.json" multiple disabled={loading} onChange={e=>{void load(Array.from(e.target.files??[]));e.target.value='';}}/></label>
    {groups.length>0&&<div className="script-options"><label style={{minWidth:0,width:'100%'}}>脚本组<select aria-label="脚本组" style={{minWidth:0,flex:1}} value={groupIndex} disabled={loading} onChange={e=>{pause();const index=Number(e.target.value);setGroupIndex(index);setBundle(groups[index].bundle);cursor.current=0;setTime(0);setError('');}}>{groups.map((group,i)=><option key={group.name} value={i}>{i+1}. {group.name}</option>)}</select></label></div>}
    <div className="script-buttons"><button disabled={!bundle||!ready||loading} onClick={playing?pause:start}>{playing?'暂停':'播放'}</button><button disabled={!bundle||!ready||loading} onClick={()=>{pause();output(0);}}>回到开头</button></div>
    <input className="script-timeline" aria-label="脚本进度" type="range" min={0} max={bundle?.duration||1} step={1} value={time} disabled={!bundle||!ready||loading} onChange={e=>{pause();output(Number(e.target.value));}}/>
    <div className="script-clock"><span>{timeLabel(time)} / {timeLabel(bundle?.duration??0)}</span><span>{playing?'播放中':'已暂停'} · 空格切换</span></div>
    <div className="script-options"><label>倍速 <select aria-label="脚本倍速" value={rate} onChange={e=>setRate(Number(e.target.value))}>{[.25,.5,1,1.5,2].map(v=><option key={v} value={v}>{v}×</option>)}</select></label><label><input type="checkbox" checked={loop} onChange={e=>setLoop(e.target.checked)}/>循环</label></div>
    {bundle&&<ul className="script-tracks">{bundle.tracks.map((t,i)=>t&&<li key={i} title={t.source}><strong>{SCRIPT_AXES[i].id}</strong><span>{t.source}</span><small>{t.actions.length} 点</small></li>)}</ul>}
    <details open><summary>每轴 0→1 总行程 / 总转角</summary><p>以装配零位为中心，0.5 对应零位。例如100 mm代表 −50→+50 mm；0可禁用该轴，缺少的轴保持零位。</p>{SCRIPT_AXES.map((a,i)=>{
      const end=settings[i].range/2*(settings[i].reverse?-1:1);
      return <div key={a.id}><div className="script-axis"><span>{a.id} {a.label}</span><label><input type="number" aria-label={`${a.id} ${a.unit==='mm'?'总行程':'总转角'}`} min={0} step={1} value={settings[i].range} onChange={e=>{const v=e.target.valueAsNumber;if(Number.isFinite(v))changeSetting(i,{range:Math.max(0,v)});}}/>{a.unit}</label><label><input type="checkbox" checked={settings[i].reverse} onChange={e=>changeSetting(i,{reverse:e.target.checked})}/>反向</label></div><small className="script-range-preview">0 → {-end} {a.unit} · 0.5 → 0 · 1 → {end} {a.unit}</small></div>;
    })}
    <div className="script-axis"><span>虚拟第七轴</span><label><input aria-label="虚拟杆长" type="number" min={1} max={300} step={1} value={stick.length} onChange={e=>{const v=e.target.valueAsNumber;if(Number.isFinite(v)){pause();onStick({...stick,length:Math.max(1,Math.min(300,v))});}}}/>mm</label><label><input type="checkbox" checked={stick.enabled} onChange={e=>{pause();onStick({...stick,enabled:e.target.checked});}}/>启用</label></div>
    <p>虚拟杆长 1–300 mm。启用后目标代表杆尖，Roll / Pitch 由接收器平移补偿；可达边界会自动缩小补偿量。R0 独立。</p>
    </details>
    {error&&<p className="script-error" role="alert">{error}</p>}
    {bundle?.warnings.map((w,i)=><p className="script-error" key={i}>{w}</p>)}
  </section>;
}
