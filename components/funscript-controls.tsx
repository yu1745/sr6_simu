"use client";
import { useEffect, useRef, useState } from 'react';
import { SCRIPT_AXES, loadScriptGroups, scriptPose, type ScriptBundle, type ScriptGroup, type AxisSetting } from '@/lib/sr6/funscript';
import type { StickSettings } from '@/lib/sr6/virtual-stick';
import type { Pose } from '@/lib/sr6/kinematics';
import { GRAVITY_MPS2 } from '@/lib/sr6/statics';
import { useStoredSetting } from '@/lib/sr6/use-stored-setting';
import { SCRIPT_STORAGE_KEY, SCRIPT_POSITION_KEY, validScriptGroups } from '@/lib/sr6/script-storage';
import type { PlaybackState, ReferenceSample } from '@/lib/sr6/buffered-trajectory';

const validRate=(v:unknown):v is number=>typeof v==='number'&&[.25,.5,1,1.5,2].includes(v);
const validLoop=(v:unknown):v is boolean=>typeof v==='boolean';
const validSettings=(v:unknown):v is AxisSetting[]=>Array.isArray(v)&&v.length===SCRIPT_AXES.length&&v.every(s=>s&&typeof s.range==='number'&&Number.isFinite(s.range)&&s.range>=0&&typeof s.reverse==='boolean');
const defaultSettings=SCRIPT_AXES.map(a=>({range:a.limit*2,reverse:false}));

const timeLabel=(ms:number)=>`${Math.floor(ms/60000)}:${(ms/1000%60).toFixed(1).padStart(4,'0')}`;
export function FunscriptControls({onPose,onPause,onPlayback,ready,stopToken,stick,onStick,weightN,onWeight}:{stick:StickSettings;onStick:(s:StickSettings)=>void;weightN:number;onWeight:(weightN:number)=>void;onPose:(p:Pose,timeMs?:number,sample?:ReferenceSample)=>boolean;onPause:()=>void;onPlayback:(state:PlaybackState)=>void;ready:boolean;stopToken:number}){
  const [bundle,setBundle]=useState<ScriptBundle|null>(null);
  const [groups,setGroups]=useState<ScriptGroup[]>([]),[groupIndex,setGroupIndex]=useState(0);
  const [playing,setPlaying]=useState(false),[time,setTime]=useState(0);
  const [rate,setRate]=useStoredSetting('sr6.settings.v1.rate',1,validRate);
  const [loop,setLoop]=useStoredSetting('sr6.settings.v1.loop',false,validLoop);
  const [loading,setLoading]=useState(false),[error,setError]=useState('');
  const [settings,setSettings]=useStoredSetting('sr6.settings.v1.axes',defaultSettings,validSettings);
  const [storageError,setStorageError]=useState('');
  const cursor=useRef(0),active=useRef(false),request=useRef(0),callback=useRef(onPose);
  const stopCallback=useRef(onPause);
  const epoch=useRef(0),playbackCallback=useRef(onPlayback);
  playbackCallback.current=onPlayback;
  function publish(){playbackCallback.current({playing:active.current,timeMs:cursor.current,epoch:epoch.current});}
  callback.current=onPose;
  stopCallback.current=onPause;
  function pause(){active.current=false;setPlaying(false);publish();stopCallback.current();}
  useEffect(()=>{active.current=false;setPlaying(false);publish();stopCallback.current();},[stopToken]);
  useEffect(()=>()=>{request.current++;active.current=false;},[]);
  useEffect(()=>{
    try{
      const raw=localStorage.getItem(SCRIPT_STORAGE_KEY);
      if(!raw)return;
      const saved:unknown=JSON.parse(raw);
      if(!validScriptGroups(saved))throw Error('invalid cache');
      let index=0,position=0;
      try{
        const p=JSON.parse(localStorage.getItem(SCRIPT_POSITION_KEY)??'null');
        if(p&&Number.isInteger(p.groupIndex)&&p.groupIndex>=0&&p.groupIndex<saved.length){
          index=p.groupIndex;
          if(Number.isFinite(p.time))position=Math.max(0,Math.min(saved[index].bundle.duration,p.time));
        }
      }catch{/* A damaged cursor does not invalidate the scripts. */}
      setGroups(saved);setGroupIndex(index);setBundle(saved[index].bundle);
      cursor.current=position;setTime(position);
    }catch{setStorageError('本地脚本缓存读取失败，请重新选择脚本。');}
  },[]);
  useEffect(()=>{
    if(!groups.length)return;
    try{
      localStorage.setItem(SCRIPT_STORAGE_KEY,JSON.stringify(groups));
      setStorageError('');
    }catch{setStorageError('脚本已加载，但本地存储空间不足或不可用，刷新后可能恢复旧脚本。');}
  },[groups]);
  useEffect(()=>{
    if(!groups.length)return;
    const savePosition=()=>{
      try{localStorage.setItem(SCRIPT_POSITION_KEY,JSON.stringify({groupIndex,time:cursor.current}));}
      catch{setStorageError('无法保存脚本进度，浏览器本地存储不可用或已满。');}
    };
    savePosition();
    const timer=window.setInterval(savePosition,1000);
    window.addEventListener('pagehide',savePosition);
    return ()=>{window.clearInterval(timer);window.removeEventListener('pagehide',savePosition);};
  },[groups,groupIndex,playing]);
  function output(next:number,source=bundle,config=settings,seek=true){
    if(seek)epoch.current++;
    cursor.current=next;setTime(next);
    publish();
    if(!source)return false;
    const valid=callback.current(scriptPose(source,next,config,rate));
    if(!valid){pause();setError('当前脚本位置不可达，已暂停。请减小轴行程或转角，或调整进度。');}
    else setError('');
    return valid;
  }
  async function load(files:File[]){
    if(!files.length)return;
    pause();epoch.current++;publish();setLoading(true);setError('');const id=++request.current;
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
    if(output(next,bundle,settings,next!==cursor.current)&&bundle.duration>0){active.current=true;publish();setPlaying(true);}
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
      if(end&&loop){epoch.current++;stopCallback.current();} // No line across the loop boundary.
      publish();
      const sample:ReferenceSample={timeMs:position,epoch:epoch.current,
        poseAtOffset:seconds=>scriptPose(bundle!,position+seconds*1000*rate,settings,rate)};
      if(!callback.current(scriptPose(bundle!,position,settings,rate),now,sample)){
        pause();setError('当前脚本位置不可达，已暂停。请减小轴行程或转角，或调整进度。');return;
      }
      if(end&&!loop){pause();return;}
      frame=requestAnimationFrame(tick);
    }
    frame=requestAnimationFrame(tick);
    const visibility=()=>{if(document.hidden)pause();};
    document.addEventListener('visibilitychange',visibility);
    return ()=>{cancelAnimationFrame(frame);document.removeEventListener('visibilitychange',visibility);};
  },[playing,bundle,rate,loop,settings]);
  function changeSetting(index:number,patch:Partial<AxisSetting>){
    pause();const next=settings.map((s,i)=>i===index?{...s,...patch}:s);setSettings(next);
    if(bundle&&ready)output(cursor.current,bundle,next);
  }
  return <section className="script-controls" aria-label="Funscript 脚本控制">
    <div className="section-label"><span>Funscript 脚本</span><small>Buffered · TCode 0.4 七次轨迹</small></div>
    <label className="script-file">{loading?'正在解析…':'选择脚本（可多选）'}<input type="file" accept=".funscript,.json" multiple disabled={loading} onChange={e=>{void load(Array.from(e.target.files??[]));e.target.value='';}}/></label>
    {groups.length>0&&<div className="script-options"><label style={{minWidth:0,width:'100%'}}>脚本组<select aria-label="脚本组" style={{minWidth:0,flex:1}} value={groupIndex} disabled={loading} onChange={e=>{pause();const index=Number(e.target.value);setGroupIndex(index);setBundle(groups[index].bundle);cursor.current=0;epoch.current++;publish();setTime(0);setError('');}}>{groups.map((group,i)=><option key={group.name} value={i}>{i+1}. {group.name}</option>)}</select></label></div>}
    <div className="script-buttons"><button disabled={!bundle||!ready||loading} onClick={playing?pause:start}>{playing?'暂停':'播放'}</button><button disabled={!bundle||!ready||loading} onClick={()=>{pause();output(0);}}>回到开头</button></div>
    <input className="script-timeline" aria-label="脚本进度" type="range" min={0} max={bundle?.duration||1} step={1} value={time} disabled={!bundle||!ready||loading} onChange={e=>{pause();output(Number(e.target.value));}}/>
    <div className="script-clock"><span>{timeLabel(time)} / {timeLabel(bundle?.duration??0)}</span><span>{playing?'播放中':'已暂停'} · 空格切换</span></div>
    <div className="script-options"><label>倍速 <select aria-label="脚本倍速" value={rate} onChange={e=>{pause();epoch.current++;publish();setRate(Number(e.target.value));}}>{[.25,.5,1,1.5,2].map(v=><option key={v} value={v}>{v}×</option>)}</select></label><label><input type="checkbox" checked={loop} onChange={e=>setLoop(e.target.checked)}/>循环</label></div>
    {bundle&&<ul className="script-tracks">{bundle.tracks.map((t,i)=>t&&<li key={i} title={t.source}><strong>{SCRIPT_AXES[i].id}</strong><span>{t.source}</span><small>{t.actions.length} 点</small></li>)}</ul>}
    <details open><summary>每轴 0→1 总行程 / 总转角</summary><p>以装配零位为中心，0.5 对应零位。例如100 mm代表 −50→+50 mm；0可禁用该轴，缺少的轴保持零位。</p>{SCRIPT_AXES.map((a,i)=>{
      const end=settings[i].range/2*(settings[i].reverse?-1:1);
      return <div key={a.id}><div className="script-axis"><span>{a.id} {a.label}</span><label><input type="number" aria-label={`${a.id} ${a.unit==='mm'?'总行程':'总转角'}`} min={0} step={1} value={settings[i].range} onChange={e=>{const v=e.target.valueAsNumber;if(Number.isFinite(v))changeSetting(i,{range:Math.max(0,v)});}}/>{a.unit}</label><label><input type="checkbox" checked={settings[i].reverse} onChange={e=>changeSetting(i,{reverse:e.target.checked})}/>反向</label></div><small className="script-range-preview">0 → {-end} {a.unit} · 0.5 → 0 · 1 → {end} {a.unit}</small></div>;
    })}
    </details>
    <section className="compensation-calibration" aria-label="补偿标定">
      <div className="section-label"><span>补偿标定</span><small>杆长 / 额外负载</small></div>
      <div className="script-axis"><span>第七轴杆长</span><label><input aria-label="虚拟杆长" type="number" min={1} max={300} step={1} value={stick.length} onChange={e=>{const v=e.target.valueAsNumber;if(Number.isFinite(v)){pause();onStick({...stick,length:Math.max(1,Math.min(300,v))});}}}/>mm</label><label><input aria-label="启用第七轴补偿" type="checkbox" checked={stick.enabled} onChange={e=>{pause();onStick({...stick,enabled:e.target.checked});}}/>补偿</label></div>
      <div className="script-axis"><span>额外负载重力</span><label><input aria-label="额外负载重力" type="number" min={0} step={.1} value={weightN} onChange={e=>{const v=e.target.valueAsNumber;if(Number.isFinite(v)){pause();onWeight(Math.max(0,v));}}}/>N</label></div>
      <p className="calibration-mass">m = G / 9.80665 = <strong>{(weightN/GRAVITY_MPS2).toFixed(3)} kg</strong>，用于 F = ma。</p>
      <p className={stick.enabled?'compensation-enabled':'script-error'}><strong>第七轴补偿：{stick.enabled?'已开启':'未开启'}</strong>。{stick.enabled?'输入控制杆尖目标，接收器按杆长反向补偿。':'输入直接控制接收器，杆尖随平台倾斜移动。'}</p>
      <p>STEP 模式：负载为杆尖集中质量，虚拟杆无质量；机构材质、自重和转动惯量另外计入。负载设为 0 仍有机构自身力矩。电机标注为总扭矩(重力 + 惯量)，单位 N·m。</p>
      <p>杆长 1–300 mm。金色杆始终显示，杆尖红 X、绿 Y、蓝 Z 随 Roll / Pitch / R0 旋转。杆长开关仅控制杆尖补偿；可达边界会缩小补偿量。</p>
    </section>
    {error&&<p className="script-error" role="alert">{error}</p>}
    {storageError&&<p className="script-error" role="alert">{storageError}</p>}
    {bundle?.warnings.map((w,i)=><p className="script-error" key={i}>{w}</p>)}
  </section>;
}
