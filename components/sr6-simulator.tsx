"use client";
import { useEffect, useRef, useState } from 'react';
import { Box, RotateCcw, Maximize2, TriangleAlert } from 'lucide-react';
import { Tabs, TabsList, TabsTrigger, TabsContent } from '@/components/ui/tabs';
import { ZERO, NAMES, inverse, forward, evaluate, type Pose } from '@/lib/sr6/kinematics';
import { geometry, shaftClearance } from '@/lib/sr6/geometry';
import { mountScene, type SceneState } from '@/lib/sr6/scene';
import { type CadStatus } from '@/lib/sr6/cad-rig';
import { type CadKinematics } from '@/lib/sr6/cad-kinematics';
import { FunscriptControls } from './funscript-controls';
import type { CollisionStatus } from '@/lib/sr6/collision';

import { DEFAULT_STICK, solveStick, tipFromReceiver, type StickSettings } from '@/lib/sr6/virtual-stick';

const rad=Math.PI/180;
const definitions: {key:keyof Pose;name:string;unit:string;factor:number;limit:number}[] = [
  {key:'x',name:'Fwd',unit:'mm',factor:1,limit:30},
  {key:'y',name:'Thrust',unit:'mm',factor:1,limit:60},
  {key:'z',name:'Side',unit:'mm',factor:1,limit:30},
  {key:'roll',name:'Roll',unit:'°',factor:rad,limit:25},
  {key:'pitch',name:'Pitch',unit:'°',factor:rad,limit:25},
  {key:'twist',name:'R0 / Twist',unit:'°',factor:rad,limit:45},
];
function Field({name,value,unit,onChange,min,max}:{name:string;value:number;unit:string;onChange:(v:number)=>void;min?:number;max?:number}) {
  return <label className="parameter-field"><span>{name}</span><div><input aria-label={name} type="number" step="0.5" min={min} max={max} value={Number(value.toFixed(4))} onChange={e=>{if(e.target.value!==''&&Number.isFinite(e.target.valueAsNumber))onChange(e.target.valueAsNumber);}}/><small>{unit}</small></div></label>;
}
export function Sr6Simulator(){
  const canvas=useRef<HTMLCanvasElement>(null);
  const [pose,setPose]=useState<Pose>({...ZERO});
  const [stick,setStick]=useState<StickSettings>({...DEFAULT_STICK});
  const [stickScale,setStickScale]=useState(1);
  const lastTarget=useRef<Pose>({...ZERO});
  const [q,setQ]=useState<number[]>(()=>Array(6).fill(0));
  const cadSolver=useRef<CadKinematics|null>(null);
  const [cadReady,setCadReady]=useState(false);
  const [scriptStopToken,setScriptStopToken]=useState(0);
  const [mode,setMode]=useState('pose');
  const [view,setView]=useState('cad');
  const [cadMessage,setCadMessage]=useState('CAD 正在加载');
  const [cadStatus,setCadStatus]=useState<CadStatus|null>(null);
  const [collision,setCollision]=useState<CollisionStatus|null>(null);
  const [error,setError]=useState('');
  const [ms,setMs]=useState<number|null>(null);
  const [iterations,setIterations]=useState(0);
  const lastGood=useRef<SceneState>({pose:{...ZERO},q:Array(6).fill(0),cad:true});
  const sceneState=useRef<SceneState>(lastGood.current);
  const valid=error==='';
  sceneState.current={...lastGood.current,cad:view==='cad'};
  const [renderError,setRenderError]=useState('');
  const residual=view==='cad' ? (cadSolver.current?.evaluate(lastGood.current.pose,lastGood.current.q) ?? {max:Infinity}) : evaluate(lastGood.current.pose,lastGood.current.q);
  const clearance=shaftClearance(geometry(lastGood.current.pose,lastGood.current.q));

  useEffect(()=>{
    if(!canvas.current)return;
    try{return mountScene(canvas.current,()=>sceneState.current,setCadMessage,setCadStatus,solver=>{cadSolver.current=solver;setCadReady(true);},setCollision);}
    catch(e){setRenderError(e instanceof Error?e.message:String(e));}
  },[mountScene]);
  function applyPose(next:Pose,fromScript=false,config=stick):boolean{
    if(!fromScript)setScriptStopToken(v=>v+1);
    if(view==='cad'&&!cadSolver.current){setError('CAD 正在加载');return false;}
    setPose(next);const start=performance.now();const result=solveStick(next,config,p=>view==='cad'?cadSolver.current!.inverse(p,lastGood.current.q):inverse(p));
    setMs(performance.now()-start);setIterations(0);
    // Script ranges are explicitly configured per axis; CAD reachability still
    // decides whether the requested target can be applied.
    const inLimits=fromScript||definitions.every(d=>Math.abs(next[d.key]/d.factor)<=d.limit+1e-9);
    const message=!inLimits?'超出当前输入范围':result.reason;
    setError(message);
    if(!message){setQ(result.q);setStickScale(result.scale);lastTarget.current=next;lastGood.current={pose:result.receiver,q:result.q,cad:view==='cad'};}
    return !message;
  }
  function applyAngles(next:number[],twist=pose.twist){
    setScriptStopToken(v=>v+1);
    if(view==='cad'&&!cadSolver.current){setError('CAD 正在加载');return;}
    setQ(next);const start=performance.now();const result=view==='cad'?cadSolver.current!.forward(next,twist,lastGood.current.pose):forward(next,twist,lastGood.current.pose);
    setMs(performance.now()-start);setIterations(result.iterations);const target=tipFromReceiver(result.pose,stick.enabled?stick.length:0);setPose(target);
    const inLimits=definitions.every(d=>Math.abs(result.pose[d.key]/d.factor)<=d.limit+1e-9);
    const message=!result.converged?'电机角度不满足闭环，正解未收敛':!inLimits?'正解超出当前输入范围':'';
    setError(message);
    if(!message){setStickScale(1);lastTarget.current=target;lastGood.current={pose:result.pose,q:next,cad:view==='cad'};}
  }
  function changeView(value:string){
    setScriptStopToken(v=>v+1);
    setView(value);setMode('pose');setPose({...ZERO});lastTarget.current={...ZERO};setStickScale(1);setError('');setMs(null);setIterations(0);
    const next=value==='cad'?Array(6).fill(0):inverse(ZERO).q;
    setQ(next);lastGood.current={pose:{...ZERO},q:next,cad:value==='cad'};
  }
  function reset(){changeView(view);}
  function changeMode(value:string){
    setScriptStopToken(v=>v+1);
    setMode(value);setPose(lastTarget.current);setQ(lastGood.current.q);setError('');
  }
  async function fullscreen(){try{if(document.fullscreenElement)await document.exitFullscreen();else await document.documentElement.requestFullscreen();}catch{setRenderError('当前浏览器不允许全屏，可继续使用窗口模式。');}}
  useEffect(()=>{
    type Registry={registerTool:(tool:{name:string;description:string;inputSchema:object;annotations:object;execute:()=>unknown},options:{signal:AbortSignal})=>void|Promise<void>};
    const context=(document as Document & {modelContext?:Registry}).modelContext;
    if(!context?.registerTool)return;
    const lifecycle=new AbortController();
    try{void Promise.resolve(context.registerTool({
      name:'read_sr6_state',description:'Read the currently displayed valid SR6 pose (mm/rad), joint angles (rad), and closure residual. Collision calibration remains incomplete.',
      inputSchema:{type:'object',properties:{},additionalProperties:false},annotations:{readOnlyHint:true},
      execute:()=>({pose:lastGood.current.pose,joints:lastGood.current.q,closure:sceneState.current.cad?cadSolver.current?.evaluate(lastGood.current.pose,lastGood.current.q).max:evaluate(lastGood.current.pose,lastGood.current.q).max,model:sceneState.current.cad?'measured CAD':'firmware',wholeMachineCollision:'uncalibrated'}),
    },{signal:lifecycle.signal})).catch(()=>{});}catch{/* Optional browser capability. */}
    return ()=>lifecycle.abort();
  },[]);

  return <main className="sim-shell">
    <header className="sim-header"><div className="brand"><span className="brand-mark"><Box size={20}/></span><div><strong>SR6 SIMU</strong><small>0909 · 机构运动学工作台</small></div></div><div className="header-state">主线程求解 <span className="divider"/>{ms===null?'等待输入':`${ms.toFixed(3)} ms`}</div><button className="icon-button" onClick={fullscreen} aria-label="切换全屏"><Maximize2 size={18}/></button></header>
    <section className="workspace">
      <aside className="control-panel">
        <div className="panel-heading"><div><span>输入</span><strong>机构控制</strong></div></div>
        <FunscriptControls stick={stick} onStick={next=>{setStick(next);setMode('pose');applyPose(pose,true,next);setScriptStopToken(v=>v+1);}} ready={view!=='cad'||cadReady} stopToken={scriptStopToken} onPose={next=>{setMode('pose');return applyPose(next,true);}}/>
        <Tabs value={mode} onValueChange={changeMode}>
          <TabsList className="segmented"><TabsTrigger value="pose">位姿 → 逆解</TabsTrigger><TabsTrigger value="motors">电机 → 正解</TabsTrigger></TabsList>
          <TabsContent value="pose"><div className="control-section"><div className="section-label"><span>{stick.enabled?'虚拟杆尖目标':'目标位姿'}</span><small>{view==='cad'?'相对当前装配零位':'相对固件零位'}</small></div><div className="field-grid">{definitions.map(d=><Field key={d.key} name={d.name} unit={d.unit} value={pose[d.key]/d.factor} min={-d.limit} max={d.limit} onChange={v=>applyPose({...pose,[d.key]:v*d.factor})}/>)}</div></div></TabsContent>
          <TabsContent value="motors"><div className="control-section"><div className="section-label"><span>{view==='cad'?'实际摇臂角增量':'逻辑关节角'}</span><small>{view==='cad'?'相对装配零位':'非驱动器原始位置'}</small></div><div className="field-grid">{q.map((angle,i)=><Field key={i} name={`M${i+1}`} unit="°" value={angle/rad} onChange={v=>applyAngles(q.map((x,j)=>j===i?v*rad:x))}/>)}<Field name="R0 / Twist" unit="°" value={pose.twist/rad} onChange={v=>applyAngles(q,v*rad)}/></div></div></TabsContent>
        </Tabs>
        <button className="reset-button" onClick={reset}><RotateCcw size={16}/>{view==='cad'?'恢复装配零位':'恢复固件零位'}</button>
        <div role="status" className={valid?'solver-status':'solver-status invalid'}>{view==='cad'&&!cadReady?'CAD 正在加载':valid?'闭环满足 · 当前位姿有效':`${error}；保持上一有效画面`}</div>
        {stick.enabled&&<div className="inspector-note"><span>虚拟杆补偿 · {(stickScale*100).toFixed(1)}%</span><p>实际接收器 Fwd {lastGood.current.pose.x.toFixed(2)} / Thrust {lastGood.current.pose.y.toFixed(2)} / Side {lastGood.current.pose.z.toFixed(2)} mm。{stickScale<1?'可达边界已缩小补偿，杆尖会偏离目标。':'按完整杆长补偿。'}</p></div>}
        <section className="joint-list"><div className="section-label"><span>{mode==='pose'?'逆解输出':'电机输入'}</span><small>°</small></div>{q.map((angle,i)=><div className="joint-row" key={i}><span>M{i+1}</span><span>{NAMES[i]}</span><strong>{(angle/rad).toFixed(3)}</strong></div>)}</section>
        {mode==='motors'&&<section className="control-section"><div className="section-label"><span>正解结果</span></div>{definitions.map(d=><div className="readout" key={d.key}><span>{d.name}</span><strong>{(pose[d.key]/d.factor).toFixed(3)} {d.unit}</strong></div>)}</section>}
      </aside>
      <section className="viewport"><canvas ref={canvas}/><div className="viewport-label"><Tabs value={view} onValueChange={changeView}><TabsList className="segmented"><TabsTrigger value="live">固件运动骨架</TabsTrigger><TabsTrigger value="cad">STEP 运动装配</TabsTrigger></TabsList></Tabs><small>{view==='cad'?cadMessage:'连杆位置由 0909 正逆解驱动'}</small><small>左键旋转 · 右键平移 · 滚轮缩放</small></div>{renderError&&<p className="render-error" role="alert">{renderError}</p>}<div className="axis-legend">{view==='cad'?(cadStatus&&!cadStatus.valid?cadStatus.reason:'光轴 Ø10 × 210 · 球心距 258 · 叠放间距 14 mm'):'固件模型：主臂 105 · 连杆 262 · 光轴 220 mm'}</div></section>
      <aside className="inspector"><div className="panel-heading"><div><span>检查</span><strong>闭环与间隙</strong></div></div>
        <div className="metric"><span>{view==='cad'?'实际装配闭环最大残差':'固件闭环最大残差'}</span><strong>{Number.isFinite(residual.max)?residual.max.toFixed(5):'—'} mm</strong><small>有效阈值 0.001 mm</small></div>
        {view==='cad'&&<div className="metric"><span>CAD 实测连杆闭环</span><strong>{cadStatus?.valid?`${cadStatus.residual.toFixed(5)} mm`:cadStatus?'不可达':'加载中'}</strong><small>按真实安装点独立求解，零件不缩放</small></div>}
        <div className="metric"><span>求解耗时</span><strong>{ms===null?'—':ms.toFixed(3)} ms</strong><small>{mode==='pose'?'闭式逆解':`阻尼 Gauss–Newton · ${iterations} 次迭代`}</small></div>
        {view==='live'&&<div className="metric"><span>固件骨架光轴包络间隙</span><strong>{clearance.min.toFixed(2)} mm</strong><small>旧尺寸胶囊近似，不适用于新 CAD 装配</small></div>}
        {view==='cad'&&<section className="collision-panel" aria-label="模型干涉检测"><div className="section-label"><span>模型干涉检测</span><small>{collision?.pending?`${collision.workers??1} 线程计算中`:collision?.ready?`${collision.elapsed.toFixed(0)} ms`:''}</small></div><strong className={collision?.pairs.length?'collision-hit':''}>{collision?.error?'检测不可用':!collision?.ready?'正在构建检测网格':collision.displayedRevision===undefined?'等待首次结果':collision.pairs.length?`${collision.pairs.length} 对零件干涉`:'本次采样未发现干涉'}</strong>{collision?.error&&<p>{collision.error}</p>}{collision?.details?.map((detail,i)=><p key={i}>{detail}</p>)}<p>按指定范围检查 {collision?.partCount??15} 个零件：接收器主体、上端鱼眼及俯仰二级摇臂。</p><p>红色：两零件布尔交集区域；特征边与视角轮廓：涉及零件（隐藏三角剖分线）。干涉消失后自动恢复。</p>{!!collision?.names.length&&<ul>{collision.names.map((name,i)=><li key={i}>{name}</li>)}</ul>}</section>}
        <div className="warning-card"><TriangleAlert size={20}/><div><strong>网格采样检测</strong><p>忽略同一刚性组件内部及已知连接配合，局部埋入补查容差0.03 mm。装配姿态与干涉结果同步更新，计算期间保持上一已检测画面，运动显示可能略滞后于脚本；高速运动、细小穿透和源模型缺陷可能漏检，不代表全行程无干涉。</p></div></div>
        <div className="inspector-note"><span>坐标与模型</span><strong>Fwd / Thrust / Side</strong><p>STEP模式按实测尺寸进行正逆解，M1–M6为相对装配零位的实际摇臂角。Fwd为前后、Thrust为升降、Side为侧移；Roll绕前后轴、Pitch绕横向轴，R0独立转动输出夹具。六个摇臂角需要满足同一接收器的闭合约束。切换模型时回到各自零位。</p></div>
      </aside>
    </section>
  </main>;
}
