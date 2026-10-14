"use client";
import type { MassProperties } from '@/lib/sr6/mass-properties';

export function MaterialInspector({data,selected,onSelect,compact=false}:{data:MassProperties;selected:number|null;onSelect:(id:number)=>void;compact?:boolean}){
  const part=data.parts.find(p=>p.id===selected);
  return <section className={compact?'material-selection':'material-inspector'} aria-label={compact?'选中零件材质':'材质与惯量'}>
    {!compact&&<>
      <div className="section-label"><span>材质与惯量</span><small>{data.includedCount} 件计入</small></div>
      <div className="material-legend">{Object.entries(data.materials).filter(([id])=>id!=='unknown').map(([id,m])=><span key={id}><i style={{background:m.color}}/>{m.name}</span>)}</div>
      <p>机构运动质量 <strong>{data.includedMassKg.toFixed(3)} kg</strong>，另加杆尖负载。点击模型或下方零件查看。</p>
    </>}
    {part&&<div className="material-part">
      <strong><i style={{background:data.materials[part.material].color}}/>#{part.id} {part.label}</strong>
      <span>{data.materials[part.material].name} · {part.included?'计入动力学':part.reason}</span>
      {!compact&&data.materials[part.material].densityKgM3!==null&&<span>密度 {data.materials[part.material].densityKgM3} kg/m³</span>}
      {part.massKg!==null&&<span>质量 {(part.massKg*1000).toFixed(3)} g</span>}
      {part.principalKgM2&&<>
        <span>重心主惯量 I₁ / I₂ / I₃ · kg·m²</span>
        <code>{part.principalKgM2.map(v=>v.toExponential(3)).join(' / ')}</code>
      </>}
      {!compact&&<>
        <span>重心 STEP XYZ · mm</span><code>{part.centerMm.map(v=>v.toFixed(3)).join(' / ')}</code>
        <span>{part.method}</span>
        {part.inertiaCentroidKgM2&&<details><summary>重心惯量张量 · STEP XYZ · kg·m²</summary><pre>{part.inertiaCentroidKgM2.map(row=>row.map(v=>v.toExponential(3).padStart(11)).join(' ')).join('\n')}</pre></details>}
      </>}
    </div>}
    {!compact&&<>
      <details className="material-parts"><summary>全部 {data.parts.length} 个零件</summary>{data.parts.map(p=><button key={p.id} aria-pressed={p.id===selected} onClick={()=>onSelect(p.id)}><i style={{background:data.materials[p.material].color}}/><span>#{p.id} {p.label}</span><small>{p.included?`${(p.massKg!*1000).toFixed(1)} g`:p.motion==='fixed'?'固定':'总成内'}</small></button>)}</details>
      <p>惯量取 STEP 实体体积积分；R0 电机采用 388 g 实测质量和圆柱外形近似。虚拟杆无质量，负载视为杆尖集中质量。未计电机内部转子相对壳体的自转。</p>
      <a href="/sr6-mass-properties.json" download>下载完整质量与惯量数据</a>
    </>}
  </section>;
}
