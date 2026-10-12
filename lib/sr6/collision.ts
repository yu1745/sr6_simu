import * as T from 'three';
import { CAD_LINKS, type CadData } from './cad-rig.ts';
import { collisionParts as selectCollisionParts } from './collision-scope.ts';
import collisionWorkerUrl from './collision-worker.ts?worker&url';
export type CollisionStatus={ready:boolean;pending:boolean;pairs:[number,number][];names:string[];elapsed:number;workers?:number;partCount?:number;error?:string;details?:string[]};
export type CollisionRequest=
  |{type:'init';shard:number;shardCount:number;parts:{id:number;positions:number[];indices:number[];group:string}[];exclusions:[number,number][]}
  |{type:'check';revision:number;matrices:{id:number;matrix:number[]}[]};
export type CollisionResult=
  |{type:'ready'}|{type:'error';message:string}
  |{type:'result';revision:number;elapsed:number;pairs:[number,number][];intersections:{a:number;b:number;positions:number[]}[];warnings:string[]};

export function createCollisionDisplay(data:CadData,meshes:Map<number,T.Mesh>,onStatus:(s:CollisionStatus)=>void){
  // Use Vite's emitted URL directly. Vinext can rewrite import.meta.url to a
  // build-time file:/// URL, which is not a valid worker origin in the browser.
  // Use the browser-reported logical CPU budget; leave one for UI/rendering.
  const workerCount=Math.max(1,Math.floor(navigator.hardwareConcurrency||2)-1);
  const workers=Array.from({length:workerCount},()=>new Worker(collisionWorkerUrl,{type:'module'}));
  const initialized=new Set<number>();
  const completed=new Map<number,Extract<CollisionResult,{type:'result'}>>();
  let batchStarted=0;
  const collisionParts=selectCollisionParts(data);
  const original=new Map([...meshes].map(([id,m])=>[id,m.material]));
  const overlays=new Map<string,T.Mesh>();
  // Cache feature edges per solid; suppress coplanar and smooth tessellation edges.
  const hidden=new T.MeshBasicMaterial({visible:false});
  const edgeMaterial=new T.LineBasicMaterial({color:0xe6a79e,transparent:true,opacity:.75,depthWrite:false});
  const edges=new Map<number,T.LineSegments>();
  function showEdges(id:number){
    const mesh=meshes.get(id)!;
    let lines=edges.get(id);
    if(!lines){lines=new T.LineSegments(new T.EdgesGeometry(mesh.geometry,20),edgeMaterial);mesh.add(lines);edges.set(id,lines);}
    lines.visible=true;mesh.material=hidden;
  }
  const red=new T.MeshBasicMaterial({color:0xff2222,side:T.DoubleSide,polygonOffset:true,polygonOffsetFactor:-2,polygonOffsetUnits:-2});
  const groups=new Map<number,string>();
  const exclusions:[number,number][]=[];
  [38,39,40,46,54,55,56,57,64,65,66,67,68,69,70,71].forEach(id=>groups.set(id,'platform'));
  const twist=[41,42,43,44,45,47,48];twist.forEach(id=>{groups.set(id,'twist');exclusions.push([39,id],[40,id]);});
  for(const link of CAD_LINKS){
    groups.set(link.crank,`crank${link.motor}`);groups.set(link.lower,`crank${link.motor}`);groups.set(link.upper,'platform');
    link.rod.forEach(id=>groups.set(id,`rod${link.motor}`));
    const spec=data.spec.links.find(l=>l.motor===link.motor)! as CadData['spec']['links'][number]&{lower_eye:number;upper_eye:number};
    exclusions.push([link.lower,spec.lower_eye],[link.upper,spec.upper_eye]);
    const motor=[8,10,12,6,4,2][link.motor];
    exclusions.push([link.crank,motor],[link.crank,motor+1]);
  }
  let ready=false,busy=false,disposed=false,revision=0,applied=-1,enabled=true;
  let status:CollisionStatus={ready:false,pending:true,pairs:[],names:[],elapsed:0,workers:workerCount,partCount:collisionParts.length};
  const names=new Map(data.parts.map(p=>[p.id,p.name]));
  function clear(){
    for(const [id,mesh] of meshes)mesh.material=original.get(id)!;
    for(const lines of edges.values())lines.visible=false;
    for(const overlay of overlays.values()){overlay.removeFromParent();overlay.geometry.dispose();}overlays.clear();
  }
  function publish(patch:Partial<CollisionStatus>){status={...status,...patch};onStatus(status);}
  function fail(message:string){
    ready=false;busy=false;clear();workers.forEach(worker=>worker.terminate());
    publish({ready:false,pending:false,pairs:[],names:[],error:message});
  }
  workers.forEach((worker,index)=>{
  worker.onmessage=(event:MessageEvent<CollisionResult>)=>{
    if(disposed)return;
    const result=event.data;
    if(result.type==='ready'){initialized.add(index);ready=initialized.size===workerCount;publish({ready,pending:true});return;}
    if(result.type==='error'){fail(result.message);return;}
    completed.set(index,result);
    if(completed.size!==workerCount)return;
    busy=false;
    // Never apply an older pose's transparency or intersection geometry.
    if(!enabled||result.revision!==revision){completed.clear();return;}
    const batches=[...completed.values()];completed.clear();
    const merged={revision:result.revision,pairs:batches.flatMap(r=>r.pairs),
      intersections:batches.flatMap(r=>r.intersections),warnings:batches.flatMap(r=>r.warnings),
      elapsed:performance.now()-batchStarted};
    clear();applied=merged.revision;
    for(const id of new Set(merged.pairs.flat()))showEdges(id);
    for(const hit of merged.intersections){
      if(!hit.positions.length)continue;
      const mesh=meshes.get(hit.a)!;
      const geometry=new T.BufferGeometry();geometry.setAttribute('position',new T.Float32BufferAttribute(hit.positions,3));
      const overlay=new T.Mesh(geometry,red);overlay.renderOrder=2;mesh.add(overlay);overlays.set(`${hit.a}:${hit.b}`,overlay);
    }
    publish({pending:applied!==revision,pairs:merged.pairs,elapsed:merged.elapsed,details:merged.warnings,
      names:merged.pairs.map(([a,b])=>`${names.get(a)} ↔ ${names.get(b)}`)});
  };
  worker.onerror=event=>{if(!disposed)fail(event.message||'干涉检测线程失败');};
  const init:CollisionRequest={type:'init',shard:index,shardCount:workerCount,parts:collisionParts.map(p=>({...p,group:groups.get(p.id)??'fixed'})),exclusions};
  worker.postMessage(init);
  });
  onStatus(status);
  return {
    changed(){
      revision++;
      if(applied>=0){clear();applied=-1;}
      if(!status.pending)publish({pending:true,pairs:[],names:[],details:[]});
    },
    update(visible:boolean){
      if(enabled!==visible){enabled=visible;if(!visible)clear();else applied=-1;}
      if(!enabled||!ready||busy||applied===revision)return;
      busy=true;batchStarted=performance.now();completed.clear();publish({pending:true});
      const request:CollisionRequest={type:'check',revision,matrices:[...meshes].map(([id,m])=>({id,matrix:m.matrix.toArray()}))};
      workers.forEach(worker=>worker.postMessage(request));
    },
    dispose(){disposed=true;workers.forEach(worker=>worker.terminate());clear();for(const lines of edges.values()){lines.removeFromParent();lines.geometry.dispose();}edges.clear();hidden.dispose();edgeMaterial.dispose();red.dispose();},
  };
}
