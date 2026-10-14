import * as T from 'three';
import { CAD_LINKS, type CadData } from './cad-rig.ts';
import { collisionParts as selectCollisionParts } from './collision-scope.ts';
import collisionWorkerUrl from './collision-worker.ts?worker&url';
import { solidOutlineGeometry, solidOutlineMaterial } from './solid-outline.ts';
export type CollisionStatus={ready:boolean;pending:boolean;pairs:[number,number][];names:string[];elapsed:number;workers?:number;partCount?:number;displayedRevision?:number;requestedRevision?:number;error?:string;details?:string[]};
export type CollisionRequest=
  |{type:'init';shard:number;shardCount:number;parts:{id:number;positions:number[];indices:number[];group:string}[];exclusions:[number,number][]}
  |{type:'check';revision:number;matrices:{id:number;matrix:number[]}[]};
export type CollisionResult=
  |{type:'ready'}|{type:'error';message:string}
  |{type:'result';revision:number;elapsed:number;pairs:[number,number][];intersections:{a:number;b:number;positions:number[]}[];warnings:string[]};

export function createCollisionDisplay<Frame=undefined>(data:CadData,meshes:Map<number,T.Mesh>,onStatus:(s:CollisionStatus)=>void,initialFrame?:Frame,onFrame?:(frame:Frame|undefined)=>void,onCommitted?:()=>void){
  // Use Vite's emitted URL directly. Vinext can rewrite import.meta.url to a
  // build-time file:/// URL, which is not a valid worker origin in the browser.
  // Use the browser-reported logical CPU budget; leave one for UI/rendering.
  const workerCount=Math.max(1,Math.floor(navigator.hardwareConcurrency||2)-1);
  const workers=Array.from({length:workerCount},()=>new Worker(collisionWorkerUrl,{type:'module'}));
  const initialized=new Set<number>();
  const completed=new Map<number,Extract<CollisionResult,{type:'result'}>>();
  let batchStarted=0,epoch=0,batchEpoch=0,failed=false;
  // Keep motion telemetry with the same snapshot as the rendered geometry.
  // In particular, a newer command's acceleration must not label an older pose.
  type Snapshot={matrices:{id:number;matrix:number[]}[];frame:Frame|undefined};
  const capture=(frame?:Frame):Snapshot=>({matrices:[...meshes].map(([id,m])=>({id,matrix:m.matrix.toArray()})),frame});
  const place=(snapshot:Snapshot)=>{for(const {id,matrix} of snapshot.matrices){const mesh=meshes.get(id)!;mesh.matrix.fromArray(matrix);mesh.matrixWorldNeedsUpdate=true;}onFrame?.(snapshot.frame);};
  let desired=capture(initialFrame),displayed=desired,inFlight=desired;
  const collisionParts=selectCollisionParts(data);
  const original=new Map([...meshes].map(([id,m])=>[id,m.material]));
  const overlays=new Map<string,T.Mesh>();
  // Cache adjacency once; the shader adds view-dependent smooth silhouettes.
  const hidden=new T.MeshBasicMaterial({visible:false});
  const edgeMaterial=solidOutlineMaterial();
  const edges=new Map<number,T.LineSegments>();
  function showEdges(id:number){
    const mesh=meshes.get(id)!;
    let lines=edges.get(id);
    if(!lines){lines=new T.LineSegments(solidOutlineGeometry(mesh.geometry,20),edgeMaterial);mesh.add(lines);edges.set(id,lines);}
    lines.visible=true;mesh.material=hidden;
  }
  const red=new T.MeshBasicMaterial({color:0xff2222,side:T.DoubleSide,polygonOffset:true,polygonOffsetFactor:-2,polygonOffsetUnits:-2});
  const groups=new Map<number,string>();
  const exclusions:[number,number][]=[];
  [38,39,40,46,54,55,56,57,64,65,66,67,68,69,70,71,72,73,74,75,76,77].forEach(id=>groups.set(id,'platform'));
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
    ready=false;busy=false;failed=true;clear();place(desired);workers.forEach(worker=>worker.terminate());
    publish({ready:false,pending:false,pairs:[],names:[],error:message});
  }
  function dispatchLatest(){
    if(!enabled||failed||!ready||busy||applied===revision)return;
    busy=true;batchStarted=performance.now();batchEpoch=epoch;inFlight=desired;completed.clear();
    publish({pending:true,requestedRevision:revision});
    const request:CollisionRequest={type:'check',revision,matrices:inFlight.matrices};
    workers.forEach(worker=>worker.postMessage(request));
  }
  workers.forEach((worker,index)=>{
  worker.onmessage=(event:MessageEvent<CollisionResult>)=>{
    if(disposed||failed)return;
    const result=event.data;
    if(result.type==='ready'){initialized.add(index);ready=initialized.size===workerCount;publish({ready,pending:true});return;}
    if(result.type==='error'){fail(result.message);return;}
    completed.set(index,result);
    if(completed.size!==workerCount)return;
    busy=false;
    // Commit the tested pose and its overlays together. A newer requested pose
    // does not invalidate this coherent display frame, otherwise playback can
    // starve forever when computation takes longer than one animation frame.
    if(!enabled||batchEpoch!==epoch){completed.clear();return;}
    const batches=[...completed.values()];completed.clear();
    const merged={revision:result.revision,pairs:batches.flatMap(r=>r.pairs),
      intersections:batches.flatMap(r=>r.intersections),warnings:batches.flatMap(r=>r.warnings),
      elapsed:performance.now()-batchStarted};
    clear();displayed=inFlight;place(displayed);applied=merged.revision;onCommitted?.();
    for(const id of new Set(merged.pairs.flat()))showEdges(id);
    for(const hit of merged.intersections){
      if(!hit.positions.length)continue;
      const mesh=meshes.get(hit.a)!;
      const geometry=new T.BufferGeometry();geometry.setAttribute('position',new T.Float32BufferAttribute(hit.positions,3));
      const overlay=new T.Mesh(geometry,red);overlay.renderOrder=2;mesh.add(overlay);overlays.set(`${hit.a}:${hit.b}`,overlay);
    }
    publish({pending:applied!==revision,displayedRevision:applied,requestedRevision:revision,pairs:merged.pairs,elapsed:merged.elapsed,details:merged.warnings,
      names:merged.pairs.map(([a,b])=>`${names.get(a)} ↔ ${names.get(b)}`)});
    // A worker batch often finishes just after a display tick. Start the next
    // newest pose now instead of waiting up to one more refresh interval.
    dispatchLatest();
  };
  worker.onerror=event=>{if(!disposed)fail(event.message||'干涉检测线程失败');};
  const init:CollisionRequest={type:'init',shard:index,shardCount:workerCount,parts:collisionParts.map(p=>({...p,group:groups.get(p.id)??'fixed'})),exclusions};
  worker.postMessage(init);
  });
  onStatus(status);
  return {
    changed(frame?:Frame){
      revision++;
      desired=capture(frame);
      if(failed)return;
      place(displayed);
      if(!status.pending)publish({pending:true,requestedRevision:revision});
    },
    update(visible:boolean){
      if(enabled!==visible){enabled=visible;epoch++;if(!visible)clear();else applied=-1;}
      dispatchLatest();
    },
    dispose(){disposed=true;workers.forEach(worker=>worker.terminate());clear();for(const lines of edges.values()){lines.removeFromParent();lines.geometry.dispose();}edges.clear();hidden.dispose();edgeMaterial.dispose();red.dispose();},
  };
}
