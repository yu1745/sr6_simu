import * as T from 'three';
import ManifoldModule, {type Manifold, type Mat4} from 'manifold-3d';
import manifoldWasmUrl from 'manifold-3d/manifold.wasm?url';
const kernel=ManifoldModule({locateFile:()=>manifoldWasmUrl}).then(m=>{m.setup();return m;});
import { MeshBVH, type ExtendedTriangle } from 'three-mesh-bvh';
import { Brush, Evaluator, INTERSECTION } from 'three-bvh-csg';
import type { CollisionRequest, CollisionResult } from './collision.ts';

const EPS = 0.03; // mm; reject tangency and coincident mounting faces.
const NUMERIC_EPS = 1e-7; // Plane-side tests need numerical, not clearance, tolerance.
type Part = {id:number;group:string;geometry:T.BufferGeometry;bvh:MeshBVH;bounds:T.Box3;matrix:T.Matrix4;brush:Brush;solid?:Manifold};
const parts:Part[]=[];
const exclusions=new Set<string>();
let shard=0,shardCount=1;
const key=(a:number,b:number)=>a<b?`${a}:${b}`:`${b}:${a}`;
const planeA=new T.Plane(),planeB=new T.Plane(),segment=new T.Line3();
function straddles(tri:T.Triangle,plane:T.Plane){
  const a=plane.distanceToPoint(tri.a),b=plane.distanceToPoint(tri.b),c=plane.distanceToPoint(tri.c);
  return Math.min(a,b,c)<-NUMERIC_EPS && Math.max(a,b,c)>NUMERIC_EPS;
}
function crosses(a:ExtendedTriangle,b:ExtendedTriangle){
  a.getPlane(planeA);b.getPlane(planeB);
  return straddles(a,planeB)&&straddles(b,planeA)&&a.intersectsTriangle(b,segment)&&segment.distance()>NUMERIC_EPS;
}
const directions=[new T.Vector3(1,.371,.217).normalize(),new T.Vector3(.193,1,.419).normalize()];
function inside(point:T.Vector3,part:Part){
  const nearest=part.bvh.closestPointToPoint(point);
  if(!nearest||nearest.distance<EPS)return false;
  return directions.every(direction=>{
    const hits=part.bvh.raycast(new T.Ray(point,direction),T.DoubleSide).sort((a,b)=>a.distance-b.distance);
    let count=0,last=-Infinity;
    for(const hit of hits)if(hit.distance-last>EPS){count++;last=hit.distance;}
    return count%2===1;
  });
}
// Also detect partially embedded surfaces, including intersections that run
// exactly along mesh edges and therefore have no strictly crossing face pair.
function embeddedFaces(a:Part,b:Part,toB:T.Matrix4,faces:Set<number>){
  const localBounds=b.bounds.clone().applyMatrix4(toB.clone().invert());
  const memo=new Map<string,boolean>();
  const centroid=new T.Vector3();
  function probe(point:T.Vector3){
    const local=point.clone().applyMatrix4(toB);
    if(!b.bounds.containsPoint(local))return false;
    const signature=local.toArray().map(n=>n.toFixed(5)).join(',');
    if(memo.has(signature))return memo.get(signature)!;
    const result=inside(local,b);memo.set(signature,result);return result;
  }
  a.bvh.shapecast({
    intersectsBounds:box=>box.intersectsBox(localBounds),
    intersectsTriangle:(triangle,index)=>{
      triangle.getMidpoint(centroid);
      if(probe(centroid)||probe(triangle.a)||probe(triangle.b)||probe(triangle.c))faces.add(index);
      return faces.size>0;
    },
  });
}
const evaluator=new Evaluator();
evaluator.attributes=['position'];
evaluator.useGroups=false;
type PairHit={a:number;b:number;positions:number[];warning?:string};
// Split intersected faces along their intersection curves. The resulting
// geometry is A ∩ B, expressed in A's local frame, not entire source triangles.
function intersection(a:Part,b:Part,toA:T.Matrix4):PairHit{
  const hit:PairHit={a:a.id,b:b.id,positions:[]};
  let result:Brush|undefined;
  try{
    a.brush.matrix.identity();a.brush.updateMatrixWorld(true);
    b.brush.matrix.copy(toA);b.brush.updateMatrixWorld(true);
    result=evaluator.evaluate(a.brush,b.brush,INTERSECTION);
    const geometry=result.geometry,position=geometry.getAttribute('position'),index=geometry.getIndex();
    const start=geometry.drawRange.start;
    const end=Math.min(index?.count??position.count,start+geometry.drawRange.count);
    for(let i=start;i<end;i++){
      const vertex=index?index.getX(i):i;
      hit.positions.push(position.getX(vertex),position.getY(vertex),position.getZ(vertex));
    }
    if(!hit.positions.length)hit.warning=`${a.id} / ${b.id}：交集网格为空，仅显示线框`;
  }catch(error){hit.warning=`${a.id} / ${b.id}：交集生成失败，仅显示线框（${error instanceof Error?error.message:String(error)}）`;}
  finally{result?.geometry.dispose();}
  return hit;
}
const cache=new Map<string,{transform:string;hit:PairHit|null}>();
const scope=self as unknown as {onmessage:((e:MessageEvent<CollisionRequest>)=>void)|null;postMessage:(data:CollisionResult)=>void};
scope.onmessage=async event=>{
  try{
    const message=event.data;
    if(message.type==='init'){
      const wasm=await kernel;
      parts.length=0;exclusions.clear();cache.clear();
      shard=message.shard;shardCount=message.shardCount;
      message.exclusions.forEach(([a,b])=>exclusions.add(key(a,b)));
      for(const p of message.parts){
        const geometry=new T.BufferGeometry();
        geometry.setAttribute('position',new T.Float32BufferAttribute(p.positions,3));geometry.setIndex(p.indices);
        const bvh=new MeshBVH(geometry,{indirect:true,targetLeafSize:12});
        geometry.computeBoundingBox();
        const brush=new Brush(geometry.clone());brush.matrixAutoUpdate=false;
        let solid:Manifold|undefined;
        try{
          const mesh=new wasm.Mesh({numProp:3,vertProperties:new Float32Array(p.positions),triVerts:new Uint32Array(p.indices)});
          mesh.merge();solid=new wasm.Manifold(mesh);
          if(solid.status()!=='NoError'){solid.delete();solid=undefined;}
        }catch{/* Non-manifold source motors retain BVH/CSG detection. */}
        parts.push({id:p.id,group:p.group,geometry,bvh,bounds:geometry.boundingBox!,matrix:new T.Matrix4(),brush,solid});
      }
      scope.postMessage({type:'ready'});return;
    }
    const start=performance.now();
    const matrices=new Map(message.matrices.map(m=>[m.id,m.matrix]));
    for(const p of parts)p.matrix.fromArray(matrices.get(p.id)!);
    const boxes=parts.map(p=>p.bounds.clone().applyMatrix4(p.matrix));
    const hits:PairHit[]=[];
    let pairIndex=0;
    for(let i=0;i<parts.length;i++)for(let j=i+1;j<parts.length;j++){
      if(pairIndex++%shardCount!==shard)continue;
      const a=parts[i],b=parts[j],pair=key(a.id,b.id);
      if(a.group===b.group||exclusions.has(pair)||!boxes[i].intersectsBox(boxes[j]))continue;
      const overlap=boxes[i].clone().intersect(boxes[j]).getSize(new T.Vector3());
      if(Math.min(overlap.x,overlap.y,overlap.z)<NUMERIC_EPS)continue;
      const toA=a.matrix.clone().invert().multiply(b.matrix);
      const signature=toA.elements.map(n=>n.toFixed(6)).join(',');
      const old=cache.get(pair);
      if(old?.transform===signature){if(old.hit)hits.push(old.hit);continue;}
      if(a.solid&&b.solid){
        let moved:Manifold|undefined,common:Manifold|undefined;
        try{
          moved=b.solid.transform(toA.elements as Mat4);common=a.solid.intersect(moved);
          if(common.status()!=='NoError')throw new Error('WASM intersection failed');
          let hit:PairHit|null=null;
          if(!common.isEmpty()&&common.volume()>1e-7){
            const mesh=common.getMesh();const positions:number[]=[];
            for(const vertex of mesh.triVerts){const offset=vertex*mesh.numProp;positions.push(mesh.vertProperties[offset],mesh.vertProperties[offset+1],mesh.vertProperties[offset+2]);}
            hit={a:a.id,b:b.id,positions};
          }
          cache.set(pair,{transform:signature,hit});if(hit)hits.push(hit);
          continue;
        }catch{/* Fall back to the mesh detector if this boolean is rejected. */}
        finally{common?.delete();moved?.delete();}
      }
      const facesA=new Set<number>(),facesB=new Set<number>();
      a.bvh.bvhcast(b.bvh,toA,{intersectsTriangles:(ta,tb,ia,ib)=>{
        if(crosses(ta,tb)){facesA.add(a.bvh.resolveTriangleIndex(ia));facesB.add(b.bvh.resolveTriangleIndex(ib));}
        return facesA.size>0;
      }});
      if(!facesA.size){
        embeddedFaces(a,b,toA.clone().invert(),facesA);
        if(!facesA.size)embeddedFaces(b,a,toA,facesB);
      }
      const hit=facesA.size||facesB.size?intersection(a,b,toA):null;
      cache.set(pair,{transform:signature,hit});if(hit)hits.push(hit);
    }
    scope.postMessage({type:'result',revision:message.revision,elapsed:performance.now()-start,
      pairs:hits.map(h=>[h.a,h.b]),intersections:hits.map(h=>({a:h.a,b:h.b,positions:h.positions})),
      warnings:hits.flatMap(h=>h.warning?[h.warning]:[])});
  }catch(error){scope.postMessage({type:'error',message:error instanceof Error?error.message:String(error)});}
};
