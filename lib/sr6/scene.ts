import * as T from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { geometry, type V3 } from './geometry.ts';
import { ZERO, inverse, type Pose } from './kinematics.ts';
import { createCadRig, type CadData, type CadStatus } from './cad-rig.ts';
import { createCadKinematics, type CadKinematics } from './cad-kinematics.ts';
import { createCollisionDisplay, type CollisionStatus } from './collision.ts';
import { createStickVisual } from './stick-visual.ts';
import { createTorqueLabels } from './torque-labels.ts';
import { createBaseReactionLabel } from './base-reaction-label.ts';
import { NO_ACCELERATION, numericalInverseJacobian, receiverTorques, type Acceleration, type TorqueAnalysis } from './statics.ts';
import { createRobotDynamics, type RobotDynamics, type RobotMotion } from './rigid-dynamics.ts';
import { type MassProperties } from './mass-properties.ts';
import { createRenderBudget } from './render-budget.ts';
import { CAD_VARIANTS, type CadVariant } from './cad-variants.ts';
import type { PlaybackState, ScriptSample } from './buffered-trajectory.ts';

export type MotionFrame = {pose:Pose;q:number[];acceleration:Acceleration;robotMotion:RobotMotion|null;timeline?:ScriptSample};
export type SceneState = MotionFrame & {cad:boolean;stickLength:number;weightN:number;selectedPart:number|null;playback?:PlaybackState};
function activityRate(report:(fps:number|null)=>void){
  let windowStart=0,count=0,last=0,idle=false;
  return {
    record(now:number){
      last=now;idle=false;
      if(!windowStart)windowStart=now;
      count++;
      const elapsed=now-windowStart;
      if(elapsed>=1000){report(Math.round((count-1)*1000/elapsed));windowStart=now;count=1;}
    },
    poll(now:number){
      if(last&&now-last>1000&&!idle){windowStart=0;count=0;idle=true;report(null);}
    },
  };
}
export function mountScene(canvas: HTMLCanvasElement, state: () => SceneState, onCad: (message: string) => void, onCadStatus: (status: CadStatus) => void, onCadReady: (solver: CadKinematics,dynamics:RobotDynamics,masses:MassProperties) => void, onCollision: (status: CollisionStatus) => void, onTorques:(result:TorqueAnalysis)=>void,onPart:(id:number|null)=>void,onFrameRate:(fps:number|null)=>void,onSolveRate:(fps:number|null)=>void,onCollisionRate:(fps:number|null)=>void, variant:CadVariant='8108') {
  const renderer = new T.WebGLRenderer({canvas, antialias:true});
  renderer.setPixelRatio(Math.min(devicePixelRatio,1.5));
  const renderBudget=createRenderBudget(120);
  const invalidate=()=>renderBudget.invalidate();
  renderer.setClearColor(0x0a0f14);
  const scene=new T.Scene(), camera=new T.PerspectiveCamera(40,1,0.1,2500);
  const torqueLabels=createTorqueLabels(canvas);
  const baseReactionLabel=createBaseReactionLabel(canvas);
  camera.position.set(510,300,610);
  const controls=new OrbitControls(camera,canvas);
  controls.target.set(0,125,0); controls.enableDamping=true;
  // Decay drag inertia quickly (default .05 drifts for much longer).
  controls.dampingFactor=0.3;
  controls.addEventListener('change',invalidate);
  scene.add(new T.HemisphereLight(0xd3e9ff,0x27303b,2.3));
  const light=new T.DirectionalLight(0xffffff,3); light.position.set(100,500,300); scene.add(light);
  const grid=new T.GridHelper(800,40,0x324858,0x172733); grid.position.y=-40; scene.add(grid);
  // CAD (x,y,z) -> viewer (x,z,-y). Proper rotation, no reflection.
  const convert=(p:V3)=>new T.Vector3(p[0],p[2],95-p[1]);
  const root=new T.Group(); scene.add(root);
  const materials = {
    plate:new T.MeshStandardMaterial({color:0x8aa8b7,metalness:0.5,roughness:0.45}),
    dark:new T.MeshStandardMaterial({color:0x344754,metalness:0.5,roughness:0.48}),
    shaft:new T.MeshStandardMaterial({color:0xc1d5de,metalness:0.7,roughness:0.3}),
    joint:new T.MeshStandardMaterial({color:0x8d7eea,metalness:0.4,roughness:0.42}),
  };
  const cylinder=new T.CylinderGeometry(1,1,1,24), sphere=new T.SphereGeometry(1,16,10);
  const box=new T.BoxGeometry(1,1,1);
  const bar=(r:number,mat:T.Material)=>{const mesh=new T.Mesh(cylinder,mat);root.add(mesh);mesh.userData.radius=r;return mesh;};
  const place=(mesh:T.Mesh,a:V3,b:V3)=>{
    const av=convert(a), bv=convert(b), d=bv.clone().sub(av);
    mesh.position.copy(av).add(bv).multiplyScalar(0.5);
    mesh.quaternion.setFromUnitVectors(new T.Vector3(0,1,0),d.clone().normalize());
    mesh.scale.set(mesh.userData.radius,d.length(),mesh.userData.radius);
  };
  const crank=Array.from({length:6},()=>bar(5,materials.plate));
  const rods=Array.from({length:6},(_,i)=>bar(i===2||i===3?4:6,materials.shaft));
  const ends=Array.from({length:12},()=>{const m=new T.Mesh(sphere,materials.joint);m.scale.setScalar(7);root.add(m);return m;});
  const motors=Array.from({length:6},()=>bar(40,materials.dark));
  const seat=bar(4,materials.plate);
  const twist=new T.Mesh(box,materials.joint);twist.scale.set(34,4,8);root.add(twist);
  const backbone=new T.Mesh(box,materials.dark);backbone.scale.set(118,8,285);backbone.position.copy(convert([0,142.5,0]));root.add(backbone);
  // sr6_mini.stp: X follows the motor rows, Y is height, Z is lateral.
  // One rigid transform for the entire assembly preserves every part placement.
  // Restore the original azimuth, then tip the assembly about horizontal X.
  // The base floor's CAD +Y normal becomes horizontal viewer +Z;
  // the motor rows (CAD +X) now run upwards, with the pitch end at the top.
  const cadPresentation=new T.Group();scene.add(cadPresentation);
  cadPresentation.rotation.x=Math.PI/2;
  const cadRoot=new T.Group();cadPresentation.add(cadRoot);
  cadRoot.rotation.y=Math.PI/2;cadRoot.position.set(54,0,100);
  let disposed=false, revision='', frame=0;
  let cadRig: ReturnType<typeof createCadRig> | undefined;
  let collisions: ReturnType<typeof createCollisionDisplay<MotionFrame>> | undefined;
  let cadSolver:CadKinematics|undefined;
  let dynamics:RobotDynamics|undefined;
  let displayedMotion:MotionFrame={pose:{...ZERO},q:Array(6).fill(0),acceleration:NO_ACCELERATION,robotMotion:null};
  const cadMeshes=new Map<number,T.Mesh>();
  const cadMaterials:T.Material[]=[];
  const selection=new T.BoxHelper(cadRoot,0xffd283);scene.add(selection);selection.visible=false;
  const raycaster=new T.Raycaster();let pointerStart:T.Vector2|null=null;
  const pointerDown=(event:PointerEvent)=>{pointerStart=new T.Vector2(event.clientX,event.clientY);};
  const pointerUp=(event:PointerEvent)=>{
    if(!state().cad||!pointerStart||event.button!==0||pointerStart.distanceTo(new T.Vector2(event.clientX,event.clientY))>4)return;
    const bounds=canvas.getBoundingClientRect();
    raycaster.setFromCamera(new T.Vector2((event.clientX-bounds.left)/bounds.width*2-1,1-(event.clientY-bounds.top)/bounds.height*2),camera);
    const hit=raycaster.intersectObjects([...cadMeshes.values()],false)[0];
    onPart(hit?hit.object.userData.partId as number:null);
  };
  canvas.addEventListener('pointerdown',pointerDown);canvas.addEventListener('pointerup',pointerUp);
  let torqueRevision='';
  let cadAnchors:T.Vector3[]=[];
  const firmwareAnchors=Array.from({length:6},()=>new T.Vector3());
  const worldAnchors=Array.from({length:6},()=>new T.Vector3());
  const baseAnchor=new T.Vector3();
  const worldBaseAnchor=new T.Vector3();
  let hasBaseAnchor=false;
  const stickVisual=createStickVisual();
  let displayedReceiver:T.Mesh|undefined;
  let displayedOutput:T.Mesh|undefined;
  const drawRate=activityRate(onFrameRate);
  const solveRate=activityRate(onSolveRate);
  const collisionRate=activityRate(onCollisionRate);
  const abort=new AbortController();
  fetch(CAD_VARIANTS[variant].data,{signal:abort.signal,cache:'no-store'}).then(r=>{if(!r.ok)throw Error(`HTTP ${r.status}`);return r.json() as Promise<CadData>;}).then(async data=>{
    if(data.partFiles){
      const chunks=await Promise.all(data.partFiles.map(async url=>{
        const response=await fetch(url,{signal:abort.signal});
        if(!response.ok)throw Error(`模型分块加载失败：HTTP ${response.status}`);
        return response.json() as Promise<CadData['parts']>;
      }));
      data.parts=chunks.flat();
    }
    if(disposed)return;
    const massResponse=await fetch(CAD_VARIANTS[variant].masses,{signal:abort.signal,cache:'no-store'});
    if(!massResponse.ok)throw Error(`质量属性 HTTP ${massResponse.status}`);
    const masses=await massResponse.json() as MassProperties;
    if(disposed)return;
    cadSolver=createCadKinematics(data);dynamics=createRobotDynamics(data,masses,cadSolver);
    const partInfo=new Map(masses.parts.map(p=>[p.id,p]));
    const colored=new Map(Object.entries(masses.materials).map(([id,info])=>{
      const material=new T.MeshStandardMaterial({color:info.color,metalness:.5,roughness:.45});cadMaterials.push(material);return [id,material];
    }));
    const meshes = cadMeshes;
    for(const p of data.parts){
      const g=new T.BufferGeometry();g.setAttribute('position',new T.Float32BufferAttribute(p.positions,3));g.setIndex(p.indices);g.computeVertexNormals();
      const info=partInfo.get(p.id)!;
      const material=colored.get(info.material)??materials.dark;
      const mesh = new T.Mesh(g,material);mesh.userData.partId=p.id;meshes.set(p.id,mesh);cadRoot.add(mesh);
    }
    cadRig = createCadRig(data, meshes);
    const baseSheet=masses.parts.find(p=>p.id===1);
    if(baseSheet){baseAnchor.fromArray(baseSheet.centerMm);hasBaseAnchor=true;}
    onCadReady(cadSolver,dynamics,masses);
    cadAnchors=[...data.spec.links].sort((a,b)=>a.motor-b.motor).map(l=>new T.Vector3(...l.pivot));
    cadPresentation.updateMatrixWorld(true);
    // Place the upright assembly on the ground instead of through the grid.
    const bounds = new T.Box3().setFromObject(cadPresentation);
    cadPresentation.position.y = grid.position.y - bounds.min.y;
    // Add after ground alignment so changing virtual rod length cannot move the base.
    displayedReceiver=meshes.get(38);
    displayedOutput=meshes.get(47);
    cadRoot.add(stickVisual.group);
    try{collisions=createCollisionDisplay<MotionFrame>(data,meshes,status=>{invalidate();onCollision(status);},displayedMotion,frame=>{if(frame)displayedMotion=frame;invalidate();},()=>collisionRate.record(performance.now()));}
    catch(error){onCollision({ready:false,pending:false,pairs:[],names:[],elapsed:0,error:error instanceof Error?error.message:String(error)});}
    revision='';
    invalidate();
    onCad(`STEP · 六组支链 / 双侧俯仰臂 · ${data.parts.length} 个实体 · 底座面垂直地面`);
  }).catch(e=>{if(!disposed)onCad(`CAD 加载失败：${e.message}`);});
  const resize=()=>{renderer.setSize(canvas.clientWidth,canvas.clientHeight,false);camera.aspect=canvas.clientWidth/Math.max(1,canvas.clientHeight);camera.updateProjectionMatrix();invalidate();};
  const observer=new ResizeObserver(resize);observer.observe(canvas);resize();
  let visualRevision='';
  const render=(now:number)=>{
    if(disposed)return;
    frame=requestAnimationFrame(render);
    drawRate.poll(now);solveRate.poll(now);collisionRate.poll(now);
    if(!renderBudget.begin(now,!document.hidden))return;
    const s=state();root.visible=!s.cad;cadRoot.visible=s.cad;
    baseReactionLabel.playback(s.playback??{playing:false,timeMs:0,epoch:0});
    const nextVisual=JSON.stringify([s.cad,s.stickLength,s.weightN,s.selectedPart]);
    if(nextVisual!==visualRevision){visualRevision=nextVisual;invalidate();}
    const next=JSON.stringify([s.pose,s.q,s.cad,s.acceleration,s.robotMotion,s.timeline]);
    if(next!==revision){
      invalidate();
      revision=next;const legs=geometry(s.pose,s.q);
      if(cadRig && s.cad){
        const status=cadRig.update(s.pose,s.q);onCadStatus(status);
        if(status.valid){
          solveRate.record(now);
          const desiredMotion={pose:s.pose,q:s.q,acceleration:s.acceleration,robotMotion:s.robotMotion,timeline:s.timeline};
          if(collisions)collisions.changed(desiredMotion);
          else displayedMotion=desiredMotion;
        }
      }
      legs.forEach((l,i)=>{
        place(crank[i],l.base,l.tip);
        // Keep body length fixed at 220; eye-to-eye length is 262, a separate quantity.
        if(i!==2&&i!==3){
          const a=new T.Vector3(...l.tip), b=new T.Vector3(...l.end), d=b.clone().sub(a).normalize();
          const midpoint=a.clone().add(b).multiplyScalar(.5);
          place(rods[i],midpoint.clone().addScaledVector(d,-110).toArray() as V3,midpoint.clone().addScaledVector(d,110).toArray() as V3);
        } else place(rods[i],l.tip,l.end);
        ends[i*2].position.copy(convert(l.tip));ends[i*2+1].position.copy(convert(l.end));
        const b=l.base;place(motors[i],[b[0]-12,b[1],b[2]],[b[0]+12,b[1],b[2]]);
        firmwareAnchors[i].copy(convert(b));
      });
      place(seat,legs[0].end,legs[5].end);
      twist.position.copy(convert(legs[0].end)).add(convert(legs[5].end)).multiplyScalar(.5);
      twist.rotation.y=s.pose.twist;
    }
    collisions?.update(s.cad);
    const motion=s.cad?displayedMotion:s;
    const nextTorque=JSON.stringify([s.cad,s.weightN,s.stickLength,motion]);
    if(nextTorque!==torqueRevision&&(!s.cad||cadSolver)){
      invalidate();
      torqueRevision=nextTorque;
      const jacobian=s.cad?cadSolver!.inverseJacobian(motion.pose,motion.q):numericalInverseJacobian(motion.pose,inverse);
      const result=s.cad&&dynamics?dynamics.torques(motion.pose,motion.q,s.stickLength,s.weightN,motion.robotMotion):receiverTorques(jacobian,s.weightN,motion.acceleration);
      torqueLabels.values(result);baseReactionLabel.values(result,motion.timeline);onTorques(result);
    }
    controls.update();
    if(!renderBudget.takeDraw())return;
    if(s.cad&&displayedReceiver&&displayedOutput)stickVisual.update(displayedReceiver.matrix,s.stickLength,displayedOutput.matrix);
    scene.updateMatrixWorld(true);camera.updateMatrixWorld(true);
    const selected=s.selectedPart===null?undefined:cadMeshes.get(s.selectedPart);
    selection.visible=s.cad&&!!selected;
    if(selected&&s.cad)selection.setFromObject(selected);
    const anchors=s.cad?cadAnchors:firmwareAnchors;
    const anchorRoot=s.cad?cadRoot:root;
    anchors.forEach((a,i)=>worldAnchors[i].copy(a).applyMatrix4(anchorRoot.matrixWorld));
    torqueLabels.update(worldAnchors,camera,anchors.length===6);
    if(hasBaseAnchor)worldBaseAnchor.copy(baseAnchor).applyMatrix4(cadRoot.matrixWorld);
    baseReactionLabel.update(worldBaseAnchor,camera,s.cad&&hasBaseAnchor);
    renderer.render(scene,camera);
    drawRate.record(now);
  };frame=requestAnimationFrame(render);
  return ()=>{disposed=true;abort.abort();cancelAnimationFrame(frame);canvas.removeEventListener('pointerdown',pointerDown);canvas.removeEventListener('pointerup',pointerUp);selection.geometry.dispose();selection.material.dispose();collisions?.dispose();stickVisual.dispose();torqueLabels.dispose();baseReactionLabel.dispose();observer.disconnect();controls.dispose();scene.traverse(o=>{if(o instanceof T.Mesh)o.geometry.dispose();});Object.values(materials).forEach(m=>m.dispose());cadMaterials.forEach(m=>m.dispose());renderer.dispose();};
}
