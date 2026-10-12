import * as T from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { geometry, type V3 } from './geometry.ts';
import { type Pose } from './kinematics.ts';
import { createCadRig, type CadData, type CadStatus } from './cad-rig.ts';
import { createCadKinematics, type CadKinematics } from './cad-kinematics.ts';
import { createCollisionDisplay, type CollisionStatus } from './collision.ts';

export type SceneState = { pose: Pose; q: number[]; cad: boolean };
export function mountScene(canvas: HTMLCanvasElement, state: () => SceneState, onCad: (message: string) => void, onCadStatus: (status: CadStatus) => void, onCadReady: (solver: CadKinematics) => void, onCollision: (status: CollisionStatus) => void) {
  const renderer = new T.WebGLRenderer({canvas, antialias:true});
  renderer.setPixelRatio(Math.min(devicePixelRatio,2));
  renderer.setClearColor(0x0a0f14);
  const scene=new T.Scene(), camera=new T.PerspectiveCamera(40,1,0.1,2500);
  camera.position.set(510,300,610);
  const controls=new OrbitControls(camera,canvas);
  controls.target.set(0,125,0); controls.enableDamping=true;
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
  let collisions: ReturnType<typeof createCollisionDisplay> | undefined;
  const abort=new AbortController();
  fetch('/sr6-cad.json',{signal:abort.signal,cache:'no-store'}).then(r=>{if(!r.ok)throw Error(`HTTP ${r.status}`);return r.json() as Promise<CadData>;}).then(async data=>{
    if(data.partFiles){
      const chunks=await Promise.all(data.partFiles.map(async url=>{
        const response=await fetch(url,{signal:abort.signal});
        if(!response.ok)throw Error(`模型分块加载失败：HTTP ${response.status}`);
        return response.json() as Promise<CadData['parts']>;
      }));
      data.parts=chunks.flat();
    }
    if(disposed)return;
    const meshes = new Map<number, T.Mesh>();
    for(const p of data.parts){
      const g=new T.BufferGeometry();g.setAttribute('position',new T.Float32BufferAttribute(p.positions,3));g.setIndex(p.indices);g.computeVertexNormals();
      const material = p.name.includes('socket_cap_bolt') || p.id === 1 ? materials.dark : [20,25,30,35].includes(p.id) || /nut|spacer/.test(p.name) ? materials.shaft : materials.plate;
      const mesh = new T.Mesh(g,material); meshes.set(p.id,mesh); cadRoot.add(mesh);
    }
    cadRig = createCadRig(data, meshes);
    onCadReady(createCadKinematics(data));
    cadPresentation.updateMatrixWorld(true);
    // Place the upright assembly on the ground instead of through the grid.
    const bounds = new T.Box3().setFromObject(cadPresentation);
    cadPresentation.position.y = grid.position.y - bounds.min.y;
    try{collisions=createCollisionDisplay(data,meshes,onCollision);}
    catch(error){onCollision({ready:false,pending:false,pairs:[],names:[],elapsed:0,error:error instanceof Error?error.message:String(error)});}
    revision='';
    onCad(`STEP · 六组支链 / 双侧俯仰臂 · ${data.parts.length} 个实体 · 底座面垂直地面`);
  }).catch(e=>{if(!disposed)onCad(`CAD 加载失败：${e.message}`);});
  const resize=()=>{renderer.setSize(canvas.clientWidth,canvas.clientHeight,false);camera.aspect=canvas.clientWidth/Math.max(1,canvas.clientHeight);camera.updateProjectionMatrix();};
  const observer=new ResizeObserver(resize);observer.observe(canvas);resize();
  const render=()=>{
    const s=state();root.visible=!s.cad;cadRoot.visible=s.cad;
    const next=JSON.stringify([s.pose,s.q,s.cad]);
    if(next!==revision){
      revision=next;const legs=geometry(s.pose,s.q);
      if(cadRig && s.cad){
        const status=cadRig.update(s.pose,s.q);onCadStatus(status);
        if(status.valid)collisions?.changed();
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
      });
      place(seat,legs[0].end,legs[5].end);
      twist.position.copy(convert(legs[0].end)).add(convert(legs[5].end)).multiplyScalar(.5);
      twist.rotation.y=s.pose.twist;
    }
    collisions?.update(s.cad);
    controls.update();renderer.render(scene,camera);frame=requestAnimationFrame(render);
  };render();
  return ()=>{disposed=true;abort.abort();cancelAnimationFrame(frame);collisions?.dispose();observer.disconnect();controls.dispose();scene.traverse(o=>{if(o instanceof T.Mesh)o.geometry.dispose();});Object.values(materials).forEach(m=>m.dispose());renderer.dispose();};
}
