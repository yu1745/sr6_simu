// Exercise the real collision display coordinator with deterministic fake workers.
// Only Vite's ?worker&url asset import is stubbed; snapshot/commit logic is unchanged.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import * as T from 'three';
import ts from 'typescript';
import { ZERO } from '../lib/sr6/kinematics.ts';
import { createCadRig } from '../lib/sr6/cad-rig.ts';

const sourceURL=new URL('../lib/sr6/collision.ts',import.meta.url);
const source=fs.readFileSync(sourceURL,'utf8').replace(/import collisionWorkerUrl from [^;]+;/,'const collisionWorkerUrl="test-worker";');
let compiled=ts.transpileModule(source,{compilerOptions:{module:ts.ModuleKind.ESNext,target:ts.ScriptTarget.ES2022}}).outputText;
compiled=compiled.replace(/from (['"])([^'"]+)\1/g,(_,quote,specifier)=>`from '${specifier.startsWith('.')?new URL(specifier,sourceURL).href:import.meta.resolve(specifier)}'`);
const directory=new URL('../work/statics/',import.meta.url);fs.mkdirSync(directory,{recursive:true});
const moduleURL=new URL('collision-under-test.mjs',directory);fs.writeFileSync(moduleURL,compiled);
const workers=[];
globalThis.Worker=class{
  messages=[];
  constructor(){workers.push(this);}
  postMessage(message){this.messages.push(message);}
  terminate(){}
  reply(data){this.onmessage({data});}
};
Object.defineProperty(navigator,'hardwareConcurrency',{value:3,configurable:true});
const {createCollisionDisplay}=await import(moduleURL.href);
const data=JSON.parse(fs.readFileSync(new URL('../public/sr6-cad.json',import.meta.url),'utf8'));
const meshes=new Map(data.parts.map(p=>[p.id,new T.Mesh()]));
const rig=createCadRig(data,meshes);
const initial={pose:{...ZERO},q:Array(6).fill(0),acceleration:[0,0,0]};
let displayed=initial;
const display=createCollisionDisplay(data,meshes,()=>{},initial,frame=>{displayed=frame;});
assert.equal(workers.length,2);
workers.forEach(w=>w.reply({type:'ready'}));
const first={pose:{...ZERO,y:8},q:[],acceleration:[1,2,3]};
const second={pose:{...ZERO,y:-6},q:[],acceleration:[-3,-2,-1]};
assert.ok(rig.update(first.pose).valid);const firstMatrix=meshes.get(38).matrix.toArray();
display.changed(first);
assert.equal(displayed,initial);
assert.ok(meshes.get(38).matrix.equals(new T.Matrix4()));
display.update(true);
assert.ok(rig.update(second.pose).valid);const secondMatrix=meshes.get(38).matrix.toArray();
display.changed(second);
assert.equal(displayed,initial);
const complete=()=>workers.forEach(w=>{
  const request=w.messages.at(-1);assert.equal(request.type,'check');
  w.reply({type:'result',revision:request.revision,elapsed:1,pairs:[],intersections:[],warnings:[]});
});
complete();
assert.equal(displayed,first,'Newer desired acceleration must not replace the committed frame');
assert.deepEqual(meshes.get(38).matrix.toArray(),firstMatrix);
assert.equal(workers[0].messages.at(-1).type,'check');
assert.equal(workers[0].messages.at(-1).revision,2,'Next pending pose starts as soon as the previous batch completes');
display.update(true);complete();
assert.equal(displayed,second);assert.deepEqual(meshes.get(38).matrix.toArray(),secondMatrix);

// Pausing keeps the last snapshot, including its nonzero acceleration.
display.update(true);assert.equal(displayed,second);assert.deepEqual(displayed.acceleration,[-3,-2,-1]);
// A manual seek can explicitly clear the derivative snapshot.
const sought={...second,acceleration:[0,0,0]};
display.changed(sought);display.update(true);complete();assert.equal(displayed,sought);
// Worker failure falls back to the latest geometry AND corresponding telemetry.
assert.ok(rig.update(first.pose).valid);display.changed(first);display.update(true);
workers[0].reply({type:'error',message:'test failure'});
assert.equal(displayed,first);assert.deepEqual(meshes.get(38).matrix.toArray(),firstMatrix);
display.dispose();
console.log('Collision display: pose/acceleration snapshots stay coherent during playback, pause and worker failure.');
