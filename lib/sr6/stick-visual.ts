import * as T from 'three';
import { CAD_PLATFORM_PIVOT } from './cad-kinematics.ts';

/** CAD-space overlay. Read the displayed receiver matrix, including collision
 * display latency; never render from a newer requested pose. */
export function createStickVisual() {
  const group = new T.Group();
  group.name = 'virtual-seventh-axis';
  const rodMaterial = new T.MeshStandardMaterial({color:0xffc45c,metalness:.25,roughness:.35});
  const rodGeometry = new T.CylinderGeometry(1.8,1.8,1,16);
  const rod = new T.Mesh(rodGeometry,rodMaterial);
  rod.name = 'virtual-stick';
  group.add(rod);

  // ROS-style right-handed tool frame. Neutral STEP directions: -Y, -Z, +X.
  // Its origin follows the tip and its orientation follows the displayed output
  // fixture, including roll/pitch and the independent R0 rotation.
  const tipFrame = new T.Group();
  tipFrame.name = 'stick-tip-frame';
  group.add(tipFrame);
  const shaftGeometry = new T.CylinderGeometry(.85,.85,32,12);
  const headGeometry = new T.ConeGeometry(2.7,8,16);
  const originGeometry = new T.SphereGeometry(2.4,16,12);
  const originMaterial = new T.MeshBasicMaterial({color:0xffffff,depthTest:false,depthWrite:false});
  const origin = new T.Mesh(originGeometry,originMaterial);
  origin.renderOrder=21;tipFrame.add(origin);
  const axisMaterials:T.MeshBasicMaterial[]=[];
  const axes = [
    {name:'X',direction:new T.Vector3(0,-1,0),color:0xff3535},
    {name:'Y',direction:new T.Vector3(0,0,-1),color:0x36e66a},
    {name:'Z',direction:new T.Vector3(1,0,0),color:0x398aff},
  ];
  for(const axis of axes){
    const material=new T.MeshBasicMaterial({color:axis.color,depthTest:false,depthWrite:false});
    axisMaterials.push(material);
    const arrow=new T.Group();arrow.name=axis.name;
    arrow.quaternion.setFromUnitVectors(new T.Vector3(0,1,0),axis.direction);
    const shaft=new T.Mesh(shaftGeometry,material),head=new T.Mesh(headGeometry,material);
    shaft.position.y=16;head.position.y=36;
    shaft.renderOrder=20;head.renderOrder=20;
    arrow.add(shaft,head);tipFrame.add(arrow);
  }
  const base=new T.Vector3(),tip=new T.Vector3(),direction=new T.Vector3();
  const up=new T.Vector3(0,1,0);
  return {
    group,
    update(displayedPlatform:T.Matrix4,length:number,displayedOutput:T.Matrix4=displayedPlatform){
      base.set(...CAD_PLATFORM_PIVOT).applyMatrix4(displayedPlatform);
      // Mounted rod points out along receiver -X in STEP, not toward +X.
      tip.set(CAD_PLATFORM_PIVOT[0]-length,CAD_PLATFORM_PIVOT[1],CAD_PLATFORM_PIVOT[2]).applyMatrix4(displayedPlatform);
      direction.copy(tip).sub(base);
      rod.position.copy(base).add(tip).multiplyScalar(.5);
      rod.scale.set(1,direction.length(),1);
      rod.quaternion.setFromUnitVectors(up,direction.normalize());
      tipFrame.position.copy(tip);
      tipFrame.quaternion.setFromRotationMatrix(displayedOutput);
    },
    dispose(){
      group.removeFromParent();
      for(const geometry of [rodGeometry,shaftGeometry,headGeometry,originGeometry])geometry.dispose();
      for(const material of [rodMaterial,originMaterial,...axisMaterials])material.dispose();
    },
  };
}
