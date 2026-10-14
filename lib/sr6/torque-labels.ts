import * as T from 'three';
import { torqueExpression, type TorqueAnalysis } from './statics.ts';

/** Screen-sized annotations linked to each motor's 3D shaft center. */
export function createTorqueLabels(canvas:HTMLCanvasElement){
  const layer=document.createElement('div');layer.className='motor-torque-overlay';
  layer.setAttribute('aria-label','电机补偿扭矩');
  const svg=document.createElementNS('http://www.w3.org/2000/svg','svg');
  svg.classList.add('motor-torque-leaders');layer.appendChild(svg);
  const labels=Array.from({length:6},(_,i)=>{
    const element=document.createElement('div');element.className='motor-torque-label';
    const heading=document.createElement('div');heading.className='motor-torque-heading';
    const title=document.createElement('strong');title.textContent=`M${i+1}`;
    const unit=document.createElement('small');unit.textContent='N·m';heading.appendChild(title);heading.appendChild(unit);
    const value=document.createElement('span');value.className='motor-torque-value';value.textContent='—';
    element.appendChild(heading);element.appendChild(value);layer.appendChild(element);
    const line=document.createElementNS('http://www.w3.org/2000/svg','line');svg.appendChild(line);
    const dot=document.createElementNS('http://www.w3.org/2000/svg','circle');dot.setAttribute('r','2.5');svg.appendChild(dot);
    return {element,value,line,dot};
  });
  canvas.parentElement?.appendChild(layer);
  const projected=new T.Vector3(),inCamera=new T.Vector3();
  return {
    values(result:TorqueAnalysis){
      labels.forEach((label,i)=>{label.value.textContent=torqueExpression(result,i);label.element.classList.toggle('invalid',!result.valid);});
      layer.title=result.valid?'总扭矩(重力补偿+惯量补偿)；正号对应各电机正角度方向':result.reason;
    },
    update(anchors:readonly T.Vector3[],camera:T.Camera,visible:boolean){
      layer.hidden=!visible;
      if(!visible)return;
      const width=canvas.clientWidth,height=canvas.clientHeight;
      const labelWidth=Math.min(204,Math.max(125,width-20));
      const labelHeight=45,gap=7,top=Math.min(122,height*.25),bottom=height-38;
      const points=anchors.map((anchor,index)=>{
        projected.copy(anchor).project(camera);inCamera.copy(anchor).applyMatrix4(camera.matrixWorldInverse);
        const visible=inCamera.z<0&&Math.abs(projected.x)<1.15&&Math.abs(projected.y)<1.15&&Math.abs(projected.z)<=1;
        labels[index].element.hidden=!visible;labels[index].line.style.display=visible?'':'none';labels[index].dot.style.display=visible?'':'none';
        return {index,x:(projected.x+1)*width/2,y:(1-projected.y)*height/2,visible};
      }).filter(p=>p.visible).sort((a,b)=>a.x-b.x);
      const half=Math.ceil(points.length/2);
      const columns=width>=500?[points.slice(0,half),points.slice(half)]:[points];
      for(const [side,column] of columns.entries()){
        column.sort((a,b)=>a.y-b.y);
        const x=side===0?10:width-labelWidth-10;
        const available=Math.max(0,bottom-top-column.length*labelHeight);
        const spacing=Math.min(gap,column.length>1?available/(column.length-1):0);
        const ys:number[]=[];
        for(const [row,p] of column.entries()){
          const maxY=bottom-(column.length-row)*labelHeight-(column.length-row-1)*spacing;
          ys.push(Math.max(row===0?top:ys[row-1]+labelHeight+spacing,Math.min(maxY,p.y-labelHeight/2)));
        }
        column.forEach((p,row)=>{
          const {element,line,dot}=labels[p.index];
          element.style.width=`${labelWidth}px`;element.style.transform=`translate(${x}px,${ys[row]}px)`;
          line.setAttribute('x1',String(p.x));line.setAttribute('y1',String(p.y));
          line.setAttribute('x2',String(side===0?x+labelWidth:x));line.setAttribute('y2',String(ys[row]+labelHeight/2));
          dot.setAttribute('cx',String(p.x));dot.setAttribute('cy',String(p.y));
        });
      }
    },
    dispose(){layer.remove();},
  };
}
