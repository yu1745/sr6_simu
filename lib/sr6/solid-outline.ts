import * as T from 'three';

// Weld coincident CAD tessellation vertices so seams between surface patches
// share adjacency. Keep face normals: interpolated shading normals cannot tell
// which two faces border an edge.
export function solidOutlineGeometry(source:T.BufferGeometry,angle=20){
  const position=source.getAttribute('position'),index=source.getIndex();
  const vertices:T.Vector3[]=[],welded:number[]=[],keys=new Map<string,number>();
  for(let i=0;i<position.count;i++){
    const p=new T.Vector3().fromBufferAttribute(position,i);
    const key=[p.x,p.y,p.z].map(v=>Math.round(v*1e5)).join(',');
    let id=keys.get(key);if(id===undefined){id=vertices.length;keys.set(key,id);vertices.push(p);}welded.push(id);
  }
  const edges=new Map<string,{a:number;b:number;normals:T.Vector3[]}>();
  const count=index?.count??position.count;
  for(let i=0;i<count;i+=3){
    const ids=[0,1,2].map(j=>welded[index?index.getX(i+j):i+j]);
    const [a,b,c]=ids.map(id=>vertices[id]);
    const normal=new T.Vector3().subVectors(b,a).cross(new T.Vector3().subVectors(c,a));
    if(normal.lengthSq()<1e-20)continue;normal.normalize();
    for(let j=0;j<3;j++){
      const a=ids[j],b=ids[(j+1)%3],key=a<b?`${a}:${b}`:`${b}:${a}`;
      const edge=edges.get(key);if(edge)edge.normals.push(normal);else edges.set(key,{a,b,normals:[normal]});
    }
  }
  const positions:number[]=[],normalA:number[]=[],normalB:number[]=[],centers:number[]=[],features:number[]=[];
  const threshold=Math.cos(T.MathUtils.degToRad(angle));
  for(const {a,b,normals} of edges.values()){
    const n0=normals[0],n1=normals[1]??n0;
    // Boundary/non-manifold edges remain visible, as do sharp feature edges.
    const feature=normals.length!==2||n0.dot(n1)<=threshold;
    if(!feature&&n0.dot(n1)>1-1e-10)continue;
    const center=vertices[a].clone().add(vertices[b]).multiplyScalar(.5);
    for(const id of [a,b]){positions.push(...vertices[id].toArray());normalA.push(...n0.toArray());normalB.push(...n1.toArray());centers.push(...center.toArray());features.push(feature?1:0);}
  }
  const geometry=new T.BufferGeometry();
  geometry.setAttribute('position',new T.Float32BufferAttribute(positions,3));
  geometry.setAttribute('faceNormalA',new T.Float32BufferAttribute(normalA,3));
  geometry.setAttribute('faceNormalB',new T.Float32BufferAttribute(normalB,3));
  geometry.setAttribute('edgeCenter',new T.Float32BufferAttribute(centers,3));
  geometry.setAttribute('feature',new T.Float32BufferAttribute(features,1));
  return geometry;
}

export function solidOutlineMaterial(){
  return new T.ShaderMaterial({
    transparent:true,depthWrite:false,
    uniforms:{color:{value:new T.Color(0xe6a79e)},opacity:{value:.75}},
    vertexShader:`
      attribute vec3 faceNormalA;
      attribute vec3 faceNormalB;
      attribute vec3 edgeCenter;
      attribute float feature;
      varying float showEdge;
      void main(){
        vec3 center=(modelViewMatrix*vec4(edgeCenter,1.0)).xyz;
        vec3 towardCamera=projectionMatrix[3][3]==1.0?vec3(0.0,0.0,1.0):-center;
        float a=dot(normalMatrix*faceNormalA,towardCamera);
        float b=dot(normalMatrix*faceNormalB,towardCamera);
        // A silhouette separates a camera-facing and a back-facing triangle.
        // Evaluated on the GPU for the current camera AND displayed solid pose.
        showEdge=max(feature,(a<=0.0&&b>=0.0)||(b<=0.0&&a>=0.0)?1.0:0.0);
        gl_Position=projectionMatrix*modelViewMatrix*vec4(position,1.0);
      }`,
    fragmentShader:`
      uniform vec3 color;
      uniform float opacity;
      varying float showEdge;
      void main(){
        if(showEdge<0.5)discard;
        gl_FragColor=vec4(color,opacity);
        #include <tonemapping_fragment>
        #include <colorspace_fragment>
      }`,
  });
}
