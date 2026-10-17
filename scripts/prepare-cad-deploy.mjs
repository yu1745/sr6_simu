import { readFile, writeFile } from 'node:fs/promises';
const directory = new URL('../dist/client/', import.meta.url);
for(const prefix of ['sr6','sr6-6010']) {
const index = new URL(`${prefix}-cad.json`,directory);
const data = JSON.parse(await readFile(index,'utf8'));
const masses=JSON.parse(await readFile(new URL(`${prefix}-mass-properties.json`,directory),'utf8'));
if(masses.version!==1||masses.sha256!==data.sha256)throw Error('CAD mass properties are stale; run pnpm cad:mass');
if (!data.partFiles) {
  const limit = 10 * 1024 * 1024;
  const chunks = []; let chunk = [], size = 2;
  for (const part of data.parts) {
    const encoded = JSON.stringify(part), bytes = Buffer.byteLength(encoded)+1;
    if (bytes>limit) throw Error(`CAD part ${part.id} exceeds deployment chunk size`);
    if (size+bytes>limit && chunk.length) {chunks.push(chunk);chunk=[];size=2;}
    chunk.push(encoded);size+=bytes;
  }
  if(chunk.length)chunks.push(chunk);
  const files = [];
  for (const [i,parts] of chunks.entries()) {
    const name = `${prefix}-cad-${data.sha256.slice(0,12)}-${i}.json`;
    await writeFile(new URL(name,directory),`[${parts.join(',')}]`);
    files.push(`/${name}`);
  }
  await writeFile(index,JSON.stringify({...data,parts:[],partFiles:files}));
  console.log(`CAD deployment: ${data.parts.length} parts in ${files.length} chunks (no precision loss)`);
}
}
