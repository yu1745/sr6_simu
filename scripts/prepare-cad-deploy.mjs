import { readFile, writeFile } from 'node:fs/promises';
const directory = new URL('../dist/client/', import.meta.url);
const index = new URL('sr6-cad.json',directory);
const data = JSON.parse(await readFile(index,'utf8'));
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
    const name = `sr6-cad-${data.sha256.slice(0,12)}-${i}.json`;
    await writeFile(new URL(name,directory),`[${parts.join(',')}]`);
    files.push(`/${name}`);
  }
  await writeFile(index,JSON.stringify({...data,parts:[],partFiles:files}));
  console.log(`CAD deployment: ${data.parts.length} parts in ${files.length} chunks (no precision loss)`);
}
