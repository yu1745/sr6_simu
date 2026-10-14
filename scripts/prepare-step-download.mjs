import { createHash } from 'node:crypto';
import { readFileSync, writeFileSync } from 'node:fs';
import { deflateRawSync } from 'node:zlib';

// The source assembly is the calibrated CAD zero pose: every part's rig
// transform is identity at ZERO (verified by scripts/test-cad-rig.mjs).
const root = new URL('../', import.meta.url);
const step = readFileSync(new URL('cad/sr6_mini_complete.stp', root));
const cad = JSON.parse(readFileSync(new URL('public/sr6-cad.json', root), 'utf8'));
const sha256 = createHash('sha256').update(step).digest('hex');
if (cad.source !== 'sr6_mini_complete.stp' || cad.sha256 !== sha256) {
  throw new Error('STEP download and displayed CAD differ; regenerate the CAD assets first');
}

// Cloudflare static assets have a per-file size limit. Keep the exact STEP
// bytes in a standard single-entry ZIP, with no geometry conversion.
const name = Buffer.from('sr6_mini_complete_centered.stp');
const compressed = deflateRawSync(step, { level: 9 });
if (compressed.length > 25 * 1024 * 1024) throw new Error('Compressed STEP exceeds the static asset limit');
const crcTable = Array.from({ length: 256 }, (_, n) => {
  let value = n;
  for (let i = 0; i < 8; i++) value = value & 1 ? 0xedb88320 ^ (value >>> 1) : value >>> 1;
  return value >>> 0;
});
let crc = 0xffffffff;
for (const byte of step) crc = crcTable[(crc ^ byte) & 255] ^ (crc >>> 8);
crc = (crc ^ 0xffffffff) >>> 0;

const local = Buffer.alloc(30);
local.writeUInt32LE(0x04034b50, 0);
local.writeUInt16LE(20, 4);
local.writeUInt16LE(8, 6); // UTF-8 filename
local.writeUInt16LE(8, 8); // raw DEFLATE
local.writeUInt32LE(crc, 14);
local.writeUInt32LE(compressed.length, 18);
local.writeUInt32LE(step.length, 22);
local.writeUInt16LE(name.length, 26);

const central = Buffer.alloc(46);
central.writeUInt32LE(0x02014b50, 0);
central.writeUInt16LE(20, 4);
central.writeUInt16LE(20, 6);
central.writeUInt16LE(8, 8);
central.writeUInt16LE(8, 10);
central.writeUInt32LE(crc, 16);
central.writeUInt32LE(compressed.length, 20);
central.writeUInt32LE(step.length, 24);
central.writeUInt16LE(name.length, 28);
const centralOffset = local.length + name.length + compressed.length;
const end = Buffer.alloc(22);
end.writeUInt32LE(0x06054b50, 0);
end.writeUInt16LE(1, 8);
end.writeUInt16LE(1, 10);
end.writeUInt32LE(central.length + name.length, 12);
end.writeUInt32LE(centralOffset, 16);

const output = new URL('public/sr6_mini_centered_step.zip', root);
writeFileSync(output, Buffer.concat([local, name, compressed, central, name, end]));
console.log(`Centered STEP download: ${step.length} bytes -> ${compressed.length} bytes (${sha256.slice(0, 12)})`);
