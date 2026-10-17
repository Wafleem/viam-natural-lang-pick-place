import { mkdir, copyFile, rm, readFile, writeFile } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import path from 'node:path';
const root = path.dirname(fileURLToPath(import.meta.url));
const output = path.join(root, 'dist');
// This directory contains generated output only. Drop any previous SDK assets.
await rm(output, { recursive: true, force: true });
await mkdir(path.join(output, 'src'), { recursive: true });
await mkdir(path.join(output, 'vendor'), { recursive: true });
for (const name of ['index.html', 'src/app.js', 'src/parser.js', 'src/speech.js', 'src/code2-contract.js', 'src/review-controller.js', 'src/style.css']) {
  await copyFile(path.join(root, name), path.join(output, name));
}
const sdkPackage = JSON.parse(await readFile(path.join(root, 'node_modules/@viamrobotics/sdk/package.json'), 'utf8'));
if (sdkPackage.version !== '0.76.0') throw new Error('Install the pinned Viam SDK with npm ci before building.');
await copyFile(path.join(root, 'node_modules/@viamrobotics/sdk/dist/main.es.js'), path.join(output, 'vendor/viam-sdk.js'));
await copyFile(path.join(root, 'node_modules/@viamrobotics/sdk/LICENSE'), path.join(output, 'vendor/VIAM-SDK-LICENSE.txt'));
const connectionSource = await readFile(path.join(root, 'src/viam-connection.js'), 'utf8');
await writeFile(path.join(output, 'src/viam-connection.js'), replaceOnce(connectionSource, "from '@viamrobotics/sdk'", "from '../vendor/viam-sdk.js'"));
const read = name => readFile(path.join(root, name), 'utf8');
let [html, css, parser, speech, contract, app] = await Promise.all([
  read('preview/index.html'), read('src/style.css'), read('src/parser.js'), read('src/speech.js'), read('src/code2-contract.js'), read('preview/app.js'),
]);
function replaceOnce(source, expected, replacement) {
  if (source.split(expected).length !== 2) throw new Error(`Standalone bundle expected exactly one occurrence of: ${expected}`);
  return source.replace(expected, () => replacement);
}
// These are the complete, fixed module boundaries. Unknown syntax fails closed.
parser = replaceOnce(parser, 'export const MAX_TEXT_CHARS', 'const MAX_TEXT_CHARS');
parser = replaceOnce(parser, 'export const MAX_TEXT_WORDS', 'const MAX_TEXT_WORDS');
parser = replaceOnce(parser, 'export function parseIntent', 'function parseIntent');
speech = replaceOnce(speech, 'export function setupSpeech', 'function setupSpeech');
contract = replaceOnce(contract, 'export const CODE2_REFERENCE', 'const CODE2_REFERENCE');
contract = replaceOnce(contract, 'export function previewCode2Command', 'function previewCode2Command');
contract = replaceOnce(contract, 'export const isSupportedImplementation', 'const isSupportedImplementation');
contract = replaceOnce(contract, 'export function placeTarget', 'function placeTarget');
contract = replaceOnce(contract, 'export function describeTarget', 'function describeTarget');
app = replaceOnce(app, "import { parseIntent, MAX_TEXT_CHARS, MAX_TEXT_WORDS } from './parser.js';\n", '');
app = replaceOnce(app, "import { setupSpeech } from './speech.js';\n", '');
app = replaceOnce(app, "import { CODE2_REFERENCE, previewCode2Command } from './code2-contract.js';\n", '');
const script = `(function () {\n'use strict';\nconst { parseIntent, MAX_TEXT_CHARS, MAX_TEXT_WORDS } = (function () {\n${parser}\nreturn { parseIntent, MAX_TEXT_CHARS, MAX_TEXT_WORDS };\n})();\nconst { setupSpeech } = (function () {\n${speech}\nreturn { setupSpeech };\n})();\nconst { CODE2_REFERENCE, previewCode2Command } = (function () {\n${contract}\nreturn { CODE2_REFERENCE, previewCode2Command };\n})();\n${app}\n})();`;
if (/^\s*(?:import|export)\b/m.test(script) || /<\/script/i.test(script) || /<\/style/i.test(css)) {
  throw new Error('Unexpected module or HTML boundary in standalone bundle.');
}
html = replaceOnce(html, '<link rel="stylesheet" href="./src/style.css">', `<style>\n${css}\n</style>`);
html = replaceOnce(html, '<script type="module" src="./src/app.js"></script>', `<script>\n${script}\n</script>`);
html = replaceOnce(html, 'href="./"', 'href="#"');
await writeFile(path.join(output, 'translator.html'), html);
console.log('Built dist/ review-and-send console with pinned local Viam SDK, plus standalone translation-only dist/translator.html.');
