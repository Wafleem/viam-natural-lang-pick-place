import { createServer } from 'node:http';
import { readFile } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import path from 'node:path';
const root = path.join(path.dirname(fileURLToPath(import.meta.url)), 'dist');
const allowed = /^(?:(?:index|translator)\.html|src\/(?:app|parser|speech|code2-contract|review-controller|viam-connection)\.js|src\/style\.css|vendor\/(?:viam-sdk\.js|VIAM-SDK-LICENSE\.txt))$/;
const types = { '.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8', '.css': 'text/css; charset=utf-8', '.txt': 'text/plain; charset=utf-8' };
const server = createServer(async (req, res) => {
  let requested;
  try { requested = decodeURIComponent(new URL(req.url, 'http://localhost').pathname).replace(/^\//, '') || 'index.html'; }
  catch { res.writeHead(400).end(); return; }
  // Allow only public application files, never package files, source backups or secrets.
  if (req.method !== 'GET' || !allowed.test(requested)) { res.writeHead(404).end(); return; }
  try {
    const data = await readFile(path.join(root, requested));
    res.writeHead(200, { 'Content-Type': types[path.extname(requested)], 'Cache-Control': 'no-store',
      'X-Content-Type-Options': 'nosniff', 'Referrer-Policy': 'no-referrer' }); res.end(data);
  } catch { res.writeHead(404).end(); }
});
server.listen(5173, '127.0.0.1', () => console.log('Armfarm review-and-send console: http://127.0.0.1:5173/ (disconnected until explicit Connect; offline translator at /translator.html)'));
