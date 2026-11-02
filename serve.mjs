// Serveur statique minimal (aucune dépendance) pour l'application web.
// Les modules ES ne se chargent pas depuis file://, d'où ce serveur.
// Usage : node serve.mjs [--port 8080]
import { createServer } from 'node:http';
import { readFile, stat } from 'node:fs/promises';
import { extname, join, normalize, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const ROOT = resolve(fileURLToPath(new URL('.', import.meta.url)));
const portArg = process.argv.indexOf('--port');
const PORT = portArg > 0 ? Number(process.argv[portArg + 1]) : Number(process.env.PORT || 8080);

const MIME = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.mjs': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
  '.csv': 'text/csv; charset=utf-8',
  '.txt': 'text/plain; charset=utf-8',
  '.md': 'text/markdown; charset=utf-8',
  '.svg': 'image/svg+xml',
  '.png': 'image/png',
};

createServer(async (req, res) => {
  try {
    const path = decodeURIComponent(new URL(req.url, 'http://x').pathname);
    if (path === '/' || path === '/app') {
      // l'appli charge style.css et app.js en relatif : il faut l'URL /app/
      res.writeHead(302, { Location: '/app/' }).end();
      return;
    }
    const file = normalize(join(ROOT, path));
    if (!file.startsWith(ROOT)) {
      res.writeHead(403).end('Forbidden');
      return;
    }
    const info = await stat(file);
    const target = info.isDirectory() ? join(file, 'index.html') : file;
    const body = await readFile(target);
    res.writeHead(200, {
      'Content-Type': MIME[extname(target).toLowerCase()] || 'application/octet-stream',
      'Cache-Control': 'no-store',
    });
    res.end(body);
  } catch {
    res.writeHead(404).end('Not found');
  }
}).listen(PORT, () => {
  console.log(`Application : http://localhost:${PORT}/`);
});
