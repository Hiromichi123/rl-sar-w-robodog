// 临时开发服务器(仅用于本地调试,不参与正式功能)
const http = require('http');
const fs = require('fs');
const path = require('path');
const zlib = require('zlib');

const MIME = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.mjs': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
  '.png': 'image/png',
  '.jpg': 'image/jpeg',
  '.jpeg': 'image/jpeg',
  '.obj': 'application/octet-stream',
  '.mtl': 'text/plain',
  '.xml': 'text/xml',
  '.svg': 'image/svg+xml',
  '.ico': 'image/x-icon',
};

// 可压缩的文本类资源扩展名(gzip 压缩率通常 70%+):
// three.core.js 1.35MB → ~268KB, 外网加载速度提升 5 倍
const COMPRESSIBLE_EXTS = new Set(['.html', '.js', '.mjs', '.css', '.json', '.obj', '.mtl', '.xml', '.svg']);

// 缓存策略:
//  - 带 ?v= 版本号的 JS/CSS → 缓存 1 年 immutable(改代码 bump 版本号自动失效)
//  - 模型文件(.obj/.stl/.xml) → 缓存 1 天(不常变,避免每次刷新加 18MB)
//  - 图片 → 缓存 1 天
//  - HTML → no-cache(始终重新校验,保证拿到最新入口)
const VERSIONED_CACHE = 'public, max-age=31536000, immutable';
const MODEL_CACHE     = 'public, max-age=86400';
const IMAGE_CACHE     = 'public, max-age=86400';
const NO_CACHE        = 'no-cache, must-revalidate';
const VERSIONED_EXTS  = new Set(['.js', '.mjs', '.css']);
const MODEL_EXTS      = new Set(['.obj', '.stl', '.xml', '.mtl']);
const IMAGE_EXTS      = new Set(['.png', '.jpg', '.jpeg', '.svg', '.ico']);

function cacheControlFor(ext, hasVersion) {
  if (hasVersion && VERSIONED_EXTS.has(ext)) return VERSIONED_CACHE;
  if (MODEL_EXTS.has(ext)) return MODEL_CACHE;
  if (IMAGE_EXTS.has(ext)) return IMAGE_CACHE;
  return NO_CACHE;
}

const ROOT = path.resolve(__dirname, '..');

http.createServer((req, res) => {
  const rawUrl = req.url;
  let urlPath = decodeURIComponent(rawUrl.split('?')[0]);
  const hasVersion = /[?&]v=/.test(rawUrl); // URL 带 ?v= 版本号
  if (urlPath === '/') urlPath = '/index.html';
  const filePath = path.join(ROOT, urlPath);
  // 防止目录穿越
  if (!filePath.startsWith(ROOT)) {
    res.writeHead(403); res.end('403 Forbidden');
    return;
  }
  fs.readFile(filePath, (err, data) => {
    if (err) {
      res.writeHead(404, { 'Content-Type': 'text/plain' });
      res.end(`404 Not Found: ${urlPath}`);
      return;
    }
    const ext = path.extname(filePath).toLowerCase();
    const headers = {
      'Content-Type': MIME[ext] || 'application/octet-stream',
      'Access-Control-Allow-Origin': '*',
      'Cache-Control': cacheControlFor(ext, hasVersion),
    };
    // 文本类资源启用 gzip 压缩(浏览器需声明 Accept-Encoding: gzip)
    const acceptEnc = req.headers['accept-encoding'] || '';
    const shouldCompress = COMPRESSIBLE_EXTS.has(ext) && /\bgzip\b/.test(acceptEnc);
    if (shouldCompress) {
      zlib.gzip(data, (gzipErr, compressed) => {
        if (gzipErr) { res.writeHead(500); res.end('500 Internal Server Error'); return; }
        headers['Content-Encoding'] = 'gzip';
        headers['Content-Length'] = compressed.length;
        res.writeHead(200, headers);
        res.end(compressed);
      });
    } else {
      headers['Content-Length'] = data.length;
      res.writeHead(200, headers);
      res.end(data);
    }
  });
}).listen(5173, '0.0.0.0', () => {
  console.log('Dev server running at http://127.0.0.1:5173/ (gzip + smart cache)');
});
