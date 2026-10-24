/* ==========================================================================
 * Service Worker — PWA 离线缓存
 * 目的:机器人机载计算机关机/重启过程中,已访问过的控制台页面仍可从缓存加载,
 *       不会出现"无法访问此网站"。WebSocket/遥测/命令不缓存,离线时前端显示断开态。
 *
 * 缓存策略:
 *   1. 应用壳(HTML 导航):network-first → 离线回退缓存
 *   2. 静态资源(js/css/png/svg/woff/json 等):stale-while-revalidate(秒开+后台更新)
 *   3. WebSocket(/ws)与 /api/* 等动态请求:网络直通,不缓存
 *   4. POST 请求:不缓存
 * 版本管理:CACHE_VERSION 每次发版递增,activate 阶段清理旧缓存。
 * ========================================================================== */

const CACHE_VERSION = 'v60-fresh-nav-20261021';
const STATIC_CACHE = `static-${CACHE_VERSION}`;
const RUNTIME_CACHE = `runtime-${CACHE_VERSION}`;

// 预缓存:应用壳的核心文件,首次安装即缓存,保证首屏可离线。
// 其余资源在首次访问时由 stale-while-revalidate 自动加入 RUNTIME_CACHE。
const PRECACHE_URLS = [
  './',
  './index.html',
  './mobile.html',
  './login.html',
  './manifest.json',
];

/* ── install:预缓存应用壳 ─────────────────────────────────────────────── */
self.addEventListener('install', (event) => {
  event.waitUntil(
    caches.open(STATIC_CACHE).then((cache) =>
      // 单个失败不阻断整体(非核心资源缺失不影响安装)
      Promise.allSettled(
        PRECACHE_URLS.map((url) =>
          cache.add(url).catch(() => {
            /* 忽略:可能此时服务未就绪,首次访问时由运行时缓存补齐 */
          })
        )
      )
    ).then(() => self.skipWaiting())
  );
});

/* ── activate:清理旧版本缓存,立即接管 ─────────────────────────────────── */
self.addEventListener('activate', (event) => {
  event.waitUntil(
    caches
      .keys()
      .then((keys) =>
        Promise.all(
          keys
            .filter((k) => k !== STATIC_CACHE && k !== RUNTIME_CACHE)
            .map((k) => caches.delete(k))
        )
      )
      .then(() => self.clients.claim())
  );
});

/* ── 是否需要缓存的请求判定 ───────────────────────────────────────────── */
function shouldBypass(request) {
  // 只处理 GET
  if (request.method !== 'GET') return true;
  const url = new URL(request.url);
  // WebSocket:绝不缓存(浏览器对 ws:// 也不会走 fetch,但显式排除更安全)
  if (url.protocol === 'ws:' || url.protocol === 'wss:') return true;
  // 动态接口:遥测/命令/文件管理,一律网络直通
  const path = url.pathname;
  if (
    path === '/ws' ||
    path.startsWith('/api/') ||
    path.startsWith('/webssh') ||
    path.startsWith('/webftp')
  ) {
    return true;
  }
  return false;
}

/* ── 是否为静态资源(可走 stale-while-revalidate) ──────────────────────── */
function isStaticAsset(url) {
  return /\.(js|css|png|jpe?g|gif|svg|ico|woff2?|ttf|eot|json|webp|wasm)$/i.test(url.pathname);
}

/* ── fetch 主路由 ─────────────────────────────────────────────────────── */
self.addEventListener('fetch', (event) => {
  const { request } = event;
  if (shouldBypass(request)) return; // 网络直通

  const url = new URL(request.url);

  // 导航请求(HTML 页面):network-first,离线回退缓存。
  // cache:'no-cache' 显式绕过浏览器 HTTP 缓存(启发式缓存曾导致"改版后刷新仍看到旧页")
  if (request.mode === 'navigate') {
    event.respondWith(
      fetch(request, { cache: 'no-cache' })
        .then((response) => {
          const copy = response.clone();
          caches.open(RUNTIME_CACHE).then((cache) => cache.put(request, copy));
          return response;
        })
        .catch(() =>
          caches.match(request).then(
            (cached) => cached || caches.match('./index.html')
          )
        )
    );
    return;
  }

  // 静态资源:stale-while-revalidate
  if (isStaticAsset(url)) {
    event.respondWith(
      caches.match(request).then((cached) => {
        const networkFetch = fetch(request)
          .then((response) => {
            if (response && response.status === 200) {
              const copy = response.clone();
              caches.open(RUNTIME_CACHE).then((cache) => cache.put(request, copy));
            }
            return response;
          })
          .catch(() => cached);
        return cached || networkFetch;
      })
    );
    return;
  }

  // 其他 GET:网络优先,失败回退缓存(如 manifest、favicon 等)
  event.respondWith(
    fetch(request)
      .then((response) => {
        const copy = response.clone();
        caches.open(RUNTIME_CACHE).then((cache) => cache.put(request, copy));
        return response;
      })
      .catch(() => caches.match(request))
  );
});

/* ── 版本更新通知:SW 更新后向所有客户端发消息提示刷新 ─────────────────── */
self.addEventListener('message', (event) => {
  if (event.data && event.data.type === 'SKIP_WAITING') {
    self.skipWaiting();
  }
});
