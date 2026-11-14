"""
Sharkee Flight Test Log Viewer Server
Standalone multi-threaded server providing UAV Log Viewer telemetry, SITL Sharkee 3D Visualizer, and Satellite Map.
"""

import os
import sys
import time
import json
import socket
import urllib.parse
import urllib.request
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import webbrowser
import threading
import parser

PORT = 8085
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_CACHE = {}

# Local Tile Cache Directory from SITL (holds 28,000+ cached airfield tiles)
LOCAL_TILE_CACHE = os.environ.get("SITL_TILE_CACHE", r"C:\Users\KARSHAK_Tamal\Desktop\Git\ArdupilotSITL\tile_cache\My_Home_Airfield")


class LogViewerHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=BASE_DIR, **kwargs)

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path
        query = urllib.parse.parse_qs(parsed.query)

        # 1. API: List Available Flight Logs
        if path == '/api/logs':
            logs = parser.discover_flight_logs()
            self._send_json({'success': True, 'logs': logs[:80]})
            return

        # 2. API: Load and Parse Flight Log Telemetry
        elif path == '/api/load':
            target_path = query.get('path', [None])[0]
            if not target_path or not os.path.exists(target_path):
                logs = parser.discover_flight_logs()
                target_path = logs[0]['path'] if logs else None

            if not target_path or not os.path.exists(target_path):
                self._send_json({'success': False, 'error': 'No log file found'})
                return

            if target_path in LOG_CACHE:
                self._send_json({'success': True, 'cached': True, **LOG_CACHE[target_path]})
                return

            try:
                res = parser.parse_flight_telemetry(target_path)
                LOG_CACHE[target_path] = res
                self._send_json({'success': True, 'cached': False, **res})
            except Exception as e:
                self._send_json({'success': False, 'error': str(e)})
            return

        # 3. SITL Tile Server Proxy: /tiles/<z>/<x>/<y>.jpg
        elif path.startswith('/tiles/'):
            parts = path.split('/')
            if len(parts) >= 5:
                try:
                    z = parts[2]
                    x = parts[3]
                    y = parts[4].split('.')[0]

                    # Check local SITL tile cache first (high-speed offline)
                    tile_bytes = None
                    if os.path.exists(LOCAL_TILE_CACHE):
                        for prov in ['google', 'esri', 'google_hybrid']:
                            candidate = os.path.join(LOCAL_TILE_CACHE, f"{z}_{x}_{y}_{prov}.jpg")
                            if os.path.exists(candidate):
                                with open(candidate, 'rb') as f:
                                    tile_bytes = f.read()
                                break

                    # Fast fallback if online
                    if not tile_bytes:
                        try:
                            online_url = f"https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
                            req = urllib.request.Request(online_url, headers={'User-Agent': 'Mozilla/5.0'})
                            with urllib.request.urlopen(req, timeout=0.8) as resp:
                                if resp.status == 200:
                                    tile_bytes = resp.read()
                                    if os.path.exists(LOCAL_TILE_CACHE):
                                        try:
                                            with open(os.path.join(LOCAL_TILE_CACHE, f"{z}_{x}_{y}_esri.jpg"), 'wb') as tf:
                                                tf.write(tile_bytes)
                                        except Exception:
                                            pass
                        except Exception:
                            pass

                    if tile_bytes:
                        self.send_response(200)
                        self.send_header('Content-Type', 'image/jpeg')
                        self.send_header('Content-Length', str(len(tile_bytes)))
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.send_header('Cache-Control', 'public, max-age=31536000')
                        self.end_headers()
                        self.wfile.write(tile_bytes)
                        return
                except Exception:
                    pass

            self.send_error(404, "Tile not found")
            return

        # 4. Mock SITL Endpoints for clean console operation
        elif path == '/api/waypoints':
            self._send_json({'success': True, 'waypoints': []})
            return
        elif path == '/api/list_ports':
            self._send_json({'ports': []})
            return
        elif path == '/api/cache_status':
            count = 0
            size_mb = 0.0
            if os.path.exists(LOCAL_TILE_CACHE):
                try:
                    files = os.listdir(LOCAL_TILE_CACHE)
                    count = len(files)
                    size_mb = count * 0.015  # avg 15KB per JPEG tile
                except Exception:
                    pass
            self._send_json({
                'success': True,
                'cached': True,
                'cached_count': count,
                'cached_mb': round(size_mb, 1),
                'preset_dir': LOCAL_TILE_CACHE,
                'is_online': True,
                'available_presets': ['My_Home_Airfield'],
                'preset_metadata': {
                    'My_Home_Airfield': {'tiles_cached': count, 'mb': round(size_mb, 1)}
                }
            })
            return

        super().do_GET()

    def do_POST(self):
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path

        if path == '/api/calculate':
            length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(length)
            try:
                req = json.loads(body.decode('utf-8'))
                target_path = req.get('path')
                t_start = float(req.get('start', 0))
                t_end = float(req.get('end', 0))
                weight_kg = float(req.get('weight_kg', 5.1))
                wing_area = float(req.get('wing_area', 0.45))

                if not target_path or target_path not in LOG_CACHE:
                    if target_path and os.path.exists(target_path):
                        LOG_CACHE[target_path] = parser.parse_flight_telemetry(target_path)
                    else:
                        self._send_json({'success': False, 'error': 'Log not parsed yet'})
                        return

                cached_data = LOG_CACHE[target_path]
                metrics = parser.compute_regime_metrics(
                    cached_data['telemetry'],
                    t_start,
                    t_end,
                    weight_kg=weight_kg,
                    wing_area_m2=wing_area
                )

                if metrics:
                    self._send_json({'success': True, 'metrics': metrics})
                else:
                    self._send_json({'success': False, 'error': 'No data in interval'})
            except Exception as e:
                self._send_json({'success': False, 'error': str(e)})
            return

        self.send_error(404, "Not found")

    def _send_json(self, data):
        payload = json.dumps(data).encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(payload)))
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
        self.end_headers()
        self.wfile.write(payload)

    def log_message(self, format, *args):
        pass


def release_port(port):
    """Kills any stale process occupying the port on Windows."""
    if sys.platform == 'win32':
        import subprocess
        try:
            out = subprocess.check_output(f'netstat -ano | findstr :{port}', shell=True).decode('utf-8')
            for line in out.strip().split('\n'):
                parts = line.strip().split()
                if len(parts) >= 5 and 'LISTENING' in parts:
                    pid = int(parts[-1])
                    if pid != os.getpid() and pid > 0:
                        print(f"Terminating old process on port {port} (PID {pid})...")
                        os.system(f'taskkill /F /PID {pid} >nul 2>&1')
                        time.sleep(0.5)
        except Exception:
            pass


def start_server():
    release_port(PORT)
    server = ThreadingHTTPServer(("", PORT), LogViewerHandler)
    server.daemon_threads = True
    print(f"Shark Hawk Digital Twin Server listening at: http://localhost:{PORT}")
    return server


if __name__ == '__main__':
    httpd = start_server()
    server_thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    server_thread.start()

    url = f"http://localhost:{PORT}/index.html"
    print(f"Opening {url} ...")
    webbrowser.open(url)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Server shutdown requested.")
        httpd.shutdown()
