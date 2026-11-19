"""
Sharkee Flight Test Log Viewer Server
Standalone multi-threaded server providing UAV Log Viewer telemetry, SITL Sharkee 3D Visualizer, and Satellite Map.
"""

import os
import sys
import time
import json
import math
import io
import socket
import urllib.parse
import urllib.request
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import webbrowser
import threading
from PIL import Image
import parser
import organizer

PORT = 8085
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_CACHE = {}

# Local Tile & DEM Cache Directories (integrated with SITL cache if present)
LOCAL_TILE_CACHE = os.environ.get("SITL_TILE_CACHE", r"C:\Users\KARSHAK_Tamal\Desktop\Git\ArdupilotSITL\tile_cache\My_Home_Airfield")
SITL_DEM_CACHE = os.environ.get("SITL_DEM_CACHE", r"C:\Users\KARSHAK_Tamal\Desktop\Git\ArdupilotSITL\tile_cache\dem")
TOOLS_DEM_CACHE = os.path.join(BASE_DIR, 'tile_cache', 'dem')
os.makedirs(TOOLS_DEM_CACHE, exist_ok=True)


def find_or_synthesize_dem(z, x, y):
    """
    Finds or synthesizes Terrarium DEM tile for 3D elevation.
    Checks SITL cached DEM tiles, local cache, fetches remote AWS Terrarium tiles,
    or falls back to dynamic LOD parent DEM synthesis / flat sea level.
    """
    dem_z = min(z, 15)
    zoom_diff = z - dem_z
    dem_x = x >> zoom_diff
    dem_y = y >> zoom_diff

    dem_filename = f"{dem_z}_{dem_x}_{dem_y}.png"

    # 1. Exact cached DEM tile in SITL cache or Tools cache
    for d in [SITL_DEM_CACHE, TOOLS_DEM_CACHE]:
        if os.path.exists(d):
            c_file = os.path.join(d, dem_filename)
            if os.path.exists(c_file) and os.path.getsize(c_file) > 100:
                try:
                    with open(c_file, 'rb') as f:
                        return f.read(), 'image/png'
                except Exception:
                    pass

    # 2. Remote DEM fetch from AWS Terrarium if online
    try:
        url = f"https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{dem_z}/{dem_x}/{dem_y}.png"
        req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0'})
        with urllib.request.urlopen(req, timeout=4.0) as resp:
            if resp.status == 200:
                data = resp.read()
                if len(data) > 100:
                    try:
                        with open(os.path.join(TOOLS_DEM_CACHE, dem_filename), 'wb') as df:
                            df.write(data)
                    except Exception:
                        pass
                    return data, 'image/png'
    except Exception:
        pass

    # 3. Dynamic LOD Parent DEM Synthesis (searches parent zoom levels)
    try:
        for pz in range(dem_z - 1, max(0, dem_z - 6), -1):
            dz = dem_z - pz
            px = dem_x >> dz
            py = dem_y >> dz
            p_name = f"{pz}_{px}_{py}.png"
            p_cand = None
            for d in [SITL_DEM_CACHE, TOOLS_DEM_CACHE]:
                cand = os.path.join(d, p_name)
                if os.path.exists(cand) and os.path.getsize(cand) > 100:
                    p_cand = cand
                    break
            if p_cand:
                p_img = Image.open(p_cand)
                sub_span = 1 << dz
                sub_mask = sub_span - 1
                sub_x = dem_x & sub_mask
                sub_y = dem_y & sub_mask

                w, h = p_img.size
                crop_w = w / sub_span
                crop_h = h / sub_span

                x0 = int(round(sub_x * crop_w))
                y0 = int(round(sub_y * crop_h))
                x1 = int(round(x0 + crop_w))
                y1 = int(round(y0 + crop_h))

                cropped = p_img.crop((x0, y0, x1, y1))
                resized = cropped.resize((256, 256), Image.Resampling.NEAREST)
                out_io = io.BytesIO()
                resized.save(out_io, format='PNG')
                synth_bytes = out_io.getvalue()
                try:
                    with open(os.path.join(TOOLS_DEM_CACHE, dem_filename), 'wb') as df:
                        df.write(synth_bytes)
                except Exception:
                    pass
                return synth_bytes, 'image/png'
    except Exception:
        pass

    # 4. Universal flat 0m sea-level Terrarium tile fallback (RGB 128, 0, 0 -> 0.0m MSL)
    try:
        flat_dem = Image.new('RGB', (256, 256), color=(128, 0, 0))
        out_io = io.BytesIO()
        flat_dem.save(out_io, format='PNG')
        return out_io.getvalue(), 'image/png'
    except Exception:
        return b'', 'image/png'


class LogViewerHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=BASE_DIR, **kwargs)

    def handle(self):
        try:
            super().handle()
        except (ConnectionResetError, BrokenPipeError, ConnectionAbortedError):
            pass

    def finish(self):
        try:
            super().finish()
        except (ConnectionResetError, BrokenPipeError, ConnectionAbortedError):
            pass

    def end_headers(self):
        try:
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Expires', '0')
            super().end_headers()
        except (ConnectionResetError, BrokenPipeError, ConnectionAbortedError):
            pass

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path
        query = urllib.parse.parse_qs(parsed.query)

        # 1. API: List Available Flight Logs (optional ?folder=<path>)
        if path == '/api/logs':
            folder = query.get('folder', [None])[0]
            logs = parser.discover_flight_logs(folder)
            active_folder = parser.get_default_logs_dir(folder)
            self._send_json({'success': True, 'logs': logs[:250], 'folder': active_folder, 'total': len(logs)})
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
                            with urllib.request.urlopen(req, timeout=4.0) as resp:
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

        # 3b. DEM Elevation Tile proxy and disk caching: /dem/<z>/<x>/<y>.png
        elif path.startswith('/dem/'):
            clean_path = path.split('?')[0]
            parts = clean_path.split('/')
            if len(parts) >= 5:
                try:
                    z = int(parts[2])
                    x = int(parts[3])
                    y = int(parts[4].split('.')[0])

                    data, content_type = find_or_synthesize_dem(z, x, y)
                    if data:
                        self.send_response(200)
                        self.send_header('Content-Type', content_type)
                        self.send_header('Content-Length', str(len(data)))
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.send_header('Cache-Control', 'public, max-age=31536000')
                        self.end_headers()
                        self.wfile.write(data)
                        return
                except (ConnectionResetError, ConnectionAbortedError, BrokenPipeError):
                    return
                except Exception:
                    pass
            self.send_error(404, "DEM tile not found")
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

        # 5. API: Folder Selection (Native Windows Directory Picker)
        elif path == '/api/select_folder':
            mode = query.get('mode', ['source'])[0]
            current_dir = query.get('current', [''])[0]
            title = "Select Source Flight Logs Directory (SD Card or Folder)" if mode == 'source' else "Select Destination Directory for Organized Logs"
            print(f"[API] Launching native folder picker: mode={mode} title='{title}' current='{current_dir}'")
            selected_path = organizer.pick_folder_native(title=title, initial_dir=current_dir)
            print(f"[API] Folder picker result: '{selected_path}'")
            if selected_path:
                self._send_json({'success': True, 'path': selected_path})
            else:
                self._send_json({'success': False, 'cancelled': True})
            return

        # 5b. API: List Subdirectories for In-App Folder Navigation
        elif path == '/api/list_subfolders':
            target_dir = query.get('path', [''])[0]
            if not target_dir:
                drives = []
                for letter in 'CDEFGHIJKLMNOPQRSTUVWXYZ':
                    d = f"{letter}:\\"
                    if os.path.exists(d):
                        drives.append(d)
                self._send_json({'success': True, 'folders': drives, 'is_drives': True})
                return
            try:
                target_dir = os.path.normpath(target_dir)
                if os.path.isdir(target_dir):
                    subdirs = []
                    for entry in os.scandir(target_dir):
                        if entry.is_dir() and not entry.name.startswith('.'):
                            subdirs.append({'name': entry.name, 'path': entry.path})
                    subdirs.sort(key=lambda x: x['name'].lower())
                    self._send_json({'success': True, 'path': target_dir, 'folders': subdirs})
                else:
                    self._send_json({'success': False, 'error': 'Not a directory'})
            except Exception as e:
                self._send_json({'success': False, 'error': str(e)})
            return

        # 6. API: Flight Logbook & Meteorological Telemetry Database
        elif path in ('/api/historical_logbook', '/api/historical_atis'):
            target_folder = query.get('folder', [None])[0]
            force_refresh = query.get('force', ['false'])[0].lower() in ('true', '1')
            max_logs_param = query.get('max_logs', [None])[0]
            max_logs = int(max_logs_param) if max_logs_param and max_logs_param.isdigit() else None
            if not target_folder:
                default_dir = r"C:\Users\KARSHAK_Tamal\Desktop\Logs"
                target_folder = default_dir if os.path.exists(default_dir) else os.path.join(BASE_DIR, 'uploads')
            try:
                data = parser.get_historical_flight_logbook(target_folder, force_refresh=force_refresh, max_logs=max_logs)
                self._send_json(data)
            except Exception as e:
                self._send_json({'success': False, 'error': str(e)})
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

        elif path == '/api/upload':
            try:
                content_length = int(self.headers.get('Content-Length', 0))
                if content_length <= 0:
                    self._send_json({'success': False, 'error': 'Empty upload payload'})
                    return
                query = urllib.parse.parse_qs(parsed.query)
                filename = query.get('filename', [None])[0] or self.headers.get('X-File-Name', 'uploaded_log.bin')
                filename = os.path.basename(filename)
                upload_dir = os.path.join(BASE_DIR, 'uploads')
                os.makedirs(upload_dir, exist_ok=True)
                save_path = os.path.join(upload_dir, filename)

                bytes_left = content_length
                with open(save_path, 'wb') as f:
                    while bytes_left > 0:
                        chunk = self.rfile.read(min(bytes_left, 65536))
                        if not chunk:
                            break
                        f.write(chunk)
                        bytes_left -= len(chunk)

                res = parser.parse_flight_telemetry(save_path)
                LOG_CACHE[save_path] = res
                self._send_json({'success': True, 'path': save_path, 'filename': filename, 'cached': False, **res})
            except Exception as e:
                self._send_json({'success': False, 'error': str(e)})
            return

        # 5b. API: Upload and Organize Flight Log Stream
        elif path == '/api/organize_upload':
            try:
                content_length = int(self.headers.get('Content-Length', 0))
                if content_length <= 0:
                    self._send_json({'success': False, 'error': 'Empty upload payload'})
                    return
                query = urllib.parse.parse_qs(parsed.query)
                filename = query.get('filename', [None])[0] or self.headers.get('X-File-Name', 'flight_log.bin')
                dest_dir = query.get('dest', [None])[0] or self.headers.get('X-Dest-Folder', r'C:\Users\KARSHAK_Tamal\Desktop\Logs')
                structure = query.get('structure', ['year_month_date'])[0]
                filename = os.path.basename(filename)
                if not filename.lower().endswith('.bin'):
                    self._send_json({'success': False, 'error': f"Only .bin flight logs are supported. '{filename}' was rejected."})
                    return

                temp_dir = os.path.join(BASE_DIR, 'uploads', '_temp_org')
                os.makedirs(temp_dir, exist_ok=True)
                temp_path = os.path.join(temp_dir, filename)

                bytes_left = content_length
                with open(temp_path, 'wb') as f:
                    while bytes_left > 0:
                        chunk = self.rfile.read(min(bytes_left, 65536))
                        if not chunk:
                            break
                        f.write(chunk)
                        bytes_left -= len(chunk)

                date_obj = organizer.get_true_log_date(temp_path)
                year_str = date_obj.strftime("%Y")
                month_str = date_obj.strftime("%B")
                date_str = date_obj.strftime("%Y-%m-%d")

                if structure == "year_date":
                    target_dir = os.path.join(dest_dir, year_str, date_str)
                elif structure == "year_month":
                    target_dir = os.path.join(dest_dir, year_str, month_str)
                elif structure == "flat_date":
                    target_dir = os.path.join(dest_dir, date_str)
                else:
                    target_dir = os.path.join(dest_dir, year_str, month_str, date_str)

                os.makedirs(target_dir, exist_ok=True)
                target_path = os.path.join(target_dir, filename)
                temp_size = os.path.getsize(temp_path)

                if os.path.exists(target_path) and os.path.getsize(target_path) == temp_size:
                    os.remove(temp_path)
                    self._send_json({
                        'success': True,
                        'status': 'skipped',
                        'filename': filename,
                        'dest': target_path,
                        'date': date_str,
                        'size_mb': round(temp_size / (1024 * 1024), 2)
                    })
                else:
                    if os.path.exists(target_path):
                        base_stem, ext = os.path.splitext(filename)
                        counter = 1
                        collision_path = os.path.join(target_dir, f"{base_stem}_{counter}{ext}")
                        is_dup = False
                        while os.path.exists(collision_path):
                            if os.path.getsize(collision_path) == temp_size:
                                is_dup = True
                                target_path = collision_path
                                break
                            counter += 1
                            collision_path = os.path.join(target_dir, f"{base_stem}_{counter}{ext}")

                        if is_dup:
                            os.remove(temp_path)
                            self._send_json({
                                'success': True,
                                'status': 'skipped',
                                'filename': os.path.basename(target_path),
                                'dest': target_path,
                                'date': date_str,
                                'size_mb': round(temp_size / (1024 * 1024), 2)
                            })
                            return

                        target_path = collision_path

                    import shutil
                    shutil.move(temp_path, target_path)
                    self._send_json({
                        'success': True,
                        'status': 'copied',
                        'filename': os.path.basename(target_path),
                        'dest': target_path,
                        'date': date_str,
                        'size_mb': round(temp_size / (1024 * 1024), 2)
                    })
            except Exception as e:
                self._send_json({'success': False, 'error': str(e)})
            return

        # 6. API: Scan Source Directory for Flight Logs
        elif path == '/api/scan_source':
            try:
                length = int(self.headers.get('Content-Length', 0))
                body = self.rfile.read(length) if length > 0 else b'{}'
                req = json.loads(body.decode('utf-8'))
                source_folder = req.get('source_folder', '')
                res = organizer.scan_source_logs(source_folder)
                self._send_json(res)
            except Exception as e:
                self._send_json({'success': False, 'error': str(e)})
            return

        # 7. API: Organize Flight Logs into Year/Month/Date Hierarchies (with Live Streaming Progress)
        elif path == '/api/organize_logs':
            try:
                length = int(self.headers.get('Content-Length', 0))
                body = self.rfile.read(length) if length > 0 else b'{}'
                req = json.loads(body.decode('utf-8'))
                source_folder = req.get('source_folder', '')
                dest_folder = req.get('dest_folder', '').strip()
                structure = req.get('structure', 'year_month_date')
                mode = req.get('mode', 'copy')
                is_stream = req.get('stream', True)

                if is_stream:
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/x-ndjson')
                    self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()

                    def on_progress(p_data):
                        try:
                            line = (json.dumps({'type': 'progress', **p_data}) + '\n').encode('utf-8')
                            self.wfile.write(line)
                            self.wfile.flush()
                        except Exception:
                            pass

                    res = organizer.organize_flight_logs(source_folder, dest_folder, structure=structure, mode=mode, progress_callback=on_progress)
                    try:
                        final_line = (json.dumps({'type': 'complete', **res}) + '\n').encode('utf-8')
                        self.wfile.write(final_line)
                        self.wfile.flush()
                    except Exception:
                        pass
                else:
                    res = organizer.organize_flight_logs(source_folder, dest_folder, structure=structure, mode=mode)
                    self._send_json(res)
            except Exception as e:
                try:
                    self._send_json({'success': False, 'error': str(e)})
                except Exception:
                    pass
            return

        # 8. API: Open File/Directory in Windows Explorer
        elif path == '/api/open_folder':
            try:
                length = int(self.headers.get('Content-Length', 0))
                body = self.rfile.read(length) if length > 0 else b'{}'
                req = json.loads(body.decode('utf-8'))
                folder_path = req.get('path', '').strip()
                opened = organizer.open_folder_in_explorer(folder_path)
                self._send_json({'success': opened})
            except Exception as e:
                self._send_json({'success': False, 'error': str(e)})
            return

        self.send_error(404, "Not found")

    def _send_json(self, data):
        try:
            payload = json.dumps(data, allow_nan=False).encode('utf-8')
        except (ValueError, TypeError):
            import math
            def sanitize(obj):
                if isinstance(obj, float):
                    return None if (math.isnan(obj) or math.isinf(obj)) else obj
                elif isinstance(obj, dict):
                    return {k: sanitize(v) for k, v in obj.items()}
                elif isinstance(obj, list):
                    return [sanitize(v) for v in obj]
                return obj
            payload = json.dumps(sanitize(data), allow_nan=False).encode('utf-8')
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
    import argparse
    parser_cli = argparse.ArgumentParser(description="ArduPilot Flight Log Analyser Server")
    parser_cli.add_argument('--browser', action='store_true', help="Open in standard browser tab instead of app window")
    parser_cli.add_argument('--headless', action='store_true', help="Run server without opening any window")
    parser_cli.add_argument('--app', action='store_true', help="Open in standalone native app window (default)")
    cli_args, _ = parser_cli.parse_known_args()

    httpd = start_server()
    server_thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    server_thread.start()

    url = f"http://localhost:{PORT}/index.html"

    if cli_args.headless:
        print("Running in headless server mode. Press Ctrl+C to stop.")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("Server shutdown requested.")
            httpd.shutdown()
    elif cli_args.browser:
        print(f"Opening in default browser: {url} ...")
        webbrowser.open(url)
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("Server shutdown requested.")
            httpd.shutdown()
    else:
        # Default mode: launch native application window
        import app
        print("Launching application window...")
        try:
            app.launch_native_window(httpd)
        finally:
            print("Application window closed. Shutting down server...")
            try:
                httpd.shutdown()
            except Exception:
                pass
            sys.exit(0)
