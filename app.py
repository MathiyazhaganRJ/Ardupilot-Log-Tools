"""
ArduPilot Flight Log Analyser - Desktop Application Launcher
Launches the flight log telemetry analyser in a dedicated native application window.
"""

import os
import sys
import time
import socket
import threading
import subprocess
import webbrowser

# Ensure current directory is in sys.path
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
if BASE_DIR not in sys.path:
    sys.path.insert(0, BASE_DIR)

import server

APP_TITLE = "ArduPilot Flight Log Analyser"
APP_ICON_ICO = os.path.join(BASE_DIR, 'app_icon.ico')
APP_ICON_PNG = os.path.join(BASE_DIR, 'app_icon.png')
PORT = 8085
URL = f"http://localhost:{PORT}/index.html"


def wait_for_server(port, timeout=5.0):
    start = time.time()
    while time.time() - start < timeout:
        try:
            with socket.create_connection(('127.0.0.1', port), timeout=0.5):
                return True
        except (OSError, ConnectionRefusedError):
            time.sleep(0.1)
    return False


def find_chromium_browser():
    candidates = [
        r'C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe',
        r'C:\Program Files\Microsoft\Edge\Application\msedge.exe',
        r'C:\Program Files\Google\Chrome\Application\chrome.exe',
        r'C:\Program Files (x86)\Google\Chrome\Application\chrome.exe',
    ]
    for p in candidates:
        if os.path.exists(p):
            return p
    return None


def launch_native_window(httpd):
    """
    Launches a dedicated standalone application window.
    Priority 1: pywebview (Microsoft Edge WebView2 Chromium runtime with WebGL)
    Priority 2: Chromium App Mode (--app=http://...)
    Priority 3: Default system browser
    """
    # 1. Attempt pywebview
    try:
        import webview
        print("Launching native application window via WebView2...")
        window = webview.create_window(
            title=APP_TITLE,
            url=URL,
            width=1480,
            height=920,
            min_size=(1024, 680),
            text_select=True,
            zoomable=True,
            background_color='#0b0f17'
        )
        ico = APP_ICON_ICO if os.path.exists(APP_ICON_ICO) else None
        webview.start(icon=ico)
        return
    except Exception as e:
        print(f"pywebview window could not be opened ({e}), attempting Chromium App Mode...")

    # 2. Attempt Chromium App Mode (--app=...)
    browser_exe = find_chromium_browser()
    if browser_exe:
        print(f"Launching application window via {os.path.basename(browser_exe)} --app mode...")
        cmd = [
            browser_exe,
            f'--app={URL}',
            '--window-size=1480,920'
        ]
        proc = subprocess.Popen(cmd)
        proc.wait()
        return

    # 3. Fallback to standard web browser
    print("Launching in standard web browser...")
    webbrowser.open(URL)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass


def main():
    # 1. Start server daemon
    httpd = server.start_server()
    server_thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    server_thread.start()

    # 2. Wait for server to be responsive
    wait_for_server(PORT)

    # 3. Launch UI Window (blocks until window is closed)
    try:
        launch_native_window(httpd)
    finally:
        print("Application window closed. Shutting down server...")
        try:
            httpd.shutdown()
        except Exception:
            pass
        sys.exit(0)


if __name__ == '__main__':
    main()
