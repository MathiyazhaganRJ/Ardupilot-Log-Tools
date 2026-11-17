"""
Creates a Windows Desktop Shortcut for the ArduPilot Flight Log Analyser.
Places a high-resolution icon on the user's Desktop that launches the app
as a native, frameless/standalone window without a console prompt.
"""

import os
import sys
import subprocess

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
APP_PY = os.path.join(BASE_DIR, "app.py")
ICON_ICO = os.path.join(BASE_DIR, "app_icon.ico")
DESKTOP_DIR = os.path.join(os.path.expanduser("~"), "Desktop")
SHORTCUT_PATH = os.path.join(DESKTOP_DIR, "ArduPilot Flight Log Analyser.lnk")

# Locate pythonw.exe
py_dir = os.path.dirname(sys.executable)
pythonw_exe = os.path.join(py_dir, "pythonw.exe")
if not os.path.exists(pythonw_exe):
    pythonw_exe = sys.executable


def create_shortcut():
    ps_script = f"""
    $WshShell = New-Object -ComObject WScript.Shell
    $Shortcut = $WshShell.CreateShortcut("{SHORTCUT_PATH}")
    $Shortcut.TargetPath = "{pythonw_exe}"
    $Shortcut.Arguments = '"{APP_PY}"'
    $Shortcut.WorkingDirectory = "{BASE_DIR}"
    $Shortcut.IconLocation = "{ICON_ICO},0"
    $Shortcut.Description = "ArduPilot Flight Log Analyser & Archiver Desktop Application"
    $Shortcut.Save()
    """
    res = subprocess.run(["powershell", "-NoProfile", "-Command", ps_script], capture_output=True, text=True)
    if res.returncode == 0 and os.path.exists(SHORTCUT_PATH):
        print(f"Desktop shortcut created successfully at: {SHORTCUT_PATH}")
        return True
    else:
        print(f"Failed to create shortcut. Error: {res.stderr}")
        return False


if __name__ == '__main__':
    create_shortcut()
