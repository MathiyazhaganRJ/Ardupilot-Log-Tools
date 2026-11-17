"""
Creates a Windows Desktop Shortcut for the ArduPilot Flight Log Analyser.
Places a high-resolution icon on the user's Desktop that launches the app
as a native, frameless/standalone window without a console prompt.
Works automatically on any Windows PC after cloning this repository.
"""

import os
import sys
import subprocess

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
APP_PY = os.path.join(BASE_DIR, "app.py")
ICON_ICO = os.path.join(BASE_DIR, "app_icon.ico")
ICON_PNG = os.path.join(BASE_DIR, "app_icon.png")


def get_desktop_dir():
    """Resolves true Desktop folder path, including OneDrive-synced desktop paths."""
    try:
        cmd = ['powershell', '-NoProfile', '-Command', '[Environment]::GetFolderPath([Environment+SpecialFolder]::Desktop)']
        out = subprocess.check_output(cmd, text=True).strip()
        if out and os.path.exists(out):
            return out
    except Exception:
        pass
    
    fallback = os.path.join(os.path.expanduser("~"), "Desktop")
    if os.path.exists(fallback):
        return fallback
    
    # Check common OneDrive desktop path
    onedrive_desktop = os.path.join(os.path.expanduser("~"), "OneDrive", "Desktop")
    if os.path.exists(onedrive_desktop):
        return onedrive_desktop
        
    return fallback


def ensure_icon():
    """Generates app_icon.ico from app_icon.png if it does not already exist."""
    if os.path.exists(ICON_ICO) and os.path.getsize(ICON_ICO) > 1000:
        return True
    if os.path.exists(ICON_PNG):
        try:
            from PIL import Image
            img = Image.open(ICON_PNG)
            img.save(ICON_ICO, format='ICO', sizes=[(16, 16), (32, 32), (48, 48), (64, 64), (128, 128), (256, 256)])
            return True
        except Exception as e:
            print(f"Warning: Could not compile .ico ({e}), using default executable icon.")
    return False


def create_shortcut():
    desktop_dir = get_desktop_dir()
    shortcut_path = os.path.join(desktop_dir, "ArduPilot Flight Log Analyser.lnk")
    
    # Locate pythonw.exe in current Python installation
    py_dir = os.path.dirname(sys.executable)
    pythonw_exe = os.path.join(py_dir, "pythonw.exe")
    if not os.path.exists(pythonw_exe):
        pythonw_exe = sys.executable

    ensure_icon()
    has_icon = os.path.exists(ICON_ICO)

    ps_script = f"""
    $WshShell = New-Object -ComObject WScript.Shell
    $Shortcut = $WshShell.CreateShortcut('{shortcut_path}')
    $Shortcut.TargetPath = '{pythonw_exe}'
    $Shortcut.Arguments = '"{APP_PY}"'
    $Shortcut.WorkingDirectory = '{BASE_DIR}'
    """
    if has_icon:
        ps_script += f"""
    $Shortcut.IconLocation = '{ICON_ICO},0'
        """
    ps_script += """
    $Shortcut.Description = 'ArduPilot Flight Log Analyser & Archiver Desktop Application'
    $Shortcut.Save()
    """

    res = subprocess.run(["powershell", "-NoProfile", "-Command", ps_script], capture_output=True, text=True)
    if res.returncode == 0 and os.path.exists(shortcut_path):
        print(f"Desktop shortcut created successfully at: {shortcut_path}")
        return True
    else:
        print(f"Failed to create shortcut. Error: {res.stderr}")
        return False


if __name__ == '__main__':
    create_shortcut()
