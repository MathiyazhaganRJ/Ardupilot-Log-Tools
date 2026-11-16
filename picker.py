import sys
import os
import tkinter as tk
from tkinter import filedialog

def pick():
    title = sys.argv[1] if len(sys.argv) > 1 else "Select Folder"
    
    # Method 1: Tkinter with mapped topmost root
    try:
        root = tk.Tk()
        root.title(title)
        root.geometry("1x1+0+0")
        root.attributes('-alpha', 0.01) # Mapped so Windows manages it as active foreground
        root.attributes('-topmost', True)
        root.lift()
        root.focus_force()
        folder = filedialog.askdirectory(parent=root, title=title)
        root.destroy()
        if folder and os.path.exists(folder):
            sys.stdout.write(os.path.normpath(folder))
            return
    except Exception:
        pass

    # Method 2: Fallback to Shell.Application BrowseForFolder rooted at This PC (Root = 17)
    try:
        import subprocess
        ps = (
            "$app = New-Object -ComObject Shell.Application; "
            f"$f = $app.BrowseForFolder(0, '{title}', 81, 17); "
            "if ($f -and $f.Self) { Write-Output $f.Self.Path }"
        )
        si = subprocess.STARTUPINFO()
        si.dwFlags = subprocess.STARTF_USESHOWWINDOW
        si.wShowWindow = 5 # SW_SHOW
        p = subprocess.run(
            ["powershell", "-NoProfile", "-STA", "-Command", ps],
            startupinfo=si,
            capture_output=True,
            text=True,
            timeout=120
        )
        out = p.stdout.strip()
        if out and os.path.exists(out):
            sys.stdout.write(os.path.normpath(out))
            return
    except Exception:
        pass

if __name__ == '__main__':
    pick()
