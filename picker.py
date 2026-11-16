import sys
import os
import ctypes

try:
    user32 = ctypes.windll.user32
    hDefault = user32.OpenDesktopW("Default", 0, False, 0x01FF)
    if hDefault:
        user32.SetThreadDesktop(hDefault)
except Exception:
    pass

import tkinter as tk
from tkinter import filedialog

def main():
    title = sys.argv[1] if len(sys.argv) > 1 else "Select Folder"
    try:
        root = tk.Tk()
        root.withdraw()
        root.wm_attributes('-topmost', 1)
        root.lift()
        root.focus_force()
        folder = filedialog.askdirectory(master=root, title=title)
        root.destroy()
        if folder and os.path.exists(folder):
            sys.stdout.write(os.path.normpath(folder))
    except Exception as e:
        sys.stderr.write(str(e))

if __name__ == '__main__':
    main()
