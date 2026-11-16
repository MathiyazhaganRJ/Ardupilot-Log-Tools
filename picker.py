import sys
import os
import tkinter as tk
from tkinter import filedialog

def main():
    title = sys.argv[1] if len(sys.argv) > 1 else "Select Folder"
    try:
        root = tk.Tk()
        root.withdraw()
        root.lift()
        root.wm_attributes('-topmost', 1)
        root.focus_force()
        folder = filedialog.askdirectory(master=root, title=title)
        root.destroy()
        if folder:
            sys.stdout.write(os.path.normpath(folder))
    except Exception as e:
        sys.stderr.write(str(e))

if __name__ == '__main__':
    main()
