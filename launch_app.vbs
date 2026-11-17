Set WshShell = CreateObject("WScript.Shell")
Set fso = CreateObject("Scripting.FileSystemObject")
strDir = fso.GetParentFolderName(WScript.ScriptFullName)
WshShell.CurrentDirectory = strDir

' Run silently with pythonw so no console window appears
WshShell.Run "pythonw.exe """ & strDir & "\app.py""", 0, False
