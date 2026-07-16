import subprocess
import time
import sys
import os

# Automatically install pywebview if it's missing
try:
    import webview
except ImportError:
    print("Installing 'pywebview' to enable Native Python Windows...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "pywebview"])
    import webview

def main():
    print("========================================================")
    print("         Starting ArduPilot Data Toolkit...")
    print("========================================================\n")
    
    # Start the Streamlit server silently in the background
    streamlit_process = subprocess.Popen(
        [sys.executable, "-m", "streamlit", "run", "1_Log_Manager.py", "--server.headless", "true"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    
    # Give the server 3 seconds to boot up
    time.sleep(3)
    
    print("Launching Native Python Window...")
    
    # Create a true, direct Python GUI Window!
    webview.create_window(
        title='ArduPilot Toolkit', 
        url='http://localhost:8501',
        width=1350, 
        height=900,
        background_color='#f9fafb'
    )
    
    # Start the window. This function blocks until the user closes the window with the 'X' button.
    webview.start()
    
    # Once the user closes the window, gracefully shut down the hidden server
    print("\nWindow closed. Shutting down server...")
    streamlit_process.terminate()
    streamlit_process.wait()
    print("Goodbye RJ!")
    sys.exit(0)

if __name__ == "__main__":
    main()
