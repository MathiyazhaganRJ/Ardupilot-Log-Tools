import streamlit as st
import os
import shutil
import math
import re
from datetime import datetime
import tkinter as tk
from tkinter import filedialog
from pymavlink import mavutil
import pandas as pd
import subprocess
from geopy.geocoders import Nominatim
import time

# --- PAGE CONFIG ---
st.set_page_config(layout="wide", page_title="ArduPilot Log Manager")

# --- SIDEBAR LOGO ---
st.sidebar.markdown("<br><br>", unsafe_allow_html=True)
try: st.sidebar.image("assets/glider_doodle.png", use_container_width=True)
except: pass

# --- STYLING (Standard Clean UI) ---
st.markdown("""
<style>
    @import url('https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500;600&family=IBM+Plex+Sans:wght@500;600;700&display=swap');
    
    html, body, [class*="css"] { font-family: 'IBM Plex Mono', monospace; font-size: 14px; }
    h1, h2, h3, h4 { font-family: 'IBM Plex Sans', sans-serif; color: #111827; }
    
    .stApp { background-color: #f9fafb; }
    [data-testid="stSidebar"] { background-color: #ffffff; border-right: 1px solid #e5e7eb; }
    [data-testid="stHeader"] { background-color: #f9fafb; }
    
    /* Buttons (Monochrome Minimalist Old School) */
    .stButton > button {
        background-color: #ffffff; border: 1px solid #9ca3af; border-radius: 4px;
        transition: all 0.2s; white-space: nowrap; box-shadow: 2px 2px 0px rgba(17, 24, 39, 0.05);
    }
    .stButton > button p { color: #374151; font-weight: 600; font-family: 'IBM Plex Mono', monospace; margin: 0; font-size: 14px; }
    .stButton > button:hover {
        border-color: #111827; background-color: #f9fafb; box-shadow: 2px 2px 0px #111827;
    }
    .stButton > button:hover p { color: #111827; }
    
    .stButton > button[kind="primary"] {
        background-color: #111827; border: 1px solid #111827; box-shadow: 2px 2px 0px rgba(17,24,39,0.3);
    }
    .stButton > button[kind="primary"] p { color: #ffffff; }
    .stButton > button[kind="primary"]:hover {
        background-color: #374151; box-shadow: 2px 2px 0px #111827;
    }
    .stButton > button[kind="primary"]:hover p { color: #ffffff; }
    
    /* Expanders */
    div[data-testid="stExpander"] { background-color: #ffffff; border-radius: 4px; border: 1px solid #e5e7eb; box-shadow: 2px 2px 0px rgba(17, 24, 39, 0.05); }
    div[data-testid="stExpander"] summary { background-color: #f9fafb; font-weight: 600; font-family: 'IBM Plex Sans', sans-serif; }
    
    /* Code Blocks */
    code { font-family: 'IBM Plex Mono', monospace; background-color: #f3f4f6; color: #111827; border-radius: 4px; padding: 2px 6px; border: 1px solid #e5e7eb; }
</style>
""", unsafe_allow_html=True)

# --- HELPER FUNCTIONS ---

def pick_folder():
    """Opens Windows folder picker"""
    try:
        root = tk.Tk()
        root.withdraw() 
        root.wm_attributes('-topmost', 1) 
        folder_path = filedialog.askdirectory(master=root)
        root.destroy()
        return folder_path
    except:
        return None

@st.cache_data(show_spinner=False)
def _get_location_cached(lat_round, lon_round):
    try:
        # Nominatim usage policy requires max 1 request per second
        time.sleep(1.1) 
        geolocator = Nominatim(user_agent="ardupilot_log_manager_tool", timeout=5)
        # zoom=14 ensures we get locality names
        location = geolocator.reverse(f"{lat_round}, {lon_round}", language='en', zoom=14)
        
        if location:
            addr = location.raw.get('address', {})
            place = (addr.get('village') or 
                     addr.get('town') or 
                     addr.get('city') or 
                     addr.get('suburb') or 
                     addr.get('county') or 
                     "Unknown Place")
            state = addr.get('state', '')
            country = addr.get('country', '')
            
            if state: return f"{place}, {state}"
            return f"{place}, {country}"
    except:
        pass
    return None

def get_smart_location_name(lat, lon):
    """
    Smarter Geocoding: Prioritizes Town/City names over 'Unnamed Roads'.
    Uses caching and coordinate rounding to group nearby flights and avoid API rate limits.
    """
    # Rounding to 2 decimal places groups locations within ~1.1km
    lat_round = round(lat, 2)
    lon_round = round(lon, 2)
    
    loc_name = _get_location_cached(lat_round, lon_round)
    if loc_name:
        return loc_name
    return f"{lat:.4f}, {lon:.4f}"

def haversine_distance(lat1, lon1, lat2, lon2):
    R = 6371000 
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2) * math.sin(dlambda/2)**2
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))

def open_file_location(filepath):
    clean_path = os.path.normpath(filepath)
    if os.path.exists(clean_path):
        subprocess.Popen(f'explorer /select,"{clean_path}"')

def detect_board_name(text_lines):
    combined_text = " ".join(text_lines)
    signatures = {
        "CubeOrange": "Cube Orange", "CubeOrangePlus": "Cube Orange+", "CubeBlack": "Cube Black",
        "MatekH743": "Matek H743", "MatekF405": "Matek F405", "SpeedyBee": "SpeedyBee",
        "Durandal": "Holybro Durandal", "Pixhawk6C": "Pixhawk 6C", "Pixhawk6X": "Pixhawk 6X",
        "QioTek": "QioTek Zealot", "SITL": "SITL (Sim)", "ChibiOS": "ChibiOS (Generic)" 
    }
    for key, name in signatures.items():
        if key.lower() in combined_text.lower(): return name
    return "Unknown Board"

def get_true_log_date(filepath):
    """
    SMART DATE LOGIC:
    1. Extract Date from Filename.
    2. VALIDITY CHECK: Is the year >= 2022?
       - YES: Use Filename Date.
       - NO (1980, 2000): Corrupt/No GPS. Fallback to Modified Date.
    3. If no date in filename, Use Modified Date.
    """
    filename = os.path.basename(filepath)
    potential_date = None
    
    # Regex 1: YYYY-MM-DD
    match = re.search(r"(\d{4})[-_](\d{2})[-_](\d{2})", filename)
    if match:
        try: potential_date = datetime(int(match.group(1)), int(match.group(2)), int(match.group(3)))
        except: pass
    
    # Regex 2: YYYYMMDD
    if not potential_date:
        match = re.search(r"(\d{4})(\d{2})(\d{2})", filename)
        if match:
            try: potential_date = datetime(int(match.group(1)), int(match.group(2)), int(match.group(3)))
            except: pass

    # VALIDITY CHECK (The "2022 Rule")
    if potential_date:
        if potential_date.year >= 2022:
            return potential_date 
        # If older than 2022 (e.g. 1980), treat as invalid and fall through to system time

    # Fallback to System Modified Time
    return datetime.fromtimestamp(os.path.getmtime(filepath))

def analyze_log_file(filepath):
    filename = os.path.basename(filepath)
    smart_date = get_true_log_date(filepath)

    meta = {
        "Filename": filename,
        "Path": filepath,
        "Size": round(os.path.getsize(filepath) / (1024 * 1024), 2),
        "SortDate": smart_date,
        "DisplayDate": smart_date.strftime("%Y-%m-%d"),
        "Year": smart_date.strftime("%Y"),
        "Month": smart_date.strftime("%B"),
        "Firmware": "Unknown",
        "Board": "Generic",
        "Type": "Unknown",
        "Duration": "0m",
        "Distance": 0.0,
        "Location": "No GPS"
    }

    try:
        mlog = mavutil.mavlink_connection(filepath)
        
        # Header Scan
        boot_msgs = []
        for i in range(1000):
            msg = mlog.recv_match(type=['MSG'], blocking=False)
            if not msg: break
            text = msg.Message
            boot_msgs.append(text)
            
            if "Ardu" in text or "PX4" in text:
                meta["Firmware"] = text.split("(")[0].strip()
                if "Plane" in text: meta["Type"] = "Fixed Wing"
                elif "Copter" in text: meta["Type"] = "Multirotor"
        
        meta["Board"] = detect_board_name(boot_msgs)

        # Flight Stats
        mlog.rewind()
        total_dist = 0
        prev_lat, prev_lon = None, None
        start_time, end_time = None, None
        gps_count = 0
        first_fix_found = False
        
        while True:
            msg = mlog.recv_match(type='GPS', blocking=False)
            if not msg: break
            
            if start_time is None: start_time = msg.TimeUS
            end_time = msg.TimeUS
            
            if msg.Status >= 3:
                if not first_fix_found and abs(msg.Lat) > 0.1:
                    meta["Location"] = get_smart_location_name(msg.Lat, msg.Lng)
                    first_fix_found = True
                
                gps_count += 1
                if gps_count % 10 == 0:
                    curr_lat, curr_lon = msg.Lat, msg.Lng
                    if abs(curr_lat) > 0.1:
                        if prev_lat:
                            d = haversine_distance(prev_lat, prev_lon, curr_lat, curr_lon)
                            if d < 500: total_dist += d
                        prev_lat, prev_lon = curr_lat, curr_lon

        meta["Distance"] = round(total_dist / 1000, 2)
        if start_time and end_time:
            duration_sec = (end_time - start_time) / 1e6
            if duration_sec > 0:
                mins = int(duration_sec // 60)
                meta["Duration"] = f"{mins}m {int(duration_sec % 60)}s"

    except:
        pass
        
    return meta

def organize_files(source_folders, dest_folder, move_mode=False):
    stats = {"copied": 0, "skipped": 0, "errors": 0}
    log_text = []
    
    all_files = []
    for folder in source_folders:
        for root, dirs, files in os.walk(folder):
            for file in files:
                if file.lower().endswith('.bin'):
                    all_files.append(os.path.join(root, file))
    
    total = len(all_files)
    if total == 0: return stats, ["No .bin files found."]

    progress_bar = st.progress(0)
    
    for i, src_path in enumerate(all_files):
        try:
            filename = os.path.basename(src_path)
            # Use same smart logic for folders
            date_obj = get_true_log_date(src_path)
            
            target_dir = os.path.join(dest_folder, date_obj.strftime("%Y"), date_obj.strftime("%B"), date_obj.strftime("%Y-%m-%d"))
            if not os.path.exists(target_dir): os.makedirs(target_dir)
            target_path = os.path.join(target_dir, filename)
            
            should_copy = True
            if os.path.exists(target_path):
                if os.path.getsize(src_path) == os.path.getsize(target_path):
                    should_copy = False
                    stats["skipped"] += 1

            if should_copy:
                if move_mode:
                    shutil.move(src_path, target_path)
                    log_text.append(f"Moved: {filename}")
                else:
                    shutil.copy2(src_path, target_path)
                    log_text.append(f"Copied: {filename}")
                stats["copied"] += 1
                
        except Exception as e:
            stats["errors"] += 1
            log_text.append(f"Error: {str(e)}")
        
        progress_bar.progress((i + 1) / total)

    progress_bar.empty()
    return stats, log_text

# --- APP LAYOUT ---
st.sidebar.title("ArduPilot Log Manager")
st.sidebar.caption("Data Organization Tool")

tab1, tab2 = st.tabs(["Log Organizer", "Log Explorer"])

# --- TAB 1: ORGANIZER ---
with tab1:
    st.header("Auto-Archive")
    if 'source_folders' not in st.session_state: st.session_state['source_folders'] = []
    if 'dest_folder' not in st.session_state: st.session_state['dest_folder'] = None

    c1, c2 = st.columns(2)
    with c1:
        st.subheader("1. Sources")
        if st.button("Add Source"):
            f = pick_folder()
            if f and f not in st.session_state['source_folders']:
                st.session_state['source_folders'].append(f)
        for f in st.session_state['source_folders']: st.code(f"{f}")
        if st.button("Clear Sources"): st.session_state['source_folders'] = []

    with c2:
        st.subheader("2. Destination")
        if st.button("Select Destination"):
            f = pick_folder()
            if f: st.session_state['dest_folder'] = f
        if st.session_state['dest_folder']: st.success(st.session_state['dest_folder'])

    st.divider()
    mode = st.radio("Mode:", ["Copy (Safe)", "Move (Delete Original)"])
    if st.button("Organize Now", type="primary", disabled=not (st.session_state['source_folders'] and st.session_state['dest_folder'])):
        s, l = organize_files(st.session_state['source_folders'], st.session_state['dest_folder'], move_mode=("Move" in mode))
        st.success(f"Processed: {s['copied']} | Skipped: {s['skipped']}")

# --- TAB 2: EXPLORER ---
with tab2:
    st.header("Flight Log Database")
    
    if 'view_root' not in st.session_state: st.session_state['view_root'] = None
    if 'df_view' not in st.session_state: st.session_state['df_view'] = pd.DataFrame()

    c1, c2 = st.columns([1,4])
    with c1:
        if st.button("Select Root"):
            f = pick_folder()
            if f: 
                st.session_state['view_root'] = f
                st.rerun()
    with c2:
        if st.session_state['view_root']: st.info(st.session_state['view_root'])

    if st.session_state['view_root']:
        if st.button("Deep Scan"):
            logs = []
            files = [os.path.join(r, f) for r, d, fs in os.walk(st.session_state['view_root']) for f in fs if f.endswith('.bin')]
            prog = st.progress(0)
            
            for i, f in enumerate(files):
                logs.append(analyze_log_file(f))
                prog.progress((i+1)/len(files))
            
            prog.empty()
            
            df = pd.DataFrame(logs)
            if not df.empty:
                st.session_state['df_view'] = df.sort_values(by=["SortDate", "Filename"], ascending=[False, False])

    if not st.session_state['df_view'].empty:
        df = st.session_state['df_view']
        search = st.text_input("Search Logs...", placeholder="Location, Date, Board...")
        if search: df = df[df.astype(str).apply(lambda x: x.str.contains(search, case=False)).any(axis=1)]

        years = sorted(df['Year'].unique(), reverse=True)
        for year in years:
            with st.expander(f"{year}", expanded=True):
                y_df = df[df['Year'] == year]
                for month in sorted(y_df['Month'].unique(), reverse=True):
                    st.caption(f"### {month}")
                    m_df = y_df[y_df['Month'] == month]
                    
                    event = st.dataframe(
                        m_df[['DisplayDate', 'Location', 'Board', 'Firmware', 'Duration', 'Distance', 'Filename']],
                        use_container_width=True, hide_index=True, on_select="rerun", selection_mode="single-row",
                        column_config={
                            "DisplayDate": "Flight Date",
                            "Distance": st.column_config.NumberColumn("Dist (km)", format="%.2f")
                        }
                    )
                    
                    if len(event.selection['rows']) > 0:
                        sel = m_df.iloc[event.selection['rows'][0]]
                        st.success(f"Selected: {sel['Filename']}")
                        c1, c2 = st.columns([1,3])
                        with c1: 
                            if st.button("Open Folder", key=f"btn_{sel['Filename']}"): open_file_location(sel['Path'])
                        with c2: st.write(f"Location: {sel['Location']} | Distance: {sel['Distance']} km")