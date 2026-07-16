import streamlit as st
import pandas as pd
import numpy as np
import math
from pymavlink import mavutil
import tempfile
import os
import glob
import time

# --- PAGE CONFIGURATION ---
st.set_page_config(page_title="Cruise Efficiency Analyzer", layout="wide")

st.markdown("""
    <link href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500;600&family=IBM+Plex+Sans:wght@500;600;700&family=Barlow+Condensed:wght@400;500;600&display=swap" rel="stylesheet">
    <style>
        
        /* Base typography */
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
        
        /* DataFrames */
        .stDataFrame { 
            background-color: #ffffff; border-radius: 8px; border: 1px solid #f3f4f6;
            box-shadow: 4px 4px 0px rgba(17, 24, 39, 0.05); 
        }
        
        /* Metric Cards */
        /* Custom HTML injected metrics do not need these overrides */
        
        
        /* File Headers */
        .file-header { 
            background-color: #ffffff; padding: 12px 16px; border-radius: 6px; 
            border-left: 4px solid #111827; margin-top: 24px; margin-bottom: 12px; 
            font-weight: 600; font-size: 1.15em; color: #111827; box-shadow: 2px 2px 0px rgba(17, 24, 39, 0.05);
            border: 1px solid #e5e7eb;
        }
        
        /* Status Boxes */
        .success-box { background-color: #f0fdf4; padding: 12px 16px; border-radius: 6px; color: #166534; font-weight: 500; border: 1px solid #bbf7d0; }
        .error-box { background-color: #fef2f2; padding: 12px 16px; border-radius: 6px; color: #991b1b; font-weight: 500; border: 1px solid #fecaca; }
    </style>
""", unsafe_allow_html=True)

# --- SESSION STATE ---
if 'history' not in st.session_state:
    st.session_state['history'] = []
if 'global_summary' not in st.session_state:
    st.session_state['global_summary'] = []

# --- MATH ---
def haversine_distance(lat1, lon1, lat2, lon2):
    R = 6371000
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))

# --- GLOBAL MISSION STATS ---
def calculate_global_stats(file_path):
    try:
        mlog = mavutil.mavlink_connection(file_path)
    except:
        return None
        
    last_time_us = 0; total_dist = 0.0; total_energy = 0.0; total_amp_sec = 0.0
    flight_start = 0; flight_end = 0; last_gps = None
    
    while True:
        msg = mlog.recv_match(type=['GPS', 'BAT'], blocking=False)
        if not msg: break
        t = getattr(msg, 'TimeUS', 0)
        
        if msg.get_type() == 'GPS' and msg.Status >= 3:
            if last_gps:
                d = haversine_distance(last_gps['lat'], last_gps['lng'], msg.Lat, msg.Lng)
                # Accumulate distance in 1-meter chunks. This completely prevents distance 
                # loss from high-frequency GPS updates (where frame delta is < 0.5m)
                if d > 1.0: 
                    total_dist += d
                    last_gps = {'lat': msg.Lat, 'lng': msg.Lng} # Anchor updated only after movement!
                    
                    if flight_start == 0: flight_start = t
                    flight_end = t
            else:
                last_gps = {'lat': msg.Lat, 'lng': msg.Lng}
            
        elif msg.get_type() == 'BAT':
            if flight_start > 0 and last_time_us != 0:
                dt = (t - last_time_us) / 1e6
                if 0 < dt < 1.0:
                    # Integrate energy only during flight window (+10s buffer for landing)
                    if t <= flight_end + 10_000_000:
                        total_energy += (msg.Volt * msg.Curr) * dt
                        total_amp_sec += msg.Curr * dt
            last_time_us = t
            
    mlog.close()
    f_time = (flight_end - flight_start) / 1e6
    if f_time < 1: f_time = 1
    
    total_mah = (total_amp_sec / 3600.0) * 1000.0
    dist_km = total_dist / 1000.0
    
    return {
        "time": f_time,
        "dist": dist_km,
        "wh_km": (total_energy/3600) / dist_km if dist_km > 0.1 else 0,
        "watts": total_energy / f_time,
        "mah": total_mah,
        "mah_km": total_mah / dist_km if dist_km > 0.1 else 0
    }

# --- REGIME ANALYSIS (Solid Accumulation) ---
def analyze_regime(file_path, target_alt, target_spd, alt_tol, spd_tol, enforce_auto):
    
    reg_dist = 0.0; reg_energy = 0.0; reg_amp_sec = 0.0; reg_volt_sec = 0.0
    reg_time = 0.0; reg_aoa_sum = 0.0; reg_count = 0
    reg_wind_sum = 0.0; wind_count = 0 # <--- Added Wind Accumulators
    
    last_time_us = 0; last_gps = None
    
    ALT_MIN = target_alt - alt_tol; ALT_MAX = target_alt + alt_tol
    SPD_MIN = target_spd - spd_tol; SPD_MAX = target_spd + spd_tol
    
    is_auto = False; cur_tecs_h = 0.0; cur_arsp = 0.0; cur_aoa = 0.0
    cur_volt = 0.0; cur_curr = 0.0

    try: mlog = mavutil.mavlink_connection(file_path)
    except: return None, "Error"

    while True:
        # Added Wind Message Types
        msg = mlog.recv_match(type=['GPS', 'BAT', 'TECS', 'ARSP', 'AOA', 'MODE', 'HEARTBEAT', 'AHR2', 'NKF2', 'XKF2'], blocking=False)
        if not msg: break
        t = getattr(msg, 'TimeUS', 0)
        mtype = msg.get_type()
        
        # State Update
        if mtype == 'MODE':
            if isinstance(msg.Mode, str): is_auto = 'AUTO' in msg.Mode.upper()
            else: is_auto = (msg.Mode == 10) 
        elif mtype == 'HEARTBEAT' and msg.custom_mode == 10: is_auto = True
        elif mtype == 'TECS': cur_tecs_h = msg.h
        elif mtype == 'ARSP': cur_arsp = msg.Airspeed
        elif mtype == 'AOA': cur_aoa = getattr(msg, 'AOA', getattr(msg, 'Aoa', 0))
        elif mtype == 'BAT': cur_volt = msg.Volt; cur_curr = msg.Curr

        mode_ok = is_auto if enforce_auto else True
        in_regime = False
        
        # Pure Accumulation
        if (mode_ok and 
            ALT_MIN <= cur_tecs_h <= ALT_MAX and 
            SPD_MIN <= cur_arsp <= SPD_MAX):
            in_regime = True

        # --- WIND SPEED CALCULATION ---
        if mtype in ['AHR2', 'NKF2', 'XKF2']:
            vn = getattr(msg, 'vwN', getattr(msg, 'VWN', None))
            ve = getattr(msg, 'vwE', getattr(msg, 'VWE', None))
            if vn is not None and ve is not None and in_regime:
                wind_speed = math.sqrt(vn**2 + ve**2)
                reg_wind_sum += wind_speed
                wind_count += 1

        # Integration
        if mtype == 'GPS' and msg.Status >= 3:
            if in_regime and last_gps:
                 d = haversine_distance(last_gps['lat'], last_gps['lng'], msg.Lat, msg.Lng)
                 if d > 0.05: 
                     reg_dist += d
                     reg_aoa_sum += cur_aoa
                     reg_count += 1
            last_gps = {'lat': msg.Lat, 'lng': msg.Lng}

        elif mtype == 'BAT':
            if last_time_us != 0:
                dt = (t - last_time_us) / 1e6
                if 0 < dt < 1.0:
                    if in_regime:
                        p = cur_volt * cur_curr
                        reg_energy += p * dt
                        reg_amp_sec += cur_curr * dt
                        reg_volt_sec += cur_volt * dt
                        reg_time += dt
            last_time_us = t

    mlog.close()
    
    if reg_time < 1.0:
        return None, "No Data"

    avg_pwr = reg_energy / reg_time if reg_time > 0 else 0
    avg_curr = reg_amp_sec / reg_time if reg_time > 0 else 0
    avg_volt = reg_volt_sec / reg_time if reg_time > 0 else 0
    avg_aoa = reg_aoa_sum / reg_count if reg_count > 0 else 0
    avg_wind = reg_wind_sum / wind_count if wind_count > 0 else 0 # <--- Avg Wind

    return {
        "Duration": reg_time, 
        "Distance": reg_dist / 1000.0,
        "Avg_Power": avg_pwr,
        "Avg_Curr": avg_curr,
        "Avg_Volt": avg_volt,
        "Avg_AoA_Sensor": avg_aoa,
        "Avg_Wind": avg_wind, # <--- Added to result
        "Wh_km": (reg_energy / 3600.0) / (reg_dist / 1000.0) if reg_dist > 0.01 else 0,
        "Tol_Spd": spd_tol,
        "Tol_Alt": alt_tol
    }, None

# --- UI HELPERS ---
def render_metric(label, value, color):
    return f"""
    <div style="background-color: #262626; border-radius: 12px; padding: 20px 10px; box-shadow: 0 8px 16px rgba(0,0,0,0.15); display: flex; flex-direction: column; align-items: center; justify-content: center; margin-bottom: 15px;">
        <div style="color: #f8fafc; font-family: 'IBM Plex Sans', sans-serif; font-size: 14px; font-weight: 500; text-align: center; margin-bottom: 8px;">{label}</div>
        <div style="color: {color}; font-family: 'Barlow Condensed', sans-serif; font-size: 44px; font-weight: 500; text-align: center; line-height: 1.1;">{value}</div>
    </div>
    """

# --- UI LAYOUT ---
st.title("Cruise Efficiency Analyzer")

# 1. INPUT
st.sidebar.header("1. Data Source")
src = st.sidebar.radio("Input", ["Drag & Drop", "Folder Path"])
files = []
is_single_file = False

if src == "Folder Path":
    fp = st.sidebar.text_input("Folder Path").replace('"', '').strip()
    rec = st.sidebar.checkbox("Recursive Scan", True)
    if fp and os.path.isdir(fp):
        pat = os.path.join(fp, "**", "*.bin") if rec else os.path.join(fp, "*.bin")
        files = glob.glob(pat, recursive=rec)
        st.sidebar.markdown(f'<div class="success-box">Found {len(files)} files</div>', unsafe_allow_html=True)
    elif fp:
        st.sidebar.markdown(f'<div class="error-box">Invalid Folder Path</div>', unsafe_allow_html=True)
else:
    upl = st.sidebar.file_uploader("Upload .bin", accept_multiple_files=True)
    if upl:
        files = upl
        if len(files) == 1: is_single_file = True
        st.sidebar.success(f"Loaded {len(files)} files")

# 2. SETTINGS
st.sidebar.header("2. Analysis Config")
mode = st.sidebar.radio("Mode", ["Single Speed", "Range Scan"])
tgt_alt = st.sidebar.number_input("Target Altitude (m)", 10.0, 1000.0, 100.0)

if mode == "Single Speed":
    tgt_spd = st.sidebar.number_input("Target Speed (m/s)", 5.0, 50.0, 16.0)
    spds = [tgt_spd]
else:
    s_start = st.sidebar.number_input("Start Speed", 10.0, 30.0, 12.0)
    s_end = st.sidebar.number_input("End Speed", 10.0, 30.0, 24.0)
    s_step = st.sidebar.number_input("Step", 0.5, 5.0, 2.0)
    spds = np.arange(s_start, s_end + 0.1, s_step).tolist()

st.sidebar.subheader("Tolerances")
t_spd = st.sidebar.number_input("Speed Tol (+/-)", 0.1, 5.0, 1.0)
t_alt = st.sidebar.number_input("Alt Tol (+/-)", 1.0, 50.0, 10.0)
enforce_auto = st.sidebar.checkbox("Enforce AUTO", True)

col1, col2 = st.sidebar.columns(2)
with col1:
    analyze_btn = st.button("Analyze", use_container_width=True)
with col2:
    clear_btn = st.button("Clear All", use_container_width=True)

if clear_btn:
    st.session_state['history'] = []
    st.session_state['global_summary'] = []
    st.rerun()

# --- EXECUTION ---
if files:
    # Auto-process Global Stats so Mission Overview is always visible
    for f in files:
        if isinstance(f, str): 
            f_path = f
            f_name = os.path.basename(f)
            tmp_file = None
        else:
            f_name = f.name
            tmp_file = None
            
        existing = [x['filename'] for x in st.session_state['global_summary']]
        if f_name not in existing:
            if not isinstance(f, str):
                with tempfile.NamedTemporaryFile(delete=False, suffix=".bin") as t:
                    t.write(f.getvalue())
                    f_path = t.name
                    tmp_file = f_path
                    
            g = calculate_global_stats(f_path)
            if g:
                g['filename'] = f_name
                st.session_state['global_summary'].append(g)
                
            if tmp_file:
                try: os.remove(tmp_file)
                except: pass

# --- DISPLAY MISSION OVERVIEW ---
if st.session_state['global_summary']:
    if is_single_file:
        g = st.session_state['global_summary'][0]
        st.markdown("### Mission Overview")
        # Added mAh/km to cards (Total 6 columns)
        c1, c2, c3, c4, c5, c6 = st.columns(6) 
        c1.markdown(render_metric("Dist (km)", f"{g['dist']:.2f}", "#d8b4e2"), unsafe_allow_html=True)
        c2.markdown(render_metric("Eff (Wh/km)", f"{g['wh_km']:.2f}", "#e5a97c"), unsafe_allow_html=True)
        c3.markdown(render_metric("Power (W)", f"{g['watts']:.1f}", "#e08e84"), unsafe_allow_html=True)
        c4.markdown(render_metric("Time (min)", f"{g['time']/60:.1f}", "#60d394"), unsafe_allow_html=True)
        c5.markdown(render_metric("Cons (mAh)", f"{g['mah']:.0f}", "#eae398"), unsafe_allow_html=True)
        c6.markdown(render_metric("Cons/Dist", f"{g['mah_km']:.0f}", "#73d2de"), unsafe_allow_html=True)
        st.divider()

# --- EXECUTION (REGIME SCAN) ---
if analyze_btn and files:
    
    progress_bar = st.progress(0, text="Starting Analysis...")
    total_steps = len(files)
    
    with st.spinner("Processing regimes..."):
        for i, f in enumerate(files):
            if isinstance(f, str): 
                f_path = f
                f_name = os.path.basename(f)
                tmp_file = None
            else:
                f_name = f.name
                with tempfile.NamedTemporaryFile(delete=False, suffix=".bin") as t:
                    t.write(f.getvalue())
                    f_path = t.name
                    tmp_file = f_path
            
            # Range Scan
            for s in spds:
                # OVERLAPPING FIX: Remove old runs for this exact speed and file before appending
                st.session_state['history'] = [x for x in st.session_state['history'] 
                                             if not (x['Filename'] == f_name and x['Target_Spd'] == s)]
                                             
                # --- UPDATE INDICATOR WITH SPEED ---
                progress_bar.progress((i + 1) / total_steps, text=f"Analyzing {f_name} @ {s:.1f} m/s...")
                
                res, err = analyze_regime(f_path, tgt_alt, s, t_alt, t_spd, enforce_auto)
                if res:
                    res['Filename'] = f_name
                    res['Target_Spd'] = s
                    st.session_state['history'].append(res)
            
            if tmp_file: 
                try: os.remove(tmp_file) 
                except: pass
                
    progress_bar.empty()

# --- DISPLAY REGIME TABLES ---
if st.session_state['global_summary']:
    if st.session_state['history']:
        unique_files = list(set([x['Filename'] for x in st.session_state['history']]))
        
        for fname in unique_files:
            if not is_single_file:
                st.markdown(f'<div class="file-header">{fname}</div>', unsafe_allow_html=True)
                f_stat = next((s for s in st.session_state['global_summary'] if s['filename'] == fname), None)
                if f_stat:
                    # Added mAh/km to file header (Total 5 columns)
                    cc1, cc2, cc3, cc4, cc5 = st.columns(5)
                    cc1.markdown(render_metric("Miss Eff (Wh/km)", f"{f_stat['wh_km']:.2f}", "#d8b4e2"), unsafe_allow_html=True)
                    cc2.markdown(render_metric("Time (min)", f"{f_stat['time']/60:.1f}", "#e5a97c"), unsafe_allow_html=True)
                    cc3.markdown(render_metric("Power (W)", f"{f_stat['watts']:.1f}", "#e08e84"), unsafe_allow_html=True)
                    cc4.markdown(render_metric("Cons (mAh)", f"{f_stat['mah']:.0f}", "#60d394"), unsafe_allow_html=True)
                    cc5.markdown(render_metric("Cons/Dist", f"{f_stat['mah_km']:.0f}", "#eae398"), unsafe_allow_html=True)

            rows = [x for x in st.session_state['history'] if x['Filename'] == fname]
            df = pd.DataFrame(rows)
            
            # Convert Seconds -> Minutes for Table
            df['Duration'] = df['Duration'] / 60.0
            
            col_map = {
                'Target_Spd': 'Speed (m/s)', 'Wh_km': 'Eff (Wh/km)', 'Avg_Power': 'Power (W)',
                'Avg_Curr': 'Curr (A)', 'Avg_Volt': 'Volt (V)', 'Avg_AoA_Sensor': 'AoA (deg)', 
                'Avg_Wind': 'Wind (m/s)', 
                'Duration': 'Valid Time (min)', 'Distance': 'Valid Dist (km)', 
                'Tol_Spd': 'Tol Spd', 'Tol_Alt': 'Tol Alt'
            }
            valid_cols = [c for c in col_map.keys() if c in df.columns]
            
            df_show = df[valid_cols].rename(columns=col_map).sort_values(by=["Speed (m/s)", "Tol Spd"])
            
            st.dataframe(
                df_show,
                column_config={
                    "Eff (Wh/km)": st.column_config.NumberColumn(format="%.2f"),
                    "Power (W)": st.column_config.NumberColumn(format="%.1f"),
                    "Curr (A)": st.column_config.NumberColumn(format="%.1f"),
                    "Volt (V)": st.column_config.NumberColumn(format="%.1f"),
                    "AoA (deg)": st.column_config.NumberColumn(format="%.1f"),
                    "Wind (m/s)": st.column_config.NumberColumn(format="%.1f"), 
                    "Valid Time (min)": st.column_config.NumberColumn(format="%.2f"),
                    "Tol Spd": st.column_config.NumberColumn(format="%.1f"),
                    "Tol Alt": st.column_config.NumberColumn(format="%.1f"),
                },
                use_container_width=True, hide_index=True
            )
            
    elif files and not analyze_btn:
        pass # Wait for user to analyze
    elif files and analyze_btn:
        st.warning("No data found matching your Tolerances.")

# --- READY PROMPT ---
elif not files:
    st.info("Select a Data Source from the sidebar to begin.")
elif not analyze_btn:
    st.info("Files ready. Click 'Analyze' in the sidebar to process.")