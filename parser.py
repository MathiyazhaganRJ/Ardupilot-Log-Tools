"""
ArduPilot DataFlash Binary Log Parser
Extracts synchronized flight telemetry, message formats, and aerodynamic performance.
"""

import os
import math
import datetime
import json
import numpy as np
from pymavlink import mavutil
import organizer

GPS_EPOCH = datetime.datetime(1980, 1, 6, tzinfo=datetime.timezone.utc)
PLANE_MODE_MAP = {
    0: 'MANUAL',
    1: 'CIRCLE',
    2: 'STABILIZE',
    3: 'TRAINING',
    4: 'ACRO',
    5: 'FBWA',
    6: 'FBWB',
    7: 'CRUISE',
    8: 'AUTOTUNE',
    10: 'AUTO',
    11: 'RTL',
    12: 'LOITER',
    13: 'TAKEOFF',
    14: 'AVOID_ADSB',
    15: 'GUIDED',
    16: 'INITIALISING',
    17: 'QSTABILIZE',
    18: 'QHOVER',
    19: 'QLOITER',
    20: 'QLAND',
    21: 'QRTL',
    22: 'QAUTOTUNE',
    23: 'QACRO',
    24: 'THERMAL',
    25: 'LOITER_ALT_QLAND'
}

COPTER_MODE_MAP = {
    0: 'STABILIZE',
    1: 'ACRO',
    2: 'ALT_HOLD',
    3: 'AUTO',
    4: 'GUIDED',
    5: 'LOITER',
    6: 'RTL',
    7: 'CIRCLE',
    9: 'LAND',
    11: 'DRIFT',
    13: 'SPORT',
    14: 'FLIP',
    15: 'AUTOTUNE',
    16: 'POSHOLD',
    17: 'BRAKE',
    18: 'THROW',
    19: 'AVOID_ADSB',
    20: 'GUIDED_NOGPS',
    21: 'SMART_RTL',
    22: 'FLOWHOLD',
    23: 'FOLLOW',
    24: 'ZIGZAG',
    25: 'SYSTEMID',
    26: 'AUTOROTATE',
    27: 'AUTO_RTL'
}

MODE_MAP = PLANE_MODE_MAP

INST_MSGS = {
    'ARSP': 'I',
    'BARO': 'I',
    'BAT': 'Inst',
    'GPA': 'I',
    'GPS': 'I',
    'IMU': 'I',
    'MAG': 'I',
    'MAV': 'chan',
    'ESC': 'Instance',
    'VIBE': 'IMU',
    'AUXF': 'function',
    'MNT': 'I',
    'RCO2': 'I',
    'RCI2': 'I',
}

SKIP_TYPES = {'FMT', 'FMTU', 'UNIT', 'MULT', 'PARM', 'MSG', 'STAK', 'VER', 'FILE'}

TARGET_TYPES = [
    'FMT', 'MODE', 'ATT', 'CTUN', 'NTUN', 'QTUN', 'RATE', 'AETR',
    'PIDR', 'PIDP', 'PIDY', 'PIDA', 'BARO', 'BAT', 'ARSP', 'GPS',
    'GPA', 'POS', 'AOA', 'TECS', 'RCOU', 'RCIN', 'RCO2', 'RCI2',
    'VIBE', 'DCM', 'AHR2', 'XKF1', 'XKF2', 'XKF4', 'ARM', 'ERR',
    'EV', 'MSG', 'PARM', 'CMD', 'DSF', 'ESC', 'AUXF', 'MNT'
]

def clean_num(v, decimals=4):
    if isinstance(v, float):
        if math.isnan(v) or math.isinf(v):
            return None
        return round(v, decimals)
    return v

def resolve_flight_mode(val, is_copter=False):
    if val is None:
        return 'MANUAL'
    mode_map = COPTER_MODE_MAP if is_copter else PLANE_MODE_MAP
    if isinstance(val, int):
        return mode_map.get(val, PLANE_MODE_MAP.get(val, f'MODE_{val}'))
    val_str = str(val).strip()
    if val_str.isdigit():
        num = int(val_str)
        return mode_map.get(num, PLANE_MODE_MAP.get(num, f'MODE_{num}'))
    upper = val_str.upper()
    alias_map = {
        'FLYBYWIREA': 'FBWA',
        'FLY_BY_WIRE_A': 'FBWA',
        'FLYBYWIREB': 'FBWB',
        'FLY_BY_WIRE_B': 'FBWB',
        'QLOITER': 'QLOITER',
        'QSTABILIZE': 'QSTABILIZE',
        'QHOVER': 'QHOVER',
        'QRTL': 'QRTL',
        'QLAND': 'QLAND'
    }
    return alias_map.get(upper, upper)

def get_default_logs_dir(base_dir=None):
    if base_dir and os.path.exists(base_dir):
        return os.path.normpath(base_dir)
    candidate_dirs = [
        os.environ.get("ARDUPILOT_LOGS_DIR"),
        r"C:\Users\KARSHAK_Tamal\Desktop\Logs",
        os.path.join(os.path.expanduser("~"), "Desktop", "Logs"),
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
    ]
    for d in candidate_dirs:
        if d and os.path.exists(d):
            return os.path.normpath(d)
    return os.path.normpath(r"C:\Users\KARSHAK_Tamal\Desktop\Logs")


def discover_flight_logs(base_dir=None):
    """Finds all .bin flight logs in the logs directory, sorted by date."""
    candidate_dirs = []
    if base_dir and os.path.exists(base_dir):
        candidate_dirs.append(base_dir)
    uploads_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "uploads")
    if os.path.exists(uploads_dir):
        candidate_dirs.append(uploads_dir)
    env_dir = os.environ.get("ARDUPILOT_LOGS_DIR")
    if env_dir and os.path.exists(env_dir):
        candidate_dirs.append(env_dir)
    candidate_dirs.append(r"C:\Users\KARSHAK_Tamal\Desktop\Logs")
    user_desktop_logs = os.path.join(os.path.expanduser("~"), "Desktop", "Logs")
    if user_desktop_logs not in candidate_dirs:
        candidate_dirs.append(user_desktop_logs)
    local_logs = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
    candidate_dirs.append(local_logs)

    valid_dir = None
    for d in candidate_dirs:
        if d and os.path.exists(d):
            valid_dir = os.path.normpath(d)
            break

    logs = []
    if valid_dir and os.path.exists(valid_dir):
        for root, dirs, files in os.walk(valid_dir):
            for f in files:
                if f.lower().endswith(('.bin', '.log')):
                    full_p = os.path.join(root, f)
                    try:
                        mtime = os.path.getmtime(full_p)
                        size_mb = os.path.getsize(full_p) / (1024 * 1024)
                        rel_p = os.path.relpath(full_p, valid_dir)
                        logs.append({
                            'path': full_p,
                            'rel_path': rel_p,
                            'filename': f,
                            'size_mb': round(size_mb, 2),
                            'mtime': mtime,
                            'date_str': datetime.datetime.fromtimestamp(mtime).strftime('%Y-%m-%d %H:%M')
                        })
                    except Exception:
                        pass
    logs.sort(key=lambda x: x['mtime'], reverse=True)
    return logs


def get_log_formats(file_path):
    """
    Scans the beginning of the .bin log to extract discovered message formats
    with instance identifiers (matching UAV Log Viewer).
    """
    mlog = mavutil.mavlink_connection(file_path)
    fmts = {}
    logged_formats = {}
    count = 0
    while count < 8000:
        msg = mlog.recv_match(type=TARGET_TYPES, blocking=False)
        if not msg:
            break
        count += 1
        mtype = msg.get_type()
        if mtype == 'FMT':
            name = getattr(msg, 'Name', '')
            cols = [c.strip() for c in getattr(msg, 'Columns', '').split(',')]
            if name and cols:
                fmts[name] = cols
            continue
        if mtype in ('PARM', 'MSG', 'CMD', 'DSF'):
            continue
        inst_col = INST_MSGS.get(mtype)
        inst = getattr(msg, inst_col, None) if inst_col else None
        prefix = f"{mtype}[{inst}]" if inst is not None else mtype
        if prefix not in logged_formats and mtype in fmts:
            col_list = [c for c in fmts[mtype] if c not in ('TimeUS', inst_col)]
            logged_formats[prefix] = col_list
    mlog.close()
    if not logged_formats:
        return fmts
    return logged_formats


def parse_flight_telemetry(file_path, target_samples=4000):
    """
    Parses ArduPilot .bin log into a synchronized timeline of flight telemetry.
    Returns:
      - metadata: dict of flight information
      - formats: dict of all message types and fields discovered (matching UAV Log Viewer)
      - telemetry: dict of synchronized time series arrays
      - mode_segments: list of flight mode intervals
      - speed_steps: list of detected airspeed test steps in AUTO mode
    """
    mlog = mavutil.mavlink_connection(file_path)

    fmts = {}
    logged_formats = {}
    parameters = {}
    raw_records = []
    flight_messages = []
    current_state = {}

    cur_mode = 'MANUAL'
    cur_roll = cur_pitch = cur_yaw = 0.0
    cur_alt = cur_crt = 0.0
    cur_amsl = 0.0
    home_elevation = 0.0
    cur_rel_alt = 0.0
    cur_press = 101325.0
    cur_temp = 25.0
    cur_baro_temp = 25.0
    cur_arsp_temp = None
    cur_tas = cur_gs = cur_tho = cur_spdem = 0.0
    cur_vwn = cur_vwe = cur_aoa = cur_ssa = 0.0
    cur_lat = cur_lng = 0.0
    cur_servos = [1000] * 16
    cur_rcin = [1500] * 16

    true_flight_date = None
    gps_week = 0
    gps_ms = 0
    firmware = "ArduPlane"
    board = "Autopilot Board"

    last_sample_t = 0.0
    sample_interval = 0.05  # 20 Hz timeline

    while True:
        msg = mlog.recv_match(type=TARGET_TYPES, blocking=False)
        if not msg:
            break
        mtype = msg.get_type()

        if mtype == 'FMT':
            name = getattr(msg, 'Name', '')
            cols = [c.strip() for c in getattr(msg, 'Columns', '').split(',')]
            if name and cols:
                fmts[name] = cols
            continue

        elif mtype == 'PARM':
            p_name = getattr(msg, 'Name', '')
            p_val = getattr(msg, 'Value', None)
            if p_name and p_val is not None:
                try:
                    vf = float(p_val)
                    parameters[p_name] = round(vf, 5) if vf != int(vf) else int(vf)
                except (ValueError, TypeError):
                    parameters[p_name] = p_val
            continue

        elif mtype == 'MSG':
            txt = getattr(msg, 'Message', '').strip()
            t_s = getattr(msg, 'TimeUS', 0) / 1e6
            if 'Ardu' in txt:
                firmware = txt.split('(')[0].strip()
            if 'Matek' in txt: board = 'Matek F405/H743'
            elif 'Cube' in txt: board = 'Cube Orange/Black'
            elif 'Speedy' in txt: board = 'SpeedyBee'
            elif 'Pixhawk' in txt: board = 'Pixhawk'

            if txt:
                txt_lower = txt.lower()
                sev = 6
                if any(w in txt_lower for w in ['fail', 'error', 'bad', 'fatal', 'panic', 'crash']):
                    sev = 3
                elif any(w in txt_lower for w in ['warn', 'glitch', 'drift', 'high', 'low', 'denied']):
                    sev = 4
                elif any(w in txt_lower for w in ['calibrat', 'init', 'arm', 'mode', 'ready', 'complete', 'active']):
                    sev = 5
                flight_messages.append({'time': round(t_s, 2), 'text': txt, 'severity': sev})
            continue

        t_us = getattr(msg, 'TimeUS', None)
        if t_us is None:
            continue
        t_s = t_us / 1e6

        # Determine instance prefix
        inst_col = INST_MSGS.get(mtype)
        inst = getattr(msg, inst_col, None) if inst_col else None
        prefix = f"{mtype}[{inst}]" if inst is not None else mtype

        # Register in logged formats
        if prefix not in logged_formats and mtype in fmts:
            col_list = [c for c in fmts[mtype] if c not in ('TimeUS', inst_col)]
            logged_formats[prefix] = col_list

        # Fast dictionary extraction & float sanitization (no NaN/Inf)
        d = msg.to_dict()
        for k, v in d.items():
            if k in ('mavpackettype', 'TimeUS', inst_col):
                continue
            if isinstance(v, (int, float)):
                v_clean = clean_num(v)
                current_state[f"{prefix}.{k}"] = v_clean
                if inst == 0:
                    current_state[f"{mtype}.{k}"] = v_clean

        # Standard navigation & attitude updates
        if mtype == 'GPS':
            status = getattr(msg, 'Status', 0)
            gwk = getattr(msg, 'GWk', 0)
            gms = getattr(msg, 'GMS', 0)
            if status >= 3 and gwk > 1500 and gps_week == 0:
                gps_week = gwk
                gps_ms = gms
            cur_gs = getattr(msg, 'Spd', cur_gs)
            lat_raw = getattr(msg, 'Lat', 0)
            lng_raw = getattr(msg, 'Lng', 0)
            if abs(lat_raw) > 1000:
                cur_lat = lat_raw / 1e7
                cur_lng = lng_raw / 1e7
            if status >= 3:
                gps_a = getattr(msg, 'Alt', None)
                if gps_a is not None and abs(gps_a) > 0.001:
                    if home_elevation == 0.0:
                        home_elevation = float(gps_a)
                    cur_alt = float(gps_a)
                    cur_amsl = cur_alt

        elif mtype == 'POS':
            lat_p = getattr(msg, 'Lat', 0)
            lng_p = getattr(msg, 'Lng', 0)
            if abs(lat_p) > 0.01:
                cur_lat = lat_p
                cur_lng = lng_p
            if home_elevation == 0.0:
                pos_alt = getattr(msg, 'Alt', None)
                if pos_alt is not None and abs(pos_alt) > 10.0:
                    home_elevation = float(pos_alt)
                    cur_alt = (home_elevation + cur_rel_alt) if home_elevation != 0.0 else cur_rel_alt
                    cur_amsl = cur_alt
            rel_home = getattr(msg, 'RelHomeAlt', None)
            if rel_home is not None:
                cur_rel_alt = float(rel_home)
                cur_alt = (home_elevation + cur_rel_alt) if home_elevation != 0.0 else cur_rel_alt
                cur_amsl = cur_alt

        elif mtype == 'MODE':
            m_val = getattr(msg, 'Mode', None)
            if m_val is None:
                m_val = getattr(msg, 'ModeNum', 'MANUAL')
            is_cop = 'Copter' in firmware or 'Quad' in firmware
            cur_mode = resolve_flight_mode(m_val, is_cop)

        elif mtype == 'ATT':
            cur_roll = getattr(msg, 'Roll', cur_roll)
            cur_pitch = getattr(msg, 'Pitch', cur_pitch)
            cur_yaw = getattr(msg, 'Yaw', cur_yaw)

        elif mtype == 'BARO':
            cur_press = getattr(msg, 'Press', cur_press)
            b_temp = getattr(msg, 'Temp', None)
            if b_temp is not None:
                cur_baro_temp = b_temp
                cur_temp = b_temp
            if cur_crt == 0.0:
                cur_crt = getattr(msg, 'CRt', cur_crt)
            if cur_alt == 0.0:
                b_alt = getattr(msg, 'Alt', None)
                if b_alt is not None and abs(b_alt) > 0.001:
                    cur_rel_alt = float(b_alt)
                    cur_alt = (home_elevation + cur_rel_alt) if home_elevation != 0.0 else cur_rel_alt
                    cur_amsl = cur_alt

        elif mtype == 'ARSP':
            cur_tas = getattr(msg, 'Airspeed', cur_tas)
            arsp_t = getattr(msg, 'Temp', None)
            if arsp_t is not None and abs(arsp_t) > 0.05:
                cur_arsp_temp = arsp_t

        elif mtype == 'CTUN':
            raw_tho = getattr(msg, 'ThO', cur_tho)
            if 0 < raw_tho <= 1.0:
                raw_tho *= 100.0
            cur_tho = raw_tho

        elif mtype == 'AOA':
            cur_aoa = getattr(msg, 'AOA', getattr(msg, 'Aoa', cur_aoa))
            cur_ssa = getattr(msg, 'SSA', getattr(msg, 'Ssa', cur_ssa))

        elif mtype == 'XKF2':
            cur_vwn = getattr(msg, 'VWN', cur_vwn)
            cur_vwe = getattr(msg, 'VWE', cur_vwe)

        elif mtype == 'TECS':
            cur_spdem = getattr(msg, 'spdem', cur_spdem)

        elif mtype == 'RCOU':
            cur_servos = [
                int(getattr(msg, f'C{ch}', 1000) or 1000) for ch in range(1, 17)
            ]

        elif mtype == 'RCIN':
            cur_rcin = [
                int(getattr(msg, f'C{ch}', 1500) or 1500) for ch in range(1, 17)
            ]

        # Sample onto the synchronized timeline at 20 Hz
        if (t_s - last_sample_t) >= sample_interval and t_s > 0:
            rec = dict(current_state)
            rec['time'] = round(t_s, 2)
            rec['mode'] = cur_mode
            rec['roll'] = clean_num(cur_roll, 1) or 0.0
            rec['pitch'] = clean_num(cur_pitch, 1) or 0.0
            rec['yaw'] = clean_num(cur_yaw, 1) or 0.0
            rec['lat'] = clean_num(cur_lat, 7) or 0.0
            rec['lng'] = clean_num(cur_lng, 7) or 0.0
            rec['alt'] = clean_num(cur_alt, 1) or 0.0
            rec['rel_alt'] = clean_num(cur_rel_alt, 1) or 0.0
            rec['field_elev'] = clean_num(home_elevation, 1) or 0.0
            rec['crt'] = clean_num(cur_crt, 2) or 0.0
            rec['tas'] = clean_num(cur_tas, 2) or 0.0
            rec['gs'] = clean_num(cur_gs, 2) or 0.0
            rec['spdem'] = clean_num(cur_spdem, 1) or 0.0
            rec['tho'] = clean_num(cur_tho, 1) or 0.0

            # BAT fields
            v_val = current_state.get('BAT[0].Volt', current_state.get('BAT.Volt', 0.0)) or 0.0
            i_val = current_state.get('BAT[0].Curr', current_state.get('BAT.Curr', 0.0)) or 0.0
            curr_tot = current_state.get('BAT[0].CurrTot', current_state.get('BAT.CurrTot', 0.0)) or 0.0
            rem_pct = current_state.get('BAT[0].RemPct', current_state.get('BAT.RemPct', 100))
            if rem_pct is None or (isinstance(rem_pct, float) and math.isnan(rem_pct)):
                rem_pct = 100
            pwr = clean_num(v_val * i_val, 1) or 0.0
            wh_km = clean_num((pwr / (cur_tas * 3.6)), 2) if (cur_tas and cur_tas > 2.0) else 0.0

            rec['volt'] = clean_num(v_val, 2) or 0.0
            rec['curr'] = clean_num(i_val, 2) or 0.0
            rec['power'] = pwr
            rec['consumed_mah'] = clean_num(curr_tot, 1) or 0.0
            rec['rem_pct'] = int(rem_pct)
            rec['wh_km'] = wh_km or 0.0

            # Atmospheric & Aerodynamic metrics
            rec['aoa'] = clean_num(cur_aoa, 2) or 0.0
            rec['ssa'] = clean_num(cur_ssa, 2) or 0.0
            rec['press'] = clean_num(cur_press, 1) or 101325.0
            rec['press_hpa'] = clean_num(cur_press / 100.0, 1) if (cur_press and cur_press > 2000) else (clean_num(cur_press, 1) or 1013.2)
            
            try:
                base_p = cur_press / 100.0 if cur_press > 2000 else cur_press
                ref_a = cur_amsl if cur_amsl != 0.0 else cur_alt
                qnh = base_p * (max(0.001, 1.0 - (ref_a / 44330.77)) ** -5.25588)
            except Exception:
                qnh = 1013.2
            rec['qnh_hpa'] = clean_num(qnh, 1) or 1013.2

            rec['temp'] = clean_num(cur_temp, 1) or 25.0
            rec['baro_temp'] = clean_num(cur_baro_temp, 1) or 25.0
            rec['arsp_temp'] = clean_num(cur_arsp_temp, 1) if cur_arsp_temp is not None else 0.0
            rec['has_arsp'] = cur_arsp_temp is not None

            amb_temp = cur_arsp_temp if cur_arsp_temp is not None else cur_baro_temp
            tk = (amb_temp or 25.0) + 273.15
            rho = (cur_press / (287.05 * tk)) if tk > 0 else 1.225
            rec['density'] = clean_num(rho, 4) or 1.225

            w_spd = math.sqrt(cur_vwn**2 + cur_vwe**2) if (cur_vwn is not None and cur_vwe is not None) else 0.0
            w_dir = (math.degrees(math.atan2(-cur_vwe, -cur_vwn)) + 360.0) % 360.0 if w_spd > 0.05 else 0.0
            rec['wind_spd'] = clean_num(w_spd, 1) or 0.0
            rec['wind_dir'] = clean_num(w_dir, 0) or 0.0
            rec['vwn'] = clean_num(cur_vwn, 2) or 0.0
            rec['vwe'] = clean_num(cur_vwe, 2) or 0.0

            rec['servos'] = cur_servos[:]
            rec['rcin'] = cur_rcin[:]
            for ch_num, s_val in enumerate(cur_servos, start=1):
                rec[f'rcou_c{ch_num}'] = s_val
                rec[f'c{ch_num}'] = s_val
            for ch_num, r_val in enumerate(cur_rcin, start=1):
                rec[f'rcin_c{ch_num}'] = r_val

            raw_records.append(rec)
            last_sample_t = t_s

    mlog.close()

    if gps_week > 1500:
        true_flight_date = GPS_EPOCH + datetime.timedelta(weeks=gps_week, milliseconds=gps_ms)
    else:
        true_flight_date = datetime.datetime.fromtimestamp(os.path.getmtime(file_path), tz=datetime.timezone.utc)

    # Backfill field elevation and AMSL datum across initial samples before GPS 3D fix
    if home_elevation != 0.0:
        for r in raw_records:
            if r['field_elev'] == 0.0:
                r['field_elev'] = round(home_elevation, 1)
            # Guarantee AMSL consistency: if alt was recorded as relative altitude or uninitialized
            if r['alt'] == 0.0 or (abs(home_elevation) > 10.0 and abs(r['alt'] - r['rel_alt']) < 1.0) or (abs(home_elevation) > 50.0 and r['alt'] < (home_elevation - 150.0)):
                r['alt'] = round(home_elevation + r['rel_alt'], 1)

    # Backfill and harmonize airspeed temperature across records
    first_valid_arsp_temp = None
    has_any_arsp = False
    for r in raw_records:
        if r.get('arsp_temp', 0.0) != 0.0:
            first_valid_arsp_temp = r['arsp_temp']
            has_any_arsp = True
            break
        elif r.get('tas', 0.0) > 1.0 or r.get('has_arsp'):
            has_any_arsp = True

    if has_any_arsp:
        # If pitot sensor did not report independent temp, fallback to first valid baro_temp
        if first_valid_arsp_temp is None:
            for r in raw_records:
                if r.get('baro_temp', 0.0) != 0.0:
                    first_valid_arsp_temp = r['baro_temp']
                    break

        if first_valid_arsp_temp is not None:
            last_known_arsp_t = first_valid_arsp_temp
            for r in raw_records:
                r['has_arsp'] = True
                if r.get('arsp_temp', 0.0) == 0.0:
                    r['arsp_temp'] = last_known_arsp_t
                else:
                    last_known_arsp_t = r['arsp_temp']
    else:
        # Truly no airspeed sensor present in flight
        for r in raw_records:
            r['arsp_temp'] = 0.0
            r['has_arsp'] = False

    # Downsample evenly for high-speed web plotting if records are huge
    total_recs = len(raw_records)
    if total_recs > target_samples:
        step = int(math.ceil(total_recs / target_samples))
        downsampled = raw_records[::step]
    else:
        downsampled = raw_records

    # Identify Flight Mode Segments
    mode_segments = []
    if raw_records:
        cur_m = raw_records[0]['mode']
        t0 = raw_records[0]['time']
        for r in raw_records:
            if r['mode'] != cur_m:
                dur = r['time'] - t0
                if dur >= 3.0:
                    mode_segments.append({
                        'id': len(mode_segments) + 1,
                        'mode': cur_m,
                        'start': round(t0, 1),
                        'end': round(r['time'], 1),
                        'dur': round(dur, 1)
                    })
                cur_m = r['mode']
                t0 = r['time']
        dur = raw_records[-1]['time'] - t0
        if dur >= 3.0:
            mode_segments.append({
                'id': len(mode_segments) + 1,
                'mode': cur_m,
                'start': round(t0, 1),
                'end': round(raw_records[-1]['time'], 1),
                'dur': round(dur, 1)
            })

    # Identify Discrete Airspeed Test Steps in AUTO mode
    speed_steps = []
    auto_recs = [r for r in raw_records if r['mode'] == 'AUTO']
    if auto_recs:
        cur_dem = round(auto_recs[0]['spdem'], 0)
        t_start = auto_recs[0]['time']
        for r in auto_recs:
            r_dem = round(r['spdem'], 0)
            if r_dem != cur_dem:
                dur = r['time'] - t_start
                if dur >= 12.0 and cur_dem > 10:
                    speed_steps.append({
                        'id': len(speed_steps) + 1,
                        'target_eas': cur_dem,
                        'start': round(t_start, 1),
                        'end': round(r['time'], 1),
                        'dur': round(dur, 1)
                    })
                cur_dem = r_dem
                t_start = r['time']
        dur = auto_recs[-1]['time'] - t_start
        if dur >= 12.0 and cur_dem > 10:
            speed_steps.append({
                'id': len(speed_steps) + 1,
                'target_eas': cur_dem,
                'start': round(t_start, 1),
                'end': round(auto_recs[-1]['time'], 1),
                'dur': round(dur, 1)
            })

    duration_s = (raw_records[-1]['time'] - raw_records[0]['time']) if raw_records else 0.0

    # Airborne Flight Time & Airspeed Alive Window Detection
    alive_start = None
    alive_end = None
    flight_dur = 0.0
    if raw_records:
        home_a = raw_records[0].get('alt', 0) or 0
        # Criterion 1: Sustained flight: tas >= 7.0 & (gs >= 3.0 or deltaAlt >= 2.0) for >= 3 points
        for i in range(len(raw_records) - 3):
            if ((raw_records[i].get('tas') or 0) >= 7.0 and 
                (raw_records[i+1].get('tas') or 0) >= 7.0 and 
                (raw_records[i+2].get('tas') or 0) >= 7.0 and 
                ((raw_records[i].get('gs') or 0) >= 3.0 or abs((raw_records[i].get('alt') or 0) - home_a) >= 2.0)):
                alive_start = raw_records[i]['time']
                break

        # Criterion 2 fallback: tas >= 5.0 for >= 2 points
        if alive_start is None:
            for i in range(len(raw_records) - 2):
                if (raw_records[i].get('tas') or 0) >= 5.0 and (raw_records[i+1].get('tas') or 0) >= 5.0:
                    alive_start = raw_records[i]['time']
                    break

        # Criterion 3 fallback for VTOL/multirotors without pitot: gs >= 2.5 & deltaAlt >= 2.0
        if alive_start is None:
            for i in range(len(raw_records) - 3):
                if ((raw_records[i].get('gs') or 0) >= 2.5 and abs((raw_records[i].get('alt') or 0) - home_a) >= 2.0):
                    alive_start = raw_records[i]['time']
                    break

        if alive_start is not None:
            for i in range(len(raw_records) - 1, 2, -1):
                if ((raw_records[i].get('tas') or 0) >= 7.0 and 
                    (raw_records[i-1].get('tas') or 0) >= 7.0 and 
                    (raw_records[i-2].get('tas') or 0) >= 7.0 and 
                    ((raw_records[i].get('gs') or 0) >= 3.0 or abs((raw_records[i].get('alt') or 0) - home_a) >= 2.0)):
                    alive_end = raw_records[i]['time']
                    break
            if alive_end is None:
                for i in range(len(raw_records) - 1, 1, -1):
                    if (raw_records[i].get('tas') or 0) >= 5.0 and (raw_records[i-1].get('tas') or 0) >= 5.0:
                        alive_end = raw_records[i]['time']
                        break
            if alive_end is None:
                alive_end = raw_records[-1]['time']
            flight_dur = max(0.0, alive_end - alive_start)

    metadata = {
        'filename': os.path.basename(file_path),
        'path': file_path,
        'date_str': true_flight_date.strftime("%Y-%m-%d %H:%M:%S UTC"),
        'firmware': firmware,
        'board': board,
        'duration_s': round(duration_s, 1),
        'flight_time_s': round(flight_dur, 1) if alive_start is not None else 0.0,
        'airspeed_alive_start_s': round(alive_start, 1) if alive_start is not None else None,
        'airspeed_alive_end_s': round(alive_end, 1) if alive_end is not None else None,
        'field_elevation': round(home_elevation, 1),
        'total_samples': total_recs,
        'plotted_samples': len(downsampled)
    }

    return {
        'metadata': metadata,
        'formats': logged_formats,
        'parameters': parameters,
        'telemetry': downsampled,
        'messages': flight_messages,
        'mode_segments': mode_segments,
        'speed_steps': speed_steps
    }


def compute_regime_metrics(raw_telemetry, t_start, t_end, weight_kg=5.1, wing_area_m2=0.45):
    """Calculates all aerodynamic and propulsive parameters for a given time window."""
    sub = [r for r in raw_telemetry if t_start <= r['time'] <= t_end]
    if not sub:
        return None

    tas = np.mean([r['tas'] for r in sub])
    gs = np.mean([r['gs'] for r in sub])
    tho = np.mean([r['tho'] for r in sub])
    pwr = np.mean([r['power'] for r in sub])
    volt = np.mean([r['volt'] for r in sub])
    curr = np.mean([r['curr'] for r in sub])
    aoa = np.mean([r['aoa'] for r in sub])
    press = np.mean([r['press'] for r in sub])
    temp = np.mean([r['temp'] for r in sub])
    roll = np.mean([r['roll'] for r in sub])
    crt = np.mean([r['crt'] for r in sub])
    vwn = np.mean([r['vwn'] for r in sub])
    vwe = np.mean([r['vwe'] for r in sub])

    # Air density: rho = P / (R * T)
    tk = temp + 273.15
    rho = press / (287.05 * tk) if tk > 0 else 1.225

    # Lift Coefficient: CL = (2 * m * g) / (rho * V^2 * S)
    g = 9.80665
    q = 0.5 * rho * (tas ** 2) if tas > 2.0 else 0.0
    cl = (weight_kg * g) / (q * wing_area_m2) if q > 0 and wing_area_m2 > 0 else 0.0

    wh_km_air = (pwr / (tas * 3.6)) if tas > 1.0 else 0.0
    wh_km_gnd = (pwr / (gs * 3.6)) if gs > 1.0 else 0.0

    w_spd = math.sqrt(vwn**2 + vwe**2)
    w_dir = (math.degrees(math.atan2(-vwe, -vwn)) + 360.0) % 360.0 if w_spd > 0.05 else 0.0

    modes = [r['mode'] for r in sub]
    main_mode = max(set(modes), key=modes.count) if modes else 'UNKNOWN'

    return {
        'start': round(t_start, 1),
        'end': round(t_end, 1),
        'duration': round(t_end - t_start, 1),
        'mode': main_mode,
        'tas': round(tas, 2),
        'gs': round(gs, 2),
        'throttle': round(tho, 1),
        'power': round(pwr, 1),
        'volt': round(volt, 2),
        'curr': round(curr, 2),
        'wh_km_air': round(wh_km_air, 2),
        'wh_km_gnd': round(wh_km_gnd, 2),
        'aoa': round(aoa, 2),
        'cl': round(cl, 4),
        'density': round(rho, 4),
        'temp': round(temp, 1),
        'press_hpa': round(press / 100.0, 1),
        'wind_spd': round(w_spd, 1),
        'wind_dir': round(w_dir, 0),
        'roll': round(roll, 1),
        'crt': round(crt, 2)
    }


PHONETIC_ALPHABET = [
    'ALPHA', 'BRAVO', 'CHARLIE', 'DELTA', 'ECHO', 'FOXTROT', 'GOLF', 'HOTEL',
    'INDIA', 'JULIETT', 'KILO', 'LIMA', 'MIKE', 'NOVEMBER', 'OSCAR', 'PAPA',
    'QUEBEC', 'ROMEO', 'SIERRA', 'TANGO', 'UNIFORM', 'VICTOR', 'WHISKEY', 'XRAY', 'YANKEE', 'ZULU'
]


def extract_flight_atis_summary(file_path):
    """
    Extracts high-fidelity aviation ATIS summary & atmospheric metrics from an ArduPilot .bin log.
    Executes in ~0.3 - 1.2 seconds per file.
    """
    import time
    t0 = time.time()
    try:
        stat = os.stat(file_path)
        size_mb = round(stat.st_size / (1024 * 1024), 2)
        mtime = stat.st_mtime
    except Exception as e:
        return {'filename': os.path.basename(file_path), 'filepath': file_path, 'error': str(e)}

    # Default fallback date from organizer
    fallback_dt = organizer.get_true_log_date(file_path)
    flight_dt = None

    try:
        mlog = mavutil.mavlink_connection(file_path, notimestamps=True)
    except Exception as e:
        return {'filename': os.path.basename(file_path), 'filepath': file_path, 'error': str(e)}

    types = ['GPS', 'BARO', 'ARSP', 'XKF2', 'NKF2', 'AHR2', 'BAT', 'MODE', 'PARM', 'MSG']

    lats, lngs, alts = [], [], []
    baro_temps, arsp_temps, press_list = [], [], []
    winds = []
    powers, tas_list, gs_list = [], [], []
    modes = []
    firmware = 'ArduPlane'
    board = 'Matek F405/H743'
    t_start, t_end = None, None
    alive_start, alive_end = None, None
    max_curr_tot = 0.0

    while True:
        m = mlog.recv_match(type=types, blocking=False)
        if m is None:
            break
        mtype = m.get_type()
        t_sec = getattr(m, 'TimeUS', None)
        if t_sec is not None:
            t_s = t_sec / 1e6
            if t_start is None:
                t_start = t_s
            t_end = t_s

        if mtype == 'GPS':
            status = getattr(m, 'Status', 0)
            if status >= 3:
                lat = getattr(m, 'Lat', 0)
                lng = getattr(m, 'Lng', 0)
                alt = getattr(m, 'Alt', 0)
                spd = getattr(m, 'Spd', 0)
                if lat != 0 and lng != 0:
                    lats.append(lat if abs(lat) <= 90 else lat * 1e-7)
                    lngs.append(lng if abs(lng) <= 180 else lng * 1e-7)
                if alt != 0:
                    alts.append(alt)
                if spd is not None and spd >= 0:
                    gs_list.append(spd)
                    if spd >= 3.0 and t_sec is not None:
                        if alive_start is None:
                            alive_start = t_s
                        alive_end = t_s
                gwk = getattr(m, 'GWk', 0)
                gms = getattr(m, 'GMS', 0)
                if flight_dt is None and gwk > 1500:
                    flight_dt = GPS_EPOCH + datetime.timedelta(weeks=gwk, milliseconds=gms)

        elif mtype == 'BARO':
            p = getattr(m, 'Press', 0)
            t = getattr(m, 'Temp', 0)
            if p > 50000:
                press_list.append(p / 100.0)
            if -40 < t < 85 and t != 0:
                baro_temps.append(t)

        elif mtype == 'ARSP':
            t = getattr(m, 'Temp', 0)
            asp = getattr(m, 'Airspeed', 0)
            if -40 < t < 85 and t != 0:
                arsp_temps.append(t)
            if asp is not None and asp > 0.5:
                tas_list.append(asp)
                if asp >= 7.0 and t_sec is not None:
                    if alive_start is None:
                        alive_start = t_s
                    alive_end = t_s

        elif mtype in ('XKF2', 'NKF2', 'AHR2'):
            vwn = getattr(m, 'VWN', None)
            vwe = getattr(m, 'VWE', None)
            if vwn is not None and vwe is not None:
                w_spd = math.sqrt(vwn*vwn + vwe*vwe)
                if 0.05 < w_spd < 50.0:
                    w_dir = (math.degrees(math.atan2(-vwe, -vwn)) + 360.0) % 360.0
                    winds.append((w_spd, w_dir))

        elif mtype == 'BAT':
            v = getattr(m, 'Volt', 0)
            c = getattr(m, 'Curr', 0)
            ctot = getattr(m, 'CurrTot', 0)
            if ctot > max_curr_tot:
                max_curr_tot = ctot
            if v > 5.0 and c >= 0:
                powers.append(v * c)

        elif mtype == 'MODE':
            mode_val = getattr(m, 'Mode', None)
            if mode_val is not None and mode_val not in modes:
                modes.append(mode_val)

        elif mtype == 'MSG':
            msg_str = getattr(m, 'Message', '')
            if 'ArduPlane' in msg_str or 'ArduCopter' in msg_str:
                firmware = msg_str.strip()
            elif 'Matek' in msg_str or 'Cube' in msg_str or 'Pixhawk' in msg_str:
                board = msg_str.strip()

    mlog.close()

    # Determine final true date
    final_dt = flight_dt if flight_dt is not None else fallback_dt
    dur_s = max(0.0, (t_end - t_start)) if (t_start is not None and t_end is not None) else 0.0
    flt_s = max(0.0, (alive_end - alive_start)) if (alive_start is not None and alive_end is not None) else 0.0

    mean_lat = round(sum(lats)/len(lats), 6) if lats else 0.0
    mean_lng = round(sum(lngs)/len(lngs), 6) if lngs else 0.0
    field_elev = round(alts[0], 1) if alts else 0.0

    # Outside Air Temperature (favor free-stream pitot temp, fallback to baro)
    primary_temps = arsp_temps if len(arsp_temps) > 50 else baro_temps
    oat_mean = round(sum(primary_temps)/len(primary_temps), 1) if primary_temps else 25.0
    oat_min = round(min(primary_temps), 1) if primary_temps else oat_mean
    oat_max = round(max(primary_temps), 1) if primary_temps else oat_mean

    # Atmospheric Pressure & QNH calculation
    p_sta_mean = round(sum(press_list)/len(press_list), 1) if press_list else 1013.2
    try:
        qnh_calc = p_sta_mean * (max(0.001, 1.0 - (field_elev / 44330.77)) ** -5.25588)
        qnh = round(qnh_calc, 1)
    except Exception:
        qnh = p_sta_mean

    # Air Density (kg/m^3)
    tk = oat_mean + 273.15
    rho = round((p_sta_mean * 100.0) / (287.05 * tk), 4) if tk > 0 else 1.225
    rho_ratio_pct = round((rho / 1.2250) * 100.0, 1)

    # Pressure Altitude, ISA Temp, and Density Altitude (m & ft)
    pa = (1013.25 - qnh) * 30.0 + field_elev
    isa_temp = round(15.0 - 0.0065 * field_elev, 1)
    isa_dev = round(oat_mean - isa_temp, 1)
    density_alt_m = round(pa + 118.8 * isa_dev, 1)
    density_alt_ft = int(round(density_alt_m * 3.28084))

    # Wind Vector
    if winds:
        w_spds = [w[0] for w in winds]
        w_dirs = [w[1] for w in winds]
        mean_w_spd_ms = round(sum(w_spds)/len(w_spds), 1)
        mean_w_spd_kt = round(mean_w_spd_ms * 1.94384, 1)
        max_w_spd_kt = round(max(w_spds) * 1.94384, 1)
        mean_w_dir = int(round(sum(w_dirs)/len(w_dirs))) % 360
    else:
        mean_w_spd_ms, mean_w_spd_kt, max_w_spd_kt, mean_w_dir = 0.0, 0.0, 0.0, 0

    # Propulsion and Energy Efficiency
    mean_pwr = round(sum(powers)/len(powers), 1) if powers else 0.0
    mean_tas = round(sum(tas_list)/len(tas_list), 1) if tas_list else 0.0
    max_tas = round(max(tas_list), 1) if tas_list else 0.0
    mean_gs = round(sum(gs_list)/len(gs_list), 1) if gs_list else 0.0
    speed_ref = mean_tas if mean_tas > 2.0 else mean_gs
    wh_km = round(mean_pwr / (speed_ref * 3.6), 2) if speed_ref > 2.0 else 0.0
    total_mah = round(max_curr_tot, 1)

    def fmt_dur(s):
        m, sec = divmod(int(round(s)), 60)
        h, m = divmod(m, 60)
        return f"{h}h {m:02d}m {sec:02d}s" if h > 0 else f"{m:02d}m {sec:02d}s"

    met_str = fmt_dur(dur_s)
    flt_str = fmt_dur(flt_s)

    parse_time = round(time.time() - t0, 2)
    return {
        'filename': os.path.basename(file_path),
        'filepath': file_path,
        'size_mb': size_mb,
        'mtime': mtime,
        'date_str': final_dt.strftime('%Y-%m-%d %H:%M:%S UTC'),
        'date_iso': final_dt.isoformat(),
        'year': final_dt.year,
        'month': final_dt.month,
        'day': final_dt.day,
        'lat': mean_lat,
        'lng': mean_lng,
        'elev_m': field_elev,
        'duration_s': round(dur_s, 1),
        'flight_time_s': round(flt_s, 1),
        'met_str': met_str,
        'flt_str': flt_str,
        'oat_mean_c': oat_mean,
        'oat_min_c': oat_min,
        'oat_max_c': oat_max,
        'p_sta_hpa': p_sta_mean,
        'qnh_hpa': qnh,
        'rho_kgm3': rho,
        'rho_pct': rho_ratio_pct,
        'density_alt_m': density_alt_m,
        'density_alt_ft': density_alt_ft,
        'isa_temp_c': isa_temp,
        'isa_dev_c': isa_dev,
        'wind_spd_kt': mean_w_spd_kt,
        'wind_spd_ms': mean_w_spd_ms,
        'wind_max_kt': max_w_spd_kt,
        'wind_dir_deg': mean_w_dir,
        'mean_pwr_w': mean_pwr,
        'mean_tas_ms': mean_tas,
        'max_tas_ms': max_tas,
        'mean_gs_ms': mean_gs,
        'wh_km': wh_km,
        'consumed_mah': total_mah,
        'board': board,
        'firmware': firmware,
        'has_valid_flight': flt_s > 30.0,
        'parse_time_s': parse_time
    }


def get_historical_atis_database(folder_path=None, force_refresh=False):
    """
    Scans folder_path for all ArduPilot .bin flight logs, leveraging a disk cache
    (.atis_flight_cache.json) to achieve sub-millisecond retrieval on subsequent calls.
    """
    if not folder_path or not os.path.exists(folder_path):
        folder_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'uploads')
    
    folder_path = os.path.normpath(folder_path)
    cache_file = os.path.join(folder_path, '.atis_flight_cache.json')
    
    cache_data = {'version': 1, 'files': {}}
    if not force_refresh and os.path.exists(cache_file):
        try:
            with open(cache_file, 'r', encoding='utf-8') as f:
                loaded = json.load(f)
                if loaded.get('version') == 1 and isinstance(loaded.get('files'), dict):
                    cache_data = loaded
        except Exception:
            cache_data = {'version': 1, 'files': {}}

    cached_files = cache_data.get('files', {})
    updated = False
    
    # Discover all .bin files in folder and direct subfolders
    found_paths = []
    for root, dirs, files in os.walk(folder_path):
        # Ignore hidden / temporary folders
        dirs[:] = [d for d in dirs if not d.startswith('.') and d != '_temp_org']
        for fn in files:
            if fn.lower().endswith('.bin'):
                found_paths.append(os.path.join(root, fn))
                
    valid_filenames = set()
    flights = []
    
    for fpath in found_paths:
        fname = os.path.basename(fpath)
        valid_filenames.add(fname)
        try:
            st = os.stat(fpath)
            fsize = st.st_size
            fmtime = st.st_mtime
        except Exception:
            continue
            
        # Cache check: key is filename, size, and mtime
        cached_entry = cached_files.get(fname)
        if (cached_entry and 
            cached_entry.get('size') == fsize and 
            abs(cached_entry.get('mtime', 0) - fmtime) < 1.0 and 
            'summary' in cached_entry):
            summary = cached_entry['summary']
            summary['filepath'] = fpath
            flights.append(summary)
        else:
            summary = extract_flight_atis_summary(fpath)
            cached_files[fname] = {
                'size': fsize,
                'mtime': fmtime,
                'summary': summary
            }
            flights.append(summary)
            updated = True

    # Purge removed files from cache
    stale_keys = [k for k in cached_files if k not in valid_filenames]
    if stale_keys:
        for k in stale_keys:
            del cached_files[k]
        updated = True

    if updated or not os.path.exists(cache_file):
        cache_data['last_updated_utc'] = datetime.datetime.now(datetime.timezone.utc).isoformat()
        cache_data['total_logs'] = len(flights)
        try:
            with open(cache_file, 'w', encoding='utf-8') as f:
                json.dump(cache_data, f, indent=2)
        except Exception as e:
            print(f"Warning: could not write ATIS cache: {e}")

    # Sort flights newest first
    flights.sort(key=lambda x: x.get('date_iso', ''), reverse=True)

    # Assign cyclic phonetic callsign (ZULU down to ALPHA)
    for idx, flt in enumerate(flights):
        p_letter = PHONETIC_ALPHABET[idx % len(PHONETIC_ALPHABET)]
        flt['atis_callsign'] = p_letter
        
        # Build standard ICAO D-ATIS ticker
        d_utc = flt.get('date_str', '').split(' ')[1].replace(':', '')[:4] if ' ' in flt.get('date_str', '') else '1200'
        day_str = f"{flt.get('day', 1):02d}"
        zulu_time = f"{day_str}{d_utc}Z"
        w_dir = f"{flt.get('wind_dir_deg', 0):03d}"
        w_kt = f"{int(round(flt.get('wind_spd_kt', 0))):02d}"
        temp_sign = '+' if flt.get('oat_mean_c', 0) >= 0 else ''
        oat_str = f"{temp_sign}{flt.get('oat_mean_c', 0):.1f}°C"
        qnh_val = flt.get('qnh_hpa', 1013.2)
        da_val = flt.get('density_alt_m', 0)
        da_ft_val = flt.get('density_alt_ft', 0)
        rho_val = flt.get('rho_kgm3', 1.225)
        flt_dur = flt.get('flt_str', '00m 00s')
        wh_km_val = flt.get('wh_km', 0.0)
        pwr_val = flt.get('mean_pwr_w', 0.0)
        tas_val = flt.get('mean_tas_ms', 0.0)
        
        flt['atis_ticker'] = (
            f"SHARKEE FIELD INFO {p_letter} · {zulu_time} · WIND {w_dir}/{w_kt}KT · VIS 10KM+ · CLR · "
            f"TEMP {oat_str} · QNH {qnh_val:.1f} HPA · DENSITY ALT {da_val:.0f}M ({da_ft_val}FT) · "
            f"RHO {rho_val:.4f} KG/M³ · CRUISE {tas_val:.1f}M/S @ {pwr_val:.0f}W ({wh_km_val:.2f} WH/KM) · "
            f"FLT {flt_dur} · AIRFRAME NOMINAL"
        )

    # Compute aggregate fleet atmospheric & performance summary
    total_flt_sec = sum(f.get('flight_time_s', 0) for f in flights)
    total_dur_sec = sum(f.get('duration_s', 0) for f in flights)
    valid_flights = [f for f in flights if f.get('has_valid_flight')]
    
    oats = [f['oat_mean_c'] for f in flights if 'oat_mean_c' in f]
    qnhs = [f['qnh_hpa'] for f in flights if 'qnh_hpa' in f]
    das = [f['density_alt_m'] for f in flights if 'density_alt_m' in f]
    winds = [f['wind_spd_kt'] for f in flights if 'wind_spd_kt' in f]
    energies = [f['wh_km'] for f in valid_flights if f.get('wh_km', 0) > 0.5]
    
    def fmt_cum_dur(s):
        h = int(s // 3600)
        m = int((s % 3600) // 60)
        return f"{h}h {m:02d}m"
        
    summary = {
        'total_logs': len(flights),
        'valid_flights_count': len(valid_flights),
        'total_airborne_time': fmt_cum_dur(total_flt_sec),
        'total_airborne_sec': round(total_flt_sec, 1),
        'total_operations_time': fmt_cum_dur(total_dur_sec),
        'date_earliest': flights[-1]['date_str'] if flights else 'N/A',
        'date_latest': flights[0]['date_str'] if flights else 'N/A',
        'temp_min_c': min(oats) if oats else 25.0,
        'temp_max_c': max(oats) if oats else 25.0,
        'temp_mean_c': round(sum(oats)/len(oats), 1) if oats else 25.0,
        'qnh_min_hpa': min(qnhs) if qnhs else 1013.2,
        'qnh_max_hpa': max(qnhs) if qnhs else 1013.2,
        'qnh_mean_hpa': round(sum(qnhs)/len(qnhs), 1) if qnhs else 1013.2,
        'density_alt_min_m': min(das) if das else 0,
        'density_alt_max_m': max(das) if das else 0,
        'density_alt_mean_m': round(sum(das)/len(das), 1) if das else 0,
        'wind_max_kt': max(winds) if winds else 0.0,
        'wind_mean_kt': round(sum(winds)/len(winds), 1) if winds else 0.0,
        'mean_wh_km': round(sum(energies)/len(energies), 2) if energies else 0.0
    }

    return {
        'success': True,
        'folder': folder_path,
        'summary': summary,
        'flights': flights
    }

