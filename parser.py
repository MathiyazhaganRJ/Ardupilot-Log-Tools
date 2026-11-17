"""
ArduPilot DataFlash Binary Log Parser
Extracts synchronized flight telemetry, message formats, and aerodynamic performance.
"""

import os
import math
import datetime
import numpy as np
from pymavlink import mavutil

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
    while count < 25000:
        msg = mlog.recv_match(blocking=False)
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
        if mtype in SKIP_TYPES:
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


def parse_flight_telemetry(file_path, target_samples=8000):
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
    last_imu_t = 0.0
    sample_interval = 0.05  # 20 Hz timeline

    while True:
        msg = mlog.recv_match(blocking=False)
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

        if mtype in SKIP_TYPES:
            continue

        t_us = getattr(msg, 'TimeUS', None)
        if t_us is None:
            continue
        t_s = t_us / 1e6

        # Downsample IMU to 25 Hz
        if mtype == 'IMU':
            if (t_s - last_imu_t) < 0.04:
                continue
            last_imu_t = t_s

        # Determine instance prefix
        inst_col = INST_MSGS.get(mtype)
        inst = getattr(msg, inst_col, None) if inst_col else None
        prefix = f"{mtype}[{inst}]" if inst is not None else mtype

        # Register in logged formats
        if prefix not in logged_formats and mtype in fmts:
            col_list = [c for c in fmts[mtype] if c not in ('TimeUS', inst_col)]
            logged_formats[prefix] = col_list

        # Extract numeric fields for state tracking
        cols = fmts.get(mtype, [])
        for col in cols:
            if col in ('TimeUS', inst_col):
                continue
            v = getattr(msg, col, None)
            if isinstance(v, (int, float)):
                v_clean = round(v, 4) if isinstance(v, float) else v
                current_state[f"{prefix}.{col}"] = v_clean
                if inst == 0:
                    current_state[f"{mtype}.{col}"] = v_clean

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
            rec['roll'] = round(cur_roll, 1)
            rec['pitch'] = round(cur_pitch, 1)
            rec['yaw'] = round(cur_yaw, 1)
            rec['lat'] = round(cur_lat, 7)
            rec['lng'] = round(cur_lng, 7)
            rec['alt'] = round(cur_alt, 1)
            rec['rel_alt'] = round(cur_rel_alt, 1)
            rec['field_elev'] = round(home_elevation, 1)
            rec['crt'] = round(cur_crt, 2)
            rec['tas'] = round(cur_tas, 2)
            rec['gs'] = round(cur_gs, 2)
            rec['spdem'] = round(cur_spdem, 1)
            rec['tho'] = round(cur_tho, 1)

            # BAT fields
            v = current_state.get('BAT[0].Volt', current_state.get('BAT.Volt', 0.0))
            i = current_state.get('BAT[0].Curr', current_state.get('BAT.Curr', 0.0))
            curr_tot = current_state.get('BAT[0].CurrTot', current_state.get('BAT.CurrTot', 0.0))
            rem_pct = current_state.get('BAT[0].RemPct', current_state.get('BAT.RemPct', 100))
            pwr = round(v * i, 1)
            wh_km = (pwr / (cur_tas * 3.6)) if cur_tas > 2.0 else 0.0

            rec['volt'] = round(v, 2)
            rec['curr'] = round(i, 2)
            rec['power'] = pwr
            rec['consumed_mah'] = round(curr_tot, 1)
            rec['rem_pct'] = int(rem_pct) if rem_pct is not None else 100
            rec['wh_km'] = round(wh_km, 2)

            # Atmospheric & Aerodynamic metrics
            rec['aoa'] = round(cur_aoa, 2)
            rec['ssa'] = round(cur_ssa, 2)
            rec['press'] = round(cur_press, 1)
            rec['press_hpa'] = round(cur_press / 100.0, 1) if cur_press > 2000 else round(cur_press, 1)
            rec['qnh_hpa'] = round((cur_press / 100.0 if cur_press > 2000 else cur_press) * (max(0.001, 1.0 - ((cur_amsl if cur_amsl != 0.0 else cur_alt) / 44330.77)) ** -5.25588), 1)
            rec['temp'] = round(cur_temp, 1)
            rec['baro_temp'] = round(cur_baro_temp, 1)
            rec['arsp_temp'] = round(cur_arsp_temp, 1) if cur_arsp_temp is not None else 0.0
            rec['has_arsp'] = cur_arsp_temp is not None

            amb_temp = cur_arsp_temp if cur_arsp_temp is not None else cur_baro_temp
            tk = amb_temp + 273.15
            rho = cur_press / (287.05 * tk) if tk > 0 else 1.225
            rec['density'] = round(rho, 4)

            w_spd = math.sqrt(cur_vwn**2 + cur_vwe**2)
            w_dir = (math.degrees(math.atan2(-cur_vwe, -cur_vwn)) + 360.0) % 360.0 if w_spd > 0.05 else 0.0
            rec['wind_spd'] = round(w_spd, 1)
            rec['wind_dir'] = round(w_dir, 0)
            rec['vwn'] = round(cur_vwn, 2)
            rec['vwe'] = round(cur_vwe, 2)

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

    metadata = {
        'filename': os.path.basename(file_path),
        'path': file_path,
        'date_str': true_flight_date.strftime("%Y-%m-%d %H:%M:%S UTC"),
        'firmware': firmware,
        'board': board,
        'duration_s': round(duration_s, 1),
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

