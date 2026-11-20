"""
ArduPilot Flight Log Organizer Engine
Categorizes and archives flight telemetry logs into structured Year/Month/Date directories.
Extracted and enhanced from the original ArduPilot Log Manager tool.
"""

import os
import sys
import re
import shutil
import struct
import datetime
import subprocess
import threading
import time

def is_valid_flight_log(filepath):
    """
    Validates whether a file is an authentic ArduPilot DataFlash flight log.
    Rejects firmware binaries (e.g. AP_Periph.bin, arduplane.bin, bootloader.bin),
    scope/multimeter data dumps, EEPROM dumps, and non-log binary files.
    """
    try:
        if not os.path.isfile(filepath):
            return False
        sz = os.path.getsize(filepath)
        if sz < 1024:
            return False
        with open(filepath, 'rb') as f:
            chunk = f.read(4096)
        # DataFlash binary logs must have packet sync header 0xA3 0x95 in lead-in
        idx = chunk.find(b'\xa3\x95')
        if idx != -1 and idx < 256:
            return True
        # Plaintext DataFlash logs (.log) start with FMT or PARM definitions
        if chunk.startswith(b'FMT,') or chunk.startswith(b'PARM,') or (b'\nFMT,' in chunk[:1024]):
            return True
        return False
    except Exception:
        return False

def extract_binary_gps_date(filepath, max_scan_bytes=262144):
    """
    Ultra-fast (5-15ms) binary reader for DataFlash GPS packets.
    Extracts true GPS epoch date (GWk + GMS) when Status >= 3.
    """
    try:
        if not os.path.exists(filepath) or os.path.getsize(filepath) < 512:
            return None
        with open(filepath, 'rb') as f:
            buf = f.read(max_scan_bytes)

        # 1. Parse FMT messages to locate GPS type ID and format layout
        gps_type = None
        gps_len = 0
        gps_fmt = ''
        pos = 0
        buf_len = len(buf)
        fmt_limit = min(buf_len, 65536) - 89
        while pos < fmt_limit:
            if buf[pos] == 0xA3 and buf[pos+1] == 0x95 and buf[pos+2] == 0x80:
                m_type = buf[pos+3]
                m_len = buf[pos+4]
                name = buf[pos+5:pos+9].decode('latin1', errors='ignore').rstrip('\x00 ')
                if name == 'GPS':
                    gps_type = m_type
                    gps_len = m_len
                    gps_fmt = buf[pos+9:pos+25].decode('latin1', errors='ignore').rstrip('\x00 ')
                    break
                pos += 89
            else:
                pos += 1

        if not gps_type or gps_len < 16:
            return None

        # ArduPilot GPS format:
        # standard QBBIH: Q(8)=TimeUS, B(1)=I, B(1)=Status, I(4)=GMS, H(2)=GWk
        # standard QBIH: Q(8)=TimeUS, B(1)=Status, I(4)=GMS, H(2)=GWk
        has_instance = gps_fmt.startswith('QBB')
        status_offset = 9 if has_instance else 8
        gms_offset = 10 if has_instance else 9
        gwk_offset = 14 if has_instance else 13

        scan_limit = buf_len - gps_len
        while pos < scan_limit:
            if buf[pos] == 0xA3 and buf[pos+1] == 0x95 and buf[pos+2] == gps_type:
                payload = buf[pos+3:pos+gps_len]
                if len(payload) >= gwk_offset + 2:
                    status = payload[status_offset]
                    gms = struct.unpack('<I', payload[gms_offset:gms_offset+4])[0]
                    gwk = struct.unpack('<H', payload[gwk_offset:gwk_offset+2])[0]
                    if status >= 3 and gwk > 1500:
                        dt = datetime.datetime(1980, 1, 6) + datetime.timedelta(weeks=gwk, milliseconds=gms)
                        if dt.year >= 2022:
                            return dt
                pos += gps_len
            else:
                pos += 1
    except Exception:
        pass
    return None

def get_true_log_date(filepath):
    """
    SMART DATE LOGIC (Enhanced with Mission Planner formats and fast binary GPS fallback):
    1. Extract Date from Filename:
       - Regex 1: YYYY-MM-DD or YYYY_MM_DD
       - Regex 2: YYYYMMDD
       - Regex 3: DD-MM-YYYY or DD_MM_YYYY (Standard Mission Planner format)
       VALIDITY CHECK: Is the year >= 2022?
       - YES: Use Filename Date.
       - NO (1980, 2000): Corrupt/No GPS. Probe fast binary GPS or fallback to Modified Date.
    2. Fast Binary GPS Inspection for generic logs without date in filename (e.g. 00000001.BIN).
    3. Fallback to System File Modified Time (mtime).
    """
    filename = os.path.basename(filepath)
    potential_date = None

    # Regex 1: YYYY-MM-DD
    match = re.search(r"(\d{4})[-_](\d{2})[-_](\d{2})", filename)
    if match:
        try:
            potential_date = datetime.datetime(int(match.group(1)), int(match.group(2)), int(match.group(3)))
        except Exception:
            pass

    # Regex 2: YYYYMMDD
    if not potential_date:
        match = re.search(r"(\d{4})(\d{2})(\d{2})", filename)
        if match:
            try:
                potential_date = datetime.datetime(int(match.group(1)), int(match.group(2)), int(match.group(3)))
            except Exception:
                pass

    # Regex 3: DD-MM-YYYY or DD_MM_YYYY (e.g. "75 02-11-2025 11-09-40.bin")
    if not potential_date:
        match = re.search(r"(?:^|[^\d])(\d{1,2})[-_](\d{1,2})[-_](\d{4})(?:[^\d]|$)", filename)
        if match:
            try:
                p1, p2, yr = int(match.group(1)), int(match.group(2)), int(match.group(3))
                if yr >= 2022:
                    if 1 <= p2 <= 12 and 1 <= p1 <= 31: # DD-MM-YYYY
                        potential_date = datetime.datetime(yr, p2, p1)
                    elif 1 <= p1 <= 12 and 1 <= p2 <= 31: # MM-DD-YYYY
                        potential_date = datetime.datetime(yr, p1, p2)
            except Exception:
                pass

    # VALIDITY CHECK (The "2022 Rule")
    if potential_date and potential_date.year >= 2022:
        return potential_date

    # Fast binary probe for generic files or missing filename dates
    if filepath and os.path.isfile(filepath):
        gps_date = extract_binary_gps_date(filepath)
        if gps_date and gps_date.year >= 2022:
            return gps_date

    # Fallback to System Modified Time
    try:
        return datetime.datetime.fromtimestamp(os.path.getmtime(filepath))
    except Exception:
        return datetime.datetime.now()


def _resolve_folders(folder_input):
    """Normalizes single path, list of paths, or delimiter-separated string into a list of valid directories."""
    if not folder_input:
        return []
    if isinstance(folder_input, list):
        raw_list = folder_input
    elif isinstance(folder_input, str):
        # Support semicolon, comma, or newline separation
        raw_list = [p.strip() for p in re.split(r'[;\r\n]+', folder_input) if p.strip()]
    else:
        raw_list = [str(folder_input)]

    valid_folders = []
    for p in raw_list:
        clean = os.path.normpath(p.strip(' "\''))
        if os.path.exists(clean) and os.path.isdir(clean):
            valid_folders.append(clean)
    return valid_folders


def scan_source_logs(source_folder):
    """
    Scans a directory or directories recursively for flight log files (.bin, .log, .tlog).
    Returns list of discovered log metadata.
    """
    folders = _resolve_folders(source_folder)
    if not folders:
        return {'success': False, 'error': f"Source folder '{source_folder}' does not exist or is not a directory.", 'logs': [], 'count': 0, 'total_size_mb': 0.0}

    logs = []
    valid_exts = ('.bin',)
    total_bytes = 0

    try:
        seen_paths = set()
        for fld in folders:
            for root, dirs, files in os.walk(fld):
                for file in files:
                    if file.lower().endswith(valid_exts):
                        full_path = os.path.join(root, file)
                        if full_path in seen_paths or not is_valid_flight_log(full_path):
                            continue
                        seen_paths.add(full_path)
                        try:
                            sz = os.path.getsize(full_path)
                            total_bytes += sz
                            dt = get_true_log_date(full_path)
                            logs.append({
                                'filename': file,
                                'path': full_path,
                                'size_mb': round(sz / (1024 * 1024), 2),
                                'date': dt.strftime("%Y-%m-%d"),
                                'year': dt.strftime("%Y"),
                                'month': dt.strftime("%B"),
                                'ext': os.path.splitext(file)[1].lower()
                            })
                        except Exception:
                            pass
    except Exception as e:
        return {'success': False, 'error': str(e), 'logs': [], 'count': 0, 'total_size_mb': 0.0}

    # Sort newest to oldest
    logs.sort(key=lambda x: (x['date'], x['filename']), reverse=True)

    return {
        'success': True,
        'count': len(logs),
        'total_size_mb': round(total_bytes / (1024 * 1024), 2),
        'source_folder': "; ".join(folders) if len(folders) > 1 else folders[0],
        'folders_scanned': folders,
        'logs': logs[:500],  # Return up to 500 for preview
        'has_more': len(logs) > 500
    }


def organize_flight_logs(source_folder, dest_folder, structure="year_month_date", mode="copy", progress_callback=None):
    """
    Organizes flight logs from source_folder into dest_folder based on Year, Month, and Date.
    
    Structure options:
      - 'year_month_date': <dest>/YYYY/Month/YYYY-MM-DD/<file>
      - 'year_date':       <dest>/YYYY/YYYY-MM-DD/<file>
      - 'year_month':      <dest>/YYYY/Month/<file>
      - 'flat_date':       <dest>/YYYY-MM-DD/<file>
    
    Modes:
      - 'copy': Safe archive, keeps original in source.
      - 'move': Moves file and deletes from source.
    """
    folders = _resolve_folders(source_folder)
    if not folders:
        return {'success': False, 'error': f"Source folder '{source_folder}' does not exist or is not a directory."}
    
    if not dest_folder:
        return {'success': False, 'error': "Destination folder not specified."}

    dest_folder = os.path.normpath(dest_folder.strip(' "\''))
    os.makedirs(dest_folder, exist_ok=True)

    valid_exts = ('.bin',)
    all_files = []
    seen_paths = set()
    for fld in folders:
        for root, dirs, files in os.walk(fld):
            for file in files:
                if file.lower().endswith(valid_exts):
                    p = os.path.join(root, file)
                    if p not in seen_paths and is_valid_flight_log(p):
                        seen_paths.add(p)
                        all_files.append(p)

    total = len(all_files)
    if total == 0:
        return {
            'success': True,
            'total': 0,
            'copied': 0,
            'moved': 0,
            'new_organized': 0,
            'skipped': 0,
            'errors': 0,
            'files': [],
            'dest_folder': dest_folder,
            'message': 'No .bin flight logs found in source folder(s).'
        }

    stats = {
        'total': total,
        'copied': 0,
        'moved': 0,
        'new_organized': 0,
        'skipped': 0,
        'errors': 0,
        'files': [],
        'dest_folder': dest_folder,
        'created_folders': set()
    }

    is_move = (str(mode).lower() == 'move')

    for idx, src_path in enumerate(all_files):
        filename = os.path.basename(src_path)
        try:
            date_obj = get_true_log_date(src_path)
            year_str = date_obj.strftime("%Y")
            month_str = date_obj.strftime("%B")
            date_str = date_obj.strftime("%Y-%m-%d")

            if structure == "year_date":
                target_dir = os.path.join(dest_folder, year_str, date_str)
            elif structure == "year_month":
                target_dir = os.path.join(dest_folder, year_str, month_str)
            elif structure == "flat_date":
                target_dir = os.path.join(dest_folder, date_str)
            else: # default: year_month_date
                target_dir = os.path.join(dest_folder, year_str, month_str, date_str)

            os.makedirs(target_dir, exist_ok=True)
            stats['created_folders'].add(target_dir)

            target_path = os.path.join(target_dir, filename)
            src_size = os.path.getsize(src_path)

            # Skip duplicate if identical size already exists in target
            if os.path.exists(target_path) and os.path.getsize(target_path) == src_size:
                stats['skipped'] += 1
                stats['files'].append({
                    'filename': filename,
                    'src': src_path,
                    'dest': target_path,
                    'size_mb': round(src_size / (1024 * 1024), 2),
                    'date': date_str,
                    'status': 'skipped'
                })
            else:
                # Collision handling: If target_path exists with a different file size,
                # do not overwrite! Append collision suffix (e.g. 00000001_1.BIN)
                if os.path.exists(target_path):
                    base_stem, ext = os.path.splitext(filename)
                    counter = 1
                    collision_path = os.path.join(target_dir, f"{base_stem}_{counter}{ext}")
                    is_dup = False
                    while os.path.exists(collision_path):
                        if os.path.getsize(collision_path) == src_size:
                            is_dup = True
                            target_path = collision_path
                            break
                        counter += 1
                        collision_path = os.path.join(target_dir, f"{base_stem}_{counter}{ext}")

                    if is_dup:
                        stats['skipped'] += 1
                        stats['files'].append({
                            'filename': os.path.basename(target_path),
                            'src': src_path,
                            'dest': target_path,
                            'size_mb': round(src_size / (1024 * 1024), 2),
                            'date': date_str,
                            'status': 'skipped'
                        })
                        continue

                    target_path = collision_path

                if is_move:
                    shutil.move(src_path, target_path)
                    stats['moved'] += 1
                    status_action = 'moved'
                else:
                    shutil.copy2(src_path, target_path)
                    stats['copied'] += 1
                    status_action = 'copied'

                stats['new_organized'] += 1
                stats['files'].append({
                    'filename': os.path.basename(target_path),
                    'src': src_path,
                    'dest': target_path,
                    'size_mb': round(src_size / (1024 * 1024), 2),
                    'date': date_str,
                    'status': status_action
                })

        except Exception as e:
            stats['errors'] += 1
            stats['files'].append({
                'filename': filename,
                'src': src_path,
                'dest': None,
                'size_mb': 0,
                'date': 'Unknown',
                'status': 'error',
                'error': str(e)
            })

        if progress_callback:
            try:
                progress_callback({
                    'current': idx + 1,
                    'total': total,
                    'filename': filename,
                    'status': stats['files'][-1]['status'] if stats['files'] else 'copied',
                    'copied': stats['copied'],
                    'moved': stats['moved'],
                    'new_organized': stats['moved'] if is_move else stats['copied'],
                    'skipped': stats['skipped'],
                    'errors': stats['errors']
                })
            except TypeError:
                progress_callback(idx + 1, total, filename)

    stats['new_organized'] = stats['moved'] if is_move else stats['copied']
    stats['created_folders'] = sorted(list(stats['created_folders']))
    stats['success'] = True
    return stats


def pick_folder_native(title="Select Folder", initial_dir=""):
    """
    Launches a dedicated subprocess to prompt the native Windows Folder Picker dialog.
    Directly mirrors the proven implementation from the original ArduPilot Log Manager tool,
    with explicit interactive user desktop routing to WinSta0\\Default.
    """
    picker_script = os.path.join(os.path.dirname(os.path.abspath(__file__)), "picker.py")
    si = subprocess.STARTUPINFO()
    si.lpDesktop = r"WinSta0\Default"
    si.dwFlags = subprocess.STARTF_USESHOWWINDOW
    si.wShowWindow = 5 # SW_SHOW
    try:
        p = subprocess.run(
            [sys.executable, picker_script, title or "Select Folder"],
            startupinfo=si,
            capture_output=True,
            text=True,
            timeout=180
        )
        out = p.stdout.strip()
        if out and os.path.exists(out):
            return os.path.normpath(out)
    except Exception as e:
        print(f"[Organizer] Folder picker error: {e}")
    return None


def open_folder_in_explorer(folder_path):
    """
    Opens target file or directory in Windows File Explorer.
    If it's a file, selects and highlights it; if it's a directory, opens it.
    """
    clean_path = os.path.normpath(folder_path)
    if os.path.isfile(clean_path):
        subprocess.Popen(f'explorer /select,"{clean_path}"')
        return True
    elif os.path.isdir(clean_path):
        subprocess.Popen(f'explorer "{clean_path}"')
        return True
    return False
