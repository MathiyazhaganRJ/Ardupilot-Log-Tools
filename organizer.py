"""
ArduPilot Flight Log Organizer Engine
Categorizes and archives flight telemetry logs into structured Year/Month/Date directories.
Extracted and enhanced from the original ArduPilot Log Manager tool.
"""

import os
import sys
import re
import shutil
import datetime
import subprocess
import threading
import time

GPS_EPOCH = datetime.datetime(1980, 1, 6, tzinfo=datetime.timezone.utc)

def get_true_log_date(filepath):
    """
    Smart Date Extraction:
    1. Extract Date from Filename (Regex YYYY-MM-DD or YYYYMMDD).
    2. Validity Check: Is Year >= 2022? (Avoids corrupt/pre-GPS 1980/2000 boot timestamps).
    3. If filename date is missing or < 2022, inspect GPS/FMT metadata from file or fallback to file modification time.
    """
    filename = os.path.basename(filepath)
    potential_date = None

    # Regex 1: YYYY-MM-DD or YYYY_MM_DD
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

    # VALIDITY CHECK (The "2022 Rule")
    if potential_date and potential_date.year >= 2022:
        return potential_date

    # Binary probe for GPS lock timestamp in .bin files
    if filepath.lower().endswith('.bin') and os.path.exists(filepath):
        try:
            gps_date = probe_bin_gps_date(filepath)
            if gps_date and gps_date.year >= 2022:
                return gps_date
        except Exception:
            pass

    # Fallback to System File Modification Time
    try:
        mtime = os.path.getmtime(filepath)
        return datetime.datetime.fromtimestamp(mtime)
    except Exception:
        return datetime.datetime.now()


def probe_bin_gps_date(filepath):
    """
    Quick binary scan of the first 256KB of an ArduPilot DataFlash .bin log
    to locate GPS / GPS_RAW / FMT timestamps if filename has no date.
    """
    try:
        with open(filepath, 'rb') as f:
            chunk = f.read(262144)  # First 256KB
            if len(chunk) < 100:
                return None

            # Look for ASCII date pattern YYYY-MM-DD in text messages inside chunk
            text_matches = re.findall(b"(\\d{4}-\\d{2}-\\d{2})", chunk)
            for m in text_matches:
                try:
                    s = m.decode('ascii', errors='ignore')
                    dt = datetime.datetime.strptime(s, "%Y-%m-%d")
                    if dt.year >= 2022:
                        return dt
                except Exception:
                    pass
    except Exception:
        pass
    return None


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
    valid_exts = ('.bin', '.log', '.tlog')
    total_bytes = 0

    try:
        seen_paths = set()
        for fld in folders:
            for root, dirs, files in os.walk(fld):
                for file in files:
                    if file.lower().endswith(valid_exts):
                        full_path = os.path.join(root, file)
                        if full_path in seen_paths:
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

    valid_exts = ('.bin', '.log', '.tlog')
    all_files = []
    seen_paths = set()
    for fld in folders:
        for root, dirs, files in os.walk(fld):
            for file in files:
                if file.lower().endswith(valid_exts):
                    p = os.path.join(root, file)
                    if p not in seen_paths:
                        seen_paths.add(p)
                        all_files.append(p)

    total = len(all_files)
    if total == 0:
        return {
            'success': True,
            'total': 0,
            'copied': 0,
            'moved': 0,
            'skipped': 0,
            'errors': 0,
            'files': [],
            'dest_folder': dest_folder,
            'message': 'No .bin, .log, or .tlog files found in source folder(s).'
        }

    stats = {
        'total': total,
        'copied': 0,
        'moved': 0,
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
                if is_move:
                    shutil.move(src_path, target_path)
                    stats['moved'] += 1
                    status_action = 'moved'
                else:
                    shutil.copy2(src_path, target_path)
                    stats['copied'] += 1
                    status_action = 'copied'

                stats['files'].append({
                    'filename': filename,
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
            progress_callback(idx + 1, total, filename)

    stats['created_folders'] = sorted(list(stats['created_folders']))
    stats['success'] = True
    return stats


def pick_folder_native(title="Select Folder", initial_dir=""):
    """
    Launches a dedicated subprocess to prompt the native Windows Folder Picker dialog.
    Directly mirrors the proven implementation from the original ArduPilot Log Manager tool,
    with STARTF_USESHOWWINDOW / SW_SHOW to ensure visibility.
    """
    picker_script = os.path.join(os.path.dirname(os.path.abspath(__file__)), "picker.py")
    si = subprocess.STARTUPINFO()
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
