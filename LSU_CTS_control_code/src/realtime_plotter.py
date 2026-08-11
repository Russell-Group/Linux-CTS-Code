"""
Real-Time File-Based Data Plotter - FINAL VERSION
- 4 vertical plots for better time visibility
- Corrected sensor labels
- Absolute time (actual clock time) on x-axis
- Shows file timestamp information
"""

import matplotlib
matplotlib.use('TkAgg')

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.dates import DateFormatter, MinuteLocator, SecondLocator
from matplotlib.widgets import Slider
from collections import deque
import numpy as np
import time
import os
import re
from datetime import datetime, timedelta
from pathlib import Path

# ============================================================================
# CONFIGURATION
# ============================================================================

# Get the directory of this script (CTS/src/)
SCRIPT_DIR = Path(__file__).resolve().parent

# Go up one level to project root (CTS/)
PROJECT_ROOT = SCRIPT_DIR.parent

# Now reference the logs directory
LOG_DIR = PROJECT_ROOT / "logs"
DATA_FILE = LOG_DIR / "CTS_Arduino_serial_log.csv"
MAX_POINTS = 500              # More points since we have more horizontal space
UPDATE_INTERVAL = 2000        # Update every 2 seconds

print("=" * 70)
print("Real-Time Sensor Data Plotter")
print("=" * 70)
print(f"Data file: {DATA_FILE}")
print(f"Current directory: {os.getcwd()}")
print()

# Check file exists
if not os.path.exists(DATA_FILE):
    print(f"ERROR: File '{DATA_FILE}' not found!")
    exit(1)

file_size = os.path.getsize(DATA_FILE)
file_modified = datetime.fromtimestamp(os.path.getmtime(DATA_FILE))
print(f"✓ File found: {file_size} bytes")
print(f"  Last modified: {file_modified.strftime('%Y-%m-%d %H:%M:%S')}")
print()

# ============================================================================
# DATA STORAGE
# ============================================================================
data_points = deque(maxlen=MAX_POINTS)
last_position = 0
data_start_time = None  # Absolute time when first data point was read
data_count = 0
update_count = 0

# Status / warning message storage (separate window)
MAX_STATUS_MESSAGES = 2000    # deep buffer so operator can scroll back through history
# STATUS_WINDOW_LINES is computed from the layout below (sized to fit window)
status_messages = deque(maxlen=MAX_STATUS_MESSAGES)
status_msg_count = 0
# Suppress repeating high-frequency [AUTOFILL] telemetry lines so the
# operator only sees them when values change meaningfully. Everything
# else (state changes, faults, warnings, init, etc.) always passes through.
last_autofill_signature = None
autofill_repeat_count = 0
AUTOFILL_REPEAT_THRESHOLD = 1   # only show first occurrence of a given signature

# ============================================================================
# FILE READING FUNCTIONS
# ============================================================================

# Regex for the absolute-time prefix used by the new CSV format:
#   "2026-06-03 17:03:26,<rest of line>"
# Anchored at start; tolerates extra whitespace.
_TIMESTAMP_PREFIX_RE = re.compile(
    r'^\s*(\d{4}-\d{2}-\d{2}[ T]\d{2}:\d{2}:\d{2}(?:\.\d+)?)\s*,\s*(.*)$'
)


def extract_timestamp_prefix(line):
    """Detect a leading absolute-timestamp prefix and return (ts, remainder).

    The new log format prefixes every line with 'YYYY-MM-DD HH:MM:SS,'.
    If that prefix is present, parse it into a datetime and return the
    rest of the line. If absent (older log format), return (None, line)
    so the caller can fall back to wall-clock timing.
    """
    if not line:
        return None, line
    m = _TIMESTAMP_PREFIX_RE.match(line)
    if not m:
        return None, line
    ts_str, remainder = m.group(1), m.group(2)
    # Normalize 'T' separator if present
    ts_str = ts_str.replace('T', ' ')
    try:
        # Handle optional fractional seconds
        if '.' in ts_str:
            ts = datetime.strptime(ts_str, '%Y-%m-%d %H:%M:%S.%f')
        else:
            ts = datetime.strptime(ts_str, '%Y-%m-%d %H:%M:%S')
    except ValueError:
        return None, line
    return ts, remainder


def parse_data_line(line):
    """Parse a line of sensor data"""
    if not line.strip():
        return None
    
    if 'LevelSensor[0]:' not in line:
        return None
    
    # Skip diagnostic markers
    if any(marker in line for marker in ['[AUTOFILL]', '[STATE]', '[DEBUG]', '[IMMERSE]']):
        return None
    
    try:
        parts = line.strip().split(',')
        data_dict = {}
        
        for part in parts:
            if ':' in part:
                try:
                    key, value = part.split(':', 1)
                    data_dict[key] = float(value)
                except:
                    continue
        
        if len(data_dict) < 5:
            return None
        
        return data_dict
    except:
        return None


def classify_status_line(line):
    """
    Classify a non-sensor-data line into a category and severity.
    Returns (category, severity) or None if line should be ignored.
    severity: 'error' (red), 'warning' (orange), 'state' (blue), 'info' (gray)
    """
    s = line.strip()
    if not s:
        return None

    # Strip PuTTY log header noise
    if s.startswith('=~=~=~') or s.startswith(':[Initalizing'):
        return ('BOOT', 'info')

    s_upper = s.upper()

    # Faults & errors (highest priority) — use word boundaries to avoid
    # false positives like "default" matching "fault".
    if re.search(r'\b(FAULT|ERROR|FAIL|FAILED|FAILURE)\b', s_upper):
        return ('FAULT', 'error')

    # Explicit warnings / saturation / cutoff / latching
    if re.search(r'\b(WARN|WARNING|SATURATED|CUTOFF|LATCHING)\b', s_upper):
        return ('WARNING', 'warning')

    # Horn alarm events are warnings (suppressed-during-init is still
    # operator-relevant info but lower priority than a true alarm)
    if '[HORN]' in s:
        return ('WARNING', 'warning')

    # State transitions (operator-relevant)
    if (s.startswith('State changed') or
            s.startswith('Setting STATE') or
            '[STATE]' in s or '[ROAH]' in s):
        return ('STATE', 'state')

    # Autofill telemetry & events
    if '[AUTOFILL]' in s:
        # Events (non-numeric telemetry) are important
        if 'LS0=' not in s and 'LS1=' not in s:
            return ('AUTOFILL', 'state')
        return ('AUTOFILL', 'info')

    # Init / boot / hardware bringup
    if ('[BOOT]' in s or '[GRENOBLE]' in s or '[AD7746]' in s or
            '[IMMERSE]' in s or '[DEBUG]' in s or
            s.startswith('Setting I/O') or s.startswith('Testing I/O') or
            s.startswith('Initializing') or s.startswith('LCD ') or
            s.startswith('Enabling') or s.startswith('Disabling')):
        return ('INIT', 'info')

    # Anything else that isn't sensor data — show as info
    return ('OTHER', 'info')


def tail_file(filename):
    """Read new lines from file since last read.

    New log format prefixes every line with 'YYYY-MM-DD HH:MM:SS,'. When
    that prefix is present we use the embedded absolute timestamp for the
    plot and the status log. If absent (older log format), we fall back
    to the wall-clock time at the moment the line is read.
    """
    global last_position, data_count, data_start_time
    global status_msg_count, last_autofill_signature, autofill_repeat_count

    if not os.path.exists(filename):
        return 0

    new_data_count = 0
    try:
        with open(filename, 'r', errors='replace') as f:
            f.seek(last_position)
            new_lines = f.readlines()
            last_position = f.tell()

            for line in new_lines:
                # Extract the absolute-time prefix if present.
                line_ts, payload = extract_timestamp_prefix(line)
                # Timestamp we will attribute to this event.
                timestamp = line_ts if line_ts is not None else datetime.now()

                # Try parsing the payload as sensor data first.
                data_dict = parse_data_line(payload)
                if data_dict is not None:
                    if data_start_time is None:
                        data_start_time = timestamp
                    data_points.append((timestamp, data_dict))
                    data_count += 1
                    new_data_count += 1
                    continue

                # Otherwise treat as a status / warning line.
                classification = classify_status_line(payload)
                if classification is None:
                    continue
                category, severity = classification
                clean = payload.strip()

                # Suppress repeated high-frequency [AUTOFILL] telemetry
                if category == 'AUTOFILL' and severity == 'info':
                    sig_parts = []
                    for tok in clean.split():
                        if '=' in tok and any(k in tok for k in
                                              ('Dewar_Level=', 'LiquidCMD=', 'latched=')):
                            sig_parts.append(tok)
                    signature = ' '.join(sig_parts)
                    if signature == last_autofill_signature:
                        autofill_repeat_count += 1
                        continue
                    else:
                        if autofill_repeat_count > 0:
                            status_messages.append((
                                timestamp, 'AUTOFILL', 'info',
                                f'  ... ({autofill_repeat_count} similar AUTOFILL telemetry rows suppressed)'
                            ))
                            status_msg_count += 1
                        last_autofill_signature = signature
                        autofill_repeat_count = 0

                status_messages.append((timestamp, category, severity, clean))
                status_msg_count += 1

    except Exception as e:
        print(f"Error reading file: {e}")

    return new_data_count


def extract_data_arrays():
    """Extract synchronized arrays from data_points"""
    if len(data_points) == 0:
        return None
    
    times = []
    level_sensors = {i: [] for i in range(10)}
    pressure1 = []
    pressure2 = []
    grenoble1 = []
    grenoble2 = []
    
    for timestamp, data in data_points:
        times.append(timestamp)
        
        for i in range(10):
            key = f'LevelSensor[{i}]'
            level_sensors[i].append(data.get(key, np.nan))
        
        pressure1.append(data.get('Pressure1_psi', np.nan))
        pressure2.append(data.get('Pressure2_psi', np.nan))
        grenoble1.append(data.get('Grenoble1_abs', np.nan))
        grenoble2.append(data.get('Grenoble2_abs', np.nan))
    
    return {
        'times': times,  # List of datetime objects
        'level_sensors': {i: np.array(level_sensors[i]) for i in range(10)},
        'pressure1': np.array(pressure1),
        'pressure2': np.array(pressure2),
        'grenoble1': np.array(grenoble1),
        'grenoble2': np.array(grenoble2)
    }

# ============================================================================
# INITIAL DATA LOAD
# ============================================================================

print("Loading existing data from file...")
initial_count = tail_file(DATA_FILE)
print(f"✓ Loaded {initial_count} data points")

if initial_count > 0:
    arrays = extract_data_arrays()
    print()
    print("Data summary:")
    print(f"  Start time: {arrays['times'][0].strftime('%H:%M:%S')}")
    print(f"  End time: {arrays['times'][-1].strftime('%H:%M:%S')}")
    duration = (arrays['times'][-1] - arrays['times'][0]).total_seconds()
    print(f"  Duration: {duration:.1f} seconds ({duration/60:.1f} minutes)")
    print(f"  Dewar sensors (0-4): {'✓' if not all(np.isnan(arrays['level_sensors'][0])) else '✗'}")
    print(f"  Basin sensors (5-9): {'✓' if not all(np.isnan(arrays['level_sensors'][5])) else '✗'}")
    print(f"  Pressure sensors: {'✓' if not all(np.isnan(arrays['pressure1'])) else '✗'}")
    print(f"  Grenoble sensors: {'✓' if not all(np.isnan(arrays['grenoble1'])) else '✗'}")

print()
print("Starting plot...")
print("=" * 70)

# ============================================================================
# PLOTTING SETUP - 4 VERTICAL SUBPLOTS
# ============================================================================

fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, figsize=(14, 12))
fig.suptitle(f'Real-Time Sensor Data - {DATA_FILE}', fontsize=14, fontweight='bold')

# Date formatter for x-axis
date_formatter = DateFormatter('%H:%M:%S')

# ============================================================================
# SUBPLOT 1: Storage Dewar Level Sensors (0-4)
# ============================================================================
lines_dewar = {}
colors_dewar = ['blue', 'green', 'orange', 'red', 'purple']
labels_dewar = ['LS0 (Overfill: ~26")','LS1 (Full: ~21")', 'LS2 (2/3 Full: ~14")', 'LS3 (1/3 Full: ~7")', 'LS4 (Empty: ~1")']

for i in range(5):
    line, = ax1.plot([], [], color=colors_dewar[i], 
                     label=labels_dewar[i], linewidth=2)
    lines_dewar[i] = line

ax1.set_ylabel('Sensor Reading (counts)', fontsize=10)
ax1.set_title('Storage Dewar Level Sensors', fontsize=11, fontweight='bold')
ax1.set_ylim(10000, 26000)
ax1.legend(loc='upper right', fontsize=8, ncol=5)
ax1.grid(True, alpha=0.3)
ax1.xaxis.set_major_formatter(date_formatter)

# ============================================================================
# SUBPLOT 2: Test Basin Level Sensors (5-9)
# ============================================================================
lines_basin = {}
colors_basin = ['cyan', 'magenta', 'brown', 'slateblue', 'gray']
labels_basin = ['L5 (Basin Overfill: 7")', 'LS6 (5")', 'LS7 (3.5")', 'LS8 (2")', 'LS9 (0.5")']

for i in range(5, 10):
    line, = ax2.plot([], [], color=colors_basin[i-5], 
                     label=labels_basin[i-5], linewidth=2)
    lines_basin[i] = line

ax2.set_ylabel('Sensor Reading (counts)', fontsize=10)
ax2.set_title('Test Basin Level Sensors', fontsize=11, fontweight='bold')
ax2.set_ylim(10000, 26000)
ax2.legend(loc='upper right', fontsize=8, ncol=5)
ax2.grid(True, alpha=0.3)
ax2.xaxis.set_major_formatter(date_formatter)

# ============================================================================
# SUBPLOT 3: Pressure Sensors (Both in Dewar)
# ============================================================================
line_p1, = ax3.plot([], [], 'b-', linewidth=2, marker='o', markersize=2,
                    label='Pressure 1', markevery=10)
line_p2, = ax3.plot([], [], 'r-', linewidth=2, marker='s', markersize=2,
                    label='Pressure 2', markevery=10)

## ax3.set_ylabel('Pressure (counts)', fontsize=10)
ax3.set_ylabel('Pressure (PSI)', fontsize=10)
ax3.set_title('Dewar Pressure Sensors', fontsize=11, fontweight='bold')
#ax3.set_ylim(0, 15000)
ax3.set_ylim(-0.5, 3)
ax3.legend(loc='upper right', fontsize=8)
ax3.grid(True, alpha=0.3)
ax3.xaxis.set_major_formatter(date_formatter)

# ============================================================================
# SUBPLOT 4: Grenoble Capacitive Sensors
# ============================================================================
line_g1, = ax4.plot([], [], 'g-', linewidth=2, marker='o', markersize=2,
                    label='Grenoble 1 (CIN1)', markevery=10)
line_g2, = ax4.plot([], [], 'm-', linewidth=2, marker='s', markersize=2,
                    label='Grenoble 2 (CIN2)', markevery=10)

ax4.set_xlabel('Time (HH:MM:SS)', fontsize=10, fontweight='bold')
ax4.set_ylabel('Capacitance (pF)', fontsize=10)
ax4.set_title('Grenoble Capacitive Level Sensors', fontsize=11, fontweight='bold')
# Y-axis is set dynamically in update_plot() based on visible data
ax4.set_ylim(-1, 10)  # initial fallback until first data arrives
ax4.legend(loc='upper right', fontsize=8)
ax4.grid(True, alpha=0.3)
ax4.xaxis.set_major_formatter(date_formatter)

# Grenoble saturation/no-signal sentinel — exclude from auto-scale calculation.
# AD7746 returns 2^25/1e6 = 33.554432 when over-range / no signal.
GRENOBLE_SATURATION = 33.554428
GRENOBLE_SAT_TOL = 0.001

# Status text - placed below the suptitle on the left edge so it
# doesn't overlap the title.
status_text = fig.text(0.02, 0.945, '', ha='left', va='top', fontsize=9,
                       bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))

# ============================================================================
# SECOND WINDOW: Status & Warning Messages
# ============================================================================
# Operator-focused log of state changes, faults, warnings, and events.
# High-frequency sensor data is NOT shown here -- only meaningful messages.
fig_status = plt.figure(figsize=(12, 8))
fig_status.canvas.manager.set_window_title('CTS Status & Warnings')
# Reserve a thin strip on the right for the scrollbar/slider.
ax_status = fig_status.add_axes([0.0, 0.0, 0.955, 1.0])
ax_status.set_axis_off()

# Header
status_header = ax_status.text(
    0.01, 0.985,
    'CTS STATUS & WARNING LOG  —  most recent at bottom',
    transform=ax_status.transAxes,
    ha='left', va='top',
    fontsize=12, fontweight='bold',
    family='monospace',
    color='black',
)

# Summary line (counts of each severity in current visible window)
status_summary = ax_status.text(
    0.01, 0.955,
    '',
    transform=ax_status.transAxes,
    ha='left', va='top',
    fontsize=9,
    family='monospace',
    color='dimgray',
)

# Pre-allocate text artists for each visible row so we can update efficiently
# without re-creating Text objects every frame.
SEVERITY_COLOR = {
    'error':   '#c0392b',   # red
    'warning': '#d68910',   # orange
    'state':   '#1f6fb4',   # blue
    'info':    '#555555',   # gray
}
SEVERITY_WEIGHT = {
    'error':   'bold',
    'warning': 'bold',
    'state':   'bold',
    'info':    'normal',
}

# Layout:
#   - top band 0.93..1.00 = header + summary
#   - bottom band 0.02..~0.30 = "recent 3" enlarged, newest at the very bottom
#   - middle band between them = older messages, oldest near top, flowing down
RECENT_HIGHLIGHT_COUNT = 3
RECENT_FONT_SIZE = 11        # between OLDER (9) and a fully emphasized size
OLDER_FONT_SIZE = 9
RECENT_LINE_HEIGHT = 0.038   # in axes-fraction units (one line of larger text)
OLDER_LINE_HEIGHT = 0.025    # in axes-fraction units (one line of smaller text)

# Bottom (recent) band: build from bottom up so newest sits at the lowest y.
# Index 0 = newest, index 1 = second-newest, index 2 = third-newest.
recent_line_artists = []
for _i in range(RECENT_HIGHLIGHT_COUNT):
    _y = 0.025 + _i * RECENT_LINE_HEIGHT
    _t = ax_status.text(
        0.01, _y, '',
        transform=ax_status.transAxes,
        ha='left', va='bottom',
        fontsize=RECENT_FONT_SIZE,
        family='monospace',
        color=SEVERITY_COLOR['info'],
    )
    recent_line_artists.append(_t)

# Separator line between older messages and the recent-highlight band
_recent_band_top = 0.025 + RECENT_HIGHLIGHT_COUNT * RECENT_LINE_HEIGHT + 0.005
status_separator = ax_status.axhline(
    y=_recent_band_top + 0.005,
    xmin=0.005, xmax=0.995,
    color='lightgray', linewidth=0.8,
)

# Middle band (older messages). Older = top of band, newest-of-the-older just
# above the separator. We compute how many lines fit between summary and band.
_older_band_top = 0.92    # just below summary
_older_band_bottom = _recent_band_top + 0.015
_older_lines_capacity = max(
    1,
    int((_older_band_top - _older_band_bottom) / OLDER_LINE_HEIGHT)
)
# Total lines we keep on screen at once
STATUS_WINDOW_LINES = _older_lines_capacity + RECENT_HIGHLIGHT_COUNT

# Build older-row artists from top down. Index 0 = topmost (oldest visible).
older_line_artists = []
for _i in range(_older_lines_capacity):
    _y = _older_band_top - _i * OLDER_LINE_HEIGHT
    _t = ax_status.text(
        0.01, _y, '',
        transform=ax_status.transAxes,
        ha='left', va='top',
        fontsize=OLDER_FONT_SIZE,
        family='monospace',
        color=SEVERITY_COLOR['info'],
    )
    older_line_artists.append(_t)

# ----------------------------------------------------------------------------
# Scrollbar (vertical Slider) on the right side of the status window
# ----------------------------------------------------------------------------
# The scrollbar applies ONLY to the older-message band. The 3 enlarged
# messages at the bottom are always the 3 newest in the buffer and do not
# move when the user scrolls.
#
# Direction convention (matches the visible layout where newest is at the
# bottom of the older band):
#   - Slider knob at the BOTTOM of its track  -> showing newest older messages
#                                                (i.e. "live" mode, view follows)
#   - Slider knob moved UPWARD                 -> view scrolls back to OLDER
#                                                history
#
# Internally, slider.val = "scroll-back offset" measured in messages:
#   val = 0           => newest possible position (live)
#   val = max_offset  => oldest position the buffer allows
# matplotlib's vertical Slider places valmin at the bottom and valmax at
# the top of the track, which gives us exactly the desired visual behavior.

# Place the slider track to span ONLY the older-message band, sitting just
# above the separator line.
_scroll_y_bottom = _older_band_bottom
_scroll_y_top = _older_band_top
ax_scroll = fig_status.add_axes([
    0.965,
    _scroll_y_bottom,
    0.020,
    _scroll_y_top - _scroll_y_bottom,
])
status_scroll = Slider(
    ax=ax_scroll,
    label='',
    valmin=0,
    valmax=1,
    valinit=0,            # start at bottom = live
    orientation='vertical',
    color='#9aa0a6',
    initcolor='none',
)
status_scroll.valtext.set_visible(False)

# Caption at the top of the slider (just above the track) and below.
# Place text in the slider's own axes coordinates so it stays anchored
# to the track itself rather than to the figure.
fig_status.text(
    0.975, _scroll_y_top + 0.008,
    '↑ older',
    ha='center', va='bottom',
    fontsize=8, color='dimgray', family='monospace',
)
fig_status.text(
    0.975, _scroll_y_bottom - 0.008,
    '↓ newer\n(live)',
    ha='center', va='top',
    fontsize=8, color='dimgray', family='monospace',
)

# Scroll state: when True, the older band auto-follows new messages.
# True whenever the slider is at val == 0 (its bottom).
_scroll_follows_live = True


def _on_scroll_changed(val):
    """User moved the slider. Follow-live only when knob is at the bottom."""
    global _scroll_follows_live
    _scroll_follows_live = (val <= 0.5)


status_scroll.on_changed(_on_scroll_changed)

# Mouse-wheel over the status window also moves the slider.
# Wheel UP = scroll back in time (older). Wheel DOWN = forward (newer).
def _on_scroll_wheel(event):
    if event.canvas.figure is not fig_status:
        return
    if status_scroll.valmax <= 0:
        return
    delta = 1 if event.button == 'up' else -1   # up => older => higher offset
    new_val = status_scroll.val + delta
    new_val = max(status_scroll.valmin, min(status_scroll.valmax, new_val))
    status_scroll.set_val(new_val)


fig_status.canvas.mpl_connect('scroll_event', _on_scroll_wheel)

# ============================================================================
# ANIMATION UPDATE FUNCTION
# ============================================================================

def refresh_status_window():
    """Redraw the status/warning messages on the second figure.

    Layout:
      - Older messages in the upper band, oldest at top, newest at bottom.
        The scrollbar moves a window through the older-message history.
      - The 3 most recent messages in the buffer, in larger font, are
        always pinned at the bottom band -- they do NOT move when the
        user scrolls.

    Scrollbar semantics:
      - Knob at BOTTOM of track => 'live' mode: older band shows the
        messages immediately preceding the recent-3, and follows new
        traffic automatically.
      - Knob moved UPWARD => view scrolls back to OLDER history. The
        recent-3 band still updates with new messages in real time.
    """
    # Severity counts across the entire buffer
    counts = {'error': 0, 'warning': 0, 'state': 0, 'info': 0}
    for _, _, sev, _ in status_messages:
        if sev in counts:
            counts[sev] += 1

    all_msgs = list(status_messages)
    n_total = len(all_msgs)

    # ---- Recent highlight band (bottom): always the 3 newest ----
    # recent_line_artists[0] = newest (lowest on screen)
    # recent_line_artists[1] = 2nd newest
    # recent_line_artists[2] = 3rd newest
    for i, artist in enumerate(recent_line_artists):
        idx = n_total - 1 - i
        if 0 <= idx < n_total:
            ts, category, severity, text = all_msgs[idx]
            ts_str = ts.strftime('%H:%M:%S')
            display = text if len(text) <= 110 else text[:107] + '...'
            artist.set_text(f'{ts_str}  [{category:<8}] {display}')
            artist.set_color(SEVERITY_COLOR.get(severity, SEVERITY_COLOR['info']))
            if i == 0:
                artist.set_fontweight('bold')
            else:
                artist.set_fontweight(SEVERITY_WEIGHT.get(severity, 'normal'))
        else:
            artist.set_text('')

    # ---- Older messages pool (everything except the 3 newest) ----
    older_pool = all_msgs[: max(0, n_total - len(recent_line_artists))]
    n_older_pool = len(older_pool)
    capacity = len(older_line_artists)

    # Maximum scroll-back offset: how many messages the user can scroll
    # past the most-recent older message. 0 if pool fits within capacity.
    max_offset = max(0, n_older_pool - capacity)

    # ---- Update slider range to match the current buffer ----
    if status_scroll.valmax != max_offset:
        # Update the slider's underlying axes ylim so the knob track
        # spans the new range, and update valmax.
        status_scroll.valmax = max_offset
        status_scroll.ax.set_ylim(0, max_offset if max_offset > 0 else 1)

        # Clamp current value into the new range without re-firing callback
        new_val = max(0, min(status_scroll.val, max_offset))
        if _scroll_follows_live:
            new_val = 0
        status_scroll.eventson = False
        status_scroll.set_val(new_val)
        status_scroll.eventson = True

    # If we're in live mode, make sure the slider sits exactly at 0
    # (snap any drift introduced by the previous clamp).
    if _scroll_follows_live and status_scroll.val != 0:
        status_scroll.eventson = False
        status_scroll.set_val(0)
        status_scroll.eventson = True

    offset = int(round(status_scroll.val))
    offset = max(0, min(max_offset, offset))

    # ---- Older band: window into older_pool ----
    # The newest message in the older band is older_pool[end-1] where
    #   end = n_older_pool - offset
    # The oldest message in the band is at start = end - capacity.
    end = n_older_pool - offset
    start = end - capacity
    actual_start = max(0, start)
    actual_end = max(actual_start, end)
    older_msgs = older_pool[actual_start:actual_end]
    n_older = len(older_msgs)
    blank_top = capacity - n_older  # blank rows at top when pool is short

    for i, artist in enumerate(older_line_artists):
        msg_idx = i - blank_top
        if 0 <= msg_idx < n_older:
            ts, category, severity, text = older_msgs[msg_idx]
            ts_str = ts.strftime('%H:%M:%S')
            display = text if len(text) <= 130 else text[:127] + '...'
            artist.set_text(f'{ts_str}  [{category:<8}] {display}')
            artist.set_color(SEVERITY_COLOR.get(severity, SEVERITY_COLOR['info']))
            artist.set_fontweight(SEVERITY_WEIGHT.get(severity, 'normal'))
        else:
            artist.set_text('')

    # Summary line — show mode and how far back we're scrolled
    if _scroll_follows_live:
        mode_str = 'LIVE'
    else:
        mode_str = f'PAUSED ({offset} msgs back — drag knob to bottom for live)'
    status_summary.set_text(
        f'Buffered: {len(status_messages)} / {MAX_STATUS_MESSAGES} msgs   |   '
        f"FAULT/ERROR: {counts['error']}   "
        f"WARNING: {counts['warning']}   "
        f"STATE: {counts['state']}   "
        f"INFO: {counts['info']}   |   "
        f'View: {mode_str}'
    )


def update_plot(frame):
    """Called periodically to update the plot"""
    global update_count
    
    update_count += 1
    
    # Read new data from file
    new_points = tail_file(DATA_FILE)
    
    # Print status
    if update_count % 10 == 0 or new_points > 0:
        now = datetime.now().strftime('%H:%M:%S')
        print(f"[{now}] Update #{update_count}: +{new_points} new points, total={data_count}")
    
    # Extract data arrays
    arrays = extract_data_arrays()
    have_data = arrays is not None and len(arrays['times']) > 0

    if not have_data:
        # ----------------------------------------------------------------
        # No sensor data yet -- fall back to status-message timestamps so
        # the plot axes show meaningful times reflecting when boot/init
        # events occurred. This is a TEMPORARY reference: as soon as real
        # sensor data starts streaming in, the axes switch to those
        # timestamps automatically (the `have_data` branch below).
        # ----------------------------------------------------------------
        if len(status_messages) > 0:
            msg_times = [ts for ts, *_ in status_messages]
            x_min_fb = msg_times[0]
            x_max_fb = msg_times[-1]
            # If all messages share a single second, expand the window so
            # the axis is readable rather than collapsing to a point.
            if (x_max_fb - x_min_fb).total_seconds() < 5:
                x_max_fb = x_min_fb + timedelta(seconds=30)
            # Add a small right-edge pad so the very last event isn't at
            # the axis boundary.
            for ax in (ax1, ax2, ax3, ax4):
                ax.set_xlim(x_min_fb, x_max_fb + timedelta(seconds=30))

            duration_fb = (msg_times[-1] - msg_times[0]).total_seconds()
            status_text.set_text(
                f'Updates: {update_count} | Data points: 0 (no sensor data yet) | '
                f'Boot window: {duration_fb/60:.1f} min | '
                f'Latest event: {msg_times[-1].strftime("%Y-%m-%d %H:%M:%S")}'
            )

    if have_data:
        times = arrays['times']
        
        # Determine x-axis limits (show last 10 minutes or all data if less)
        time_window = timedelta(minutes=10)
        x_max = times[-1]
        x_min = max(times[0], x_max - time_window)
        
        # Update status text with current time info
        duration = (times[-1] - times[0]).total_seconds()
        # "Latest data" is the timestamp of the most recent point on the
        # plot (from the log file's absolute timestamps when available,
        # otherwise wall-clock). Includes the date so it's unambiguous
        # across midnight boundaries.
        latest_data_str = times[-1].strftime('%Y-%m-%d %H:%M:%S')
        status_text.set_text(
            f'Updates: {update_count} | Data points: {data_count} | '
            f'Duration: {duration/60:.1f} min | '
            f'Latest data: {latest_data_str}'
        )
        
        # ================================================================
        # Update Dewar sensors (0-4) with DYNAMIC Y-AXIS
        # ================================================================
        dewar_visible_data = []
        for i in range(5):
            data = arrays['level_sensors'][i]
            valid_mask = ~np.isnan(data)
            if np.any(valid_mask):
                valid_times = [times[j] for j in range(len(times)) if valid_mask[j]]
                lines_dewar[i].set_data(valid_times, data[valid_mask])
                # Collect visible data within current time window for dynamic y-axis
                visible_indices = [j for j in range(len(times)) if valid_mask[j] and times[j] >= x_min and times[j] <= x_max]
                if visible_indices:
                    visible_values = data[visible_indices]
                    # Filter: ignore data below 100 and above 22000 for scale calculation
                    filtered_values = visible_values[(visible_values >= 100) & (visible_values <= 22000) & (~np.isnan(visible_values))]
                    dewar_visible_data.extend(filtered_values)
            else:
                lines_dewar[i].set_data([], [])
        
        ax1.set_xlim(x_min, x_max + timedelta(seconds=30))
        
        # Dynamic y-axis for Dewar sensors - zoom to visible data range
        if len(dewar_visible_data) > 0:
            y_min = np.nanmin(dewar_visible_data)
            y_max = np.nanmax(dewar_visible_data)
            y_range = y_max - y_min
            y_margin = max(y_range * 0.15, 500)  # 15% margin or minimum 500 counts
            ax1.set_ylim(y_min - y_margin, y_max + y_margin)
        else:
            ax1.set_ylim(10000, 26000)  # Fallback to default if no data
        
        # ================================================================
        # Update Basin sensors (5-9) with DYNAMIC Y-AXIS
        # ================================================================
        basin_visible_data = []
        for i in range(5, 10):
            data = arrays['level_sensors'][i]
            valid_mask = ~np.isnan(data)
            if np.any(valid_mask):
                valid_times = [times[j] for j in range(len(times)) if valid_mask[j]]
                lines_basin[i].set_data(valid_times, data[valid_mask])
                # Collect visible data within current time window for dynamic y-axis
                visible_indices = [j for j in range(len(times)) if valid_mask[j] and times[j] >= x_min and times[j] <= x_max]
                if visible_indices:
                    visible_values = data[visible_indices]
                    # Filter: ignore data below 100 and above 22000 for scale calculation
                    filtered_values = visible_values[(visible_values >= 100) & (visible_values <= 22000) & (~np.isnan(visible_values))]
                    basin_visible_data.extend(filtered_values)
            else:
                lines_basin[i].set_data([], [])
        
        ax2.set_xlim(x_min, x_max + timedelta(seconds=30))
        
        # Dynamic y-axis for Basin sensors - zoom to visible data range
        if len(basin_visible_data) > 0:
            y_min = np.nanmin(basin_visible_data)
            y_max = np.nanmax(basin_visible_data)
            y_range = y_max - y_min
            y_margin = max(y_range * 0.15, 500)  # 15% margin or minimum 500 counts
            ax2.set_ylim(y_min - y_margin, y_max + y_margin)
        else:
            ax2.set_ylim(10000, 26000)  # Fallback to default if no data
        
        # ================================================================
        # Update Pressure sensors
        # ================================================================
        p1_valid = ~np.isnan(arrays['pressure1'])
        p2_valid = ~np.isnan(arrays['pressure2'])
        
        if np.any(p1_valid):
            p1_times = [times[j] for j in range(len(times)) if p1_valid[j]]
            line_p1.set_data(p1_times, arrays['pressure1'][p1_valid])
        else:
            line_p1.set_data([], [])
        
        if np.any(p2_valid):
            p2_times = [times[j] for j in range(len(times)) if p2_valid[j]]
            line_p2.set_data(p2_times, arrays['pressure2'][p2_valid])
        else:
            line_p2.set_data([], [])
        
        ax3.set_xlim(x_min, x_max + timedelta(seconds=30))
        
        # ================================================================
        # Update Grenoble sensors with DYNAMIC Y-AXIS
        # ================================================================
        g1_valid = ~np.isnan(arrays['grenoble1'])
        g2_valid = ~np.isnan(arrays['grenoble2'])

        if np.any(g1_valid):
            g1_times = [times[j] for j in range(len(times)) if g1_valid[j]]
            line_g1.set_data(g1_times, arrays['grenoble1'][g1_valid])
        else:
            line_g1.set_data([], [])

        if np.any(g2_valid):
            g2_times = [times[j] for j in range(len(times)) if g2_valid[j]]
            line_g2.set_data(g2_times, arrays['grenoble2'][g2_valid])
        else:
            line_g2.set_data([], [])

        ax4.set_xlim(x_min, x_max + timedelta(seconds=30))

        # Collect Grenoble data within current time window for dynamic y-axis,
        # excluding the AD7746 saturation/no-signal sentinel (~33.554428)
        # and obvious out-of-band values.
        gren_visible = []
        for src_valid, src_data in (
            (g1_valid, arrays['grenoble1']),
            (g2_valid, arrays['grenoble2']),
        ):
            for j in range(len(times)):
                if not src_valid[j]:
                    continue
                if times[j] < x_min or times[j] > x_max:
                    continue
                v = src_data[j]
                if np.isnan(v):
                    continue
                if abs(v - GRENOBLE_SATURATION) < GRENOBLE_SAT_TOL:
                    continue  # skip saturated readings
                gren_visible.append(v)

        if len(gren_visible) > 0:
            gy_min = float(np.nanmin(gren_visible))
            gy_max = float(np.nanmax(gren_visible))
            gy_range = gy_max - gy_min
            # Use a small absolute floor so a flat trace still has visible margin
            gy_margin = max(gy_range * 0.15, 0.05)
            ax4.set_ylim(gy_min - gy_margin, gy_max + gy_margin)
        else:
            ax4.set_ylim(-1, 10)  # fallback when no valid data yet

    # Always refresh the status window, even if no sensor data yet
    refresh_status_window()
    fig_status.canvas.draw_idle()

# ============================================================================
# START ANIMATION
# ============================================================================

try:
    ani = animation.FuncAnimation(
        fig, 
        update_plot, 
        interval=UPDATE_INTERVAL,
        blit=False,
        cache_frame_data=False
    )
    
    # Apply tight_layout ONLY to the main 4-subplot figure. The status
    # figure (fig_status) uses hand-tuned manual axes positioning that
    # isn't compatible with tight_layout. Reserve the top ~7% for the
    # suptitle + status text box so they aren't squeezed.
    fig.tight_layout(rect=[0, 0, 1, 0.93])
    plt.show(block=True)
    
except KeyboardInterrupt:
    print("\nStopped by user (Ctrl+C)")
except Exception as e:
    print(f"\nERROR: {e}")
    import traceback
    traceback.print_exc()

print()
print("=" * 70)
print(f"Final statistics:")
print(f"  Total updates: {update_count}")
print(f"  Total data points: {data_count}")
if data_start_time:
    print(f"  Data collection time: {data_start_time.strftime('%Y-%m-%d %H:%M:%S')} to {datetime.now().strftime('%H:%M:%S')}")
print("=" * 70)
