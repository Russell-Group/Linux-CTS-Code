import datetime
import os
import serial
import signal
import sys
import time
from pathlib import Path

# --- Configuration ---
SERIAL_PORT = "/dev/ttyACM0"  # Adjust to your port
BAUD_RATE = 115200             # Adjust to match Arduino Serial.begin()

# Get the directory of this script (CTS/src/)
SCRIPT_DIR = Path(__file__).resolve().parent

# Go up one level to project root (CTS/)
PROJECT_ROOT = SCRIPT_DIR.parent

# Now reference other directories
LOG_DIR = PROJECT_ROOT / "logs"
PIPE_NAME = "arduino_input.pipe"
PIPE_PATH = LOG_DIR / PIPE_NAME
# ---------------------

# Global tracking variables so signal handlers can see them
current_dated_path = ""
dated_file = None
static_file = None
pipe_fd = None
ser = None

def generate_start_filename():
    """Generates a temporary active filename using the format 'hhmm_ACTIVE.csv'."""
    now = datetime.datetime.now()
    return os.path.join(LOG_DIR, f"CTS_Arduino_serial_log_{now.strftime('%Y-%m-%d-%H%M')}_ACTIVE.csv")

def generate_midnight_filename():
    """Generates a temporary active filename for a midnight rollover starting at 0000."""
    now = datetime.datetime.now()
    return os.path.join(LOG_DIR, f"CTS_Arduino_serial_log_{now.strftime('%Y-%m-%d')}-0000_ACTIVE.csv")

def finalize_filename(active_path):
    """Renames the active file to replace '_ACTIVE.csv' with the clear '-hhmm.csv' end time."""
    if not active_path or not os.path.exists(active_path):
        return
    
    # Drops the trailing placeholder text
    base_path = active_path.replace("_ACTIVE.csv", "")
    end_time_str = datetime.datetime.now().strftime("%H%M")
    final_path = f"{base_path}-{end_time_str}.csv"
    
    try:
        os.rename(active_path, final_path)
        print(f"[LOG] File finalized and saved as: {final_path}")
    except Exception as e:
        print(f"[ERROR] Rename failed: {e}")

def handle_shutdown_signal(signum, frame):
    """Triggers clean shutdown when Bash requests it via kill signal."""
    print("\n[LOG] Shutdown signal received. Cleaning up...")
    cleanup_and_exit()

def cleanup_and_exit():
    """Safely closes file descriptors, renames logs, and terminates instantly."""
    global dated_file, static_file, pipe_fd, ser, current_dated_path
    try:
        if dated_file and not dated_file.closed: dated_file.close()
        if static_file and not static_file.closed: static_file.close()
        if pipe_fd is not None: os.close(pipe_fd)
        if ser and ser.is_open: ser.close()
    except Exception:
        pass
    
    finalize_filename(current_dated_path)
    
    pipe_path = os.path.join(LOG_DIR, PIPE_NAME)
    if os.path.exists(pipe_path):
        os.remove(pipe_path)
        
    # Force Linux to reclaim memory immediately and avoid background hanging
    sys.stdout.flush()
    os._exit(0)

def main():
    global current_dated_path, dated_file, static_file, pipe_fd, ser
    
    if not os.path.exists(LOG_DIR):
        os.makedirs(LOG_DIR)

    pipe_path = os.path.join(LOG_DIR, PIPE_NAME)
    if os.path.exists(pipe_path):
        os.remove(pipe_path)
    os.mkfifo(pipe_path)

    # Register termination signals to ensure file renaming happens on exits
    signal.signal(signal.SIGINT, handle_shutdown_signal)
    signal.signal(signal.SIGTERM, handle_shutdown_signal)

    print(f"Connecting to {SERIAL_PORT}...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return

    # Flush hardware buffers
    ser.flushInput()
    ser.flushOutput()

    current_tracking_date = datetime.datetime.now().strftime("%Y-%m-%d")
    current_dated_path = generate_start_filename()
    static_path = os.path.join(LOG_DIR, "CTS_Arduino_serial_log.csv")

    dated_file = open(current_dated_path, "a", encoding="utf-8")
    static_file = open(static_path, "w", encoding="utf-8")

    # Open named pipe using low-level non-blocking read descriptors
    pipe_fd = os.open(pipe_path, os.O_RDONLY | os.O_NONBLOCK)

    print(f"Logging started.")
    print("--------------------------------------------------")

    command_buffer = ""

    try:
        while True:
            # 1. READ INCOMING SERIAL FROM ARDUINO
            if ser.in_waiting > 0:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if line:
                    now_date = datetime.datetime.now().strftime("%Y-%m-%d")
                    if now_date != current_tracking_date:
                        dated_file.close()
                        static_file.close()
                        finalize_filename(current_dated_path)
                        
                        current_tracking_date = now_date
                        current_dated_path = generate_midnight_filename()
                        dated_file = open(current_dated_path, "a", encoding="utf-8")
                        static_file = open(static_path, "w", encoding="utf-8")
                        print(f"Midnight reached! Logs rotated.")

                    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    log_entry = f"{timestamp},{line}\n"
                    dated_file.write(log_entry)
                    static_file.write(log_entry)
                    dated_file.flush()
                    static_file.flush()

            # 2. PERSISTENT LOW-LEVEL PIPE READING FOR COMMANDS
            try:
                # Read up to 1024 raw bytes from the descriptor
                read_data = os.read(pipe_fd, 1024).decode("utf-8", errors="ignore")
                if read_data:
                    command_buffer += read_data
                    # If whole commands arrived, parse out the line breaks
                    if "\n" in command_buffer:
                        lines = command_buffer.split("\n")
                        command_buffer = lines[-1]  # Keep trailing fragment
                        
                        for cmd in lines[:-1]:
                            cmd = cmd.strip()
                            if cmd:
                                cmd_bytes = (cmd + "\n").encode("utf-8")
                                bytes_written = ser.write(cmd_bytes)
                                ser.flush()  # Force immediate hardware transmission
            except OSError:
                # Expected exception block when pipe is empty; step through safely
                pass

            time.sleep(0.01)  # Limit CPU cycling
            
    except SystemExit:
        pass
    except Exception as e:
        print(f"Unexpected loop crash: {e}")
        cleanup_and_exit()

if __name__ == "__main__":
    main()
