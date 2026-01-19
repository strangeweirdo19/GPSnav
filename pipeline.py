#!/usr/bin/env python3
"""
Full Stack OSM Pipeline Manager - OPTIMIZED BULLETPROOF VERSION

FEATURES:
- Always uses MAX batch size instead of learning optimal
- Small files (<95 bytes) removed immediately during extraction
- Updated Valhalla commands (no timezone)
- Correct tilemaker command with bbox
- No cooldown period - continuous monitoring
- Memory-aware prediction with max batch
- OOM detection and recovery
- Auto-scrolling logs (last 10 lines)
- Parallel MBTiles conversion
- Resource monitoring
- Checkpoint system
- Crash recovery
- Garbage collection
- VERBOSE terminal output (shows ALL commands and responses)
"""

import os
import sys
import time
import shutil
import json
import hashlib
import subprocess
import threading
import requests
import multiprocessing
from multiprocessing import Pool, Manager
import uuid
import logging
import sqlite3
import socket
import argparse
import signal
import psutil
import gc
import tarfile
from functools import partial
from collections import deque
import traceback
from datetime import datetime, timedelta
from requests.adapters import HTTPAdapter
from urllib3.util.retry import Retry

# Import osmium for POI extraction
try:
    import osmium
except ImportError:
    osmium = None

# --- Import Rich for UI ---
try:
    from rich.console import Console, Group
    from rich.progress import Progress, SpinnerColumn, BarColumn, TextColumn, TimeRemainingColumn, TransferSpeedColumn, DownloadColumn, TaskID
    from rich.panel import Panel
    from rich.table import Table
    from rich.logging import RichHandler
    from rich.layout import Layout
    from rich.live import Live
    from rich.text import Text
except ImportError as e:
    print(f"Rich Import Error: {e}")
    print("Error: 'rich' library missing. Please run: pip install rich psutil --upgrade")
    sys.exit(1)

# --- Configuration ---
BASE_DIR = os.getcwd()
DOWNLOAD_DIR = os.path.join(BASE_DIR, "downloads")
WORK_DIR = os.path.join(BASE_DIR, "work_temp")
PUBLIC_DIR = os.path.join(BASE_DIR, "public_export")
CONFIG_DIR = os.path.join(BASE_DIR, "config")
VALHALLA_DIR = os.path.join(BASE_DIR, "valhalla")
PARTIAL_DIR = "/app/partial"
STATE_FILE = os.path.join(WORK_DIR, "pipeline_state.json")
LOG_FILE = os.path.join(BASE_DIR, "pipeline.log")
STATS_FILE = os.path.join(BASE_DIR, "pipeline_stats.json")
CHECKPOINT_FILE = os.path.join(WORK_DIR, "checkpoint.json")
OOM_HISTORY_FILE = os.path.join(WORK_DIR, "oom_history.json")
BATCH_HISTORY_FILE = os.path.join(WORK_DIR, "batch_history.json")
LAST_RUN_FILE = os.path.join(WORK_DIR, "last_successful_run.txt")

INDIA_PBF_URL = "https://download.geofabrik.de/asia/india-latest.osm.pbf"
INDIA_MD5_URL = "https://download.geofabrik.de/asia/india-latest.osm.pbf.md5"
INDIA_PBF_FILE = os.path.join(DOWNLOAD_DIR, "india-latest.osm.pbf")
LOCAL_MD5_FILE = os.path.join(DOWNLOAD_DIR, "india-latest.osm.pbf.md5")

CHECK_INTERVAL_HOURS = 0  # NO COOLDOWN - continuous checking
MIN_FILE_SIZE_BYTES = 95  # Updated to 95 bytes as requested
CPU_COUNT = multiprocessing.cpu_count()

# ALWAYS USE MAX BATCH SIZE
INITIAL_BATCH_SIZE = 100  # Start at max
MAX_BATCH_SIZE = 100  # Max batch size
MIN_BATCH_SIZE = 1  # Minimum

INDIA_BBOX = "68.0,6.5,97.5,35.5"

# Parallelism settings
MAX_WORKERS = max(1, CPU_COUNT - 1)
TILEMAKER_WORKERS = max(1, CPU_COUNT // 2)

# NO TIMEOUTS - Remove all timeouts for faster execution
SUBPROCESS_TIMEOUT = None
VALHALLA_TIMEOUT = None
EXTRACT_TIMEOUT = None

# Minimum free disk space required
MIN_FREE_SPACE_GB = 20
MIN_FREE_SPACE_BYTES = MIN_FREE_SPACE_GB * 1024 * 1024 * 1024

# NO RETRIES - Fail fast and move on
MAX_RETRIES = 0
BASE_RETRY_DELAY = 0

# Remote MD5 fetch settings
REMOTE_MD5_RETRIES = 3
REMOTE_MD5_TIMEOUT = 12
REMOTE_MD5_BACKOFF = 5

# Memory monitoring during execution - VERY AGGRESSIVE
MEMORY_MONITOR_INTERVAL = 1.5
MEMORY_EXECUTION_THRESHOLD = 0.85
MEMORY_SAFETY_THRESHOLD = 0.85
MEMORY_CRITICAL_THRESHOLD = 0.85
MIN_FREE_MEMORY_GB = 0.5
ENABLE_MEMORY_AWARE_EXTRACTION = True

# Swap monitoring - preflight and runtime
SWAP_MONITOR_INTERVAL = 2
SWAP_THRESHOLD = 0.10          # kill batch if swap exceeds 10% while running
SWAP_HEADROOM = 0.05           # avoid starting batches if swap already >5%
MEMORY_HEADROOM = 0.80         # avoid starting batches if RAM already >80%

# OOM handling - NO COOLDOWN
OOM_DETECTION_ENABLED = True
OOM_BATCH_SIZE_DIVISOR = 2
OOM_COOLDOWN_SECONDS = 0

# More logs visible
LOG_SCROLL_SIZE = 10
TERMINAL_SCROLL_SIZE = 10

# Persistence Configuration
SCREEN_SESSION_NAME = "osm_pipeline_master"
ENV_WORKER_FLAG = "OSM_PIPELINE_WORKER_MODE"

# --- Global State ---
console = Console()
log_deque = deque(maxlen=LOG_SCROLL_SIZE)
terminal_deque = deque(maxlen=TERMINAL_SCROLL_SIZE)
layout_lock = threading.Lock()
shutdown_event = threading.Event()
memory_exceeded_event = threading.Event()
last_oom_time = 0
oom_count = 0
oom_history = []

# Terminal rate-limiting to reduce Live redraw flicker
TERMINAL_MIN_INTERVAL = 0.08  # seconds between non-critical terminal entries
last_terminal_time = 0.0

# Statistics tracking
stats = {
    "pipeline_runs": 0,
    "successful_runs": 0,
    "failed_runs": 0,
    "total_tiles_processed": 0,
    "failed_tiles": [],
    "last_run_duration": 0,
    "average_run_duration": 0,
    "last_error": None,
    "oom_kills": 0,
    "oom_recoveries": 0,
    "memory_profile": {
        "batch_size_to_memory": {},
        "successful_batches": [],
        "failed_batches": [],
        "oom_batches": []
    }
}

# Batch history for tracking
batch_history = {
    "last_successful_size": None,
    "successful_sizes": [],
    "optimal_size": None
}

# File Logging
file_handler = logging.FileHandler(LOG_FILE, mode='a')
file_handler.setLevel(logging.INFO)
file_handler.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))
logger = logging.getLogger("osm_pipeline")
logger.setLevel(logging.INFO)
logger.addHandler(file_handler)

# Global Progress Object
job_progress = Progress(
    SpinnerColumn(),
    TextColumn("[progress.description]{task.description}"),
    BarColumn(),
    TextColumn("{task.percentage:>3.0f}%"),
    DownloadColumn(),
    TransferSpeedColumn(),
    TimeRemainingColumn(),
)

def signal_handler(signum, frame):
    """Handle shutdown signals gracefully."""
    log(f"Received signal {signum}. Shutting down...", "WARN")
    save_checkpoint()
    save_statistics()
    save_batch_history()
    shutdown_event.set()
    sys.exit(0)

# Register signal handlers
signal.signal(signal.SIGTERM, signal_handler)
signal.signal(signal.SIGINT, signal_handler)

def log(msg, level="INFO"):
    """Styled logging to Side Panel + File (auto-scrolls to last 10)."""
    timestamp = datetime.now().strftime("%H:%M:%S")
    
    if level == "INFO": color = "blue"
    elif level == "SUCCESS": color = "green bold"
    elif level == "WARN": color = "yellow"
    elif level == "ERROR": color = "red bold"
    elif level == "HEADER": color = "magenta bold"
    elif level == "STAT": color = "cyan"
    elif level == "OOM": color = "red bold blink"
    elif level == "LEARN": color = "green"
    else: color = "white"
    
    log_deque.append(f"[{timestamp}] [{color}]{msg}[/{color}]")
    
    if level == "ERROR" or level == "OOM": logger.error(msg)
    elif level == "WARN": logger.warning(msg)
    else: logger.info(msg)

def log_terminal(line, error=False, oom=False, command=False):
    """Log terminal output (auto-scrolls to last 10). Shows ALL output."""
    timestamp = datetime.now().strftime("%H:%M:%S")
    if oom: color = "red bold blink"
    elif error: color = "red"
    elif command: color = "yellow bold"
    else: color = "cyan"
    # Rate-limit non-critical terminal entries to reduce UI flicker.
    global last_terminal_time
    now = time.time()
    # Always allow errors, oom, and explicit commands through immediately.
    if not (error or oom or command):
        if now - last_terminal_time < TERMINAL_MIN_INTERVAL:
            return
        last_terminal_time = now

    terminal_deque.append(f"[dim]{timestamp}[/dim] [{color}]{line}[/{color}]")

def get_system_stats():
    """Get current system resource usage."""
    try:
        cpu_percent = psutil.cpu_percent(interval=0.1)
        memory = psutil.virtual_memory()
        swap = psutil.swap_memory()
        disk = psutil.disk_usage(BASE_DIR)
        
        return {
            "cpu": cpu_percent,
            "memory_used_gb": memory.used / (1024**3),
            "memory_available_gb": memory.available / (1024**3),
            "memory_percent": memory.percent,
            "swap_used_gb": swap.used / (1024**3),
            "swap_total_gb": swap.total / (1024**3),
            "swap_percent": swap.percent,
            "disk_free_gb": disk.free / (1024**3),
            "disk_percent": disk.percent
        }
    except Exception:
        return {
            "cpu": 0, "memory_used_gb": 0, "memory_available_gb": 0,
            "memory_percent": 0, "swap_used_gb": 0, "swap_total_gb": 0,
            "swap_percent": 0, "disk_free_gb": 0, "disk_percent": 0
        }

# --- Batch History Management ---

def load_batch_history():
    """Load batch history from previous runs."""
    global batch_history
    if os.path.exists(BATCH_HISTORY_FILE):
        try:
            with open(BATCH_HISTORY_FILE, 'r') as f:
                loaded = json.load(f)
                batch_history.update(loaded)
                log(f"Loaded batch history: Last successful size = {batch_history.get('last_successful_size', 'None')}", "LEARN")
        except Exception:
            pass

def save_batch_history():
    """Save batch history for next run."""
    try:
        os.makedirs(WORK_DIR, exist_ok=True)
        with open(BATCH_HISTORY_FILE, 'w') as f:
            json.dump(batch_history, f, indent=2)
    except Exception as e:
        log(f"Failed to save batch history: {e}", "DEBUG")

def record_successful_batch(batch_size):
    """Record a successful batch size."""
    batch_history["last_successful_size"] = batch_size
    
    if "successful_sizes" not in batch_history:
        batch_history["successful_sizes"] = []
    batch_history["successful_sizes"].append({
        "size": batch_size,
        "timestamp": datetime.now().isoformat()
    })
    
    batch_history["successful_sizes"] = batch_history["successful_sizes"][-100:]
    
    save_batch_history()
    log(f"✓ Recorded: Batch size {batch_size} successful", "LEARN")

def get_starting_batch_size():
    """
    ALWAYS USE MAX BATCH SIZE - No learning, just max.
    Only reduce if previous run had OOM crash.
    """
    # If previous run OOM'd, start at half
    if os.path.exists(CHECKPOINT_FILE):
        try:
            with open(CHECKPOINT_FILE, 'r') as f:
                checkpoint = json.load(f)
                oom_batch = checkpoint.get("last_oom_batch_size")
                if oom_batch and oom_batch > 0:
                    safe_size = max(1, oom_batch // 2)
                    log(f"🔥 Previous OOM at {oom_batch} tiles → Starting at {safe_size} (halved)", "WARN")
                    return safe_size
        except Exception as e:
            log(f"Failed to load checkpoint: {e}", "DEBUG")
    
    # Otherwise, ALWAYS USE MAX
    return MAX_BATCH_SIZE

# --- Memory Monitoring Thread ---

def monitor_memory_during_extraction(process, batch_size):
    """Monitor memory during extraction. Kill early on RAM >85% or swap >10%."""
    while process.poll() is None and not shutdown_event.is_set():
        mem_stats = get_system_stats()
        
        # Check swap first (70% threshold)
        if mem_stats["swap_percent"] > SWAP_THRESHOLD * 100:
            log(f"🛑 SWAP 70%+ EXCEEDED ({mem_stats['swap_percent']:.1f}%)! Halting batch {batch_size} immediately", "WARN")
            log_terminal(f"⚠️ Swap {mem_stats['swap_percent']:.1f}% - STOP NOW", error=True)
            
            try:
                process.kill()
            except:
                pass
            
            memory_exceeded_event.set()
            return False
        
        # Check RAM (95% threshold)
        if mem_stats["memory_percent"] > MEMORY_EXECUTION_THRESHOLD * 100:
            log(f"🛑 MEMORY 95%+ EXCEEDED! Halting batch {batch_size} immediately", "WARN")
            log_terminal(f"⚠️ Memory {mem_stats['memory_percent']:.1f}% - STOP NOW", error=True)
            
            try:
                process.kill()
            except:
                pass
            
            memory_exceeded_event.set()
            return False
        
        time.sleep(MEMORY_MONITOR_INTERVAL)
    
    return True

def check_oom_killed(pid):
    """Check if a process was killed by OOM killer."""
    try:
        result = subprocess.run(
            ["dmesg", "-T"],
            capture_output=True,
            text=True,
            timeout=5
        )
        
        if result.stdout is None:
            return False
            
        recent_lines = result.stdout.split('\n')[-100:]
        
        for line in recent_lines:
            if "Out of memory" in line or "oom-kill" in line.lower() or "Killed process" in line:
                if str(pid) in line or "osmium" in line.lower():
                    return True
    except Exception as e:
        log(f"check_oom_killed failed: {e}", "DEBUG")
    
    return False

def record_oom_event(batch_size, coords_sample):
    """Record an OOM kill event."""
    global oom_count, last_oom_time, oom_history
    
    oom_count += 1
    last_oom_time = time.time()
    stats["oom_kills"] += 1
    
    event = {
        "timestamp": datetime.now().isoformat(),
        "batch_size": batch_size,
        "coords_sample": coords_sample[:5] if coords_sample else [],
        "memory_at_kill": get_system_stats()
    }
    
    stats["memory_profile"]["oom_batches"].append(event)
    stats["memory_profile"]["oom_batches"] = stats["memory_profile"]["oom_batches"][-20:]
    
    if os.path.exists(OOM_HISTORY_FILE):
        try:
            with open(OOM_HISTORY_FILE, 'r') as f:
                oom_history = json.load(f)
        except Exception as e:
            log(f"Failed to load OOM history: {e}", "DEBUG")
            oom_history = []
    
    oom_history.append(event)
    oom_history = oom_history[-50:]
    
    try:
        with open(OOM_HISTORY_FILE, 'w') as f:
            json.dump(oom_history, f, indent=2)
    except Exception as e:
        log(f"Failed to save OOM history: {e}", "DEBUG")
    
    save_checkpoint()
    
    log(f"🔥 OOM KILL! Event #{oom_count} - Batch {batch_size}", "OOM")
    log_terminal(f"⚠️⚠️⚠️ OOM KILLER - Size {batch_size} too large", oom=True)

def generate_layout():
    """Creates the split-screen layout with 10 lines for logs and terminal."""
    layout = Layout()
    layout.split_column(
        Layout(name="top", ratio=3),
        Layout(name="bottom", ratio=2)
    )
    
    layout["top"].update(Panel(job_progress, title="Active Tasks", border_style="blue"))
    
    layout["bottom"].split_row(
        Layout(name="logs", ratio=2),
        Layout(name="right_panel", ratio=2)
    )
    
    layout["right_panel"].split_column(
        Layout(name="terminal", ratio=3),
        Layout(name="stats", ratio=1)
    )
    
    log_text = "\n".join(log_deque)
    layout["logs"].update(Panel(Text.from_markup(log_text), title=f"Logs (last {LOG_SCROLL_SIZE})", border_style="green"))
    
    terminal_text = "\n".join(terminal_deque) if terminal_deque else "[dim]No output yet...[/dim]"
    layout["terminal"].update(Panel(Text.from_markup(terminal_text), title=f"Terminal (last {TERMINAL_SCROLL_SIZE})", border_style="yellow"))
    
    sys_stats = get_system_stats()
    stats_table = Table.grid(padding=(0, 1))
    stats_table.add_row(
        f"[cyan]CPU:[/cyan] {sys_stats['cpu']:.1f}%",
        f"[cyan]Mem:[/cyan] {sys_stats['memory_used_gb']:.1f}GB ({sys_stats['memory_percent']:.1f}%)"
    )
    stats_table.add_row(
        f"[cyan]Avail:[/cyan] {sys_stats['memory_available_gb']:.1f}GB",
        f"[cyan]Swap:[/cyan] {sys_stats['swap_used_gb']:.1f}GB ({sys_stats['swap_percent']:.1f}%)"
    )
    
    stats_table.add_row(
        f"[green]Max Batch:[/green] {MAX_BATCH_SIZE}",
        f"[cyan]Tiles:[/cyan] {stats['total_tiles_processed']}"
    )
    
    if stats['oom_kills'] > 0:
        stats_table.add_row(
            f"[red bold]OOM:[/red bold] {stats['oom_kills']}",
            f"[green]Recovered:[/green] {stats['oom_recoveries']}"
        )
    
    layout["stats"].update(Panel(stats_table, title="Stats", border_style="magenta"))
    
    return layout

def check_disk_space():
    """Verify sufficient disk space is available."""
    stat = shutil.disk_usage(BASE_DIR)
    free_gb = stat.free / (1024**3)
    if stat.free < MIN_FREE_SPACE_BYTES:
        log(f"Disk space low: {free_gb:.1f}GB", "ERROR")
        return False
    return True

def check_dependencies():
    """Verifies all required tools are installed."""
    missing = []
    for tool in ['osmium', 'tilemaker', 'tar', 'lz4', 'screen']:
        if not shutil.which(tool): missing.append(tool)
    
    for v_tool in ['valhalla_build_config', 'valhalla_build_admins', 'valhalla_build_tiles', 'valhalla_build_extract']:
        if not shutil.which(v_tool): missing.append(v_tool)
    
    if missing:
        console.print(Panel(f"[bold red]Missing:[/bold red] {', '.join(missing)}", title="Error"))
        sys.exit(1)

# --- Checkpoint System ---

def save_checkpoint():
    """Save current progress for crash recovery."""
    checkpoint = {
        "timestamp": datetime.now().isoformat(),
        "stats": stats.copy(),
        "oom_count": oom_count,
        "last_oom_time": last_oom_time,
        "batch_history": batch_history.copy(),
        "last_oom_batch_size": None
    }
    
    if oom_history and len(oom_history) > 0:
        try:
            latest_oom = oom_history[-1]
            checkpoint["last_oom_batch_size"] = latest_oom.get("batch_size")
        except (IndexError, TypeError) as e:
            log(f"Error accessing oom_history[-1]: {e}", "DEBUG")
    
    try:
        os.makedirs(WORK_DIR, exist_ok=True)
        with open(CHECKPOINT_FILE, 'w') as f:
            json.dump(checkpoint, f, indent=2)
    except:
        pass

def load_checkpoint():
    """Load checkpoint from previous run."""
    global oom_count, last_oom_time
    if os.path.exists(CHECKPOINT_FILE):
        try:
            with open(CHECKPOINT_FILE, 'r') as f:
                checkpoint = json.load(f)
                stats.update(checkpoint.get("stats", {}))
                oom_count = checkpoint.get("oom_count", 0)
                last_oom_time = checkpoint.get("last_oom_time", 0)
                if "batch_history" in checkpoint:
                    batch_history.update(checkpoint["batch_history"])
        except:
            pass

# --- Statistics Management ---

def load_statistics():
    """Load pipeline statistics from disk."""
    global stats
    if os.path.exists(STATS_FILE):
        try:
            with open(STATS_FILE, 'r') as f:
                loaded = json.load(f)
                stats.update(loaded)
        except Exception as e:
            log(f"Failed to load statistics: {e}", "DEBUG")

def save_statistics():
    """Save pipeline statistics to disk."""
    try:
        with open(STATS_FILE, 'w') as f:
            json.dump(stats, f, indent=2)
    except Exception as e:
        log(f"Failed to save statistics: {e}", "DEBUG")

# --- Helper Functions ---

def run_cmd(cmd, description="", show_error=True, show_output=True, monitor_memory=False, batch_size=0):
    """Execute command with optional memory monitoring. Shows ALL commands and output verbosely."""
    
    cmd_str = " ".join(cmd)
    log_terminal(f"$ {cmd_str}", command=True)  # Highlight commands
    
    if description:
        log(f"Exec: {description}...", "INFO")
    
    if shutdown_event.is_set():
        return False
    
    memory_exceeded_event.clear()
    
    process = None
    monitor_thread = None
    
    try:
        if show_output:
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                universal_newlines=True
            )
            
            if monitor_memory and batch_size > 0:
                monitor_thread = threading.Thread(
                    target=monitor_memory_during_extraction,
                    args=(process, batch_size),
                    daemon=True
                )
                monitor_thread.start()
            
            output_lines = []
            for line in process.stdout:
                line = line.rstrip()
                if line:
                    log_terminal(line)  # Show ALL output
                    output_lines.append(line)
            
            # Always show completion
            log_terminal("[Process completed]")
            
            process.wait()
            pid = process.pid
            returncode = process.returncode
            
            if memory_exceeded_event.is_set():
                log_terminal("⚠️ Memory 95%+ - Halting immediately", error=True)
                return "MEMORY_EXCEEDED"
            
            if returncode != 0:
                if returncode == -9 or check_oom_killed(pid):
                    log_terminal("⚠️ OOM KILL!", oom=True)
                    return "OOM"
            
            if returncode != 0:
                error_msg = "".join(output_lines[-5:]) if output_lines else "Unknown error"
                log_terminal(f"✗ Exit code: {returncode}", error=True)
                log(f"Failed: {error_msg[:200]}", "ERROR")
                return False
            
            log_terminal("✓ Success")
            return True
        else:
            cmd_name = cmd[0] if cmd else "unknown"
            log_terminal(f"[Executing: {cmd_name}]")
            try:
                result = subprocess.run(
                    cmd, 
                    check=True, 
                    text=True, 
                    capture_output=True
                )
                log_terminal("✓ Complete")
                return True
            except subprocess.CalledProcessError as e:
                if show_error:
                    log_terminal(f"✗ Failed", error=True)
                return False
                
    except Exception as e:
        if show_error:
            log(f"Exception: {str(e)[:200]}", "ERROR")
            log_terminal(f"✗ Exception: {str(e)[:100]}", error=True)
        return False
    finally:
        # Ensure process is properly terminated and resources are cleaned up
        if process is not None:
            try:
                if process.stdout:
                    process.stdout.close()
            except Exception:
                pass
            try:
                if process.poll() is None:  # Still running
                    process.terminate()
                    try:
                        process.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        process.kill()
            except Exception:
                pass

def get_remote_md5():
    """Fetch remote MD5 with retries and backoff. Returns MD5 string or None on persistent failure."""
    session = requests.Session()
    for attempt in range(1, REMOTE_MD5_RETRIES + 1):
        try:
            r = session.get(INDIA_MD5_URL, timeout=REMOTE_MD5_TIMEOUT)
            r.raise_for_status()
            parts = r.text.strip().split()
            if not parts:
                log(f"MD5 response empty or invalid format", "WARN")
                raise ValueError("Empty MD5 response")
            md5 = parts[0]
            return md5
        except requests.exceptions.RequestException as e:
            log(f"MD5 fetch attempt {attempt} failed: {e}", "WARN")
            # On transient network errors, back off then retry
            if attempt < REMOTE_MD5_RETRIES:
                sleep_time = REMOTE_MD5_BACKOFF * attempt
                log(f"Retrying MD5 fetch in {sleep_time}s... (attempt {attempt+1}/{REMOTE_MD5_RETRIES})", "INFO")
                time.sleep(sleep_time)
            else:
                log("All MD5 fetch attempts failed", "ERROR")
                return None

def get_local_md5():
    if os.path.exists(LOCAL_MD5_FILE):
        with open(LOCAL_MD5_FILE, 'r') as f: return f.read().strip()
    return None

def save_local_md5(md5_hash):
    with open(LOCAL_MD5_FILE, 'w') as f: f.write(md5_hash)

def calculate_file_md5(filepath):
    hash_md5 = hashlib.md5()
    with open(filepath, "rb") as f:
        for chunk in iter(lambda: f.read(4096), b""): hash_md5.update(chunk)
    return hash_md5.hexdigest()

def save_single_file_md5(filepath):
    md5_hash = calculate_file_md5(filepath)
    with open(filepath + ".md5", 'w') as f: f.write(md5_hash)
    return md5_hash

def download_file_resumable(url, dest_path):
    """Downloads file with Resume support."""
    session = requests.Session()
    retry = Retry(connect=5, backoff_factor=0.5)
    adapter = HTTPAdapter(max_retries=retry)
    session.mount('http://', adapter)
    session.mount('https://', adapter)

    existing_size = 0
    if os.path.exists(dest_path):
        existing_size = os.path.getsize(dest_path)

    headers = {}
    mode = 'wb'
    if existing_size > 0:
        headers['Range'] = f'bytes={existing_size}-'
        mode = 'ab' 
        log(f"Resuming from {existing_size // 1024 // 1024} MB", "INFO")

    try:
        with session.get(url, headers=headers, stream=True) as r:
            if r.status_code == 416:
                log("Download complete", "SUCCESS")
                return True
            r.raise_for_status()
            
            if existing_size > 0 and r.status_code == 200:
                existing_size = 0
                mode = 'wb'

            total_length = r.headers.get('content-length')
            total_size = int(total_length) + existing_size if total_length else None

            task_id = job_progress.add_task("[cyan]Downloading...", total=total_size, completed=existing_size)
            
            try:
                with open(dest_path, mode) as f:
                    for data in r.iter_content(chunk_size=8192):
                        if shutdown_event.is_set():
                            return False
                        f.write(data)
                        job_progress.update(task_id, advance=len(data))
            finally:
                job_progress.remove_task(task_id)
                
        return True
    except Exception as e:
        log(f"Download error: {e}", "ERROR")
        return False

# --- State Management ---

def load_state():
    if os.path.exists(STATE_FILE):
        try:
            with open(STATE_FILE, 'r') as f: return json.load(f)
        except json.JSONDecodeError: return {}
    return {}

def save_state(state):
    os.makedirs(os.path.dirname(STATE_FILE), exist_ok=True)
    with open(STATE_FILE, 'w') as f: json.dump(state, f)

def update_step_status(step_name, status=True):
    s = load_state()
    s[step_name] = status
    save_state(s)
    save_checkpoint()

def is_step_done(step_name):
    return load_state().get(step_name, False)

def update_last_run_timestamp():
    """Update the timestamp of the last successful run."""
    try:
        os.makedirs(WORK_DIR, exist_ok=True)
        timestamp = datetime.now().isoformat()
        with open(LAST_RUN_FILE, 'w') as f:
            f.write(timestamp)
        log(f"✓ Updated last run timestamp: {timestamp}", "SUCCESS")
    except Exception as e:
        log(f"Failed to update last run timestamp: {e}", "DEBUG")

def get_last_run_timestamp():
    """Get the timestamp of the last successful run."""
    if os.path.exists(LAST_RUN_FILE):
        try:
            with open(LAST_RUN_FILE, 'r') as f:
                return f.read().strip()
        except Exception as e:
            log(f"Failed to read last run timestamp: {e}", "DEBUG")
    return None

def init_workspace(target_md5):
    current_state = load_state()
    if current_state.get('target_md5') != target_md5:
        log(f"New MD5. Wiping workspace.", "WARN")
        if os.path.exists(WORK_DIR): safe_rmtree(WORK_DIR)
        os.makedirs(WORK_DIR, exist_ok=True)
        os.makedirs(os.path.join(WORK_DIR, "mbtiles"), exist_ok=True)
        os.makedirs(os.path.join(WORK_DIR, "extracts"), exist_ok=True)
        save_state({'target_md5': target_md5})
    else:
        log(f"Resuming for MD5 {target_md5[:8]}...", "SUCCESS")
        os.makedirs(os.path.join(WORK_DIR, "mbtiles"), exist_ok=True)
        os.makedirs(os.path.join(WORK_DIR, "extracts"), exist_ok=True)


def reuse_existing_poi_if_current(target_md5):
    state = load_state()
    if state.get("poi_source_md5") != target_md5:
        return None

    dest = os.path.join(WORK_DIR, "poi.sqlite")
    if os.path.exists(dest):
        return dest

    src = os.path.join(PUBLIC_DIR, "poi.sqlite")
    if os.path.exists(src):
        os.makedirs(WORK_DIR, exist_ok=True)
        shutil.copy2(src, dest)
        if os.path.exists(src + ".md5"):
            shutil.copy2(src + ".md5", dest + ".md5")
        return dest

    return None


def record_poi_source(target_md5):
    state = load_state()
    state["poi_source_md5"] = target_md5
    save_state(state)

# --- Memory-Aware Processing ---

def create_batch_config(coords_list, batch_id, output_dir):
    extracts = []
    for coord in coords_list:
        lat, lon = map(int, coord.split('_'))
        output_filename = f"{coord}.osm.pbf"
        extracts.append({"output": output_filename, "bbox": [lon, lat, lon + 1, lat + 1]})
    config = {"extracts": extracts}
    # Ensure WORK_DIR exists before writing
    os.makedirs(WORK_DIR, exist_ok=True)
    config_file = os.path.join(WORK_DIR, f"batch_{batch_id}.json")
    with open(config_file, 'w') as f: json.dump(config, f, indent=2)
    return config_file

def remove_small_files_in_batch(coords_batch, extracts_output_dir):
    """Remove files smaller than MIN_FILE_SIZE_BYTES immediately after extraction."""
    removed_count = 0
    for coord in coords_batch:
        pbf_path = os.path.join(extracts_output_dir, f"{coord}.osm.pbf")
        if os.path.exists(pbf_path):
            try:
                size = os.path.getsize(pbf_path)
                if size < MIN_FILE_SIZE_BYTES:
                    os.remove(pbf_path)
                    removed_count += 1
                    log(f"Removed small file: {coord}.osm.pbf ({size} bytes)", "INFO")
            except OSError:
                pass
    
    if removed_count > 0:
        log(f"Removed {removed_count} small files (< {MIN_FILE_SIZE_BYTES} bytes)", "SUCCESS")
    
    return removed_count

def process_adaptive_batch(source_pbf, coords_to_extract, extracts_output_dir):
    """
    AGGRESSIVE OPTIMIZED - ALWAYS USE MAX BATCH SIZE
    - Starts at 100 tiles (MAX)
    - Monitors memory at 95%
    - NO retries, NO delays
    - Removes small files immediately
    """
    
    pending_coords = []
    for c in coords_to_extract:
        if not os.path.exists(os.path.join(extracts_output_dir, f"{c}.osm.pbf")):
            pending_coords.append(c)
    
    if len(pending_coords) < len(coords_to_extract):
        skipped = len(coords_to_extract) - len(pending_coords)
        log(f"Resume: {skipped} done", "INFO")

    total_tiles = len(pending_coords)
    if total_tiles == 0:
        log("All tiles done!", "SUCCESS")
        return []

    starting_size = get_starting_batch_size()
    if starting_size <= 0:
        log(f"Invalid starting batch size: {starting_size}, using 1", "WARN")
        starting_size = 1
    log(f"Starting batch size: {starting_size} (MAX MODE)", "LEARN")
    
    if not pending_coords:
        log(f"No pending coordinates to extract", "WARN")
        return {"total": 0, "success": 0, "failed": []}
    
    queue = deque()
    chunks = [pending_coords[i:i + starting_size] for i in range(0, len(pending_coords), starting_size)]
    for chunk in chunks:
        queue.append((chunk, 0))

    failed_coords = []
    completed = 0
    
    task_id = job_progress.add_task("[green]Extracting (MAX)...", total=total_tiles)

    try:
        while queue and not shutdown_event.is_set():
            coords_batch, depth = queue.popleft()
            current_batch_size = len(coords_batch)
            
            if current_batch_size > 5:
                gc.collect()
            
            mem_before = get_system_stats()
            
            if current_batch_size == 1:
                log(f"Tile: {coords_batch[0] if coords_batch else 'unknown'} (d:{depth})", "INFO")
            else:
                log(f"Batch: {current_batch_size} tiles (d:{depth}, mem:{mem_before['memory_percent']:.0f}%)", "INFO")

            # Pre-flight guard: if system is already under swap or high RAM, shrink batch before starting (more aggressive)
            if mem_before["swap_percent"] > SWAP_HEADROOM * 100 or mem_before["memory_percent"] > MEMORY_HEADROOM * 100:
                if current_batch_size > 2:
                    new_size = max(1, current_batch_size // 3)
                    left = coords_batch[:new_size]
                    right = coords_batch[new_size:]
                    log(f"⚠️ High system load (mem {mem_before['memory_percent']:.1f}%, swap {mem_before['swap_percent']:.1f}%) → reduce batch {current_batch_size} → {len(left)}+{len(right)} (third split)", "WARN")
                    if right:
                        queue.appendleft((right, depth + 1))
                    coords_batch = left
                    current_batch_size = len(coords_batch)
                elif current_batch_size == 2:
                    # run as singles to avoid spike
                    a, b = coords_batch
                    log(f"⚠️ High load → split pair into singles: {a}, {b}", "WARN")
                    queue.appendleft(([b], depth + 1))
                    coords_batch = [a]
                    current_batch_size = 1
                else:
                    log(f"✗ High memory/swap (mem {mem_before['memory_percent']:.1f}%, swap {mem_before['swap_percent']:.1f}%) – skipping tile {coords_batch[0]}", "ERROR")
                    failed_coords.append(coords_batch[0])
                    completed += 1
                    job_progress.update(task_id, completed=completed)
                    continue
            
            batch_id = f"{uuid.uuid4().hex[:8]}"
            config_file = create_batch_config(coords_batch, batch_id, extracts_output_dir)
            cmd = ["osmium", "extract", "-c", config_file, "-d", extracts_output_dir, source_pbf, "--overwrite"]
            
            start_time = time.time()
            result = run_cmd(
                cmd, 
                description=f"Extracting {current_batch_size} tile(s)", 
                show_error=True,
                show_output=True,
                monitor_memory=True,
                batch_size=current_batch_size
            )
            duration = time.time() - start_time
            
            mem_after = get_system_stats()
            
            if os.path.exists(config_file):
                os.remove(config_file)

            # Handle MEMORY_EXCEEDED
            if result == "MEMORY_EXCEEDED":
                log(f"⚡ Memory 95%! Halving {current_batch_size} NOW", "WARN")
                stats["oom_recoveries"] += 1
                gc.collect()
                
                if current_batch_size > 1:
                    mid = max(1, current_batch_size // 2)
                    left = coords_batch[:mid]
                    right = coords_batch[mid:]
                    log(f"⚡ Split: {current_batch_size} → {len(left)} + {len(right)}", "WARN")
                    
                    queue.appendleft((right, depth + 1))
                    queue.appendleft((left, depth + 1))
                else:
                    log(f"✗ Single tile 95%+ memory!", "ERROR")
                    failed_coords.append(coords_batch[0])
                    completed += 1
                    job_progress.update(task_id, completed=completed)
                
                save_checkpoint()
                continue

            # Handle OOM kill
            if result == "OOM":
                log(f"🔥 OOM! Halving {current_batch_size} NOW", "OOM")
                record_oom_event(current_batch_size, coords_batch)
                stats["oom_recoveries"] += 1
                gc.collect()
                
                if current_batch_size > 1:
                    mid = max(1, current_batch_size // 2)
                    left = coords_batch[:mid]
                    right = coords_batch[mid:]
                    log(f"🔥 OOM split: {current_batch_size} → {len(left)} + {len(right)}", "WARN")
                    
                    queue.appendleft((right, depth + 1))
                    queue.appendleft((left, depth + 1))
                else:
                    log(f"✗ Single tile OOM!", "ERROR")
                    failed_coords.append(coords_batch[0])
                    completed += 1
                    job_progress.update(task_id, completed=completed)
                
                save_checkpoint()
                continue

            if result == True:
                # SUCCESS! Remove small files immediately
                removed = remove_small_files_in_batch(coords_batch, extracts_output_dir)
                
                record_successful_batch(current_batch_size)
                
                log(f"✓ {current_batch_size} tiles ({duration:.0f}s, mem:{mem_after['memory_percent']:.0f}%)", "SUCCESS")
                completed += current_batch_size
                stats["total_tiles_processed"] += current_batch_size
                job_progress.update(task_id, completed=completed)
            else:
                # Regular failure - split immediately
                if current_batch_size > 1:
                    mid = max(1, current_batch_size // 2)
                    left = coords_batch[:mid]
                    right = coords_batch[mid:]
                    
                    log(f"✗ Failed - Split NOW: {current_batch_size} → {len(left)}+{len(right)}", "WARN")
                    
                    queue.appendleft((right, depth + 1))
                    queue.appendleft((left, depth + 1))
                else:
                    if coords_batch:
                        log(f"✗ Tile {coords_batch[0]} failed - SKIP", "ERROR")
                        failed_coords.append(coords_batch[0])
                        stats["failed_tiles"].append(coords_batch[0])
                    completed += 1
                    job_progress.update(task_id, completed=completed)
            
            if completed % 50 == 0:
                save_checkpoint()
                    
    finally:
        job_progress.remove_task(task_id)
        save_statistics()
        save_checkpoint()
        save_batch_history()
    
    if failed_coords:
        log(f"Done with {len(failed_coords)} failures", "WARN")
    else:
        log("All tiles extracted!", "SUCCESS")
    
    return failed_coords

def generate_coordinates(bbox_str):
    min_lon, min_lat, max_lon, max_lat = map(float, bbox_str.split(','))
    coords = []
    # Round coordinates for proper grid coverage instead of truncating
    min_lat_int = int(min_lat if min_lat == int(min_lat) else (min_lat + 0.5))
    max_lat_int = int(max_lat if max_lat == int(max_lat) else (max_lat + 0.5))
    min_lon_int = int(min_lon if min_lon == int(min_lon) else (min_lon + 0.5))
    max_lon_int = int(max_lon if max_lon == int(max_lon) else (max_lon + 0.5))
    for lat in range(min_lat_int, max_lat_int + 1):
        for lon in range(min_lon_int, max_lon_int + 1):
            coords.append(f"{lat}_{lon}")
    return coords

# --- Parallel MBTiles Conversion ---

def convert_single_mbtile(args):
    """Worker function for parallel mbtiles conversion."""
    pbf_file, mbtiles_dir, extracts_dir, tm_config_nav, tm_process = args
    
    coord_name = pbf_file.replace(".osm.pbf", "")
    out_f = os.path.join(mbtiles_dir, f"{coord_name}.mbtiles")
    in_f = os.path.join(extracts_dir, pbf_file)
    
    if os.path.exists(out_f) and os.path.getsize(out_f) > MIN_FILE_SIZE_BYTES:
        return (coord_name, True, "skipped")
    
    try:
        lat, lon = map(int, coord_name.split('_'))
        bbox = f"{lon},{lat},{lon+1},{lat+1}"
    except:
        bbox = None
    
    try:
        cmd = ["tilemaker", "--input", in_f, "--output", out_f, "--config", tm_config_nav, "--process", tm_process]
        if bbox:
            cmd.extend(["--bbox", bbox])
        
        result = subprocess.run(
            cmd, 
            stdout=subprocess.DEVNULL, 
            stderr=subprocess.PIPE,
            timeout=SUBPROCESS_TIMEOUT,
            text=True
        )
        
        if result.returncode == 0 and os.path.exists(out_f):
            try:
                save_single_file_md5(out_f)
            except Exception as e:
                log(f"Failed to save MD5 for {out_f}: {e}", "DEBUG")
            try: os.remove(in_f)
            except OSError: pass
            return (coord_name, True, "success")
        else:
            return (coord_name, False, result.stderr[:200] if result.stderr else "Unknown error")
            
    except subprocess.TimeoutExpired:
        return (coord_name, False, "timeout")
    except Exception as e:
        return (coord_name, False, str(e)[:200])

def parallel_mbtiles_conversion(pbf_files, mbtiles_dir, extracts_dir, tm_config_nav, tm_process):
    """Convert PBF files to MBTiles in parallel."""
    
    log(f"Converting {len(pbf_files)} tiles ({TILEMAKER_WORKERS} workers)", "INFO")
    
    args_list = [(pbf, mbtiles_dir, extracts_dir, tm_config_nav, tm_process) for pbf in pbf_files]
    
    task_id = job_progress.add_task("[cyan]Converting...", total=len(pbf_files))
    
    completed = 0
    failed = []
    
    try:
        with Pool(processes=TILEMAKER_WORKERS) as pool:
            for coord_name, success, message in pool.imap_unordered(convert_single_mbtile, args_list):
                completed += 1
                job_progress.update(task_id, completed=completed)
                
                if not success and message != "skipped":
                    failed.append(coord_name)
                    
                if shutdown_event.is_set():
                    pool.terminate()
                    break
                    
                if completed % 100 == 0:
                    save_checkpoint()
    finally:
        job_progress.remove_task(task_id)
    
    if failed:
        log(f"Conversion: {len(failed)} failed", "WARN")
    else:
        log(f"Conversion: All {len(pbf_files)} done!", "SUCCESS")
    
    return failed

def cleanup_small_files(directory):
    """Remove files smaller than MIN_FILE_SIZE_BYTES."""
    count = 0
    for root, dirs, files in os.walk(directory):
        for file in files:
            path = os.path.join(root, file)
            try:
                if os.path.getsize(path) < MIN_FILE_SIZE_BYTES:
                    os.remove(path)
                    if os.path.exists(path + ".md5"): os.remove(path + ".md5")
                    count += 1
            except OSError: pass
    if count > 0: log(f"Cleaned {count} files", "SUCCESS")

def create_tiles_index_json(directory: str, index_name: str = "tiles_index.json"):
    """
    Create a comprehensive tiles index JSON with MD5 for all tiles in directory.
    Used for client-side tile update checking.
    
    Format:
    {
        "generated_at": "2026-01-17T...",
        "tiles": {
            "28_77": {
                "filename": "28_77.mbtiles",
                "size_bytes": 1234567,
                "md5": "abc123..."
            },
            ...
        },
        "total_tiles": 900,
        "total_size_gb": 45.2
    }
    """
    if not os.path.exists(directory):
        log(f"Tiles directory not found: {directory}", "WARN")
        return None
    
    index = {
        "generated_at": datetime.now().isoformat(),
        "tiles": {},
        "total_tiles": 0,
        "total_size_bytes": 0,
        "bbox": INDIA_BBOX
    }
    
    # Collect all tile files (.mbtiles or .mbtiles.lz4)
    # Note: MD5 in this index represents the MAP CONTENT (uncompressed
    # .mbtiles) when available, not the compressed .lz4 container.
    tile_files = [f for f in os.listdir(directory) if f.endswith(".mbtiles") or f.endswith(".mbtiles.lz4")]
    
    for tile_file in sorted(tile_files):
        file_path = os.path.join(directory, tile_file)
        try:
            # Extract tile coordinate from filename:
            # supports "28_77.mbtiles" and "28_77.mbtiles.lz4" -> "28_77"
            if tile_file.endswith(".mbtiles.lz4"):
                tile_name = tile_file[:-len(".mbtiles.lz4")]
            elif tile_file.endswith(".mbtiles"):
                tile_name = tile_file[:-len(".mbtiles")]
            else:
                continue

            file_size = os.path.getsize(file_path)

            # Prefer MD5 of the uncompressed map content if available.
            # We expect a sidecar like "28_77.mbtiles.md5" created
            # before compression. If not present, fall back to the
            # MD5 of the actual file on disk (compressed or not).
            md5_hash = None

            # Look for uncompressed map MD5 sidecar first
            uncompressed_md5_path = os.path.join(directory, f"{tile_name}.mbtiles.md5")
            if os.path.exists(uncompressed_md5_path):
                try:
                    with open(uncompressed_md5_path, 'r') as f:
                        md5_hash = f.read().strip().split()[0]
                except Exception as e:
                    log(f"Failed to read uncompressed MD5 for {tile_name}: {e}", "WARN")

            # Fall back to MD5 of the actual file (.mbtiles or .mbtiles.lz4)
            if not md5_hash:
                md5_file = file_path + ".md5"
                if os.path.exists(md5_file):
                    with open(md5_file, 'r') as f:
                        md5_hash = f.read().strip().split()[0]
                else:
                    md5_hash = calculate_file_md5(file_path)

            index["tiles"][tile_name] = {
                "filename": tile_file,
                "size_bytes": file_size,
                "size_mb": file_size / (1024**2),
                "md5": md5_hash
            }
            
            index["total_tiles"] += 1
            index["total_size_bytes"] += file_size
        
        except Exception as e:
            log(f"Error processing {tile_file}: {e}", "WARN")
    
    # Convert total size to GB
    index["total_size_gb"] = round(index["total_size_bytes"] / (1024**3), 2)
    
    # Save index file
    index_path = os.path.join(directory, index_name)
    try:
        with open(index_path, 'w') as f:
            json.dump(index, f, indent=2)
        log(f"✓ Created tiles index: {index_path} ({index['total_tiles']} tiles)", "SUCCESS")
        return index_path
    except Exception as e:
        log(f"Failed to create tiles index: {e}", "ERROR")
        return None

def create_md5_manifest_json(mbtiles_files, output_dir):
    """Create JSON manifest with MD5 hashes of uncompressed MBTiles files."""
    manifest = {
        "generated_at": datetime.now().isoformat(),
        "type": "mobile_mbtiles",
        "files": {}
    }
    
    for mbtiles_path in mbtiles_files:
        if os.path.exists(mbtiles_path):
            file_name = os.path.basename(mbtiles_path)
            file_size = os.path.getsize(mbtiles_path)
            md5_hash = calculate_file_md5(mbtiles_path)
            
            manifest["files"][file_name] = {
                "path": mbtiles_path,
                "size_bytes": file_size,
                "size_mb": file_size / (1024**2),
                "md5": md5_hash
            }
    
    manifest_path = os.path.join(output_dir, "mbtiles_manifest.json")
    try:
        os.makedirs(output_dir, exist_ok=True)
        with open(manifest_path, 'w') as f:
            json.dump(manifest, f, indent=2)
        log(f"✓ Created MD5 manifest: {manifest_path}", "SUCCESS")
        return manifest_path
    except Exception as e:
        log(f"Failed to create manifest: {e}", "ERROR")
        return None

def compress_mbtiles_batch(mbtiles_files, output_dir):
    """Compress multiple MBTiles files with lz4 and create MD5 manifest."""
    log(f"Compressing {len(mbtiles_files)} MBTiles files to {output_dir}...", "INFO")
    
    os.makedirs(output_dir, exist_ok=True)
    compressed_files = []
    
    for mbtiles_path in mbtiles_files:
        if not os.path.exists(mbtiles_path):
            log(f"Skipping missing file: {mbtiles_path}", "WARN")
            continue
        
        file_name = os.path.basename(mbtiles_path)
        compressed_path = os.path.join(output_dir, f"{file_name}.lz4")
        
        try:
            original_size = os.path.getsize(mbtiles_path)
            log(f"Compressing {file_name} ({original_size / (1024**2):.1f} MB)...", "INFO")
            
            result = subprocess.run(
                ["lz4", "-9", "-f", mbtiles_path, compressed_path],
                capture_output=True,
                text=True,
                timeout=600
            )
            
            if result.returncode == 0 and os.path.exists(compressed_path):
                compressed_size = os.path.getsize(compressed_path)
                ratio = (1 - compressed_size / original_size) * 100
                log(f"✓ {file_name}: {original_size / (1024**2):.1f}MB → {compressed_size / (1024**2):.1f}MB ({ratio:.1f}% saved)", "SUCCESS")
                compressed_files.append(mbtiles_path)
                save_single_file_md5(compressed_path)
            else:
                log(f"Compression failed for {file_name}", "ERROR")
        except Exception as e:
            log(f"Error compressing {file_name}: {e}", "ERROR")
    
    manifest_path = create_md5_manifest_json(compressed_files, output_dir)
    
    return compressed_files, manifest_path

def generate_metadata_json(directory, source_md5):
    log("Generating metadata...", "INFO")
    meta = {
        "last_updated": datetime.now().isoformat(),
        "source_pbf_md5": source_md5,
        "pipeline_stats": stats.copy(),
        "batch_history": batch_history.copy(),
        "files": {}
    }
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file == "metadata.json" or file.endswith(".md5"): continue
            path = os.path.join(root, file)
            rel_path = os.path.relpath(path, directory)
            md5_path = path + ".md5"
            md5 = None
            if os.path.exists(md5_path):
                try:
                    with open(md5_path, 'r') as f: md5 = f.read().strip()
                except Exception as e:
                    log(f"Failed to read MD5 from {md5_path}: {e}", "DEBUG")
            if not md5:
                md5 = save_single_file_md5(path) or "unknown"
            if os.path.exists(path):
                meta["files"][rel_path] = {"size_bytes": os.path.getsize(path), "md5": md5}
    with open(os.path.join(directory, "metadata.json"), "w") as f:
        json.dump(meta, f, indent=2)

class OSMToSQLiteHandler(osmium.SimpleHandler):
    """Convert OSM PBF to SQLite with FTS search support."""
    def __init__(self, db_path):
        super().__init__()
        self.db_path = db_path
        self.batch = []
        self.batch_size = 10000
        self.total_processed = 0
        self.setup_database()
    
    def setup_database(self):
        """Create database schema with FTS support."""
        conn = sqlite3.connect(self.db_path)
        c = conn.cursor()
        
        c.execute('''
            CREATE TABLE IF NOT EXISTS places (
                id TEXT PRIMARY KEY,
                name TEXT NOT NULL,
                lat REAL NOT NULL,
                lon REAL NOT NULL,
                type TEXT,
                category TEXT,
                geometry_type TEXT,
                tags TEXT
            )
        ''')
        
        c.execute('CREATE INDEX IF NOT EXISTS idx_name ON places(name COLLATE NOCASE)')
        c.execute('CREATE INDEX IF NOT EXISTS idx_type ON places(type)')
        c.execute('CREATE INDEX IF NOT EXISTS idx_location ON places(lat, lon)')
        
        c.execute('''
            CREATE VIRTUAL TABLE IF NOT EXISTS places_fts 
            USING fts5(name, type, content=places, content_rowid=rowid)
        ''')
        
        conn.commit()
        conn.close()
    
    def get_category_and_type(self, tags):
        """Determine category and type from OSM tags."""
        priority_categories = [
            ('amenity', 'Amenity'),
            ('shop', 'Shop'),
            ('tourism', 'Tourism'),
            ('leisure', 'Leisure'),
            ('highway', 'Road'),
            ('building', 'Building'),
            ('natural', 'Natural'),
            ('place', 'Place'),
            ('railway', 'Railway'),
            ('public_transport', 'Transport')
        ]
        
        for tag, category in priority_categories:
            if tag in tags:
                return category, tags[tag]
        
        return 'Other', 'unknown'
    
    def node(self, n):
        """Process node POIs."""
        if 'name' in n.tags:
            lat, lon = n.location.lat, n.location.lon
            category, type_val = self.get_category_and_type(n.tags)
            
            self.batch.append({
                'id': f'n{n.id}',
                'name': n.tags['name'],
                'lat': round(lat, 6),
                'lon': round(lon, 6),
                'type': type_val,
                'category': category,
                'geometry_type': 'point',
                'tags': json.dumps(dict(n.tags))
            })
            
            if len(self.batch) >= self.batch_size:
                self.flush_batch()
    
    def way(self, w):
        """Process way POIs."""
        if 'name' in w.tags:
            try:
                nodes = list(w.nodes)
                if nodes:
                    lat = sum(n.lat for n in nodes) / len(nodes)
                    lon = sum(n.lon for n in nodes) / len(nodes)
                    
                    category, type_val = self.get_category_and_type(w.tags)
                    
                    if 'highway' in w.tags or 'railway' in w.tags:
                        geom_type = 'line'
                    elif 'building' in w.tags or 'area' in w.tags:
                        geom_type = 'polygon'
                    else:
                        geom_type = 'way'
                    
                    self.batch.append({
                        'id': f'w{w.id}',
                        'name': w.tags['name'],
                        'lat': round(lat, 6),
                        'lon': round(lon, 6),
                        'type': type_val,
                        'category': category,
                        'geometry_type': geom_type,
                        'tags': json.dumps(dict(w.tags))
                    })
                    
                    if len(self.batch) >= self.batch_size:
                        self.flush_batch()
            except:
                pass
    
    def flush_batch(self):
        """Write batch to database."""
        if not self.batch:
            return
        
        conn = sqlite3.connect(self.db_path)
        c = conn.cursor()
        
        c.executemany('''
            INSERT OR REPLACE INTO places 
            (id, name, lat, lon, type, category, geometry_type, tags)
            VALUES (:id, :name, :lat, :lon, :type, :category, :geometry_type, :tags)
        ''', self.batch)
        
        for item in self.batch:
            c.execute('''
                INSERT INTO places_fts(rowid, name, type)
                SELECT rowid, name, type FROM places WHERE id = ?
            ''', (item['id'],))
        
        conn.commit()
        conn.close()
        
        self.total_processed += len(self.batch)
        log_terminal(f"📍 Processed {self.total_processed:,} places...")
        self.batch = []
    
    def finalize(self):
        """Finalize database and return count."""
        self.flush_batch()
        
        conn = sqlite3.connect(self.db_path)
        c = conn.cursor()
        c.execute('VACUUM')
        c.execute('ANALYZE')
        c.execute('SELECT COUNT(*) FROM places')
        count = c.fetchone()[0]
        conn.close()
        
        return count


def generate_poi_database(pbf_file):
    """Extracts places from PBF using osmium and stores in SQLite with FTS. ALWAYS regenerates."""
    log("Generating POI database from PBF...", "HEADER")
    
    # Check if osmium is available
    if osmium is None:
        log("ERROR: osmium module not found", "ERROR")
        log("Install with: pip install osmium", "INFO")
        return False

    db_path = os.path.join(WORK_DIR, "poi.sqlite")
    
    # Always remove and regenerate
    if os.path.exists(db_path):
        log(f"Removing existing POI database for fresh generation...", "INFO")
        try:
            os.remove(db_path)
            if os.path.exists(db_path + ".md5"):
                os.remove(db_path + ".md5")
        except Exception as e:
            log(f"Warning: Could not remove existing POI DB: {e}", "WARN")

    log("Converting PBF to SQLite with FTS...", "INFO")
    log_terminal("📍 Extracting places from PBF...")
    
    try:
        handler = OSMToSQLiteHandler(db_path)
        handler.apply_file(pbf_file, locations=True)
        total = handler.finalize()
        
        log("=" * 50, "HEADER")
        log(f"POI Processing Complete:", "SUCCESS")
        log(f"  ✓ Total places: {total:,}", "SUCCESS")
        log(f"  ✓ Database: {db_path}", "INFO")
        log("=" * 50, "HEADER")

        if total == 0:
            log("⚠️  No places extracted! Check PBF file", "WARN")
            return db_path
        
        return db_path
        
    except Exception as e:
        log(f"Error processing PBF: {e}", "ERROR")
        log(f"Traceback: {traceback.format_exc()}", "DEBUG")
        return False


def compress_file_with_lz4(source_path, keep_original=False):
    """Compress a file with lz4 maximum compression.
    
    Args:
        source_path: Path to file to compress
        keep_original: If True, keep the original file. If False, delete it.
    
    Returns:
        Path to compressed file, or None on failure
    """
    if not os.path.exists(source_path):
        return None
    
    compressed_path = source_path + ".lz4"
    
    try:
        original_size = os.path.getsize(source_path)
        log(f"Compressing {os.path.basename(source_path)} ({original_size / (1024**2):.1f} MB)...", "INFO")
        
        result = subprocess.run(
            ["lz4", "-9", "-f", source_path, compressed_path],
            capture_output=True,
            text=True,
            timeout=600
        )
        
        if result.returncode == 0 and os.path.exists(compressed_path):
            compressed_size = os.path.getsize(compressed_path)
            ratio = (compressed_size / original_size) * 100
            saved = original_size - compressed_size
            log(f"✓ Compressed to {compressed_size / (1024**2):.1f} MB ({ratio:.1f}%, saved {saved / (1024**2):.1f} MB)", "SUCCESS")
            
            # Delete original if requested
            if not keep_original:
                try:
                    os.remove(source_path)
                    # Also remove original's MD5 if it exists
                    if os.path.exists(source_path + ".md5"):
                        os.remove(source_path + ".md5")
                except Exception as e:
                    log(f"Warning: Could not remove original: {e}", "DEBUG")
            
            return compressed_path
        else:
            log(f"lz4 compression failed: {result.stderr}", "ERROR")
            return None
    except Exception as e:
        log(f"Failed to compress {source_path}: {e}", "ERROR")
        return None


def ensure_valhalla_tar_from_tiles(tile_dir, work_dir):
    """Ensure valhalla_tiles.tar.lz4 exists by rebuilding from tiles if needed."""
    tar_path = os.path.join(work_dir, "valhalla_tiles.tar.lz4")
    if os.path.exists(tar_path) and os.path.getsize(tar_path) > MIN_FILE_SIZE_BYTES:
        return tar_path

    if not os.path.exists(tile_dir):
        log(f"Valhalla tiles dir missing: {tile_dir}", "ERROR")
        return None

    # Count .gph tiles
    tile_files = []
    for root, dirs, files in os.walk(tile_dir):
        tile_files.extend([os.path.join(root, f) for f in files if f.endswith('.gph')])

    if len(tile_files) == 0:
        log(f"No .gph tiles found in {tile_dir}; cannot build tar", "ERROR")
        return None

    log(f"Rebuilding valhalla_tiles.tar.lz4 from {len(tile_files)} tiles...", "INFO")

    temp_tar = os.path.join(work_dir, "valhalla_tiles_temp.tar")
    try:
        with tarfile.open(temp_tar, "w") as tar:
            tar.add(tile_dir, arcname="valhalla_tiles")

        result = subprocess.run([
            "lz4", "-9", "-f", temp_tar, tar_path
        ], capture_output=True, text=True)

        if os.path.exists(temp_tar):
            try:
                os.remove(temp_tar)
            except OSError:
                pass

        if result.returncode != 0:
            log(f"lz4 compression failed: {result.stderr}", "ERROR")
            return None

        if os.path.exists(tar_path) and os.path.getsize(tar_path) > MIN_FILE_SIZE_BYTES:
            save_single_file_md5(tar_path)
            tar_size_mb = os.path.getsize(tar_path) / (1024**2)
            log(f"✓ Rebuilt valhalla_tiles.tar.lz4 ({tar_size_mb:.2f} MB)", "SUCCESS")
            return tar_path
        else:
            log("Rebuilt tar.lz4 is missing or empty", "ERROR")
            return None
    except Exception as e:
        log(f"Failed to rebuild valhalla tar: {e}", "ERROR")
        return None

def create_manifest_for_directory(directory, name="manifest"):
    """Create a manifest file listing all files with their MD5 hashes."""
    manifest = {
        "name": name,
        "created": datetime.now().isoformat(),
        "files": {}
    }
    
    if not os.path.exists(directory):
        return None
    
    try:
        for root, dirs, files in os.walk(directory):
            for file in files:
                file_path = os.path.join(root, file)
                rel_path = os.path.relpath(file_path, directory)
                
                try:
                    file_md5 = calculate_file_md5(file_path)
                    file_size = os.path.getsize(file_path)
                    
                    manifest["files"][rel_path] = {
                        "md5": file_md5,
                        "size_bytes": file_size,
                        "size_mb": round(file_size / (1024**2), 2)
                    }
                except Exception as e:
                    log(f"Failed to hash {file_path}: {e}", "DEBUG")
        
        return manifest
    except Exception as e:
        log(f"Failed to create manifest for {directory}: {e}", "ERROR")
        return None


def calculate_all_md5_in_directory(directory):
    """Calculate and save MD5 for all files in directory."""
    if not os.path.exists(directory):
        return 0
    
    count = 0
    try:
        for root, dirs, files in os.walk(directory):
            for file in files:
                file_path = os.path.join(root, file)
                
                # Skip if already .md5 file or manifest
                if file.endswith('.md5') or file == 'manifest.json':
                    continue
                
                try:
                    md5_hash = calculate_file_md5(file_path)
                    md5_file = file_path + ".md5"
                    
                    with open(md5_file, 'w') as f:
                        f.write(md5_hash)
                    
                    count += 1
                except Exception as e:
                    log(f"Failed to create MD5 for {file_path}: {e}", "DEBUG")
    except Exception as e:
        log(f"Error walking directory {directory}: {e}", "ERROR")
    
    return count

def safe_rmtree(path):
    """Safely remove directory tree, handling symlinks."""
    if not os.path.exists(path):
        return
    try:
        for root, dirs, files in os.walk(path, topdown=False):
            for f in files:
                try:
                    os.remove(os.path.join(root, f))
                except OSError:
                    pass
            for d in dirs:
                d_path = os.path.join(root, d)
                if os.path.islink(d_path):
                    try:
                        os.unlink(d_path)
                    except OSError:
                        pass
                else:
                    try:
                        os.rmdir(d_path)
                    except OSError:
                        pass
        os.rmdir(path)
    except Exception as e:
        log(f"Warning: Could not fully clean {path}: {e}", "WARN")

# --- Valhalla Manifest Support ---

def build_valhalla_graph(pbf_file, force_rebuild=False):
    """Build Valhalla routing graph with correct commands."""
    if not force_rebuild and is_step_done("valhalla"):
        tar_path = os.path.join(WORK_DIR, "valhalla_tiles.tar")
        if os.path.exists(tar_path):
            log("Valhalla done", "SUCCESS")
            return tar_path

    # Clean up any partial Valhalla data from previous failed runs
    if os.path.exists(VALHALLA_DIR):
        log("Cleaning up previous Valhalla data...", "INFO")
        safe_rmtree(VALHALLA_DIR)
    
    # Create fresh directories
    os.makedirs(VALHALLA_DIR, exist_ok=True)
    
    config_path = os.path.join(VALHALLA_DIR, "valhalla.json")
    tile_dir = os.path.join(VALHALLA_DIR, "valhalla_tiles")
    elevation_dir = os.path.join(VALHALLA_DIR, "elevation")
    
    # Create all necessary directories
    for d in [tile_dir, elevation_dir]:
        os.makedirs(d, exist_ok=True)

    log("Generating Valhalla config with command-line args...", "INFO")
    # Generate config with all paths specified
    tz_sqlite_path = os.path.join(VALHALLA_DIR, "timezones.sqlite")
    # Point tile-extract to a throwaway path so tiles.tar is never produced/used
    skip_tile_extract = os.path.join(WORK_DIR, "valhalla_tiles_skip.tar")
    cmd = [
        "valhalla_build_config",
        "--mjolnir-tile-dir", tile_dir,
        "--mjolnir-tile-extract", skip_tile_extract,
        "--mjolnir-traffic-extract", os.path.join(VALHALLA_DIR, "traffic.tar"),
        "--mjolnir-admin", os.path.join(VALHALLA_DIR, "admin.sqlite"),
        "--mjolnir-landmarks", os.path.join(VALHALLA_DIR, "landmarks.sqlite"),
        "--mjolnir-timezone", tz_sqlite_path,
        "--mjolnir-transit-dir", os.path.join(VALHALLA_DIR, "transit"),
        "--mjolnir-transit-feeds-dir", os.path.join(VALHALLA_DIR, "transit_feeds"),
        "--additional-data-elevation", elevation_dir + "/"
    ]
    try:
        with open(config_path, "w") as f:
            subprocess.check_call(cmd, stdout=f, stderr=subprocess.STDOUT, timeout=60)
        log("Config generated successfully", "SUCCESS")
    except Exception as e:
        log(f"Config generation failed: {e}", "ERROR")
        return False

    task_id = job_progress.add_task("[bold blue]Building Valhalla...", total=None)
    
    try:
        # Build timezones - direct subprocess execution with stdout redirect
        log("Building timezones...", "INFO")
        log_terminal("$ valhalla_build_timezones > timezones.sqlite", command=True)
        with open(tz_sqlite_path, "w") as tz_file:
            result = subprocess.run(
                ["valhalla_build_timezones"],
                stdout=tz_file,
                stderr=subprocess.PIPE,
                text=True
            )
        log(f"Timezones exit code: {result.returncode}", "INFO")
        if result.stderr:
            log_terminal(result.stderr)

        # Build admins
        log("Building admins...", "INFO")
        log_terminal("$ valhalla_build_admins -c valhalla.json pbf_file", command=True)
        result = subprocess.run(
            ["valhalla_build_admins", "-c", config_path, pbf_file],
            capture_output=False,
            text=True
        )
        log(f"Admins exit code: {result.returncode}", "INFO")
        if result.returncode != 0:
            log("Admins command returned non-zero, but continuing...", "WARN")

        # Build tiles (no valhalla_build_extract, so no tiles.tar is produced)
        log("Building tiles...", "INFO")
        log_terminal("$ valhalla_build_tiles -c config pbf_file", command=True)
        result = subprocess.run(
            ["valhalla_build_tiles", "-c", config_path, pbf_file],
            capture_output=True,
            text=True
        )
        
        if result.stdout:
            log_terminal(result.stdout)
        if result.stderr:
            log_terminal(result.stderr, error=True)
            
        log(f"Tiles exit code: {result.returncode}", "INFO")
        
        if result.returncode != 0:
            log("ERROR: valhalla_build_tiles failed!", "ERROR")
            log("Tiles were not generated - cannot continue", "ERROR")
            return False

        # Skip valhalla_build_extract entirely; we package tiles directory ourselves below
        log("Skipping valhalla_build_extract and tiles.tar generation", "INFO")
        
        # Verify tiles were actually created before continuing
        log("Verifying tile generation...", "INFO")
        tile_files = []
        for root, dirs, files in os.walk(tile_dir):
            tile_files.extend([f for f in files if f.endswith('.gph')])
        
        if len(tile_files) == 0:
            log(f"ERROR: No .gph tiles found in {tile_dir}", "ERROR")
            log("valhalla_build_tiles completed but produced no output", "ERROR")
            return False
        
        log(f"✓ Verified: {len(tile_files)} tile files created", "SUCCESS")
            
    except Exception as e:
        log(f"Exception during Valhalla build: {e}", "ERROR")
        tb = traceback.format_exc()
        log_terminal(tb, error=True)
        return False
    finally:
        job_progress.remove_task(task_id)

    # Create tar from valhalla_tiles directory directly
    log("Archiving Valhalla tiles with manifest...", "INFO")
    os.makedirs(WORK_DIR, exist_ok=True)
    
    tar_path = os.path.join(WORK_DIR, "valhalla_tiles.tar.lz4")
    
    try:
        # First verify that tile_dir exists and has .gph files
        if not os.path.exists(tile_dir):
            log(f"ERROR: Tile directory not found: {tile_dir}", "ERROR")
            log("Valhalla build may have failed silently", "ERROR")
            return False
        
        # Count tile files
        tile_files = []
        for root, dirs, files in os.walk(tile_dir):
            tile_files.extend([f for f in files if f.endswith('.gph')])
        
        if len(tile_files) == 0:
            log(f"ERROR: No .gph tile files found in {tile_dir}", "ERROR")
            log("Valhalla tiles were not generated", "ERROR")
            return False
        
        log(f"Found {len(tile_files)} tile files to archive", "INFO")
        
        # Calculate total tile size
        total_size = 0
        for root, dirs, files in os.walk(tile_dir):
            for file in files:
                fp = os.path.join(root, file)
                try:
                    total_size += os.path.getsize(fp)
                except:
                    pass
        
        log(f"Total tile data size: {total_size / (1024**2):.2f} MB", "INFO")
        
        # Create manifest for valhalla_tiles directory
        log("Creating Valhalla tiles manifest...", "INFO")
        manifest = create_manifest_for_directory(tile_dir, "valhalla_tiles_manifest")
        
        if manifest:
            manifest_file = os.path.join(tile_dir, "manifest.json")
            with open(manifest_file, 'w') as f:
                json.dump(manifest, f, indent=2)
            
            log(f"✓ Manifest created with {len(manifest['files'])} files", "SUCCESS")
        else:
            log("Warning: Could not create manifest", "WARN")
        
        # Create compressed tar with lz4 (includes manifest)
        log("Compressing tar archive with lz4 (this may take several minutes)...", "INFO")
        
        # Create uncompressed tar first
        temp_tar = os.path.join(WORK_DIR, "valhalla_tiles.tar")
        def _sanitize_tarinfo(tarinfo):
            # Ensure group/other write bits are cleared without breaking TarInfo contract
            tarinfo.mode = tarinfo.mode & ~0o022
            return tarinfo

        with tarfile.open(temp_tar, "w") as tar:
            tar.add(tile_dir, arcname="valhalla_tiles", filter=_sanitize_tarinfo)
        
        # Compress with lz4
        result = subprocess.run(
            ["lz4", "-9", "-f", temp_tar, tar_path],
            capture_output=True,
            text=True
        )
        
        # Clean up temp tar
        if os.path.exists(temp_tar):
            os.remove(temp_tar)
        
        if result.returncode != 0:
            log(f"lz4 compression failed: {result.stderr}", "ERROR")
            return False
        
        if os.path.exists(tar_path) and os.path.getsize(tar_path) > 0:
            tar_size_mb = os.path.getsize(tar_path) / (1024**2)
            log(f"✓ Created tar archive: {tar_path} ({tar_size_mb:.2f} MB)", "SUCCESS")
            log(f"  Contains {len(tile_files)} .gph tiles", "INFO")
            if manifest:
                log(f"  Includes manifest.json with {len(manifest['files'])} file hashes", "INFO")
        else:
            log(f"ERROR: Tar archive created but is empty or missing", "ERROR")
            log(f"  Expected: {tar_path}", "ERROR")
            tar_path = None
            return False
    except Exception as e:
        log(f"Failed to create tar archive: {e}", "ERROR")
        tb = traceback.format_exc()
        log_terminal(tb, error=True)
        tar_path = None
    
    # Copy config and databases, calculate their MD5s
    log("Copying Valhalla supporting files...", "INFO")
    
    if os.path.exists(config_path):
        config_dest = os.path.join(WORK_DIR, "valhalla.json")
        shutil.copy(config_path, config_dest)
        save_single_file_md5(config_dest)
    else:
        log(f"Warning: config_path not found: {config_path}", "WARN")
    
    if os.path.exists(os.path.join(VALHALLA_DIR, "admin.sqlite")):
        admin_dest = os.path.join(WORK_DIR, "admin.sqlite")
        shutil.copy(os.path.join(VALHALLA_DIR, "admin.sqlite"), admin_dest)
        save_single_file_md5(admin_dest)
    
    if os.path.exists(os.path.join(VALHALLA_DIR, "timezones.sqlite")):
        tz_dest = os.path.join(WORK_DIR, "timezones.sqlite")
        shutil.copy(os.path.join(VALHALLA_DIR, "timezones.sqlite"), tz_dest)
        save_single_file_md5(tz_dest)
    
    # Calculate MD5 for the tar file itself
    if tar_path and os.path.exists(tar_path):
        save_single_file_md5(tar_path)
    # Ensure any throwaway tile extract is removed
    try:
        if 'skip_tile_extract' in locals() and os.path.exists(skip_tile_extract):
            os.remove(skip_tile_extract)
    except Exception:
        pass
    
    update_step_status("valhalla")
    return tar_path

# --- Firmware Build ---

def build_firmware():
    """Build ESP32 firmware using git pull + pio run and copy to public_export."""
    log("=" * 50, "HEADER")
    log("BUILDING ESP32 FIRMWARE", "HEADER")
    log("=" * 50, "HEADER")
    
    FIRMWARE_SOURCE_DIR = os.getenv('FIRMWARE_SOURCE_DIR', "/home/navigator/GPSnav")
    FIRMWARE_SOURCE_PATH = os.path.join(FIRMWARE_SOURCE_DIR, ".pio/build/esp-wrover-kit/firmware.bin")
    
    try:
        # Check if firmware source directory exists
        if not os.path.exists(FIRMWARE_SOURCE_DIR):
            log(f"Firmware source directory not found: {FIRMWARE_SOURCE_DIR}", "WARN")
            log("Skipping firmware build", "INFO")
            return True
        
        # Change to firmware directory
        original_dir = os.getcwd()
        os.chdir(FIRMWARE_SOURCE_DIR)
        log(f"Changed to: {FIRMWARE_SOURCE_DIR}", "INFO")
        
        # Git pull
        log("Running: git pull origin main", "INFO")
        log_terminal("$ cd " + FIRMWARE_SOURCE_DIR, command=True)
        log_terminal("$ git pull origin main", command=True)
        result = subprocess.run(
            ["git", "pull", "origin", "main"],
            capture_output=True,
            text=True,
            timeout=60
        )
        
        if result.returncode == 0:
            log(f"✓ Git pull successful", "SUCCESS")
            if result.stdout:
                log_terminal(result.stdout[:200], command=True)
        else:
            log(f"⚠ Git pull failed: {result.stderr[:100]}", "WARN")
            log("Continuing with firmware build anyway...", "INFO")
        
        # Build firmware with PlatformIO
        log("Running: pio run", "INFO")
        log("(This may take a few minutes...)", "INFO")
        log_terminal("$ pio run", command=True)
        
        result = subprocess.run(
            ["pio", "run"],
            capture_output=True,
            text=True,
            timeout=600  # 10 minute timeout
        )
        
        if result.returncode != 0:
            log(f"❌ PlatformIO build failed", "ERROR")
            log_terminal(result.stderr[:500] if result.stderr else "Unknown error", error=True)
            os.chdir(original_dir)
            return False
        
        log("✓ Firmware build successful!", "SUCCESS")
        
        # Return to original directory
        os.chdir(original_dir)
        
        # Check if firmware file exists
        if not os.path.exists(FIRMWARE_SOURCE_PATH):
            log(f"❌ Firmware file not found after build: {FIRMWARE_SOURCE_PATH}", "ERROR")
            return False
        
        # Get firmware info
        firmware_size = os.path.getsize(FIRMWARE_SOURCE_PATH)
        log(f"Firmware size: {firmware_size / (1024**2):.2f} MB", "INFO")
        
        # Calculate MD5
        log("Calculating firmware MD5...", "INFO")
        md5_hash = calculate_file_md5(FIRMWARE_SOURCE_PATH)
        if not md5_hash:
            log("Failed to calculate firmware MD5", "ERROR")
            return False
        
        log(f"✓ Firmware MD5: {md5_hash}", "SUCCESS")
        
        # Copy firmware to public_export
        firmware_public_dir = os.path.join(PUBLIC_DIR, "esp32wrover")
        os.makedirs(firmware_public_dir, exist_ok=True)
        
        firmware_public_path = os.path.join(firmware_public_dir, "firmware.bin")
        log(f"Copying firmware to public_export...", "INFO")
        shutil.copy2(FIRMWARE_SOURCE_PATH, firmware_public_path)
        
        # Write MD5 sidecar file
        save_single_file_md5(firmware_public_path)
        log(f"✓ Firmware published to: {firmware_public_path}", "SUCCESS")
        
        log("=" * 50, "HEADER")
        log("FIRMWARE BUILD COMPLETED", "SUCCESS")
        log("=" * 50, "HEADER")
        
        return True
    
    except subprocess.TimeoutExpired:
        log("❌ Firmware build timed out (>10 min)", "ERROR")
        try:
            os.chdir(original_dir)
        except:
            pass
        return False
    except Exception as e:
        log(f"❌ Error building firmware: {e}", "ERROR")
        log_terminal(str(e)[:500], error=True)
        try:
            os.chdir(original_dir)
        except:
            pass
        return False

# --- Main Pipeline ---

def process_pipeline(target_md5, start_from_step=None):
    """Main pipeline with MAX batch size always.
    
    Args:
        target_md5: MD5 hash of the source PBF file
        start_from_step: Optional step to resume from ('poi', 'filter', 'extract', 'india', 'valhalla', 'publish')
    """
    start_time = time.time()
    stats["pipeline_runs"] += 1
    
    try:
        init_workspace(target_md5)
        
        # Backdoor: skip to specific step if requested
        skip_to_step = start_from_step

        # 0. POI Database - only regenerate when source PBF changes
        if skip_to_step and skip_to_step != 'poi':
            log(f"⏭️  Skipping POI (backdoor: starting from {skip_to_step})", "WARN")
        else:
            log("=" * 50, "HEADER")
            log("STEP 1: POI Database Generation", "HEADER")
            log("=" * 50, "HEADER")
        poi_db_path = reuse_existing_poi_if_current(target_md5)
        if poi_db_path:
            log("Skipping POI generation (unchanged source)", "SUCCESS")
        else:
            poi_db_path = generate_poi_database(INDIA_PBF_FILE)
            if poi_db_path and os.path.exists(poi_db_path):
                save_single_file_md5(poi_db_path)
                record_poi_source(target_md5)
                log("✓ POI database ready for publishing", "SUCCESS")
            else:
                log("✗ POI database generation failed", "ERROR")
            if skip_to_step == 'poi':
                skip_to_step = None  # Completed requested step, continue normally

        # 1. Filter
        if skip_to_step and skip_to_step not in ['poi', 'filter']:
            log(f"⏭️  Skipping filter (backdoor: starting from {skip_to_step})", "WARN")
        else:
            log("=" * 50, "HEADER")
            log("STEP 2: PBF Filtering", "HEADER")
            log("=" * 50, "HEADER")
        filtered_pbf = os.path.join(WORK_DIR, "filtered.osm.pbf")
        if not is_step_done("filter"):
            log("Filtering PBF...", "HEADER")
            
            tags_list = [
                "nwr/natural", "nwr/landuse", "nwr/landcover", "nwr/leisure", "nwr/waterway",
                "nwr/place=sea", "nwr/railway", "nwr/aeroway", "nwr/amenity", "nwr/shop", "nwr/place", "nwr/highway"
            ]
            
            cmd_filter = ["osmium", "tags-filter", INDIA_PBF_FILE, "-o", filtered_pbf, "--overwrite"] + tags_list
            
            task_id = job_progress.add_task("Filtering...", total=None)
            try:
                if run_cmd(cmd_filter, "Filtering", show_output=True): 
                    update_step_status("filter")
                else: 
                    filtered_pbf = INDIA_PBF_FILE
            finally:
                job_progress.remove_task(task_id)
            if skip_to_step == 'filter':
                skip_to_step = None
        else:
            if not os.path.exists(filtered_pbf): filtered_pbf = INDIA_PBF_FILE

        # 2. Extract
        if skip_to_step and skip_to_step not in ['poi', 'filter', 'extract']:
            log(f"⏭️  Skipping extraction (backdoor: starting from {skip_to_step})", "WARN")
        else:
            log("=" * 50, "HEADER")
            log("STEP 3: Tile Extraction", "HEADER")
            log("=" * 50, "HEADER")
        tm_config_mobile = os.path.join(CONFIG_DIR, "config-mobile.json")
        tm_config_nav = os.path.join(CONFIG_DIR, "config-navigator.json")
        tm_process = os.path.join(CONFIG_DIR, "process-openmaptiles.lua")
        tm_process_app = os.path.join(CONFIG_DIR, "process-openmapphone.lua")        
        if not os.path.exists(tm_config_mobile):
            log("Missing config-mobile.json", "ERROR")
            return False
        if not os.path.exists(tm_config_nav): 
            tm_config_nav = tm_config_mobile

        # Check if extraction already complete (from rescan)
        current_state = load_state()
        if current_state.get("extraction_complete"):
            log("=" * 50, "HEADER")
            log("SKIPPING EXTRACTION - Already Complete", "SUCCESS")
            log("=" * 50, "HEADER")
            
            # Verify MBTiles exist
            mbtiles_dir = os.path.join(WORK_DIR, "mbtiles")
            if os.path.exists(mbtiles_dir):
                mbtiles_files = [f for f in os.listdir(mbtiles_dir) if f.endswith(".mbtiles")]
                log(f"✓ Using existing {len(mbtiles_files)} MBTiles files", "SUCCESS")
            else:
                log("⚠ MBTiles directory missing - will re-extract", "WARN")
                current_state["extraction_complete"] = False
                save_state(current_state)
        
        # Only do extraction if not already complete
        if not current_state.get("extraction_complete"):
            coords = generate_coordinates(INDIA_BBOX)
            log(f"Total: {len(coords)} tiles", "INFO")
            failed = process_adaptive_batch(filtered_pbf, coords, os.path.join(WORK_DIR, "extracts"))
            
            if failed:
                log(f"{len(failed)} tiles failed", "WARN")

            # 3. MBTiles (PARALLEL)
            pbf_files = [f for f in os.listdir(os.path.join(WORK_DIR, "extracts")) if f.endswith(".pbf")]
            mbtiles_dir = os.path.join(WORK_DIR, "mbtiles")
            
            if pbf_files:
                parallel_mbtiles_conversion(
                    pbf_files, 
                    mbtiles_dir, 
                    os.path.join(WORK_DIR, "extracts"),
                    tm_config_nav,
                    tm_process
                )
            if skip_to_step == 'extract':
                skip_to_step = None
        else:
            log("Extraction skipped - using cached files", "SUCCESS")
                
        # 4. Individual 1x1 MBTiles for Navigator (using navigator config)
        # Skip monolithic india.mbtiles - use individual tiles instead
        if skip_to_step and skip_to_step not in ['poi', 'filter', 'extract', 'india']:
            log(f"⏭️  Skipping to {skip_to_step}", "WARN")
        
        log("Skipping monolithic india.mbtiles - using individual 1x1 tiles", "INFO")
        update_step_status("india_mbtiles")  # Mark as done (using individual tiles instead)
        
        # The main MBTiles directory (mbtiles/) already has individual tiles with navigator config
        mbtiles_dir = os.path.join(WORK_DIR, "mbtiles")
        if os.path.exists(mbtiles_dir):
            nav_files = [f for f in os.listdir(mbtiles_dir) if f.endswith(".mbtiles")]
            log(f"✓ Navigator MBTiles: {len(nav_files)} individual tiles ready", "SUCCESS")
        
        # 4b. Mobile MBTiles - Create individual 1x1 tiles with mobile config
        if not is_step_done("mobile_mbtiles"):
            log("Building mobile MBTiles (1x1 tiles with mobile config)...", "INFO")
            
            extracts_dir = os.path.join(WORK_DIR, "extracts")
            if os.path.exists(extracts_dir):
                pbf_files = [f for f in os.listdir(extracts_dir) if f.endswith(".osm.pbf")]
                
                if pbf_files:
                    log(f"Found {len(pbf_files)} extract PBFs for mobile conversion", "INFO")
                    
                    mobile_mbtiles_dir = os.path.join(WORK_DIR, "mbtiles_mobile")
                    os.makedirs(mobile_mbtiles_dir, exist_ok=True)
                    
                    failed = parallel_mbtiles_conversion(
                        pbf_files,
                        mobile_mbtiles_dir,
                        extracts_dir,
                        tm_config_mobile,
                        tm_process_app
                    )
                    
                    if failed:
                        log(f"Mobile conversion: {len(failed)} tiles failed", "WARN")
                    else:
                        log(f"Mobile conversion: All {len(pbf_files)} tiles successful", "SUCCESS")
                    
                    # Mark mobile MBTiles step complete (index will be created after publishing)
                    if os.path.exists(mobile_mbtiles_dir):
                        mobile_files = [f for f in os.listdir(mobile_mbtiles_dir) if f.endswith(".mbtiles")]
                        log(f"✓ Mobile MBTiles: {len(mobile_files)} individual tiles ready", "SUCCESS")
                        update_step_status("mobile_mbtiles")
                else:
                    log("No PBF extracts found for mobile conversion", "WARN")
            else:
                log("Extracts directory not found for mobile conversion", "WARN")
        
        # Navigator tiles index will be created after publishing compressed tiles

        # 5. Valhalla - Updated commands (using unfiltered PBF)
        if skip_to_step and skip_to_step not in ['poi', 'filter', 'extract', 'india', 'valhalla']:
            log(f"⏭️  Skipping Valhalla (backdoor: starting from {skip_to_step})", "WARN")
            valhalla_tar = None
        else:
            valhalla_tar = build_valhalla_graph(INDIA_PBF_FILE)
            if valhalla_tar and os.path.exists(valhalla_tar): 
                save_single_file_md5(valhalla_tar)
            if skip_to_step == 'valhalla':
                skip_to_step = None

        # 6. Publish - Always run (or start here with backdoor)
        if skip_to_step == 'publish':
            log("⏩ Starting from publish step (backdoor)", "WARN")
            skip_to_step = None
        
        log("Publishing files...", "INFO")
        if os.path.exists(PUBLIC_DIR):
            try:
                for f in os.listdir(PUBLIC_DIR):
                    path = os.path.join(PUBLIC_DIR, f)
                    if os.path.isdir(path):
                        safe_rmtree(path)
                    else:
                        try:
                            os.remove(path)
                        except OSError as e:
                            log(f"Failed to remove {path}: {e}", "WARN")
            except Exception as e:
                log(f"Error clearing PUBLIC_DIR: {e}", "WARN")
        else:
            os.makedirs(PUBLIC_DIR, exist_ok=True)

        # Move and compress POI database if it exists
        poi_db = os.path.join(WORK_DIR, "poi.sqlite")
        if os.path.exists(poi_db):
            try:
                poi_dest = os.path.join(PUBLIC_DIR, "poi.sqlite")
                shutil.move(poi_db, poi_dest)
                # Compress and replace original
                compressed = compress_file_with_lz4(poi_dest, keep_original=False)
                if compressed:
                    save_single_file_md5(compressed)
                    log("✓ POI database compressed and published", "SUCCESS")
                else:
                    log("✓ POI database published (compression failed)", "SUCCESS")
            except Exception as e:
                log(f"Failed to publish POI database: {e}", "WARN")

        # Note: india.mbtiles no longer generated (using individual 1x1 tiles instead)
        
        # Export Navigator MBTiles directory with compression
        nav_mbtiles_src = os.path.join(WORK_DIR, "mbtiles")
        if os.path.exists(nav_mbtiles_src):
            nav_mbtiles_public = os.path.join(PUBLIC_DIR, "mbtiles_navigator")
            try:
                if os.path.exists(nav_mbtiles_public):
                    safe_rmtree(nav_mbtiles_public)
                os.makedirs(nav_mbtiles_public, exist_ok=True)
                
                # Copy, compress and create MD5 for each navigator tile
                nav_files = [f for f in os.listdir(nav_mbtiles_src) if f.endswith(".mbtiles")]
                log(f"Publishing {len(nav_files)} Navigator MBTiles with compression...", "INFO")
                
                task_id = job_progress.add_task("[cyan]Publishing Navigator tiles...", total=len(nav_files))
                try:
                    for mbtiles_file in nav_files:
                        src = os.path.join(nav_mbtiles_src, mbtiles_file)
                        dest = os.path.join(nav_mbtiles_public, mbtiles_file)
                        
                        # Copy file
                        shutil.copy2(src, dest)

                        # Calculate MD5 of the uncompressed MAP content and save sidecar
                        try:
                            tile_name = mbtiles_file[:-len(".mbtiles")] if mbtiles_file.endswith(".mbtiles") else os.path.splitext(mbtiles_file)[0]
                            map_md5 = calculate_file_md5(dest)
                            if map_md5:
                                uncompressed_md5_path = os.path.join(nav_mbtiles_public, f"{tile_name}.mbtiles.md5")
                                with open(uncompressed_md5_path, "w") as f:
                                    f.write(f"{map_md5}  {tile_name}.mbtiles\n")
                        except Exception as e:
                            log(f"Failed to write uncompressed MD5 for {mbtiles_file}: {e}", "WARN")

                        # Compress and replace with .lz4 version
                        compressed = compress_file_with_lz4(dest, keep_original=False)
                        if compressed:
                            save_single_file_md5(compressed)
                        
                        job_progress.update(task_id, advance=1)
                finally:
                    job_progress.remove_task(task_id)
                
                # Create tiles index JSON for navigator tiles (compressed files in public_export)
                nav_index = create_tiles_index_json(nav_mbtiles_public, "tiles_index.json")
                if nav_index:
                    log(f"✓ Navigator tiles_index.json created: {nav_index}", "SUCCESS")
                
                log(f"✓ Navigator MBTiles: {len(nav_files)} tiles → {nav_mbtiles_public}", "SUCCESS")
            except Exception as e:
                log(f"Failed to export Navigator MBTiles: {e}", "ERROR")
        
        # Export Mobile/App MBTiles directory with compression
        mobile_mbtiles_src = os.path.join(WORK_DIR, "mbtiles_mobile")
        if os.path.exists(mobile_mbtiles_src):
            mobile_mbtiles_public = os.path.join(PUBLIC_DIR, "mbtiles_mobile")
            try:
                if os.path.exists(mobile_mbtiles_public):
                    safe_rmtree(mobile_mbtiles_public)
                os.makedirs(mobile_mbtiles_public, exist_ok=True)
                
                # Copy, compress and create MD5 for each mobile tile
                mobile_files = [f for f in os.listdir(mobile_mbtiles_src) if f.endswith(".mbtiles")]
                log(f"Publishing {len(mobile_files)} Mobile/App MBTiles with compression...", "INFO")
                
                task_id = job_progress.add_task("[cyan]Publishing Mobile tiles...", total=len(mobile_files))
                try:
                    for mbtiles_file in mobile_files:
                        src = os.path.join(mobile_mbtiles_src, mbtiles_file)
                        dest = os.path.join(mobile_mbtiles_public, mbtiles_file)
                        
                        # Copy file
                        shutil.copy2(src, dest)

                        # Calculate MD5 of the uncompressed MAP content and save sidecar
                        try:
                            tile_name = mbtiles_file[:-len(".mbtiles")] if mbtiles_file.endswith(".mbtiles") else os.path.splitext(mbtiles_file)[0]
                            map_md5 = calculate_file_md5(dest)
                            if map_md5:
                                uncompressed_md5_path = os.path.join(mobile_mbtiles_public, f"{tile_name}.mbtiles.md5")
                                with open(uncompressed_md5_path, "w") as f:
                                    f.write(f"{map_md5}  {tile_name}.mbtiles\n")
                        except Exception as e:
                            log(f"Failed to write uncompressed MD5 for {mbtiles_file}: {e}", "WARN")

                        # Compress and replace with .lz4 version
                        compressed = compress_file_with_lz4(dest, keep_original=False)
                        if compressed:
                            save_single_file_md5(compressed)
                        
                        job_progress.update(task_id, advance=1)
                finally:
                    job_progress.remove_task(task_id)
                
                # Create tiles index JSON for mobile tiles (compressed files in public_export)
                mobile_index = create_tiles_index_json(mobile_mbtiles_public, "tiles_index.json")
                if mobile_index:
                    log(f"✓ Mobile tiles_index.json created: {mobile_index}", "SUCCESS")
                
                log(f"✓ Mobile MBTiles: {len(mobile_files)} tiles → {mobile_mbtiles_public}", "SUCCESS")
            except Exception as e:
                log(f"Failed to export Mobile MBTiles: {e}", "ERROR")

        # Move Valhalla directory (exclude tiles.tar) and publish archive tar.gz
        valhalla_public = os.path.join(PUBLIC_DIR, "valhalla")
        valhalla_tar_lz4 = os.path.join(WORK_DIR, "valhalla_tiles.tar.lz4")
        valhalla_tar_md5 = valhalla_tar_lz4 + ".md5"

        # Ensure valhalla public directory exists
        os.makedirs(valhalla_public, exist_ok=True)

        # Ensure tar exists; if missing but tiles exist, rebuild it
        if not os.path.exists(valhalla_tar_lz4):
            log("valhalla_tiles.tar.lz4 missing; attempting rebuild from tiles", "WARN")
            rebuilt_tar = ensure_valhalla_tar_from_tiles(os.path.join(VALHALLA_DIR, "valhalla_tiles"), WORK_DIR)
            if rebuilt_tar:
                valhalla_tar_lz4 = rebuilt_tar
                valhalla_tar_md5 = valhalla_tar_lz4 + ".md5"
            else:
                log("Could not rebuild valhalla tar; skipping copy", "ERROR")

        # Always copy the compressed Valhalla tiles archive first
        try:
            if os.path.exists(valhalla_tar_lz4):
                shutil.copy2(valhalla_tar_lz4, os.path.join(valhalla_public, "valhalla_tiles.tar.lz4"))
                log("✓ Valhalla tar.lz4 copied to public_export", "SUCCESS")
                if os.path.exists(valhalla_tar_md5):
                    shutil.copy2(valhalla_tar_md5, os.path.join(valhalla_public, "valhalla_tiles.tar.lz4.md5"))
                    log("✓ Valhalla tar.lz4 MD5 copied", "SUCCESS")
                
                # Copy manifest if it exists in valhalla_tiles directory
                valhalla_tiles_dir = os.path.join(VALHALLA_DIR, "valhalla_tiles")
                manifest_src = os.path.join(valhalla_tiles_dir, "manifest.json")
                if os.path.exists(manifest_src):
                    manifest_dst = os.path.join(valhalla_public, "valhalla_tiles_manifest.json")
                    shutil.copy2(manifest_src, manifest_dst)
                    log("✓ Valhalla tiles manifest copied", "SUCCESS")
            else:
                log("Valhalla tar.lz4 not found; skipping", "WARN")
        except Exception as e:
            log(f"Failed to publish valhalla_tiles.tar.lz4: {e}", "WARN")

        # Move Valhalla directory contents (if exists)
        if os.path.exists(VALHALLA_DIR):
            try:
                # Remove tiles.tar if it exists (not needed)
                tiles_tar = os.path.join(VALHALLA_DIR, "tiles.tar")
                if os.path.exists(tiles_tar):
                    try:
                        os.remove(tiles_tar)
                        log("Removed tiles.tar (not needed)", "INFO")
                    except OSError as e:
                        log(f"Warning: could not remove tiles.tar: {e}", "WARN")

                # Copy other Valhalla files (config, databases, tiles directory, etc.)
                for item in os.listdir(VALHALLA_DIR):
                    src = os.path.join(VALHALLA_DIR, item)
                    dst = os.path.join(valhalla_public, item)
                    try:
                        if os.path.isfile(src):
                            shutil.copy2(src, dst)
                        elif os.path.isdir(src):
                            if os.path.exists(dst):
                                shutil.rmtree(dst)
                            shutil.copytree(src, dst)
                    except Exception as e:
                        log(f"Failed to copy {item}: {e}", "WARN")
                
                log("✓ Valhalla directory contents published", "SUCCESS")
            except Exception as e:
                log(f"Failed to publish Valhalla directory: {e}", "WARN")

        cleanup_small_files(PUBLIC_DIR)
        
        # Build and publish firmware AFTER all other files are published
        log("=" * 50, "HEADER")
        log("Building firmware for deployment...", "HEADER")
        log("=" * 50, "HEADER")
        if not build_firmware():
            log("⚠ Firmware build failed, continuing with deployment", "WARN")
        
        # Calculate MD5 for all files in WORK_DIR before publishing
        log("=" * 50, "HEADER")
        log("Calculating MD5 for all generated files...", "INFO")
        log("=" * 50, "HEADER")
        md5_count = calculate_all_md5_in_directory(WORK_DIR)
        log(f"✓ Created {md5_count} MD5 files", "SUCCESS")
        
        generate_metadata_json(PUBLIC_DIR, target_md5)
        
        # Update last successful run timestamp BEFORE cleanup
        update_last_run_timestamp()
        
        # Create deployment marker for server blue-green deployment
        try:
            marker_file = "/home/navigator/completed.done"
            with open(marker_file, 'w') as f:
                f.write(datetime.now().isoformat())
            log(f"✓ Created deployment marker: {marker_file}", "SUCCESS")
        except Exception as e:
            log(f"Warning: Failed to create deployment marker: {e}", "WARN")
        
        safe_rmtree(WORK_DIR)
        
        # Update stats
        duration = time.time() - start_time
        stats["successful_runs"] += 1
        stats["last_run_duration"] = duration
        stats["average_run_duration"] = (
            (stats["average_run_duration"] * (stats["successful_runs"] - 1) + duration) / stats["successful_runs"]
        )
        
        log(f"Completed in {duration/3600:.1f}h", "SUCCESS")
        save_statistics()
        save_checkpoint()
        save_batch_history()
        
        return True
        
    except Exception as e:
        stats["failed_runs"] += 1
        stats["last_error"] = str(e)
        log(f"Pipeline failed: {e}", "ERROR")
        save_statistics()
        save_checkpoint()
        return False

# --- Main Loop ---

def clear_all_files():
    """Cleans up all generated content including learned data."""
    log("Full cleanup...", "WARN")
    dirs_to_clean = [DOWNLOAD_DIR, WORK_DIR, PUBLIC_DIR, VALHALLA_DIR]
    for d in dirs_to_clean:
        if os.path.exists(d):
            safe_rmtree(d)
        os.makedirs(d, exist_ok=True)
    for f in [LOG_FILE, STATS_FILE, BATCH_HISTORY_FILE, CHECKPOINT_FILE, OOM_HISTORY_FILE, STATE_FILE]:
        if os.path.exists(f): 
            try:
                os.remove(f)
            except OSError:
                pass
    log("Cleanup done - all learned data reset", "SUCCESS")

def collect_and_export_all_files():
    """
    Collect all files ever created/worked on and copy them to public_export.
    Uses copy (not move) to avoid breaking the pipeline.
    """
    files_to_export = []
    
    # Directories to scan
    scan_dirs = [
        DOWNLOAD_DIR,
        WORK_DIR,
        CONFIG_DIR,
        VALHALLA_DIR,
        BASE_DIR
    ]
    
    log("=" * 50, "HEADER")
    log("COLLECTING ALL PIPELINE FILES", "HEADER")
    log("=" * 50, "HEADER")
    
    for scan_dir in scan_dirs:
        if not os.path.exists(scan_dir):
            log(f"Scan directory does not exist: {scan_dir}", "INFO")
            continue
        
        try:
            for root, dirs, files in os.walk(scan_dir):
                # Skip public_export and hidden dirs
                dirs[:] = [d for d in dirs if d != 'public_export' and not d.startswith('.')]
                
                for file in files:
                    file_path = os.path.join(root, file)
                    # Skip certain files and extensions
                    if any(file.endswith(ext) for ext in ['.pyc', '.log', '.tmp', '.swp', '.bak', '.lock']):
                        continue
                    if any(skip in file_path for skip in ['__pycache__', '.git', '.venv']):
                        continue
                    
                    try:
                        if os.path.isfile(file_path) and not os.path.islink(file_path):
                            files_to_export.append(file_path)
                            log(f"Found: {file_path}", "INFO")
                    except OSError as e:
                        log(f"Error scanning {file_path}: {e}", "WARN")
                        pass
        except Exception as e:
            log(f"Error scanning directory {scan_dir}: {e}", "WARN")
            continue
    
    # Deduplicate by basename (in case files exist in multiple places)
    seen_basenames = set()
    unique_files = []
    for f in files_to_export:
        basename = os.path.basename(f)
        if basename not in seen_basenames:
            unique_files.append(f)
            seen_basenames.add(basename)
    
    log(f"Collected {len(unique_files)} unique files", "SUCCESS")
    
    # Create public_export directory
    if not os.path.exists(PUBLIC_DIR):
        try:
            os.makedirs(PUBLIC_DIR, exist_ok=True)
            log(f"Created export directory: {PUBLIC_DIR}", "INFO")
        except Exception as e:
            log(f"Failed to create export directory: {e}", "ERROR")
            raise
    
    moved_count = 0
    skipped_count = 0
    error_count = 0
    
    for file_path in unique_files:
        try:
            basename = os.path.basename(file_path)
            dest_path = os.path.join(PUBLIC_DIR, basename)
            
            # Skip if already exists in public_export and is the same file
            if os.path.exists(dest_path):
                try:
                    if os.path.samefile(file_path, dest_path):
                        log(f"Skip (same file): {basename}", "INFO")
                        skipped_count += 1
                        continue
                except OSError:
                    pass  # Files don't exist or can't compare, proceed with copy
            
            # Copy file (not move) to avoid breaking the pipeline
            log(f"Copying: {basename} to {dest_path}", "INFO")
            shutil.copy2(file_path, dest_path)
            log(f"✓ Copied: {basename}", "SUCCESS")
            moved_count += 1
            
        except Exception as e:
            log(f"Failed to copy {file_path}: {e}", "ERROR")
            log(f"  Exception: {traceback.format_exc()}", "DEBUG")
            error_count += 1
    
    log("=" * 50, "HEADER")
    log(f"EXPORT COMPLETE: {moved_count} files copied", "SUCCESS")
    log(f"Skipped: {skipped_count} files (already in place)", "INFO")
    log(f"Errors: {error_count} files failed to copy", "INFO")
    log("=" * 50, "HEADER")
    
    return moved_count

def rescan_and_mark_complete():
    """
    Scan work directory and mark steps as complete based on existing files.
    This allows resuming from Valhalla step if previous steps are already done.
    """
    log("=" * 50, "HEADER")
    log("RESCANNING EXISTING FILES", "HEADER")
    log("=" * 50, "HEADER")
    
    state = load_state()
    
    # Check filtered PBF
    filtered_pbf = os.path.join(WORK_DIR, "filtered.osm.pbf")
    if os.path.exists(filtered_pbf) and os.path.getsize(filtered_pbf) > MIN_FILE_SIZE_BYTES:
        log(f"✓ Found filtered.osm.pbf ({os.path.getsize(filtered_pbf) / (1024**3):.2f} GB)", "SUCCESS")
        state["filter"] = True
    else:
        log("✗ No filtered.osm.pbf found", "WARN")
        state["filter"] = False
    
    # Check extracts
    extracts_dir = os.path.join(WORK_DIR, "extracts")
    extracts_exist = False
    pbf_count = 0
    if os.path.exists(extracts_dir):
        pbf_files = [f for f in os.listdir(extracts_dir) if f.endswith(".osm.pbf")]
        pbf_count = len(pbf_files)
        if pbf_files:
            log(f"✓ Found {len(pbf_files)} extract PBF files", "SUCCESS")
            extracts_exist = True
        else:
            log("✗ No extract PBF files found", "WARN")
    else:
        log("✗ Extracts directory not found", "WARN")
    
    # Check MBTiles
    mbtiles_dir = os.path.join(WORK_DIR, "mbtiles")
    mbtiles_count = 0
    mbtiles_exist = False
    if os.path.exists(mbtiles_dir):
        mbtiles_files = [f for f in os.listdir(mbtiles_dir) if f.endswith(".mbtiles")]
        mbtiles_count = len(mbtiles_files)
        if mbtiles_files:
            log(f"✓ Found {len(mbtiles_files)} MBTiles files", "SUCCESS")
            mbtiles_exist = True
        else:
            log("✗ No MBTiles files found", "WARN")
    else:
        log("✗ MBTiles directory not found", "WARN")
    
    # Check India MBTiles
    india_mbtiles = os.path.join(WORK_DIR, "india.mbtiles")
    if os.path.exists(india_mbtiles) and os.path.getsize(india_mbtiles) > MIN_FILE_SIZE_BYTES:
        size_gb = os.path.getsize(india_mbtiles) / (1024**3)
        log(f"✓ Found india.mbtiles ({size_gb:.2f} GB)", "SUCCESS")
        state["india_mbtiles"] = True
    else:
        log("✗ No india.mbtiles found", "WARN")
        state["india_mbtiles"] = False
    
    # Check Valhalla tar.lz4
    valhalla_tar = os.path.join(WORK_DIR, "valhalla_tiles.tar.lz4")
    if os.path.exists(valhalla_tar) and os.path.getsize(valhalla_tar) > MIN_FILE_SIZE_BYTES:
        size_gb = os.path.getsize(valhalla_tar) / (1024**3)
        log(f"✓ Found valhalla_tiles.tar.lz4 ({size_gb:.2f} GB)", "SUCCESS")
        state["valhalla"] = True
    else:
        log("✗ No valhalla_tiles.tar.lz4 found", "WARN")
        state["valhalla"] = False
    
    # Mark extraction as complete if we have MBTiles (means extraction already done)
    if mbtiles_exist:
        state["extraction_complete"] = True
        log("✓ Marking extraction as COMPLETE (MBTiles exist)", "SUCCESS")
    else:
        state["extraction_complete"] = False
    
    # Save updated state
    save_state(state)
    
    log("=" * 50, "HEADER")
    log("RESCAN SUMMARY:", "HEADER")
    log(f"  Filter: {'✓ DONE' if state.get('filter') else '✗ TODO'}", "INFO")
    log(f"  Extraction: {'✓ SKIP (already done)' if state.get('extraction_complete') else '✗ TODO'}", "INFO")
    log(f"  India MBTiles: {'✓ DONE' if state.get('india_mbtiles') else '✗ TODO'}", "INFO")
    log(f"  Valhalla: {'✓ DONE' if state.get('valhalla') else '✗ TODO'}", "INFO")
    log(f"  Extract PBFs: {pbf_count} files", "INFO")
    log(f"  MBTiles: {mbtiles_count} files", "INFO")
    log("=" * 50, "HEADER")
    
    if mbtiles_exist and not state.get("valhalla"):
        log("🎯 READY: Will skip extraction and jump to Valhalla!", "SUCCESS")
    
    return state

def main_loop():
    parser = argparse.ArgumentParser()
    parser.add_argument("--clean", action="store_true", help="Clear all files")
    parser.add_argument("--rescan", action="store_true", help="Rescan existing files and continue from Valhalla")
    parser.add_argument("--export", action="store_true", help="Collect and export all files to public_export")
    parser.add_argument("--valhalla", action="store_true", help="Rebuild Valhalla only")
    parser.add_argument("--continue-from", type=str, choices=['poi', 'filter', 'extract', 'india', 'valhalla', 'publish'], 
                        help="Backdoor: continue pipeline from specific step")
    parser.add_argument("--fix-publish", action="store_true", help="Quick fix: copy missing files to public_export")
    parser.add_argument("--rebuild-tar", action="store_true", help="Rebuild valhalla_tiles.tar.lz4 from valhalla directory")
    args, unknown = parser.parse_known_args()

    # Handle rebuild-tar before UI initialization
    if args.rebuild_tar:
        print("\n" + "="*60)
        print("🔧 REBUILD-TAR: Creating valhalla_tiles.tar.lz4 from tiles")
        print("="*60 + "\n")
        
        import glob
        import tarfile
        import hashlib
        
        # Use the same VALHALLA_DIR as the pipeline to avoid path mismatch
        valhalla_dir = VALHALLA_DIR
        tile_dir = os.path.join(valhalla_dir, "valhalla_tiles")
        os.makedirs(WORK_DIR, exist_ok=True)
        tar_path = os.path.join(WORK_DIR, "valhalla_tiles.tar.lz4")
        
        print(f"Checking for tiles in: {tile_dir}")
        
        if not os.path.exists(tile_dir):
            print(f"✗ Tile directory not found: {tile_dir}")
            print(f"  Valhalla directory: {valhalla_dir} {'EXISTS' if os.path.exists(valhalla_dir) else 'NOT FOUND'}")
            print(f"  Cannot rebuild tar without tiles")
            sys.exit(1)
        
        # Count tiles
        tile_files = glob.glob(os.path.join(tile_dir, "**/*.gph"), recursive=True)
        print(f"Found {len(tile_files)} tile files in {tile_dir}")
        
        if len(tile_files) == 0:
            print(f"✗ No .gph tile files found in {tile_dir}")
            print(f"  Run pipeline to rebuild Valhalla tiles first")
            sys.exit(1)
        
        # Calculate total size (streaming over files)
        total_size = 0
        for f in tile_files:
            try:
                total_size += os.path.getsize(f)
            except OSError:
                pass
        print(f"Total tile size: {total_size / (1024**2):.2f} MB")
        
        # Create manifest
        print(f"\nCreating manifest for all {len(tile_files)} files...")
        
        manifest = {
            "created": datetime.now().isoformat(),
            "tile_count": len(tile_files),
            "total_size_bytes": total_size,
            "files": {}
        }
        
        # Hash all files for complete manifest
        for i, tf in enumerate(tile_files, 1):
            if i % 100 == 0:
                print(f"  Hashed {i}/{len(tile_files)} files...")
            rel_path = os.path.relpath(tf, tile_dir)
            # Stream hash to avoid loading file into memory
            h = hashlib.md5()
            with open(tf, 'rb') as fh:
                for chunk in iter(lambda: fh.read(8192), b""):
                    h.update(chunk)
            manifest["files"][rel_path] = {
                "size": os.path.getsize(tf),
                "md5": h.hexdigest()
            }
        
        manifest_file = os.path.join(tile_dir, "manifest.json")
        with open(manifest_file, 'w') as f:
            json.dump(manifest, f, indent=2)
        print(f"✓ Created complete manifest with {len(manifest['files'])} file hashes")
        
        # Create tar.lz4
        print(f"\nCompressing {len(tile_files)} tiles to tar.lz4...")
        print(f"This may take several minutes...")
        
        # Create uncompressed tar first
        temp_tar = os.path.join(WORK_DIR, "valhalla_tiles_temp.tar")
        with tarfile.open(temp_tar, "w") as tar:
            tar.add(tile_dir, arcname="valhalla_tiles")
        
        # Compress with lz4
        print(f"Compressing with lz4...")
        result = subprocess.run(
            ["lz4", "-9", "-f", temp_tar, tar_path],
            capture_output=True,
            text=True
        )
        
        # Clean up temp tar
        if os.path.exists(temp_tar):
            os.remove(temp_tar)
        
        if result.returncode != 0:
            print(f"✗ lz4 compression failed: {result.stderr}")
            sys.exit(1)
        
        if os.path.exists(tar_path):
            tar_size = os.path.getsize(tar_path)
            print(f"\n✓ Created: {tar_path}")
            print(f"  Size: {tar_size / (1024**2):.2f} MB")
            
            # Create MD5
            md5_hash = hashlib.md5()
            with open(tar_path, "rb") as f:
                for chunk in iter(lambda: f.read(8192), b""):
                    md5_hash.update(chunk)
            
            md5_file = tar_path + ".md5"
            with open(md5_file, "w") as f:
                f.write(f"{md5_hash.hexdigest()}  valhalla_tiles.tar.lz4\n")
            print(f"  ✓ MD5: {md5_hash.hexdigest()}")
            
            print(f"\nNow run: python3 pipeline.py --fix-publish")
        else:
            print(f"✗ Failed to create tar file")
            sys.exit(1)
        
        print("\n" + "="*60)
        sys.exit(0)

    # Handle fix-publish before UI initialization
    if args.fix_publish:
        print("\n" + "="*60)
        print("🔧 FIX-PUBLISH: Copying missing files to public_export")
        print("="*60 + "\n")
        
        valhalla_public = os.path.join(PUBLIC_DIR, "valhalla")
        print(f"Creating directory: {valhalla_public}")
        os.makedirs(valhalla_public, exist_ok=True)
        
        copied_count = 0
        
        # Copy valhalla_tiles.tar.lz4
        valhalla_tar_lz4 = os.path.join(WORK_DIR, "valhalla_tiles.tar.lz4")
        print(f"\nChecking: {valhalla_tar_lz4}")
        if os.path.exists(valhalla_tar_lz4):
            dest = os.path.join(valhalla_public, "valhalla_tiles.tar.lz4")
            print(f"  Source size: {os.path.getsize(valhalla_tar_lz4) / (1024*1024):.2f} MB")
            print(f"  Copying to: {dest}")
            shutil.copy2(valhalla_tar_lz4, dest)
            print(f"  ✓ Copied valhalla_tiles.tar.lz4")
            copied_count += 1
            
            valhalla_tar_md5 = valhalla_tar_lz4 + ".md5"
            if os.path.exists(valhalla_tar_md5):
                shutil.copy2(valhalla_tar_md5, os.path.join(valhalla_public, "valhalla_tiles.tar.lz4.md5"))
                print(f"  ✓ Copied valhalla_tiles.tar.lz4.md5")
                copied_count += 1
            
            # Copy manifest if it exists
            valhalla_dir = os.path.join(os.path.expanduser("~"), "valhalla")
            manifest_src = os.path.join(valhalla_dir, "valhalla_tiles", "manifest.json")
            if os.path.exists(manifest_src):
                manifest_dst = os.path.join(valhalla_public, "valhalla_tiles_manifest.json")
                shutil.copy2(manifest_src, manifest_dst)
                print(f"  ✓ Copied valhalla_tiles_manifest.json")
                copied_count += 1
        else:
            print(f"  ✗ NOT FOUND: {valhalla_tar_lz4}")
        
        # Copy and compress valhalla config files
        print("\nCopying and compressing config files:")
        for fname in ["valhalla.json", "admin.sqlite", "timezones.sqlite"]:
            src = os.path.join(WORK_DIR, fname)
            if os.path.exists(src):
                dest = os.path.join(valhalla_public, fname)
                shutil.copy2(src, dest)
                print(f"  ✓ Copied {fname}")
                copied_count += 1
                
                # Compress .sqlite files only
                if fname.endswith(".sqlite"):
                    print(f"  🔄 Compressing {fname}...")
                    result = subprocess.run(
                        ["lz4", "-9", "-f", dest, dest + ".lz4"],
                        capture_output=True,
                        text=True
                    )
                    if result.returncode == 0 and os.path.exists(dest + ".lz4"):
                        os.remove(dest)  # Remove uncompressed
                        if os.path.exists(dest + ".md5"):
                            os.remove(dest + ".md5")  # Remove old MD5
                        # Create MD5 for compressed file
                        subprocess.run(["md5sum", dest + ".lz4"], capture_output=True)
                        print(f"  ✓ Compressed to {fname}.lz4")
                        copied_count += 1
                    else:
                        print(f"  ⚠️  Compression failed for {fname}")
                else:
                    # For .json files, just copy MD5
                    md5_src = src + ".md5"
                    if os.path.exists(md5_src):
                        shutil.copy2(md5_src, os.path.join(valhalla_public, fname + ".md5"))
                        print(f"  ✓ Copied {fname}.md5")
                        copied_count += 1
            else:
                print(f"  - {fname} not found, skipping")
        
        print("\n" + "="*60)
        print(f"✓ Fix complete - copied {copied_count} files")
        print("="*60)
        print("\nVerify with:")
        print(f"  ls -lh {valhalla_public}/")
        print(f"  ls -lh {valhalla_public}/ | grep valhalla_tiles.tar.gz")
        print("\n")
        sys.exit(0)  # Explicit exit

    for d in [DOWNLOAD_DIR, CONFIG_DIR]: 
        os.makedirs(d, exist_ok=True)
    
    check_dependencies()
    load_statistics()
    load_checkpoint()
    load_batch_history()
    
    layout = generate_layout()
    # Reduce refresh rate to minimize flicker
    with Live(layout, refresh_per_second=10, screen=True) as live:

        def update_layout_loop():
            """Thread-safe layout updater - update only when content changes."""
            last_log = None
            last_terminal = None
            last_stats = None
            last_update_time = 0
            while not shutdown_event.is_set():
                try:
                    current_time = time.time()
                    # Skip update if less than 2 seconds since last update
                    if current_time - last_update_time < 2.0:
                        continue

                    # Build current content snapshots
                    log_text = "\n".join(log_deque)
                    terminal_text = "\n".join(terminal_deque) if terminal_deque else "[dim]No output...[/dim]"

                    sys_stats = get_system_stats()
                    stats_snapshot = (
                        int(sys_stats['cpu']),
                        int(sys_stats['memory_percent']),
                        int(sys_stats['memory_available_gb']),
                        stats['total_tiles_processed'],
                        stats['oom_kills'],
                        stats['oom_recoveries']
                    )

                    # Only update if something actually changed
                    if log_text == last_log and terminal_text == last_terminal and stats_snapshot == last_stats:
                        continue

                    # Only take layout lock and update panels when something changed
                    if log_text != last_log or terminal_text != last_terminal or stats_snapshot != last_stats:
                        with layout_lock:
                            if log_text != last_log:
                                layout["logs"].update(Panel(Text.from_markup(log_text), title=f"Logs (last {LOG_SCROLL_SIZE})", border_style="green"))
                                last_log = log_text

                            if terminal_text != last_terminal:
                                layout["terminal"].update(Panel(Text.from_markup(terminal_text), title=f"Terminal (last {TERMINAL_SCROLL_SIZE})", border_style="yellow"))
                                last_terminal = terminal_text

                            if stats_snapshot != last_stats:
                                stats_table = Table.grid(padding=(0, 1))
                                stats_table.add_row(
                                    f"[cyan]CPU:[/cyan] {sys_stats['cpu']:.1f}%",
                                    f"[cyan]Mem:[/cyan] {sys_stats['memory_used_gb']:.1f}GB ({sys_stats['memory_percent']:.1f}%)"
                                )
                                # Add swap row with color warning if >= 70%
                                swap_color = "[yellow]" if sys_stats['swap_percent'] >= 70 else "[cyan]"
                                swap_end_color = "[/yellow]" if sys_stats['swap_percent'] >= 70 else "[/cyan]"
                                stats_table.add_row(
                                    f"{swap_color}Swap:{swap_end_color} {sys_stats['swap_used_gb']:.1f}GB ({sys_stats['swap_percent']:.1f}%)",
                                    f"[cyan]Workers:[/cyan] {TILEMAKER_WORKERS}/{CPU_COUNT}"
                                )
                                stats_table.add_row(
                                    f"[green]Max Batch:[/green] {MAX_BATCH_SIZE}",
                                    f"[cyan]Tiles:[/cyan] {stats['total_tiles_processed']}"
                                )
                                if stats['oom_kills'] > 0:
                                    stats_table.add_row(
                                        f"[red]OOM:[/red] {stats['oom_kills']}",
                                        f"[green]OK:[/green] {stats['oom_recoveries']}"
                                    )
                                layout["stats"].update(Panel(stats_table, title="Stats", border_style="magenta"))
                                last_stats = stats_snapshot

                except Exception as e:
                    log(f"UI error: {e}", "WARN")
                    break

        t_ui = threading.Thread(target=update_layout_loop, daemon=True)
        t_ui.start()
        time.sleep(0.5)
        
        log_terminal("=" * 50)
        log_terminal("OSM Pipeline - MAX MODE")
        log_terminal("=" * 50)
        log_terminal("System initialized")

        if args.clean:
            clear_all_files()
        
        if args.rescan:
            rescan_and_mark_complete()
            log("Rescan complete. Pipeline will skip completed steps.", "SUCCESS")
            log_terminal("✓ Rescan complete - will continue from where left off")

        if args.export:
            try:
                collect_and_export_all_files()
                log("Export complete, continuing in main loop", "SUCCESS")
                log_terminal("✓ Export complete - continuing")
            except Exception as e:
                log(f"Export failed with error: {e}", "ERROR")
                log(f"Traceback: {traceback.format_exc()}", "ERROR")
                log_terminal(f"✗ Export failed: {e}", error=True)
            # Don't exit - continue in the main loop
        
        # Backdoor: continue from specific step
        if args.continue_from:
            log(f"🔓 BACKDOOR: Continuing from {args.continue_from} step", "WARN")
            log_terminal(f"🔓 Backdoor activated: starting from {args.continue_from}", command=True)
            
            # Get current MD5
            local_md5 = get_local_md5()
            if not local_md5:
                log("No local MD5 found - cannot use backdoor without MD5", "ERROR")
                log_terminal("✗ Need local MD5 to continue", error=True)
                return
            
            log(f"Using local MD5: {local_md5[:8]}...", "INFO")
            if process_pipeline(local_md5, start_from_step=args.continue_from):
                log("Pipeline completed from backdoor entry point", "SUCCESS")
                log_terminal("✓ Backdoor pipeline complete")
            else:
                log("Pipeline failed from backdoor", "ERROR")
                log_terminal("✗ Backdoor pipeline failed", error=True)
            return
        
        if args.valhalla:
            # Rebuild Valhalla only - force rebuild regardless of previous state
            log("Rebuilding Valhalla only (forced)...", "HEADER")
            log_terminal("🔄 Force rebuilding Valhalla...", command=True)
            
            if not os.path.exists(INDIA_PBF_FILE):
                log("No PBF file found for Valhalla rebuild", "ERROR")
                log_terminal("✗ No PBF file found", error=True)
                return
            
            # Force rebuild by passing force_rebuild=True
            log_terminal("Calling build_valhalla_graph(..., force_rebuild=True)", command=True)
            valhalla_tar = build_valhalla_graph(INDIA_PBF_FILE, force_rebuild=True)
            log(f"build_valhalla_graph returned: {valhalla_tar}", "INFO")
            if valhalla_tar and os.path.exists(valhalla_tar):
                # Copy rebuilt files to public export
                log("Copying Valhalla files to export...", "INFO")
                log_terminal("📦 Copying files to export folder...", command=True)
                export_valhalla_dir = os.path.join(PUBLIC_DIR, "valhalla")
                os.makedirs(export_valhalla_dir, exist_ok=True)
                
                # Copy config and databases
                for file in ["valhalla.json", "admin.sqlite", "timezones.sqlite"]:
                    src = os.path.join(WORK_DIR, file)
                    if os.path.exists(src):
                        shutil.copy(src, os.path.join(export_valhalla_dir, file))
                        log_terminal(f"✓ Copied {file}")
                
                # Copy tiles.tar from valhalla directory
                tiles_tar = os.path.join(VALHALLA_DIR, "tiles.tar")
                if os.path.exists(tiles_tar):
                    shutil.copy(tiles_tar, os.path.join(export_valhalla_dir, "tiles.tar"))
                    log_terminal("✓ Copied tiles.tar")
                
                # Copy valhalla_tiles directory
                valhalla_tiles_src = os.path.join(VALHALLA_DIR, "valhalla_tiles")
                valhalla_tiles_dst = os.path.join(export_valhalla_dir, "valhalla_tiles")
                if os.path.exists(valhalla_tiles_src):
                    if os.path.exists(valhalla_tiles_dst):
                        shutil.rmtree(valhalla_tiles_dst)
                    shutil.copytree(valhalla_tiles_src, valhalla_tiles_dst)
                    log_terminal("✓ Copied valhalla_tiles directory")
                
                    try:
                        save_single_file_md5(valhalla_tar)
                    except Exception as e:
                        log(f"Failed to save MD5: {e}", "WARN")
                log("Valhalla rebuild complete and files copied", "SUCCESS")
                log_terminal("✓ Valhalla rebuild complete!", command=True)
            else:
                log("Valhalla rebuild failed", "ERROR")
                log_terminal("✗ Valhalla rebuild failed", error=True)
            return

        if not check_disk_space():
            log("Insufficient disk space", "ERROR")
            log_terminal("✗ Insufficient disk space")
            return

        log("🚀 MAX BATCH MODE:", "SUCCESS")
        log(f"  Batch: {MAX_BATCH_SIZE} tiles (always max)", "INFO")
        log(f"  Monitor: Halt at {MEMORY_EXECUTION_THRESHOLD*100:.0f}%, halve batch", "INFO")
        log(f"  Small files: Auto-remove < {MIN_FILE_SIZE_BYTES} bytes", "INFO")
        log(f"  Cooldown: NONE - continuous monitoring", "INFO")
        
        log_terminal(f"✓ Max Batch: {MAX_BATCH_SIZE}, Threshold: {MEMORY_EXECUTION_THRESHOLD*100:.0f}%")
        log_terminal("✓ Terminal: VERBOSE mode (showing all commands)")
        log_terminal("Starting main loop...")

        while not shutdown_event.is_set():
            log("Checking updates...", "INFO")
            log_terminal("Checking for updates...")
            remote_md5 = get_remote_md5()
            local_md5 = get_local_md5()

            # If we couldn't fetch the remote MD5 (network/transient error), don't assume 'Up to date'.
            if remote_md5 is None:
                log("Could not fetch remote MD5; will retry shortly", "WARN")
                log_terminal("MD5 fetch failed; retrying in 15s")
                time.sleep(15)
                continue

            if remote_md5 != local_md5:
                log(f"Update found: {remote_md5[:8]}...", "SUCCESS")
                log_terminal(f"✓ Update found: {remote_md5[:8]}...")
                
                if not check_disk_space():
                    log("Low disk, sleeping 1h", "ERROR")
                    log_terminal("✗ Low disk space, sleeping 1h...")
                    time.sleep(3600)
                    continue
                
                download_needed = True
                if os.path.exists(INDIA_PBF_FILE):
                    if calculate_file_md5(INDIA_PBF_FILE) == remote_md5:
                        download_needed = False
                        log("File valid, skip download", "SUCCESS")
                
                if download_needed:
                    if not download_file_resumable(INDIA_PBF_URL, INDIA_PBF_FILE):
                        log("Download failed, retry in 5m", "ERROR")
                        time.sleep(300)
                        continue
                    if calculate_file_md5(INDIA_PBF_FILE) != remote_md5:
                        log("MD5 mismatch", "ERROR")
                        continue

                if process_pipeline(remote_md5):
                    log("Pipeline done! Moving files...", "SUCCESS")
                    save_local_md5(remote_md5)
                    log("Ready for next update", "SUCCESS")
                else:
                    log("Pipeline failed", "ERROR")
            elif not os.path.exists(PUBLIC_DIR) or not os.listdir(PUBLIC_DIR):
                log("No output files found - running initial pipeline", "WARN")
                if process_pipeline(remote_md5):
                    log("Initial pipeline done!", "SUCCESS")
                    save_local_md5(remote_md5)
                else:
                    log("Initial pipeline failed", "ERROR")
            else:
                log("Up to date", "INFO")
            
            # Check more frequently after update/failure, less frequently when up to date
            if remote_md5 == local_md5:
                # Already up to date - check every 30 minutes
                check_interval = 1800
                log(f"Waiting 30 minutes before next check...", "INFO")
            else:
                # After pipeline run or failure - check again in 5 minutes
                check_interval = 300
                log(f"Waiting 5 minutes before next check...", "INFO")
            
            time.sleep(check_interval)

def ensure_screen_session():
    parser = argparse.ArgumentParser()
    parser.add_argument("--clean", action="store_true", help="Clear all files")
    parser.add_argument("--rescan", action="store_true", help="Rescan existing files and continue from Valhalla")
    parser.add_argument("--export", action="store_true", help="Collect and export all files to public_export")
    parser.add_argument("--valhalla", action="store_true", help="Rebuild Valhalla only")
    parser.add_argument("--continue-from", type=str, choices=['poi', 'filter', 'extract', 'india', 'valhalla', 'publish'],
                        help="Backdoor: continue pipeline from specific step")
    parser.add_argument("--fix-publish", action="store_true", help="Quick fix: copy missing files to public_export")
    parser.add_argument("--rebuild-tar", action="store_true", help="Rebuild valhalla_tiles.tar.gz from valhalla directory")
    args, unknown = parser.parse_known_args()
    
    if os.environ.get(ENV_WORKER_FLAG) == "1":
        try: 
            main_loop()
        except KeyboardInterrupt: 
            log("Shutting down...", "WARN")
            save_statistics()
            save_checkpoint()
            save_batch_history()
            sys.exit(0)
        return

    if not shutil.which("screen"):
        print("Error: 'screen' not installed.")
        sys.exit(1)

    result = subprocess.run(["screen", "-ls"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if SCREEN_SESSION_NAME in result.stdout:
        os.execvp("screen", ["screen", "-r", SCREEN_SESSION_NAME])
    else:
        script_path = os.path.abspath(__file__)
        extra_args = ""
        if args.clean:
            extra_args += " --clean"
        if args.rescan:
            extra_args += " --rescan"
        if args.export:
            extra_args += " --export"
        if args.valhalla:
            extra_args += " --valhalla"
        if args.continue_from:
            extra_args += f" --continue-from {args.continue_from}"
        if args.fix_publish:
            extra_args += " --fix-publish"
        if args.rebuild_tar:
            extra_args += " --rebuild-tar"
        inner_cmd = f"export {ENV_WORKER_FLAG}=1; {sys.executable} {script_path}{extra_args}; exec bash"
        subprocess.run(["screen", "-dmS", SCREEN_SESSION_NAME, "bash", "-c", inner_cmd])
        time.sleep(1) 
        os.execvp("screen", ["screen", "-r", SCREEN_SESSION_NAME])

if __name__ == "__main__":
    ensure_screen_session()