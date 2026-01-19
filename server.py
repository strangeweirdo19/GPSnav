#!/usr/bin/env python3
"""
OSM Pipeline File Server
A file distribution and inventory system for OSM pipeline outputs.
"""

import os
import json
import hashlib
import mimetypes
import shutil
import threading
import subprocess
import argparse
import logging
import sys
import re
import time
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Optional
from flask import Flask, jsonify, send_file, request, render_template_string, abort, Response
from werkzeug.utils import safe_join
from werkzeug.middleware.proxy_fix import ProxyFix
import time

# Logging Configuration
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('server.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

# Configuration (supports environment variables for production)
BASE_DIR = os.getenv('SERVER_BASE_DIR', os.getcwd())
PUBLIC_DIR = os.getenv('PUBLIC_DIR', os.path.join(BASE_DIR, "public_export"))

# Blue-Green Deployment: Two host directories for zero-downtime updates
HOST1_DIR = os.path.join(BASE_DIR, "host1")
HOST2_DIR = os.path.join(BASE_DIR, "host2")
ACTIVE_HOST_FILE = os.path.join(BASE_DIR, ".active_host")
COMPLETED_MARKER = "/home/navigator/completed.done"

# Active host directory (dynamically determined)
active_host_lock = threading.Lock()
HOST_DIR = None  # Will be set during initialization
MANIFEST_FILE = None  # Will be set during initialization
SERVER_VERSION = "2.1.0"

# Firmware Configuration (supports environment variables)
FIRMWARE_SOURCE_DIR = os.getenv('FIRMWARE_SOURCE_DIR', "/home/navigator/GPSnav")
FIRMWARE_SOURCE_PATH = os.path.join(FIRMWARE_SOURCE_DIR, ".pio/build/esp-wrover-kit/firmware.bin")
FIRMWARE_HOST_DIR = None  # Will be set based on active host
FIRMWARE_HOST_PATH = None  # Will be set based on active host

# Security Configuration
MAX_CONTENT_LENGTH = int(os.getenv('MAX_CONTENT_LENGTH', 16 * 1024 * 1024 * 1024))  # 16GB default
ALLOWED_EXTENSIONS = set(os.getenv('ALLOWED_EXTENSIONS', '.bin,.mbtiles,.lz4,.sqlite,.db,.json,.xml,.tar,.gph,.md5').split(','))

# Statistics
server_stats = {
    "start_time": time.time(),
    "total_downloads": 0,
    "active_connections": 0
}

app = Flask(__name__)
app.config['MAX_CONTENT_LENGTH'] = MAX_CONTENT_LENGTH
app.config['SEND_FILE_MAX_AGE_DEFAULT'] = 31536000  # 1 year cache

# Trust Cloudflare proxy headers (X-Forwarded-For, X-Forwarded-Proto, etc.)
# This allows the server to work correctly behind Cloudflare Tunnel
app.wsgi_app = ProxyFix(
    app.wsgi_app, 
    x_for=1,      # Trust X-Forwarded-For (1 proxy: Cloudflare)
    x_proto=1,    # Trust X-Forwarded-Proto for HTTPS detection
    x_host=1,     # Trust X-Forwarded-Host
    x_prefix=1    # Trust X-Forwarded-Prefix
)

# Security headers middleware
@app.after_request
def add_security_headers(response):
    """Add security headers to all responses."""
    response.headers['X-Content-Type-Options'] = 'nosniff'
    response.headers['X-Frame-Options'] = 'DENY'
    response.headers['X-XSS-Protection'] = '1; mode=block'
    # CORS for API access
    response.headers['Access-Control-Allow-Origin'] = os.getenv('CORS_ORIGIN', '*')
    response.headers['Access-Control-Allow-Methods'] = 'GET, POST, OPTIONS'
    response.headers['Access-Control-Allow-Headers'] = 'Content-Type, Range'
    # Cloudflare compatibility
    if request.headers.get('CF-Ray'):
        response.headers['X-Served-By'] = 'Cloudflare'
    return response

# Global error handlers
@app.errorhandler(404)
def not_found(error):
    """Handle 404 errors."""
    if request.path.startswith('/api/'):
        return jsonify({'error': 'Resource not found', 'path': request.path}), 404
    return jsonify({'error': 'Not found'}), 404

@app.errorhandler(500)
def internal_error(error):
    """Handle 500 errors."""
    logger.error(f"Internal server error: {error}", exc_info=True)
    return jsonify({'error': 'Internal server error'}), 500

@app.errorhandler(413)
def request_entity_too_large(error):
    """Handle file too large errors."""
    return jsonify({'error': 'File too large'}), 413

def get_active_host() -> str:
    """Get the currently active host directory (host1 or host2)."""
    if os.path.exists(ACTIVE_HOST_FILE):
        try:
            with open(ACTIVE_HOST_FILE, 'r') as f:
                active = f.read().strip()
                if active in ['host1', 'host2']:
                    return active
        except Exception as e:
            logger.error(f"Error reading active host file: {e}")
    # Default to host1
    return 'host1'

def set_active_host(host_name: str):
    """Set the active host directory and update global variables."""
    global HOST_DIR, MANIFEST_FILE, FIRMWARE_HOST_DIR, FIRMWARE_HOST_PATH
    
    with active_host_lock:
        try:
            with open(ACTIVE_HOST_FILE, 'w') as f:
                f.write(host_name)
            
            # Update global variables
            if host_name == 'host1':
                HOST_DIR = HOST1_DIR
            else:
                HOST_DIR = HOST2_DIR
            
            MANIFEST_FILE = os.path.join(HOST_DIR, "manifest.json")
            FIRMWARE_HOST_DIR = os.path.join(HOST_DIR, "esp32wrover")
            FIRMWARE_HOST_PATH = os.path.join(FIRMWARE_HOST_DIR, "firmware.bin")
            
            logger.info(f"Active host switched to: {host_name} ({HOST_DIR})")
            return True
        except Exception as e:
            logger.error(f"Error setting active host: {e}")
            return False

def get_inactive_host_dir() -> str:
    """Get the path to the inactive host directory."""
    active = get_active_host()
    return HOST2_DIR if active == 'host1' else HOST1_DIR

def ensure_directories():
    """Ensure public_export and both host directories exist."""
    os.makedirs(PUBLIC_DIR, exist_ok=True)
    os.makedirs(HOST1_DIR, exist_ok=True)
    os.makedirs(HOST2_DIR, exist_ok=True)
    logger.info(f"Ensured directories: {PUBLIC_DIR}, {HOST1_DIR}, {HOST2_DIR}")

def load_last_processed_run() -> Optional[str]:
    """Load the last processed run timestamp from disk."""
    state_file = os.path.join(HOST_DIR, ".last_processed_run")
    try:
        if os.path.exists(state_file):
            with open(state_file, 'r') as f:
                return f.read().strip()
    except Exception as e:
        logger.error(f"Error loading last processed run: {e}")
    return None

def save_last_processed_run(timestamp: str):
    """Save the last processed run timestamp to disk."""
    state_file = os.path.join(HOST_DIR, ".last_processed_run")
    try:
        with open(state_file, 'w') as f:
            f.write(timestamp)
    except Exception as e:
        logger.error(f"Error saving last processed run: {e}")


def calculate_file_md5(filepath: str) -> Optional[str]:
    """Calculate MD5 hash of a file."""
    try:
        md5_hash = hashlib.md5()
        with open(filepath, "rb") as f:
            for chunk in iter(lambda: f.read(8192), b""):
                md5_hash.update(chunk)
        return md5_hash.hexdigest()
    except Exception as e:
        logger.error(f"Error calculating MD5 for {filepath}: {e}")
        return None


def write_md5_sidecar(filepath: str, md5_hash: str):
    """Write MD5 hash to sidecar file."""
    try:
        with open(f"{filepath}.md5", 'w') as f:
            f.write(md5_hash)
    except Exception as e:
        logger.error(f"Error writing MD5 sidecar for {filepath}: {e}")




def scan_directory_path(directory: str) -> Dict:
    """Scan specified directory and generate file manifest."""
    logger.info(f"Scanning directory: {directory}")
    
    if not os.path.exists(directory):
        logger.error(f"ERROR: Directory does not exist: {directory}")
        raise FileNotFoundError(f"Required directory not found: {directory}")
    
    manifest = {
        "generated_at": datetime.utcnow().isoformat(),
        "server_version": SERVER_VERSION,
        "total_files": 0,
        "total_size": 0,
        "categories": {key: {"name": val["name"], "files": []} for key, val in CATEGORIES.items()},
        "structure": {}
    }
    
    # Add "other" category for uncategorized files
    manifest["categories"]["other"] = {"name": "Other Files", "files": []}
    
    all_files = []
    
    # Walk through directory
    for root, dirs, files in os.walk(directory):
        for filename in files:
            # Skip manifest and MD5 checksum files
            if filename == "manifest.json" or filename == "metadata.json" or filename.endswith(".md5"):
                continue
            
            # Skip firmware binaries as they are handled by the firmware section
            if filename == "firmware.bin":
                continue
            
            file_path = os.path.join(root, filename)
            rel_path = os.path.relpath(file_path, directory)
            
            try:
                file_size = os.path.getsize(file_path)
                
                # Read MD5 from sidecar file (created by pipeline)
                file_md5 = read_md5_sidecar(file_path)
                if not file_md5:
                    file_md5 = "unknown"
                
                category = categorize_file(rel_path)
                
                file_info = {
                    "name": filename,
                    "path": rel_path.replace("\\", "/"),
                    "size": file_size,
                    "md5": file_md5,
                    "url": f"/files/{rel_path.replace(os.sep, '/')}",
                    "description": f"{filename}"
                }
                
                manifest["categories"][category]["files"].append(file_info)
                manifest["total_files"] += 1
                manifest["total_size"] += file_size
                
                all_files.append(file_info)
            
            except Exception as e:
                logger.error(f"Error processing {file_path}: {e}")
    
    # Build tree structure
    manifest["structure"] = build_tree_structure(all_files)
    
    return manifest


# File categorization rules
CATEGORIES = {
    "maps": {
        "name": "MBTiles Maps",
        "extensions": [".mbtiles", ".mbtiles.lz4"],
        "paths": ["maps/", "tiles/"]
    },
    "mobile": {
        "name": "Mobile MBTiles",
        "extensions": [".mbtiles", ".mbtiles.lz4"],
        "paths": ["mobile_mbtiles/"]
    },
    "databases": {
        "name": "SQLite Databases",
        "extensions": [".sqlite", ".sqlite.lz4", ".db", ".db.lz4"],
        "paths": ["databases/"]
    },
    "valhalla": {
        "name": "Valhalla Routing Data",
        "extensions": [".tar.lz4", ".tar", ".json", ".gph"],
        "paths": ["valhalla/"]
    },
    "config": {
        "name": "Configuration Files",
        "extensions": [".json", ".yaml", ".yml", ".xml"],
        "paths": ["config/"]
    }
}

def read_md5_sidecar(file_path: str) -> Optional[str]:
    """Read MD5 from sidecar .md5 file created by pipeline."""
    md5_file_path = f"{file_path}.md5"
    try:
        if os.path.exists(md5_file_path):
            with open(md5_file_path, 'r') as f:
                return f.read().strip().split()[0]
    except Exception:
        pass
    return None

def format_bytes(bytes_value: int) -> str:
    """Format bytes to human readable format."""
    for unit in ['B', 'KB', 'MB', 'GB', 'TB']:
        if bytes_value < 1024.0:
            return f"{bytes_value:.2f} {unit}"
        bytes_value /= 1024.0
    return f"{bytes_value:.2f} PB"

def categorize_file(rel_path: str) -> str:
    """Determine file category based on path and extension."""
    rel_path_lower = rel_path.lower()
    file_ext = os.path.splitext(rel_path_lower)[1]
    
    # First pass: Check path prefixes
    for cat_key, cat_info in CATEGORIES.items():
        for path_prefix in cat_info["paths"]:
            if rel_path_lower.startswith(path_prefix):
                return cat_key
                
    # Second pass: Check extensions
    for cat_key, cat_info in CATEGORIES.items():
        if file_ext in cat_info["extensions"]:
            return cat_key
    
    return "other"

def build_tree_structure(files_list):
    """Build a hierarchical tree structure from flat file list."""
    tree = {'_files': [], '_dirs': {}}
    
    for file_info in files_list:
        path_parts = file_info['path'].split('/')
        
        # If it's a root-level file (no subdirectories)
        if len(path_parts) == 1:
            tree['_files'].append(file_info)
        else:
            # Navigate/create the tree structure for nested files
            current = tree
            
            # Navigate through all directory levels except the last (filename)
            for part in path_parts[:-1]:
                if part not in current['_dirs']:
                    current['_dirs'][part] = {'_dirs': {}, '_files': []}
                current = current['_dirs'][part]
            
            # Add file to the final directory's files list
            current['_files'].append(file_info)
    
    return tree

def get_last_successful_run() -> Optional[str]:
    """Get timestamp of last successful run from pipeline."""
    last_run_file = os.path.join(os.path.dirname(PUBLIC_DIR), "work_temp", "last_successful_run.txt")
    try:
        if os.path.exists(last_run_file):
            with open(last_run_file, 'r') as f:
                return f.read().strip()
    except Exception as e:
        logger.error(f"Error reading last run timestamp: {e}")
    return None

def sync_to_inactive_host():
    """Sync all files from public_export to inactive host directory."""
    inactive_dir = get_inactive_host_dir()
    logger.info("="*60)
    logger.info(f"SYNCING TO INACTIVE HOST: {inactive_dir}")
    logger.info("="*60)
    
    try:
        # Copy firmware from public_export (built by pipeline)
        firmware_src = os.path.join(PUBLIC_DIR, "esp32wrover")
        if os.path.exists(firmware_src):
            logger.info("Copying firmware from public_export...")
            firmware_dst = os.path.join(inactive_dir, "esp32wrover")
            if os.path.exists(firmware_dst):
                shutil.rmtree(firmware_dst)
            shutil.copytree(firmware_src, firmware_dst)
            logger.info(f"✓ Firmware copied to: {firmware_dst}")
        else:
            logger.warning("No firmware found in public_export (pipeline may not have built it)")
        
        # Sync files from public_export to inactive directory
        if not os.path.exists(PUBLIC_DIR):
            logger.warning("public_export does not exist")
            return False
        
        os.makedirs(inactive_dir, exist_ok=True)
        
        updated = 0
        copied = 0
        
        # Walk through public_export
        for root, dirs, files in os.walk(PUBLIC_DIR):
            rel_root = os.path.relpath(root, PUBLIC_DIR)
            
            # Create corresponding directories in inactive host
            if rel_root != '.':
                inactive_subdir = os.path.join(inactive_dir, rel_root)
                os.makedirs(inactive_subdir, exist_ok=True)
            
            for file in files:
                if file == "metadata.json":
                    continue
                
                src_path = os.path.join(root, file)
                if rel_root == '.':
                    dst_path = os.path.join(inactive_dir, file)
                else:
                    dst_path = os.path.join(inactive_dir, rel_root, file)
                
                should_copy = False
                
                if not os.path.exists(dst_path):
                    should_copy = True
                    copied += 1
                else:
                    src_mtime = os.path.getmtime(src_path)
                    dst_mtime = os.path.getmtime(dst_path)
                    if src_mtime > dst_mtime:
                        should_copy = True
                        updated += 1
                
                if should_copy:
                    try:
                        shutil.copy2(src_path, dst_path)
                        logger.info(f"Synced: {os.path.join(rel_root, file) if rel_root != '.' else file}")
                    except Exception as e:
                        logger.error(f"Failed to copy {file}: {e}")
        
        # Generate manifest for inactive directory
        logger.info("Generating manifest for inactive host...")
        inactive_manifest = scan_directory_path(inactive_dir)
        inactive_manifest_file = os.path.join(inactive_dir, "manifest.json")
        with open(inactive_manifest_file, "w") as f:
            json.dump(inactive_manifest, f, indent=2)
        
        logger.info("="*60)
        logger.info(f"SYNC COMPLETED: {copied} new, {updated} updated")
        logger.info(f"Total files in inactive host: {inactive_manifest['total_files']}")
        logger.info("="*60)
        
        return True
    
    except Exception as e:
        logger.error(f"Error syncing to inactive host: {e}")
        return False

def switch_active_host():
    """Switch to the inactive host directory."""
    global manifest
    
    current_active = get_active_host()
    new_active = 'host2' if current_active == 'host1' else 'host1'
    old_host_dir = HOST1_DIR if current_active == 'host1' else HOST2_DIR
    
    logger.info("="*60)
    logger.info(f"SWITCHING ACTIVE HOST: {current_active} -> {new_active}")
    logger.info("="*60)
    
    # Set new active host
    if set_active_host(new_active):
        # Reload manifest from new active directory
        manifest = load_manifest()
        logger.info(f"Now serving from: {HOST_DIR}")
        logger.info(f"Files available: {manifest.get('total_files', 0)}")
        
        # Clean up old host directory
        logger.info(f"Cleaning up old host directory: {old_host_dir}")
        try:
            for root, dirs, files in os.walk(old_host_dir, topdown=False):
                for file in files:
                    os.remove(os.path.join(root, file))
                for dir in dirs:
                    os.rmdir(os.path.join(root, dir))
            logger.info("Old host directory emptied")
        except Exception as e:
            logger.error(f"Error cleaning up old host: {e}")
        
        logger.info("="*60)
        logger.info("SWITCH COMPLETED SUCCESSFULLY")
        logger.info("="*60)
        return True
    
    return False

def monitor_for_updates():
    """Monitor for completed.done file and trigger blue-green deployment."""
    global manifest
    
    logger.info("Blue-green deployment monitor started")
    logger.info(f"Watching for: {COMPLETED_MARKER}")
    
    while True:
        try:
            # Check for completed.done file every 60 seconds
            if os.path.exists(COMPLETED_MARKER):
                logger.info("\n" + "="*60)
                logger.info("DEPLOYMENT MARKER DETECTED!")
                logger.info("="*60)
                
                # Sync to inactive host
                if sync_to_inactive_host():
                    # Switch active host
                    if switch_active_host():
                        # Delete the marker file
                        try:
                            os.remove(COMPLETED_MARKER)
                            logger.info(f"Deleted marker file: {COMPLETED_MARKER}")
                        except Exception as e:
                            logger.error(f"Error deleting marker file: {e}")
                        
                        logger.info("\nZERO-DOWNTIME DEPLOYMENT COMPLETED!")
                    else:
                        logger.error("Failed to switch active host")
                else:
                    logger.error("Failed to sync to inactive host")
            
            time.sleep(60)  # Check every minute
        
        except Exception as e:
            logger.error(f"Error in monitor thread: {e}")
            time.sleep(60)

def load_valhalla_tiles_manifest():
    """
    Load Valhalla tiles manifest that was created by the pipeline.
    Now stored as valhalla_tiles_manifest.json in the valhalla directory.
    """
    # Try new location first
    manifest_path = os.path.join(HOST_DIR, "valhalla", "valhalla_tiles_manifest.json")
    
    if not os.path.exists(manifest_path):
        # Fallback to old location for backwards compatibility
        manifest_path = os.path.join(HOST_DIR, "valhalla", "manifest.json")
    
    try:
        if os.path.exists(manifest_path):
            with open(manifest_path, 'r') as f:
                return json.load(f)
    except Exception as e:
        logger.error(f"Error loading Valhalla manifest: {e}")
    
    return None

def scan_directory() -> Dict:
    """Scan host directory and generate file manifest."""
    logger.info(f"Scanning directory: {HOST_DIR}")
    
    if not os.path.exists(HOST_DIR):
        logger.error(f"ERROR: Directory does not exist: {HOST_DIR}")
        logger.error("Please ensure the host directory exists before starting the server.")
        raise FileNotFoundError(f"Required directory not found: {HOST_DIR}")
    
    manifest = {
        "generated_at": datetime.utcnow().isoformat(),
        "server_version": SERVER_VERSION,
        "total_files": 0,
        "total_size": 0,
        "categories": {key: {"name": val["name"], "files": []} for key, val in CATEGORIES.items()},
        "structure": {}
    }
    
    # Add "other" category for uncategorized files
    manifest["categories"]["other"] = {"name": "Other Files", "files": []}
    
    all_files = []
    
    # Walk through directory
    for root, dirs, files in os.walk(HOST_DIR):
        for filename in files:
            # Skip manifest and MD5 checksum files
            if filename == "manifest.json" or filename == "metadata.json" or filename.endswith(".md5"):
                continue
            
            # Skip firmware binaries as they are handled by the firmware section
            if filename == "firmware.bin":
                continue
            
            file_path = os.path.join(root, filename)
            rel_path = os.path.relpath(file_path, HOST_DIR)
            
            try:
                file_size = os.path.getsize(file_path)
                
                # Read MD5 from sidecar file (created by pipeline)
                file_md5 = read_md5_sidecar(file_path)
                if not file_md5:
                    file_md5 = "unknown"
                
                category = categorize_file(rel_path)
                
                file_info = {
                    "name": filename,
                    "path": rel_path.replace("\\", "/"),
                    "size": file_size,
                    "md5": file_md5,
                    "url": f"/files/{rel_path.replace(os.sep, '/')}",
                    "description": f"{filename}"
                }
                
                manifest["categories"][category]["files"].append(file_info)
                manifest["total_files"] += 1
                manifest["total_size"] += file_size
                
                all_files.append(file_info)
                
                logger.debug(f"Added: {rel_path} ({file_size} bytes) [{category}]")
            
            except Exception as e:
                logger.error(f"Error processing {file_path}: {e}")
    
    # Build tree structure
    manifest["structure"] = build_tree_structure(all_files)
    
    return manifest

def generate_manifest() -> Dict:
    """Generate or update the manifest file for active host directory."""
    # Scan and create main manifest (MD5s come from pipeline)
    manifest = scan_directory_path(HOST_DIR)
    
    try:
        with open(MANIFEST_FILE, "w") as f:
            json.dump(manifest, f, indent=2)
        logger.info(f"Manifest saved to: {MANIFEST_FILE}")
    except Exception as e:
        logger.error(f"Error saving manifest: {e}")
    
    return manifest

def load_manifest() -> Optional[Dict]:
    """Load manifest from active host directory or generate if not exists."""
    if MANIFEST_FILE and os.path.exists(MANIFEST_FILE):
        try:
            with open(MANIFEST_FILE, "r") as f:
                return json.load(f)
        except Exception as e:
            logger.error(f"Error loading manifest: {e}")
    
    return generate_manifest()

# HTML Template for the UI
HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Navion - File Server</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
            transition: all 0.3s ease;
        }
        body {
            background: linear-gradient(135deg, #0f172a 0%, #1e293b 50%, #0f172a 100%);
            min-height: 100vh;
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            color: #f1f5f9;
        }
        .container {
            max-width: 1280px;
            margin: 0 auto;
            padding: 0 24px;
        }
        .glass {
            background: rgba(15, 23, 42, 0.4);
            backdrop-filter: blur(20px);
            border: 1px solid rgba(148, 163, 184, 0.15);
            border-radius: 16px;
        }
        .glass:hover {
            background: rgba(15, 23, 42, 0.6);
            border-color: rgba(148, 163, 184, 0.25);
            box-shadow: 0 20px 50px rgba(0, 0, 0, 0.3);
            transform: translateY(-2px);
        }
        .gradient-text {
            background: linear-gradient(135deg, #60a5fa 0%, #a78bfa 100%);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
            background-clip: text;
            font-size: 3rem;
            font-weight: 900;
            margin-bottom: 0.5rem;
        }
        .header {
            position: sticky;
            top: 0;
            z-index: 50;
            background: rgba(15, 23, 42, 0.4);
            backdrop-filter: blur(20px);
            border-bottom: 1px solid rgba(148, 163, 184, 0.2);
            padding: 32px 0;
        }
        .header-grid {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 32px;
        }
        .stats-grid {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 16px;
        }
        .stat-card {
            padding: 16px;
            background: rgba(15, 23, 42, 0.4);
            backdrop-filter: blur(20px);
            border: 1px solid rgba(148, 163, 184, 0.15);
            border-radius: 12px;
        }
        .stat-label {
            font-size: 0.875rem;
            color: #94a3b8;
            margin-bottom: 4px;
        }
        .stat-value {
            font-size: 1.5rem;
            font-weight: 700;
            color: #34d399;
        }
        .file-grid {
            display: grid;
            gap: 16px;
            margin-bottom: 48px;
        }
        .file-card {
            position: relative;
            padding: 24px;
            background: rgba(15, 23, 42, 0.4);
            backdrop-filter: blur(20px);
            border: 1px solid rgba(148, 163, 184, 0.15);
            border-radius: 16px;
            overflow: hidden;
        }
        .file-card::before {
            content: '';
            position: absolute;
            top: 0;
            left: 0;
            right: 0;
            height: 2px;
            background: linear-gradient(90deg, transparent, #60a5fa, transparent);
        }
        .file-content {
            display: flex;
            justify-content: space-between;
            align-items: flex-start;
            gap: 16px;
        }
        .file-info {
            flex: 1;
            min-width: 0;
        }
        .file-name {
            font-size: 1.125rem;
            font-weight: 600;
            margin-bottom: 8px;
            overflow: hidden;
            text-overflow: ellipsis;
            white-space: nowrap;
        }
        .file-meta {
            display: flex;
            flex-wrap: wrap;
            gap: 12px;
            font-size: 0.875rem;
            color: #94a3b8;
        }
        .btn {
            padding: 12px 24px;
            border-radius: 8px;
            font-weight: 600;
            text-decoration: none;
            display: inline-block;
            cursor: pointer;
            border: none;
            white-space: nowrap;
            position: relative;
            overflow: hidden;
        }
        .btn-primary {
            background: linear-gradient(135deg, #3b82f6 0%, #2563eb 100%);
            color: white;
        }
        .btn-primary:hover {
            background: linear-gradient(135deg, #2563eb 0%, #1d4ed8 100%);
            box-shadow: 0 10px 25px rgba(59, 130, 246, 0.3);
        }
        .btn-danger {
            background: linear-gradient(135deg, #ef4444 0%, #dc2626 100%);
            color: white;
        }
        .btn-danger:hover {
            background: linear-gradient(135deg, #dc2626 0%, #b91c1c 100%);
            box-shadow: 0 10px 25px rgba(239, 68, 68, 0.3);
        }
        .btn-secondary {
            background: rgba(15, 23, 42, 0.4);
            backdrop-filter: blur(20px);
            border: 1px solid rgba(148, 163, 184, 0.15);
            color: #f1f5f9;
        }
        .btn-secondary:hover {
            background: rgba(51, 65, 85, 0.6);
        }
        .icon-wrapper {
            width: 56px;
            height: 56px;
            border-radius: 16px;
            display: flex;
            align-items: center;
            justify-content: center;
            font-size: 28px;
            flex-shrink: 0;
        }
        .icon-maps { background: linear-gradient(135deg, #10b981 0%, #34d399 100%); }
        .icon-databases { background: linear-gradient(135deg, #8b5cf6 0%, #c4b5fd 100%); }
        .icon-valhalla { background: linear-gradient(135deg, #f59e0b 0%, #fbbf24 100%); }
        .icon-config { background: linear-gradient(135deg, #06b6d4 0%, #22d3ee 100%); }
        .icon-firmware { background: linear-gradient(135deg, #ef4444 0%, #f87171 100%); }
        .icon-other { background: linear-gradient(135deg, #6366f1 0%, #818cf8 100%); }
        .firmware-card {
            margin: 32px 0;
            border-left: 4px solid #ef4444;
        }
        .firmware-grid {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 16px;
            margin-bottom: 16px;
        }
        .firmware-meta {
            color: #94a3b8;
        }
        .firmware-meta-label {
            font-size: 0.75rem;
            color: #64748b;
            margin-bottom: 4px;
        }
        .firmware-meta-value {
            font-size: 1.125rem;
            font-weight: 600;
        }
        .section-title {
            font-size: 1.875rem;
            font-weight: 700;
            margin-bottom: 32px;
            color: #f1f5f9;
        }
        .category-title {
            font-size: 1.25rem;
            font-weight: 700;
            margin-bottom: 16px;
            color: #cbd5e1;
            display: flex;
            align-items: center;
            gap: 8px;
        }
        .loading, .error {
            min-height: 100vh;
            display: flex;
            flex-direction: column;
            align-items: center;
            justify-content: center;
            text-align: center;
        }
        .spinner {
            width: 64px;
            height: 64px;
            border: 4px solid rgba(96, 165, 250, 0.2);
            border-top-color: #60a5fa;
            border-radius: 50%;
            animation: spin 1s linear infinite;
            margin-bottom: 16px;
        }
        @keyframes spin {
            to { transform: rotate(360deg); }
        }
        @keyframes shimmer {
            0% { opacity: 0.5; }
            50% { opacity: 1; }
            100% { opacity: 0.5; }
        }
        .animate-shimmer { animation: shimmer 2s infinite; }
        .empty-state {
            text-align: center;
            padding: 80px 0;
        }
        .empty-icon {
            font-size: 4rem;
            margin-bottom: 16px;
            color: #475569;
        }
        .truncate {
            overflow: hidden;
            text-overflow: ellipsis;
            white-space: nowrap;
        }
        .font-mono {
            font-family: 'Courier New', monospace;
        }
        .accordion {
            background: rgba(15, 23, 42, 0.4);
            backdrop-filter: blur(20px);
            border: 1px solid rgba(148, 163, 184, 0.15);
            border-radius: 16px;
            margin-bottom: 16px;
            overflow: hidden;
        }
        .accordion-header {
            padding: 24px;
            display: flex;
            align-items: center;
            cursor: pointer;
            user-select: none;
        }
        .accordion-header:hover {
            background: rgba(51, 65, 85, 0.3);
        }
        .accordion-title {
            flex: 1;
            display: flex;
            align-items: center;
            gap: 12px;
            font-size: 1.25rem;
            font-weight: 700;
            color: #cbd5e1;
        }
        .accordion-content {
            max-height: 0;
            overflow: hidden;
            transition: max-height 0.3s ease-out;
            background: rgba(15, 23, 42, 0.2);
        }
        .accordion-content.open {
            max-height: 2000px; /* Arbitrary large height */
            transition: max-height 0.5s ease-in;
        }
        .accordion-icon {
            font-size: 1.5rem;
            transition: transform 0.3s ease;
        }
        .accordion-icon.rotate {
            transform: rotate(180deg);
        }
        .file-list-padding {
            padding: 24px;
            display: grid;
            gap: 16px;
        }
    </style>
</head>
<body>
    <div id="root"></div>
    
    <script>
        const API_BASE = '';
        let manifest = null;
        let firmwareInfo = null;
        let loading = true;
        
        async function fetchData() {
            try {
                const [manifestRes, firmwareRes] = await Promise.all([
                    fetch('/manifest'),
                    fetch('/firmware/info')
                ]);
                
                if (!manifestRes.ok) throw new Error('Manifest failed');
                manifest = await manifestRes.json();
                if (firmwareRes.ok) firmwareInfo = await firmwareRes.json();
                
                loading = false;
                render();
            } catch (err) {
                console.error(err);
                loading = false;
                render();
            }
        }
        
        function formatBytes(bytes) {
            if (bytes === 0) return '0.00 B';
            const k = 1024, sizes = ['B', 'KB', 'MB', 'GB'];
            let i = Math.floor(Math.log(bytes) / Math.log(k));
            return (bytes / Math.pow(k, i)).toFixed(2) + ' ' + sizes[i];
        }
        
        function getCategoryIcon(cat) {
            const icons = {
                maps: '🗺️', databases: '📊', valhalla: '🛣️',
                config: '⚙️', firmware: '⚡', other: '📦'
            };
            return icons[cat] || '📦';
        }
        
        function getCategoryColor(cat) {
            const colors = {
                maps: 'icon-maps', databases: 'icon-databases', valhalla: 'icon-valhalla',
                config: 'icon-config', firmware: 'icon-firmware', other: 'icon-other'
            };
            return colors[cat] || 'icon-other';
        }
        
        function render() {
            const root = document.getElementById('root');
            
            if (loading) {
                root.innerHTML = `
                    <div class="loading">
                        <div class="spinner"></div>
                        <p style="font-size: 1.25rem; color: #94a3b8;">Loading Navigator...</p>
                    </div>
                `;
                return;
            }
            
            if (!manifest) {
                root.innerHTML = `
                    <div class="error">
                        <div class="glass" style="padding: 48px; max-width: 28rem;">
                            <div style="font-size: 3rem; color: #f87171; margin-bottom: 16px;">⚠️</div>
                            <p style="font-size: 1.25rem; margin-bottom: 16px;">Connection Error</p>
                            <button onclick="location.reload()" class="btn btn-primary">
                                Retry
                            </button>
                        </div>
                    </div>
                `;
                return;
            }
            
            const totalFiles = manifest.total_files || 0;
            const totalSize = formatBytes(manifest.total_size || 0);
            
            let html = `
                <div>
                    <!-- HEADER -->
                    <div class="header">
                        <div class="container">
                            <div class="header-grid">
                                <div>
                                    <h1 class="gradient-text">Navion</h1>
                                    <p style="color: #94a3b8;">OpenStreetMap Data & Firmware Server</p>
                                </div>
                                <div style="text-align: right;">
                                    <div style="font-size: 2.25rem; font-weight: 700; color: #60a5fa;">${totalFiles}</div>
                                    <div style="color: #94a3b8;">Files Available</div>
                                </div>
                            </div>
                            
                            <div class="stats-grid">
                                <div class="stat-card">
                                    <div class="stat-label">Total Size</div>
                                    <div class="stat-value">${totalSize}</div>
                                </div>
                                <div class="stat-card">
                                    <div class="stat-label">Last Updated</div>
                                    <div style="font-size: 0.875rem; color: #cbd5e1;">${new Date(manifest.generated_at).toLocaleString()}</div>
                                </div>
                            </div>
                        </div>
                    </div>
            `;
            
            // FIRMWARE CARD
            if (firmwareInfo && firmwareInfo.length > 0) {
                html += `
                    <div class="container">
                        <div class="accordion">
                            <div class="accordion-header" onclick="toggleAccordion('ota-firmware')">
                                <div class="accordion-title">
                                    <span>⚡</span>
                                    Firmware Updates
                                    <span style="font-size: 0.875rem; color: #94a3b8; font-weight: 400; margin-left: auto;">${firmwareInfo.length} devices</span>
                                </div>
                                <div class="accordion-icon" id="icon-ota-firmware">▼</div>
                            </div>
                            <div class="accordion-content" id="content-ota-firmware">
                                <div class="file-list-padding">
                `;
                
                for (const fw of firmwareInfo) {
                    html += `
                        <div class="file-card firmware-card" style="margin-bottom: 16px;">
                            <div style="display: flex; gap: 24px; align-items: flex-start;">
                                <div class="icon-wrapper icon-firmware">⚡</div>
                                <div style="flex: 1;">
                                    <div style="margin-bottom: 24px;">
                                        <h3 style="font-size: 1rem; color: #94a3b8; text-transform: uppercase; letter-spacing: 0.05em; margin-bottom: 8px;">OTA Group</h3>
                                        <div style="display: flex; align-items: center; gap: 12px;">
                                            <h2 style="font-size: 1.5rem; font-weight: 700; color: #f1f5f9;">${fw.device_name}</h2>
                                            <span style="background: rgba(34, 197, 94, 0.2); color: #4ade80; padding: 4px 12px; border-radius: 99px; font-size: 0.75rem; font-weight: 600;">Latest</span>
                                        </div>
                                    </div>

                                    <div class="firmware-grid">
                                        <div class="firmware-meta">
                                            <div class="firmware-meta-label">Size</div>
                                            <div class="firmware-meta-value">${fw.size_human}</div>
                                        </div>
                                        <div class="firmware-meta">
                                            <div class="firmware-meta-label">MD5</div>
                                            <div class="firmware-meta-value truncate font-mono" style="font-size: 0.875rem;">${fw.md5 ? fw.md5.substring(0, 12) + '...' : 'N/A'}</div>
                                        </div>
                                        <div class="firmware-meta">
                                            <div class="firmware-meta-label">Updated</div>
                                            <div class="firmware-meta-value" style="font-size: 0.875rem;">${new Date(fw.modified).toLocaleDateString()}</div>
                                        </div>
                                    </div>
                                    <div style="display: flex; gap: 12px; flex-wrap: wrap; margin-top: 24px;">
                                        <a href="${fw.download_url}" class="btn btn-danger" style="display: flex; align-items: center; gap: 8px;">
                                            <span>⬇</span> Download Firmware
                                        </a>
                                        <button onclick="navigator.clipboard.writeText('${fw.md5 || ''}'); alert('MD5 copied!')" class="btn btn-secondary">
                                            📋 Copy MD5
                                        </button>
                                    </div>
                                </div>
                            </div>
                        </div>
                    `;
                }
                
                html += `
                                </div>
                            </div>
                        </div>
                    </div>
                `;
            }
            
            // FILES GRID
            html += `
                <div class="container" style="padding-top: 48px; padding-bottom: 48px;">
                    <h2 class="section-title">Data Files</h2>
            `;
            
            let hasFiles = false;
            for (const [catKey, cat] of Object.entries(manifest.categories || {})) {
                if (cat.files.length === 0) continue;
                hasFiles = true;
                
                html += `
                    <div class="accordion">
                        <div class="accordion-header" onclick="toggleAccordion('${catKey}')">
                            <div class="accordion-title">
                                <span>${getCategoryIcon(catKey)}</span>
                                ${cat.name}
                                <span style="font-size: 0.875rem; color: #94a3b8; font-weight: 400; margin-left: auto;">${cat.files.length} files</span>
                            </div>
                            <div class="accordion-icon" id="icon-${catKey}">▼</div>
                        </div>
                        <div class="accordion-content" id="content-${catKey}">
                            <div class="file-list-padding">
                `;
                
                for (const file of cat.files) {
                    html += `
                        <div class="file-card" style="margin: 0;">
                            <div class="file-content">
                                <div class="file-info">
                                    <h4 class="file-name">${file.name}</h4>
                                    <div class="file-meta">
                                        <span>💾 ${formatBytes(file.size)}</span>
                                        <span class="font-mono">🔐 ${file.md5.substring(0, 8)}...</span>
                                    </div>
                                </div>
                                <a href="${file.url}" class="btn btn-primary">
                                    ⬇
                                </a>
                            </div>
                        </div>
                    `;
                }
                
                html += `
                            </div>
                        </div>
                    </div>
                `;
            }
            
            if (!hasFiles) {
                html += `
                    <div class="empty-state">
                        <div class="empty-icon">📦</div>
                        <p style="font-size: 1.25rem; color: #94a3b8; margin-bottom: 8px;">No files available yet</p>
                        <p style="font-size: 0.875rem; color: #64748b;">Pipeline is working...</p>
                    </div>
                `;
            }
            
            html += `
                </div>
            `;
            
            root.innerHTML = html;
        }

        function toggleAccordion(id) {
            const content = document.getElementById(`content-${id}`);
            const icon = document.getElementById(`icon-${id}`);
            content.classList.toggle('open');
            icon.classList.toggle('rotate');
        }



        // Initialize the app
        fetchData();
    </script>
</body>
</html>
"""

# Routes

@app.route('/')
def index():
    """Serve the main UI."""
    return render_template_string(HTML_TEMPLATE)

@app.route('/manifest')
def get_manifest():
    """Get the file manifest as JSON."""
    manifest = load_manifest()
    return jsonify(manifest)

@app.route('/manifest/refresh', methods=['POST'])
def refresh_manifest():
    """Regenerate the manifest by scanning the directory."""
    manifest = generate_manifest()
    return jsonify({"status": "success", "message": "Manifest refreshed", "total_files": manifest["total_files"]})

# Route for /sync-folders removed - use CLI flag --resync for offline sync

@app.route('/api/files/list', methods=['GET'])
def api_files_list():
    """Get list of all available files with MD5 checksums and metadata."""
    manifest = load_manifest()
    
    files_info = []
    for category, files in manifest.get("categories", {}).items():
        for file_info in files.get("files", []):
            download_url = f"http://{request.host}/files/{file_info['path']}"
            md5_url = f"http://{request.host}/api/md5/{file_info['path']}"
            
            file_details = {
                "name": file_info.get("name", ""),
                "path": file_info.get("path", ""),
                "category": category,
                "size_bytes": file_info.get("size", 0),
                "size_human": format_bytes(file_info.get("size", 0)),
                "md5": file_info.get("md5", ""),
                "download_url": download_url,
                "md5_url": md5_url,
                "description": file_info.get("description", "")
            }
            files_info.append(file_details)
    
    return jsonify({
        "status": "success",
        "timestamp": manifest.get("generated_at", ""),
        "server_url": f"http://{request.host}",
        "total_files": len(files_info),
        "total_size_bytes": manifest.get("total_size", 0),
        "total_size_human": format_bytes(manifest.get("total_size", 0)),
        "files": files_info
    })

@app.route('/api/md5/<path:filepath>', methods=['GET'])
def api_get_md5(filepath):
    """Get MD5 checksum for a specific file from pipeline-generated sidecar."""
    try:
        file_path = safe_join(HOST_DIR, filepath)
        
        if not file_path or not os.path.exists(file_path):
            return jsonify({"status": "error", "message": "File not found"}), 404
        
        if not os.path.isfile(file_path):
            return jsonify({"status": "error", "message": "Path is not a file"}), 400
        
        # Read MD5 from sidecar file
        md5_hash = read_md5_sidecar(file_path)
        
        if not md5_hash:
            return jsonify({
                "status": "error",
                "message": "MD5 file not found - pipeline may not have processed this file yet"
            }), 404
        
        return jsonify({
            "status": "success",
            "filepath": filepath,
            "md5": md5_hash,
            "download_url": f"http://{request.host}/files/{filepath}",
            "file_size": os.path.getsize(file_path)
        })
    
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/api/check-update/<path:filepath>', methods=['GET'])
def api_check_update(filepath):
    """Check if a file has been updated since a given timestamp or MD5."""
    server_md5 = request.args.get('md5')
    client_timestamp = request.args.get('timestamp')
    
    try:
        file_path = safe_join(HOST_DIR, filepath)
        
        if not file_path or not os.path.exists(file_path):
            return jsonify({"status": "error", "message": "File not found"}), 404
        
        if not os.path.isfile(file_path):
            return jsonify({"status": "error", "message": "Path is not a file"}), 400
        
        # Get file MD5
        md5_file_path = f"{file_path}.md5"
        current_md5 = None
        
        if os.path.exists(md5_file_path):
            try:
                with open(md5_file_path, 'r') as f:
                    current_md5 = f.read().strip().split()[0]
            except Exception:
                pass
        
        if not current_md5:
            current_md5 = calculate_md5(file_path)
        
        # Compare MD5
        if server_md5:
            has_update = server_md5 != current_md5
            return jsonify({
                "status": "success",
                "has_update": has_update,
                "server_md5": current_md5,
                "client_md5": server_md5,
                "file_size": os.path.getsize(file_path),
                "download_url": f"http://{request.host}/files/{filepath}" if has_update else None
            })
        
        # Compare timestamp
        if client_timestamp:
            try:
                client_ts = float(client_timestamp)
                file_mtime = os.path.getmtime(file_path)
                has_update = file_mtime > client_ts
                return jsonify({
                    "status": "success",
                    "has_update": has_update,
                    "server_timestamp": file_mtime,
                    "client_timestamp": client_ts,
                    "server_md5": current_md5,
                    "file_size": os.path.getsize(file_path),
                    "download_url": f"http://{request.host}/files/{filepath}" if has_update else None
                })
            except ValueError:
                return jsonify({"status": "error", "message": "Invalid timestamp format"}), 400
        
        return jsonify({
            "status": "success",
            "server_md5": current_md5,
            "file_size": os.path.getsize(file_path),
            "file_mtime": os.path.getmtime(file_path),
            "download_url": f"http://{request.host}/files/{filepath}"
        })
    
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/api/file-info/<path:filepath>', methods=['GET'])
def api_file_info(filepath):
    """Get detailed information about a specific file."""
    try:
        file_path = safe_join(HOST_DIR, filepath)
        
        if not file_path or not os.path.exists(file_path):
            return jsonify({"status": "error", "message": "File not found"}), 404
        
        if not os.path.isfile(file_path):
            return jsonify({"status": "error", "message": "Path is not a file"}), 400
        
        file_size = os.path.getsize(file_path)
        file_mtime = os.path.getmtime(file_path)
        
        # Get MD5 from sidecar
        md5_hash = read_md5_sidecar(file_path) or "unknown"
        
        return jsonify({
            "status": "success",
            "filepath": filepath,
            "filename": os.path.basename(file_path),
            "size_bytes": file_size,
            "size_human": format_bytes(file_size),
            "md5": md5_hash,
            "modified_timestamp": file_mtime,
            "modified_datetime": datetime.fromtimestamp(file_mtime).isoformat(),
            "download_url": f"http://{request.host}/files/{filepath}",
            "md5_url": f"http://{request.host}/api/md5/{filepath}",
            "md5_file_url": f"http://{request.host}/files/{filepath}.md5",
            "mime_type": mimetypes.guess_type(file_path)[0] or "application/octet-stream"
        })
    
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/api/md5-list', methods=['GET'])
def api_md5_list():
    """Get complete list of all files with MD5 checksums from pipeline sidecars."""
    md5_list = []
    
    for root, dirs, files in os.walk(HOST_DIR):
        for filename in files:
            if filename.endswith(".md5") or filename in ["manifest.json", "metadata.json"]:
                continue
            
            file_path = os.path.join(root, filename)
            rel_path = os.path.relpath(file_path, HOST_DIR)
            
            try:
                # Read MD5 from sidecar file
                file_md5 = read_md5_sidecar(file_path)
                
                if file_md5:
                    md5_list.append({
                        "path": rel_path.replace(os.sep, '/'),
                        "md5": file_md5,
                        "size": os.path.getsize(file_path)
                    })
            except Exception:
                pass
    
    return jsonify({
        "status": "success",
        "total_files": len(md5_list),
        "timestamp": datetime.utcnow().isoformat(),
        "server_url": f"http://{request.host}",
        "files": md5_list
    })

@app.route('/api/valhalla-tiles', methods=['GET'])
def api_valhalla_tiles():
    """Get Valhalla tiles manifest created by pipeline and packed in tar."""
    # Try to load manifest from valhalla directory
    tiles_manifest = load_valhalla_tiles_manifest()
    
    if not tiles_manifest:
        return jsonify({"status": "error", "message": "No Valhalla manifest found - data may not be available yet"}), 404
    
    return jsonify({
        "status": "success",
        "timestamp": datetime.utcnow().isoformat(),
        "server_url": f"http://{request.host}",
        "total_files": len(tiles_manifest.get("files", {})),
        "valhalla_manifest": tiles_manifest
    })

@app.route('/files/<path:filename>.md5', methods=['GET'])
def download_md5(filename):
    """Serve MD5 sidecar files created by pipeline."""
    try:
        file_path = safe_join(HOST_DIR, filename)
        md5_file_path = f"{file_path}.md5"
        
        if not os.path.exists(md5_file_path):
            return "MD5 file not found - pipeline may not have processed this file yet", 404
        
        # Serve existing MD5 file
        with open(md5_file_path, 'r') as f:
            md5_content = f.read()
        
        return md5_content, 200, {'Content-Type': 'text/plain'}
    
    except Exception as e:
        return f"Error: {str(e)}", 500

@app.route('/api/ota-manifest', methods=['GET'])
def api_ota_manifest():
    """
    OTA Update manifest - provides all file information for automated update checking.
    Perfect for mobile apps and automated deployment systems.
    Includes last successful run timestamp for version tracking.
    """
    current_manifest = load_manifest()
    last_run = get_last_successful_run()
    
    ota_files = []
    for category, files in current_manifest.get("categories", {}).items():
        for file_info in files.get("files", []):
            ota_entry = {
                "name": file_info.get("name", ""),
                "path": file_info.get("path", ""),
                "category": category,
                "size_bytes": file_info.get("size", 0),
                "md5": file_info.get("md5", ""),
                "urls": {
                    "download": f"http://{request.host}/files/{file_info['path']}",
                    "md5": f"http://{request.host}/api/md5/{file_info['path']}",
                    "info": f"http://{request.host}/api/file-info/{file_info['path']}"
                }
            }
            ota_files.append(ota_entry)
    
    return jsonify({
        "status": "success",
        "version": "1.0",
        "server_version": SERVER_VERSION,
        "timestamp": current_manifest.get("generated_at", ""),
        "last_successful_run": last_run,
        "server_url": f"http://{request.host}",
        "base_url": f"http://{request.host}/files/",
        "total_files": len(ota_files),
        "total_size_bytes": current_manifest.get("total_size", 0),
        "files": ota_files
    })

@app.route('/stats')
def get_stats():
    """Get server statistics."""
    uptime_seconds = time.time() - server_stats["start_time"]
    days = int(uptime_seconds // 86400)
    hours = int((uptime_seconds % 86400) // 3600)
    minutes = int((uptime_seconds % 3600) // 60)
    
    uptime_str = f"{days}d {hours}h {minutes}m"
    
    return jsonify({
        "uptime": uptime_str,
        "total_downloads": server_stats["total_downloads"],
        "active_connections": server_stats["active_connections"]
    })

@app.route('/health')
def health():
    """Health check endpoint with detailed status."""
    uptime = time.time() - server_stats['start_time']
    
    active_host = get_active_host()
    inactive_host_dir = get_inactive_host_dir()
    
    # Check critical paths
    checks = {
        'active_host_dir': os.path.exists(HOST_DIR),
        'host1_dir': os.path.exists(HOST1_DIR),
        'host2_dir': os.path.exists(HOST2_DIR),
        'manifest': os.path.exists(MANIFEST_FILE) if MANIFEST_FILE else False,
        'public_dir': os.path.exists(PUBLIC_DIR),
        'deployment_marker_present': os.path.exists(COMPLETED_MARKER)
    }
    
    status = "healthy" if checks['active_host_dir'] and checks['manifest'] else "degraded"
    
    return jsonify({
        "status": status,
        "version": SERVER_VERSION,
        "timestamp": datetime.utcnow().isoformat(),
        "uptime_seconds": int(uptime),
        "deployment": {
            "mode": "blue-green",
            "active_host": active_host,
            "active_dir": HOST_DIR,
            "inactive_dir": inactive_host_dir,
            "deployment_pending": checks['deployment_marker_present']
        },
        "checks": checks,
        "stats": {
            "total_downloads": server_stats['total_downloads'],
            "active_connections": server_stats['active_connections']
        }
    }), 200 if status == "healthy" else 503

@app.route('/firmware/md5')
@app.route('/firmware.md5')
def firmware_md5():
    """Get firmware MD5 hash."""
    try:
        md5_file = f"{FIRMWARE_HOST_PATH}.md5"
        if os.path.exists(md5_file):
            with open(md5_file, 'r') as f:
                md5_hash = f.read().strip()
            return Response(md5_hash + "\n", mimetype='text/plain')
        elif os.path.exists(FIRMWARE_HOST_PATH):
            # Calculate on the fly if sidecar missing
            md5_hash = calculate_file_md5(FIRMWARE_HOST_PATH)
            if md5_hash:
                return Response(md5_hash + "\n", mimetype='text/plain')
        
        return jsonify({"error": "Firmware not available"}), 404
    except Exception as e:
        logger.error(f"Error serving firmware MD5: {e}")
        return jsonify({"error": str(e)}), 500

@app.route('/firmware/info')
def firmware_info():
    """Get list of available firmware for all devices."""
    try:
        firmware_list = []
        
        # Scan for directories containing firmware.bin
        if os.path.exists(HOST_DIR):
            for item in os.listdir(HOST_DIR):
                item_path = os.path.join(HOST_DIR, item)
                if os.path.isdir(item_path):
                    fw_path = os.path.join(item_path, "firmware.bin")
                    if os.path.exists(fw_path):
                        # Found a firmware device
                        device_name = item
                        file_size = os.path.getsize(fw_path)
                        file_mtime = os.path.getmtime(fw_path)
                        
                        # Get MD5
                        md5_file = f"{fw_path}.md5"
                        md5_hash = None
                        if os.path.exists(md5_file):
                            with open(md5_file, 'r') as f:
                                md5_hash = f.read().strip()
                        else:
                            md5_hash = calculate_file_md5(fw_path)
                            
                        firmware_list.append({
                            "device_name": device_name,
                            "filename": "firmware.bin",
                            "size": file_size,
                            "size_human": format_bytes(file_size),
                            "md5": md5_hash,
                            "modified": datetime.fromtimestamp(file_mtime).isoformat(),
                            "download_url": f"/firmware/{device_name}/firmware.bin",
                            "md5_url": f"/firmware/{device_name}/md5"
                        })
        
        return jsonify(firmware_list)
    except Exception as e:
        logger.error(f"Error getting firmware info: {e}")
        return jsonify({"error": str(e)}), 500

@app.route('/firmware/<device_name>/firmware.bin')
def download_firmware(device_name):
    """Download firmware with range support for a specific device."""
    try:
        server_stats["active_connections"] += 1
        server_stats["total_downloads"] += 1
        
        # Sanitize device name
        device_name = os.path.basename(device_name)
        firmware_path = os.path.join(HOST_DIR, device_name, "firmware.bin")
        
        if not os.path.exists(firmware_path):
            return jsonify({"error": f"Firmware not available for {device_name}"}), 404
        
        file_size = os.path.getsize(firmware_path)
        
        # Handle range requests for resumable downloads
        range_header = request.headers.get('Range')
        if range_header:
            try:
                range_match = re.match(r'bytes=(\d+)-(\d*)', range_header)
                start = int(range_match.group(1))
                end = int(range_match.group(2)) if range_match.group(2) else file_size - 1
                
                logger.info(f"Firmware range request: {start}-{end}/{file_size}")
                
                with open(firmware_path, 'rb') as f:
                    f.seek(start)
                    data = f.read(end - start + 1)
                
                response = Response(data, status=206)
                response.headers['Content-Range'] = f'bytes {start}-{end}/{file_size}'
                response.headers['Content-Length'] = len(data)
                response.headers['Accept-Ranges'] = 'bytes'
                response.headers['Content-Type'] = 'application/octet-stream'
                response.headers['Content-Disposition'] = 'attachment; filename=firmware.bin'
                return response
            except Exception as e:
                logger.warning(f"Range request error: {e}")
        
        # Regular download with caching
        response = send_file(
            firmware_path,
            mimetype='application/octet-stream',
            as_attachment=True,
            download_name='firmware.bin'
        )
        
        # Add caching headers for immutable firmware
        md5_file = f"{firmware_path}.md5"
        if os.path.exists(md5_file):
            with open(md5_file, 'r') as f:
                md5_hash = f.read().strip()
            response.headers['ETag'] = f'"{md5_hash}"'
        
        response.headers['Cache-Control'] = 'public, max-age=31536000, immutable'
        response.headers['Accept-Ranges'] = 'bytes'
        
        return response
    
    except Exception as e:
        logger.error(f"Error downloading firmware: {e}")
        return jsonify({"error": str(e)}), 500
    finally:
        server_stats["active_connections"] -= 1

@app.route('/firmware/<device_name>/md5')
def download_firmware_md5(device_name):
    """Get firmware MD5 hash for a specific device."""
    try:
        # Sanitize device name to prevent directory traversal
        device_name = os.path.basename(device_name)
        firmware_path = os.path.join(HOST_DIR, device_name, "firmware.bin")
        
        md5_file = f"{firmware_path}.md5"
        if os.path.exists(md5_file):
            with open(md5_file, 'r') as f:
                md5_hash = f.read().strip()
            return Response(md5_hash + "\n", mimetype='text/plain')
        elif os.path.exists(firmware_path):
            md5_hash = calculate_file_md5(firmware_path)
            if md5_hash:
                return Response(md5_hash + "\n", mimetype='text/plain')
        
        return jsonify({"error": f"Firmware MD5 not available for {device_name}"}), 404
    except Exception as e:
        logger.error(f"Error serving firmware MD5: {e}")
        return jsonify({"error": str(e)}), 500

@app.route('/files/<path:filename>')
def download_file(filename):
    """Serve a file from the host directory with range support for resumable downloads."""
    try:
        server_stats["active_connections"] += 1
        
        file_path = safe_join(HOST_DIR, filename)
        
        if not file_path or not os.path.exists(file_path):
            logger.warning(f"File not found: {filename}")
            abort(404)
        
        if not os.path.isfile(file_path):
            logger.warning(f"Path is not a file: {filename}")
            abort(400, description="Path is not a file")
        
        file_size = os.path.getsize(file_path)
        file_mtime = os.path.getmtime(file_path)
        etag = f'"{file_size}-{int(file_mtime)}"'
        
        # Check If-None-Match for caching
        if request.headers.get('If-None-Match') == etag:
            return Response(status=304)
        
        # Determine MIME type
        mime_type = mimetypes.guess_type(file_path)[0]
        if file_path.endswith('.lz4'):
            mime_type = 'application/x-lz4'
        elif file_path.endswith('.mbtiles'):
            mime_type = 'application/x-sqlite3'
        if not mime_type:
            mime_type = 'application/octet-stream'
        
        # Check for Range header (for resumable downloads)
        range_header = request.headers.get('Range')
        
        if range_header:
            # Parse range header
            try:
                byte_range = range_header.replace('bytes=', '').split('-')
                start = int(byte_range[0]) if byte_range[0] else 0
                end = int(byte_range[1]) if byte_range[1] else file_size - 1
                
                # Validate range
                if start >= file_size or end >= file_size or start > end:
                    return Response(status=416)  # Range Not Satisfiable
                
                length = end - start + 1
                
                # Stream the requested range
                def generate():
                    with open(file_path, 'rb') as f:
                        f.seek(start)
                        remaining = length
                        chunk_size = 8192
                        while remaining > 0:
                            chunk = f.read(min(chunk_size, remaining))
                            if not chunk:
                                break
                            remaining -= len(chunk)
                            yield chunk
                
                response = Response(generate(), 206, mimetype=mime_type)
                response.headers['Content-Range'] = f'bytes {start}-{end}/{file_size}'
                response.headers['Accept-Ranges'] = 'bytes'
                response.headers['Content-Length'] = str(length)
                response.headers['Content-Disposition'] = f'attachment; filename="{os.path.basename(file_path)}"'
                response.headers['ETag'] = etag
                response.headers['Cache-Control'] = 'public, max-age=31536000, immutable'
                
                logger.info(f"Serving range {start}-{end} of {filename}")
                return response
            except (ValueError, IndexError) as e:
                logger.error(f"Invalid range header: {range_header}")
                return Response(status=416)
        
        # Normal download (no range request) - use streaming
        def generate():
            with open(file_path, 'rb') as f:
                chunk_size = 8192  # 8KB chunks for faster streaming
                while True:
                    chunk = f.read(chunk_size)
                    if not chunk:
                        break
                    yield chunk
        
        response = Response(generate(), mimetype=mime_type)
        response.headers['Content-Length'] = str(file_size)
        response.headers['Content-Disposition'] = f'attachment; filename="{os.path.basename(file_path)}"'
        response.headers['Accept-Ranges'] = 'bytes'
        response.headers['ETag'] = etag
        response.headers['Cache-Control'] = 'public, max-age=31536000, immutable'
        response.headers['X-Content-Type-Options'] = 'nosniff'
        
        server_stats["total_downloads"] += 1
        logger.info(f"Serving file: {filename} ({format_bytes(file_size)})")
        return response
    
    except Exception as e:
        logger.error(f"Error serving file {filename}: {e}", exc_info=True)
        abort(500, description=str(e))
    finally:
        server_stats["active_connections"] -= 1

def run_resync_only():
    """Run sync operation and exit (offline mode). Firmware built by pipeline."""
    logger.info("=" * 60)
    logger.info("OFFLINE RESYNC MODE")
    logger.info("=" * 60)
    
    # Initialize active host
    active = get_active_host()
    set_active_host(active)
    
    ensure_directories()
    
    # Track overall success
    overall_success = True
    
    # 1. Copy firmware from public_export (built by pipeline)
    logger.info("\n[1/2] Copying firmware from public_export...")
    firmware_src = os.path.join(PUBLIC_DIR, "esp32wrover")
    if os.path.exists(firmware_src):
        firmware_dst = os.path.join(HOST_DIR, "esp32wrover")
        try:
            if os.path.exists(firmware_dst):
                shutil.rmtree(firmware_dst)
            shutil.copytree(firmware_src, firmware_dst)
            logger.info(f"✓ Firmware copied to: {firmware_dst}")
        except Exception as e:
            logger.warning(f"Failed to copy firmware: {e}")
            overall_success = False
    else:
        logger.warning("No firmware in public_export (pipeline may not have built it)")
    
    # 2. Sync files from public_export
    logger.info("\n[2/2] Syncing data files from public_export...")
    public_has_files = os.path.exists(PUBLIC_DIR) and any(
        f for f in os.listdir(PUBLIC_DIR) 
        if f not in ["manifest.json", "metadata.json"] and not f.endswith(".md5")
    )
    
    if public_has_files:
        logger.info("Syncing files from public_export to host...")
        # Copy files directly
        if not os.path.exists(PUBLIC_DIR):
            logger.warning("public_export does not exist")
        else:
            os.makedirs(HOST_DIR, exist_ok=True)
            
            try:
                for root, dirs, files in os.walk(PUBLIC_DIR):
                    rel_root = os.path.relpath(root, PUBLIC_DIR)
                    
                    if rel_root != '.':
                        host_dir = os.path.join(HOST_DIR, rel_root)
                        os.makedirs(host_dir, exist_ok=True)
                    
                    for file in files:
                        if file == "metadata.json":
                            continue
                        
                        src_path = os.path.join(root, file)
                        if rel_root == '.':
                            dst_path = os.path.join(HOST_DIR, file)
                        else:
                            dst_path = os.path.join(HOST_DIR, rel_root, file)
                        
                        should_copy = False
                        
                        if not os.path.exists(dst_path):
                            should_copy = True
                        else:
                            src_mtime = os.path.getmtime(src_path)
                            dst_mtime = os.path.getmtime(dst_path)
                            if src_mtime > dst_mtime:
                                should_copy = True
                        
                        if should_copy:
                            try:
                                shutil.copy2(src_path, dst_path)
                                logger.info(f"Synced: {os.path.join(rel_root, file) if rel_root != '.' else file}")
                            except Exception as e:
                                logger.error(f"Failed to copy {file}: {e}")
                
                logger.info("✓ Data sync completed")
            except Exception as e:
                logger.error(f"Data sync failed: {e}")
                overall_success = False
    else:
        logger.warning("No files found in public_export directory")
    
    # 3. Generate manifest
    logger.info("\nGenerating manifest...")
    manifest = generate_manifest()
    logger.info(f"Manifest updated: {manifest['total_files']} files, {format_bytes(manifest['total_size'])}")
    
    logger.info("\n" + "=" * 60)
    if overall_success:
        logger.info("RESYNC COMPLETED SUCCESSFULLY")
        logger.info("=" * 60)
        return 0
    else:
        logger.warning("RESYNC COMPLETED WITH WARNINGS")
        logger.info("=" * 60)
        return 1

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='OSM Pipeline File Server (Cloudflare Tunnel Ready)')
    parser.add_argument('--resync', action='store_true', 
                       help='Run resync operation and exit (offline mode)')
    parser.add_argument('--host', default='127.0.0.1', 
                       help='Host to bind to (default: 127.0.0.1 for Cloudflare Tunnel; use 0.0.0.0 for direct network access)')
    parser.add_argument('--port', type=int, default=8080, 
                       help='Port to bind to (default: 8080)')
    parser.add_argument('--no-monitor', action='store_true',
                       help='Disable automatic monitoring for updates')
    args = parser.parse_args()
    
    # Handle resync-only mode
    if args.resync:
        sys.exit(run_resync_only())
    
    # Initialize active host directory
    logger.info("=" * 60)
    logger.info("BLUE-GREEN DEPLOYMENT INITIALIZATION")
    logger.info("=" * 60)
    active = get_active_host()
    set_active_host(active)
    logger.info(f"Active host: {active}")
    logger.info(f"Serving from: {HOST_DIR}")
    logger.info("=" * 60)
    
    # Normal server startup
    logger.info("=" * 60)
    logger.info("OSM Pipeline File Server v" + SERVER_VERSION)
    logger.info("=" * 60)
    logger.info(f"Base Directory: {BASE_DIR}")
    logger.info(f"Public Directory (build): {PUBLIC_DIR}")
    logger.info(f"Host1 Directory: {HOST1_DIR}")
    logger.info(f"Host2 Directory: {HOST2_DIR}")
    logger.info(f"Active Host Directory: {HOST_DIR}")
    logger.info(f"Manifest File: {MANIFEST_FILE}")
    logger.info(f"Deployment Marker: {COMPLETED_MARKER}")
    logger.info("=" * 60)
    
    # Ensure all directories exist
    logger.info("\nSetting up directories...")
    ensure_directories()

    # Generate initial manifest from active host
    logger.info("\nGenerating initial manifest...")
    manifest = generate_manifest()
    logger.info(f"\nFound {manifest['total_files']} files in active host")
    logger.info(f"Total size: {format_bytes(manifest['total_size'])}")
    logger.info("\n" + "=" * 60)
    logger.info(f"Server starting on http://{args.host}:{args.port}")
    if not args.no_monitor:
        logger.info(f"Monitoring for deployment marker: {COMPLETED_MARKER}")
        logger.info("Blue-green deployment enabled (zero downtime updates)")
    logger.info("=" * 60)
    
    # Start monitor thread unless disabled
    if not args.no_monitor:
        monitor_thread = threading.Thread(target=monitor_for_updates, daemon=True)
        monitor_thread.start()
        logger.info("Monitor thread started\n")
    
    # Run the server with production settings
    try:
        app.run(host=args.host, port=args.port, debug=False, threaded=True)
    except KeyboardInterrupt:
        logger.info("\nServer shutting down...")
    except Exception as e:
        logger.error(f"Server error: {e}", exc_info=True)
        sys.exit(1)