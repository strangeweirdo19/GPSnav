#!/usr/bin/env python3
"""
OTA Firmware Server
Serves firmware file and calculates MD5 hash only on file changes.
"""

import http.server
import socketserver
import hashlib
import os
import time
from pathlib import Path
from threading import Thread
import socket

# Configuration
PORT = 8080
FIRMWARE_PATH = ".pio/build/esp-wrover-kit/firmware.bin"

# Global state
current_md5 = None
last_modified_time = None


def calculate_md5(filepath):
    """Calculate MD5 hash of a file."""
    md5_hash = hashlib.md5()
    try:
        with open(filepath, "rb") as f:
            # Read in chunks to handle large files
            for chunk in iter(lambda: f.read(4096), b""):
                md5_hash.update(chunk)
        return md5_hash.hexdigest()
    except FileNotFoundError:
        return None


def get_local_ip():
    """Get local IP address."""
    try:
        # Create a socket to determine local IP
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
        s.close()
        return local_ip
    except:
        return "127.0.0.1"


def monitor_firmware_file():
    """Monitor firmware file for changes and update MD5."""
    global current_md5, last_modified_time
    
    print(f"\n📁 Monitoring: {FIRMWARE_PATH}")
    
    while True:
        try:
            if os.path.exists(FIRMWARE_PATH):
                # Get file modification time
                mod_time = os.path.getmtime(FIRMWARE_PATH)
                
                # Check if file was modified
                if last_modified_time is None or mod_time != last_modified_time:
                    last_modified_time = mod_time
                    file_size = os.path.getsize(FIRMWARE_PATH)
                    
                    print(f"\n🔄 File changed detected!")
                    print(f"   Size: {file_size:,} bytes ({file_size/1024/1024:.2f} MB)")
                    print(f"   Calculating MD5...", end=" ", flush=True)
                    
                    current_md5 = calculate_md5(FIRMWARE_PATH)
                    
                    if current_md5:
                        print(f"✓")
                        print(f"   MD5: {current_md5}")
                        print(f"   Modified: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(mod_time))}")
            else:
                if last_modified_time is not None:
                    print(f"\n⚠️  Firmware file not found: {FIRMWARE_PATH}")
                    current_md5 = None
                    last_modified_time = None
        
        except Exception as e:
            print(f"\n❌ Error monitoring file: {e}")
        
        # Check every 1 second for changes (but only calculate MD5 when changed)
        time.sleep(1)


class OTARequestHandler(http.server.SimpleHTTPRequestHandler):
    """Custom HTTP request handler with logging."""

    def log_message(self, format, *args):
        """Override to customize logging."""
        client_ip = self.client_address[0]
        print(f"📥 {client_ip} - {args[0]} - {args[1]}")

    def end_headers(self):
        """Add CORS headers before finalizing response."""
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET")
        super().end_headers()

    def do_GET(self):
        """Handle GET requests."""
        # Serve MD5 at /md5 or /md5.txt
        if self.path in ("/md5", "/md5.txt"):
            if current_md5:
                payload = f"{current_md5}\n"
                self.send_response(200)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.send_header("Content-Length", str(len(payload)))
                self.end_headers()
                self.wfile.write(payload.encode("utf-8"))
            else:
                payload = "MD5 unavailable\n"
                self.send_response(503)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.send_header("Content-Length", str(len(payload)))
                self.end_headers()
                self.wfile.write(payload.encode("utf-8"))
            return

        return super().do_GET()


def main():
    """Main server function."""
    global current_md5
    
    # Print banner
    print("=" * 60)
    print("🚀 OTA Firmware Server")
    print("=" * 60)
    
    # Check if firmware file exists
    if not os.path.exists(FIRMWARE_PATH):
        print(f"\n⚠️  Firmware file not found: {FIRMWARE_PATH}")
        print("   Build your project first with: platformio run")
        print(f"\n   Waiting for firmware file to appear...\n")
    else:
        # Calculate initial MD5
        file_size = os.path.getsize(FIRMWARE_PATH)
        print(f"\n✓ Firmware file found!")
        print(f"   Size: {file_size:,} bytes ({file_size/1024/1024:.2f} MB)")
        print(f"   Calculating initial MD5...", end=" ", flush=True)
        current_md5 = calculate_md5(FIRMWARE_PATH)
        print(f"✓")
        print(f"   MD5: {current_md5}")
    
    # Start file monitoring thread
    monitor_thread = Thread(target=monitor_firmware_file, daemon=True)
    monitor_thread.start()
    
    # Get local IP
    local_ip = get_local_ip()
    
    # Print connection info
    print(f"\n" + "=" * 60)
    print(f"🌐 Server running on:")
    print(f"   Local:   http://localhost:{PORT}")
    print(f"   Network: http://{local_ip}:{PORT}")
    print("=" * 60)
    print(f"\n📲 BLE Command to send:")
    print(f"   OTA_FW http://{local_ip}:{PORT}/{FIRMWARE_PATH}")
    print("=" * 60)
    print(f"\n🔍 Direct firmware URL:")
    print(f"   http://{local_ip}:{PORT}/{FIRMWARE_PATH}")
    print("=" * 60)
    print(f"\n⌨️  Press Ctrl+C to stop the server\n")
    
    # Start HTTP server
    try:
        with socketserver.TCPServer(("", PORT), OTARequestHandler) as httpd:
            httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n\n👋 Server stopped")
    except OSError as e:
        if "Address already in use" in str(e):
            print(f"\n❌ Error: Port {PORT} is already in use!")
            print(f"   Try stopping other servers or use a different port.")
        else:
            print(f"\n❌ Error: {e}")


if __name__ == "__main__":
    main()
