#!/usr/bin/env python3
"""
Simple MBTiles Generation Pipeline for India
============================================

tile_row convention
-------------------
The MBTiles spec mandates TMS (row 0 = southernmost tile).
tilemaker respects this on some builds/platforms and not others.
Rather than hardcode either assumption, detect_tile_row_convention()
samples a few tiles from the source and decides automatically.
"""

import os
import sys
import subprocess
import sqlite3
import hashlib
import json
import math
import shlex
import shutil
import tarfile
import multiprocessing
from datetime import datetime
from pathlib import Path
from tempfile import NamedTemporaryFile

import requests

try:
    import osmium
except ImportError:
    osmium = None

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
WORK_DIR      = Path("./simple_pipeline_work")
DOWNLOADS_DIR = WORK_DIR / "downloads"
OUTPUT_DIR    = WORK_DIR / "output"
FINAL_DIR     = WORK_DIR / "final_mbtiles"

INDIA_PBF_URL      = "https://download.geofabrik.de/asia/india-latest.osm.pbf"
INDIA_MD5_URL      = "https://download.geofabrik.de/asia/india-latest.osm.pbf.md5"
INDIA_PBF          = DOWNLOADS_DIR / "india-latest.osm.pbf"
INDIA_FILTERED_PBF = DOWNLOADS_DIR / "india-filtered.osm.pbf"

APP_MBTILES       = OUTPUT_DIR / "app.mbtiles"
NAVIGATOR_MBTILES = OUTPUT_DIR / "navigator.mbtiles"
POI_DB            = WORK_DIR / "poi.sqlite"

APP_PROCESS       = "config/process-openmapphone.lua"
APP_CONFIG        = "config/config-navigator.json"
NAVIGATOR_PROCESS = "config/process-openmapphone.lua"
NAVIGATOR_CONFIG  = "config/config-navigator.json"

INDIA_BBOX   = "68.1,6.5,97.4,35.5"
INDIA_LAT_MIN, INDIA_LAT_MAX =  6.0, 36.0
INDIA_LON_MIN, INDIA_LON_MAX = 68.0, 98.0
MIN_GRID_ZOOM = 14

VALHALLA_DIR     = WORK_DIR / "valhalla"
VALHALLA_TILES   = VALHALLA_DIR / "valhalla_tiles"
VALHALLA_CONFIG  = VALHALLA_DIR / "valhalla.json"
VALHALLA_TAR_LZ4 = WORK_DIR / "valhalla_tiles.tar.lz4"

NUM_WORKERS     = max(1, os.cpu_count() or 4)
COMMIT_INTERVAL = 2000
TOTAL_STEPS     = 10

PATCH_FILE_TYPE = "mbtiles"
PATCH_STATE_DIR = Path("/home/navigator/patch_state")
SNAPSHOTS_DIR   = PATCH_STATE_DIR / "snapshots"
PATCHES_PERSISTENT_DIR = PATCH_STATE_DIR / "patches"
APP_BASELINE_MBTILES = PATCH_STATE_DIR / "latest_app.mbtiles"
POI_BASELINE_DB = PATCH_STATE_DIR / "latest_poi.sqlite"
TIMEZONE_HASH_STATE = PATCH_STATE_DIR / "timezone_hash_state.json"

PATCH_SCHEDULES = {
    "incremental": 1,
    "weekly": 7,
    "fortnight": 14,
    "monthly": 28,
    "six_month": 182,
    "yearly": 365,
}

PATCH_RETENTION_COUNT = {
    "incremental": 7,
    "weekly": 4,
    "fortnight": 4,
    "monthly": 6,
    "six_month": 2,
    "yearly": 1,
}

for d in (WORK_DIR, DOWNLOADS_DIR, OUTPUT_DIR, FINAL_DIR):
    d.mkdir(parents=True, exist_ok=True)


# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
def log(msg, level="INFO"):
    print(f"[{level}] {msg}", flush=True)


# ---------------------------------------------------------------------------
# Shell helpers
# ---------------------------------------------------------------------------
def _format_cmd(cmd):
    return " ".join(shlex.quote(part) for part in cmd)


def run_cmd(cmd, description="", timeout=None):
    if description:
        log(description)
    log(f"$ {_format_cmd(cmd)}", "CMD")

    popen_cmd = cmd
    if shutil.which("stdbuf"):
        popen_cmd = ["stdbuf", "-oL", "-eL", *cmd]

    try:
        process = subprocess.Popen(
            popen_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
    except Exception as e:
        log(f"Failed to start command: {e}", "ERROR")
        return False

    try:
        assert process.stdout is not None
        for line in process.stdout:
            print(line.rstrip(), flush=True)
        return_code = process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        process.kill()
        log(f"Command timed out after {timeout} seconds", "ERROR")
        return False

    if return_code != 0:
        log(f"Command failed with exit code {return_code}", "ERROR")
        return False

    return True


# ---------------------------------------------------------------------------
# Download helpers
# ---------------------------------------------------------------------------
def download_with_resume(url, dest):
    dest = Path(dest)
    headers, mode = {}, 'wb'
    if dest.exists():
        current_size = dest.stat().st_size
        headers['Range'] = f'bytes={current_size}-'
        log(f"Resuming download from byte {current_size:,}")
        mode = 'ab'
    log(f"Downloading {url}")
    response = requests.get(url, headers=headers, stream=True, timeout=30)
    response.raise_for_status()
    total_size   = int(response.headers.get('content-length', 0))
    downloaded   = dest.stat().st_size if dest.exists() else 0
    with open(dest, mode) as f:
        for chunk in response.iter_content(chunk_size=65536):
            if chunk:
                f.write(chunk)
                downloaded += len(chunk)
                if downloaded % (100 * 1024 * 1024) < 65536:
                    pct = downloaded / total_size * 100 if total_size else 0
                    log(f"  {downloaded/(1024**3):.2f} GB / "
                        f"{total_size/(1024**3):.2f} GB ({pct:.1f}%)")
    log(f"Download complete: {dest}")
    return True


def fetch_expected_md5(md5_url: str, retries: int = 3) -> str | None:
    for attempt in range(1, retries + 1):
        try:
            response = requests.get(md5_url, timeout=15)
            response.raise_for_status()
            token = response.text.split()[0].strip()
            if token and len(token) == 32:
                return token.lower()
        except Exception as e:
            log(f"MD5 fetch attempt {attempt}/{retries} failed: {e}", "WARN")
    return None


def verify_md5(pbf_file, md5_url, expected_md5: str | None = None):
    if expected_md5 is None:
        log("Downloading MD5 checksum...")
        expected_md5 = fetch_expected_md5(md5_url)
        if not expected_md5:
            log("Could not fetch expected MD5 checksum", "ERROR")
            return False
    log("Calculating local MD5...")
    md5_hash  = hashlib.md5()
    file_size = pbf_file.stat().st_size
    processed = 0
    with open(pbf_file, 'rb') as f:
        for chunk in iter(lambda: f.read(65536), b''):
            md5_hash.update(chunk)
            processed += len(chunk)
            if processed % (200 * 1024 * 1024) < 65536:
                log(f"  MD5: {processed/(1024**3):.2f} / {file_size/(1024**3):.2f} GB")
    actual_md5 = md5_hash.hexdigest()
    if actual_md5 == expected_md5:
        log("✓ MD5 verified", "SUCCESS")
        return True
    log(f"✗ MD5 mismatch! Expected {expected_md5}, got {actual_md5}", "ERROR")

    # Force a clean re-download on any mismatch
    try:
        if pbf_file.exists():
            pbf_file.unlink()
            log(f"Removed stale file after MD5 mismatch: {pbf_file}", "WARN")
        sidecar = Path(str(pbf_file) + ".md5")
        if sidecar.exists():
            sidecar.unlink()
            log(f"Removed stale MD5 sidecar: {sidecar}", "WARN")
    except Exception as e:
        log(f"Failed to remove stale files after MD5 mismatch: {e}", "WARN")

    return False


def _sqlite_connect_ro(path: Path | str):
    # immutable=1 prevents sqlite from trying to create/write sidecar files (.wal/.shm)
    # which can trigger "attempt to write a readonly database" on read-only pipelines.
    conn = sqlite3.connect(f"file:{path}?mode=ro&immutable=1", uri=True)
    conn.execute("PRAGMA query_only=ON")
    return conn


class OSMToSQLiteHandler(osmium.SimpleHandler if osmium is not None else object):
    """Convert OSM PBF to SQLite with FTS search support."""

    def __init__(self, db_path):
        if osmium is None:
            raise RuntimeError("python-osmium is not installed")
        super().__init__()
        self.db_path = db_path
        self.batch = []
        self.batch_size = 10000
        self.total_processed = 0
        self.setup_database()

    def setup_database(self):
        conn = sqlite3.connect(self.db_path)
        cur = conn.cursor()
        cur.execute(
            """
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
            """
        )
        cur.execute("CREATE INDEX IF NOT EXISTS idx_name ON places(name COLLATE NOCASE)")
        cur.execute("CREATE INDEX IF NOT EXISTS idx_type ON places(type)")
        cur.execute("CREATE INDEX IF NOT EXISTS idx_location ON places(lat, lon)")
        cur.execute(
            """
            CREATE VIRTUAL TABLE IF NOT EXISTS places_fts
            USING fts5(name, type, content=places, content_rowid=rowid)
            """
        )
        conn.commit()
        conn.close()

    def get_category_and_type(self, tags):
        priority_categories = [
            ("amenity", "Amenity"),
            ("shop", "Shop"),
            ("tourism", "Tourism"),
            ("leisure", "Leisure"),
            ("highway", "Road"),
            ("building", "Building"),
            ("natural", "Natural"),
            ("place", "Place"),
            ("railway", "Railway"),
            ("public_transport", "Transport"),
        ]

        for tag, category in priority_categories:
            if tag in tags:
                return category, tags[tag]

        return "Other", "unknown"

    def node(self, node):
        if "name" not in node.tags:
            return

        category, type_value = self.get_category_and_type(node.tags)
        self.batch.append({
            "id": f"n{node.id}",
            "name": node.tags["name"],
            "lat": round(node.location.lat, 6),
            "lon": round(node.location.lon, 6),
            "type": type_value,
            "category": category,
            "geometry_type": "point",
            "tags": json.dumps(dict(node.tags)),
        })

        if len(self.batch) >= self.batch_size:
            self.flush_batch()

    def way(self, way):
        if "name" not in way.tags:
            return

        try:
            nodes = list(way.nodes)
            if not nodes:
                return

            lat = sum(node.lat for node in nodes) / len(nodes)
            lon = sum(node.lon for node in nodes) / len(nodes)
            category, type_value = self.get_category_and_type(way.tags)

            if "highway" in way.tags or "railway" in way.tags:
                geometry_type = "line"
            elif "building" in way.tags or "area" in way.tags:
                geometry_type = "polygon"
            else:
                geometry_type = "way"

            self.batch.append({
                "id": f"w{way.id}",
                "name": way.tags["name"],
                "lat": round(lat, 6),
                "lon": round(lon, 6),
                "type": type_value,
                "category": category,
                "geometry_type": geometry_type,
                "tags": json.dumps(dict(way.tags)),
            })

            if len(self.batch) >= self.batch_size:
                self.flush_batch()
        except Exception:
            return

    def flush_batch(self):
        if not self.batch:
            return

        conn = sqlite3.connect(self.db_path)
        cur = conn.cursor()
        cur.executemany(
            """
            INSERT OR REPLACE INTO places
            (id, name, lat, lon, type, category, geometry_type, tags)
            VALUES (:id, :name, :lat, :lon, :type, :category, :geometry_type, :tags)
            """,
            self.batch,
        )

        for item in self.batch:
            cur.execute(
                """
                INSERT INTO places_fts(rowid, name, type)
                SELECT rowid, name, type FROM places WHERE id = ?
                """,
                (item["id"],),
            )

        conn.commit()
        conn.close()
        self.total_processed += len(self.batch)
        log(f"POI rows processed: {self.total_processed:,}")
        self.batch = []

    def finalize(self):
        self.flush_batch()
        conn = sqlite3.connect(self.db_path)
        cur = conn.cursor()
        cur.execute("VACUUM")
        cur.execute("ANALYZE")
        cur.execute("SELECT COUNT(*) FROM places")
        count = cur.fetchone()[0]
        conn.close()
        return count


def generate_poi_database(pbf_file):
    if osmium is None:
        log("python-osmium not installed; cannot generate poi.sqlite", "ERROR")
        return False

    if POI_DB.exists():
        POI_DB.unlink()

    log("Generating poi.sqlite from PBF...")
    handler = OSMToSQLiteHandler(str(POI_DB))
    handler.apply_file(str(pbf_file), locations=True)
    total_places = handler.finalize()
    log(f"✓ POI database ready: {POI_DB} ({total_places:,} places)", "SUCCESS")
    return POI_DB if POI_DB.exists() else False


# ---------------------------------------------------------------------------
# Tile coordinate utilities
# ---------------------------------------------------------------------------

def _south_for_xyz(z, x, xyz_y):
    """South edge of tile treating tile_row as XYZ (row 0 = north)."""
    n = 2 ** z
    return math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * (xyz_y + 1) / n))))


def _south_for_tms(z, x, tms_y):
    """South edge of tile treating tile_row as TMS (row 0 = south)."""
    n     = 2 ** z
    xyz_y = (n - 1) - tms_y
    return math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * (xyz_y + 1) / n))))


def detect_tile_row_convention(source_path):
    """
    Auto-detect whether tile_row is stored as XYZ or TMS by sampling
    tiles and checking which interpretation lands inside India's bbox.

    Returns 'xyz' or 'tms'.
    """
    conn = _sqlite_connect_ro(source_path)
    cur  = conn.cursor()
    # Sample evenly by picking every Nth row
    cur.execute("SELECT zoom_level, tile_column, tile_row FROM tiles LIMIT 50")
    rows = cur.fetchall()
    conn.close()

    xyz_hits = 0
    tms_hits = 0
    for z, x, tr in rows:
        s_xyz = _south_for_xyz(z, x, tr)
        s_tms = _south_for_tms(z, x, tr)
        if INDIA_LAT_MIN <= s_xyz <= INDIA_LAT_MAX:
            xyz_hits += 1
        if INDIA_LAT_MIN <= s_tms <= INDIA_LAT_MAX:
            tms_hits += 1

    log(f"  Convention probe: xyz_hits={xyz_hits}, tms_hits={tms_hits} (out of {len(rows)} samples)")

    if tms_hits > xyz_hits:
        log("  → tile_row is TMS (row 0 = south) — will apply flip", "INFO")
        return 'tms'
    else:
        log("  → tile_row is XYZ (row 0 = north) — no flip needed", "INFO")
        return 'xyz'


def tile_bounds(z, x, tile_row, convention):
    """
    Return (west, south, east, north) for a tile.
    convention: 'xyz' or 'tms'
    """
    n     = 2 ** z
    xyz_y = ((n - 1) - tile_row) if convention == 'tms' else tile_row
    west  = x       / n * 360.0 - 180.0
    east  = (x + 1) / n * 360.0 - 180.0
    north = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 *  xyz_y      / n))))
    south = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * (xyz_y + 1) / n))))
    return west, south, east, north


def tile_to_grid_key(z, x, tile_row, convention):
    """
    Map a tile to its 1° grid cell (SW corner).
    Output: "12_80"  positive, "S12_80" negative lat, "12_W10" negative lon.
    """
    west, south, _e, _n = tile_bounds(z, x, tile_row, convention)
    lat_f = math.floor(south)
    lon_f = math.floor(west)
    lat_s = f"S{abs(lat_f)}" if lat_f < 0 else str(lat_f)
    lon_s = f"W{abs(lon_f)}" if lon_f < 0 else str(lon_f)
    return f"{lat_s}_{lon_s}"


def tile_intersects_india(z, x, tile_row, convention):
    west, south, east, north = tile_bounds(z, x, tile_row, convention)
    return not (
        east <= INDIA_LON_MIN or
        west >= INDIA_LON_MAX or
        north <= INDIA_LAT_MIN or
        south >= INDIA_LAT_MAX
    )


def should_keep_grid_tile(z, x, tile_row, convention):
    if z < MIN_GRID_ZOOM:
        return False
    return tile_intersects_india(z, x, tile_row, convention)


# ---------------------------------------------------------------------------
# Metadata helpers
# ---------------------------------------------------------------------------

def read_source_metadata(source_path):
    conn = _sqlite_connect_ro(source_path)
    cur  = conn.cursor()
    cur.execute("SELECT name, value FROM metadata")
    meta = {row[0]: row[1] for row in cur.fetchall()}
    conn.close()
    return meta


def build_grid_metadata(grid_key, source_meta, tile_coords, convention):
    """
    Full metadata for one grid file.
    Copies source verbatim except name/bounds/center/minzoom/maxzoom
    which are calculated from the actual tiles in this file.
    """
    meta = dict(source_meta)

    min_lon, min_lat =  180.0,  90.0
    max_lon, max_lat = -180.0, -90.0
    zoom_levels_seen = set()

    for z, x, tr in tile_coords:
        west, south, east, north = tile_bounds(z, x, tr, convention)
        min_lon = min(min_lon, west)
        min_lat = min(min_lat, south)
        max_lon = max(max_lon, east)
        max_lat = max(max_lat, north)
        zoom_levels_seen.add(z)

    center_lon  = (min_lon + max_lon) / 2
    center_lat  = (min_lat + max_lat) / 2
    center_zoom = max(zoom_levels_seen) if zoom_levels_seen else 16

    meta['name']    = f"India Navigator {grid_key}"
    meta['bounds']  = f"{min_lon:.6f},{min_lat:.6f},{max_lon:.6f},{max_lat:.6f}"
    meta['center']  = f"{center_lon:.6f},{center_lat:.6f},{center_zoom}"
    if zoom_levels_seen:
        meta['minzoom'] = str(min(zoom_levels_seen))
        meta['maxzoom'] = str(max(zoom_levels_seen))

    return meta


# ---------------------------------------------------------------------------
# Worker process
# ---------------------------------------------------------------------------

def _worker(args):
    """
    Subprocess worker. Reads tiles for assigned grid keys, writes output DBs.
    """
    worker_id, source_path, output_dir, grid_keys, commit_interval, \
        source_meta, convention = args
    output_dir = Path(output_dir)

    src = _sqlite_connect_ro(source_path)

    grid_tile_coords = {gk: [] for gk in grid_keys}
    dbs = {}

    def get_db(gk):
        if gk in dbs:
            return dbs[gk]
        path = output_dir / f"{gk}.mbtiles"
        conn = sqlite3.connect(str(path))
        cur  = conn.cursor()
        cur.executescript("""
            PRAGMA synchronous  = OFF;
            PRAGMA journal_mode = MEMORY;
            PRAGMA temp_store   = MEMORY;
            PRAGMA cache_size   = -65536;
            CREATE TABLE IF NOT EXISTS metadata (
                name  TEXT PRIMARY KEY,
                value TEXT
            );
            CREATE TABLE IF NOT EXISTS tiles (
                zoom_level  INTEGER,
                tile_column INTEGER,
                tile_row    INTEGER,
                tile_data   BLOB,
                PRIMARY KEY (zoom_level, tile_column, tile_row)
            );
        """)
        conn.commit()
        dbs[gk] = (conn, cur, 0)
        return dbs[gk]

    src_cur = src.cursor()
    src_cur.execute(
        "SELECT zoom_level, tile_column, tile_row, tile_data FROM tiles"
    )

    processed = 0
    skipped   = 0
    skipped_low_zoom = 0
    skipped_outside_india = 0

    for z, x, tr, tile_data in src_cur:
        if z < MIN_GRID_ZOOM:
            skipped += 1
            skipped_low_zoom += 1
            continue

        if not tile_intersects_india(z, x, tr, convention):
            skipped += 1
            skipped_outside_india += 1
            continue

        gk = tile_to_grid_key(z, x, tr, convention)
        if gk not in grid_keys:
            skipped += 1
            continue

        conn, cur, pending = get_db(gk)
        cur.execute(
            "INSERT OR REPLACE INTO tiles "
            "(zoom_level, tile_column, tile_row, tile_data) VALUES (?, ?, ?, ?)",
            (z, x, tr, tile_data)
        )
        grid_tile_coords[gk].append((z, x, tr))
        pending += 1
        dbs[gk]  = (conn, cur, pending)

        if pending >= commit_interval:
            conn.commit()
            dbs[gk] = (conn, cur, 0)

        processed += 1

    # Write correct metadata for each grid file
    for gk, (conn, cur, _pending) in dbs.items():
        meta = build_grid_metadata(gk, source_meta, grid_tile_coords[gk], convention)
        cur.executemany(
            "INSERT OR REPLACE INTO metadata (name, value) VALUES (?, ?)",
            meta.items()
        )
        conn.commit()
        conn.close()

    src.close()
    return worker_id, processed, skipped, skipped_low_zoom, skipped_outside_india


# ---------------------------------------------------------------------------
# Pipeline steps
# ---------------------------------------------------------------------------

def step1_download():
    log("=" * 70)
    log("STEP 1: Download India OSM Data")
    log("=" * 70)
    max_attempts = 3
    for attempt in range(1, max_attempts + 1):
        expected_md5 = fetch_expected_md5(INDIA_MD5_URL)
        if not expected_md5:
            log(f"Unable to fetch expected MD5 (attempt {attempt}/{max_attempts})", "ERROR")
            continue

        if INDIA_PBF.exists():
            if verify_md5(INDIA_PBF, INDIA_MD5_URL, expected_md5):
                log("Using existing file", "SUCCESS")
                return True
            INDIA_PBF.unlink(missing_ok=True)

        log(f"Download attempt {attempt}/{max_attempts}", "INFO")
        download_with_resume(INDIA_PBF_URL, INDIA_PBF)

        if verify_md5(INDIA_PBF, INDIA_MD5_URL, expected_md5):
            return True

        latest_expected = fetch_expected_md5(INDIA_MD5_URL)
        if latest_expected and latest_expected != expected_md5:
            log(
                "Upstream MD5 changed during download window; retrying with new checksum baseline",
                "WARN",
            )
        else:
            log("Checksum mismatch after download; retrying with a clean re-download", "WARN")

        INDIA_PBF.unlink(missing_ok=True)

    log("Download failed after repeated checksum retries", "ERROR")
    return False


def step2_generate_app_mbtiles():
    log("=" * 70)
    log("STEP 2: Generate app.mbtiles (Full OpenMapTiles)")
    log("=" * 70)
    if APP_MBTILES.exists():
        log(f"✓ Already exists ({APP_MBTILES.stat().st_size/(1024**2):.1f} MB) - skipping", "SUCCESS")
        return True
    return run_cmd([
        "tilemaker", "--input", str(INDIA_PBF), "--output", str(APP_MBTILES),
        "--process", APP_PROCESS, "--config", APP_CONFIG, "--bbox", INDIA_BBOX,
    ], "Generating app.mbtiles...")


def step3_filter_pbf():
    log("=" * 70)
    log("STEP 3: Filter PBF for Navigator")
    log("=" * 70)
    if INDIA_FILTERED_PBF.exists():
        log(f"✓ Already exists ({INDIA_FILTERED_PBF.stat().st_size/(1024**2):.1f} MB) - skipping", "SUCCESS")
        return True
    return run_cmd([
        "osmium", "tags-filter", str(INDIA_PBF),
        "-o", str(INDIA_FILTERED_PBF), "--overwrite",
        "w/highway", "w/waterway", "w/railway", "w/aeroway", "w/route",
        "a/boundary",
        "n/place", "w/place", "r/place",
        "n/name",  "w/name",  "r/name",
    ], "Filtering OSM data for navigation...")


def step3b_generate_poi():
    log("=" * 70)
    log("STEP 3b: Generate POI Database")
    log("=" * 70)
    if POI_DB.exists():
        log(f"✓ Already exists ({POI_DB.stat().st_size/(1024**2):.1f} MB) - skipping", "SUCCESS")
        return True
    return bool(generate_poi_database(INDIA_PBF))


def step4_generate_navigator_mbtiles():
    log("=" * 70)
    log("STEP 4: Generate navigator.mbtiles (Filtered)")
    log("=" * 70)
    if NAVIGATOR_MBTILES.exists():
        log(f"✓ Already exists ({NAVIGATOR_MBTILES.stat().st_size/(1024**2):.1f} MB) - skipping", "SUCCESS")
        return True
    return run_cmd([
        "tilemaker", "--input", str(INDIA_FILTERED_PBF), "--output", str(NAVIGATOR_MBTILES),
        "--process", NAVIGATOR_PROCESS, "--config", NAVIGATOR_CONFIG, "--bbox", INDIA_BBOX,
    ], "Generating navigator.mbtiles...")


def step5_repack_tiles():
    """
    Parallel repack of navigator.mbtiles into per-degree grid files.

    Phase 0: clear any stale output files from previous runs.
    Phase A: detect tile_row convention, then scan index (no blobs).
    Phase B: parallel write — N workers each own a disjoint set of grid keys.
    """
    log("=" * 70)
    log(f"STEP 5: Parallel Repack into 1° Grid MBTiles  ({NUM_WORKERS} workers)")
    log("=" * 70)

    source = str(NAVIGATOR_MBTILES)

    # ------------------------------------------------------------------
    # Phase 0: wipe stale output files so we start clean
    # ------------------------------------------------------------------
    existing = list(FINAL_DIR.glob("*.mbtiles"))
    if existing:
        log(f"Removing {len(existing)} stale files from previous run...")
        for f in existing:
            f.unlink()

    # ------------------------------------------------------------------
    # Phase A: detect convention + index scan (no blobs)
    # ------------------------------------------------------------------
    log("Phase A: detecting tile_row convention...")
    convention = detect_tile_row_convention(source)

    source_meta = read_source_metadata(source)
    log("Source metadata from navigator.mbtiles:")
    for k, v in source_meta.items():
        display = v if len(v) < 100 else v[:97] + "..."
        log(f"  {k:20s} = {display}")

    log("Phase A: scanning tile index...")
    src = _sqlite_connect_ro(source)
    cur = src.cursor()

    cur.execute("SELECT COUNT(*) FROM tiles")
    total_tiles = cur.fetchone()[0]
    log(f"  Total tiles: {total_tiles:,}")

    cur.execute("SELECT DISTINCT zoom_level FROM tiles ORDER BY zoom_level")
    zoom_levels = [r[0] for r in cur.fetchall()]
    log(f"  Zoom levels: {zoom_levels}")

    cur.execute("SELECT zoom_level, tile_column, tile_row FROM tiles")
    grid_tile_count: dict = {}
    skipped_low_zoom = 0
    skipped_outside_india = 0
    for z, x, tr in cur:
        if z < MIN_GRID_ZOOM:
            skipped_low_zoom += 1
            continue
        if not tile_intersects_india(z, x, tr, convention):
            skipped_outside_india += 1
            continue
        gk = tile_to_grid_key(z, x, tr, convention)
        grid_tile_count[gk] = grid_tile_count.get(gk, 0) + 1
    src.close()

    all_grid_keys = sorted(grid_tile_count.keys())
    log(f"  Unique 1° grid cells: {len(all_grid_keys)}")
    log(f"  Filtered out low-zoom tiles (< z{MIN_GRID_ZOOM}): {skipped_low_zoom:,}")
    log(f"  Filtered out tiles outside India bbox: {skipped_outside_india:,}")

    # Sanity check — all keys should be inside India's range
    bad_keys = [k for k in all_grid_keys if k.startswith('S') or k.startswith('W')]
    if bad_keys:
        log(f"  ⚠ {len(bad_keys)} unexpected keys outside India: {bad_keys[:10]}", "WARN")
    else:
        lat_vals = [int(k.split('_')[0]) for k in all_grid_keys]
        lon_vals = [int(k.split('_')[1]) for k in all_grid_keys]
        log(f"  ✓ lat range {min(lat_vals)}–{max(lat_vals)}, "
            f"lon range {min(lon_vals)}–{max(lon_vals)} — looks correct for India")

    # ------------------------------------------------------------------
    # Phase B: distribute round-robin by load, run workers
    # ------------------------------------------------------------------
    log(f"Phase B: writing with {NUM_WORKERS} parallel workers...")
    keys_by_load = sorted(all_grid_keys, key=lambda k: -grid_tile_count[k])
    worker_keys  = [set() for _ in range(NUM_WORKERS)]
    for i, k in enumerate(keys_by_load):
        worker_keys[i % NUM_WORKERS].add(k)

    worker_args = [
        (wid, source, str(FINAL_DIR), keys, COMMIT_INTERVAL, source_meta, convention)
        for wid, keys in enumerate(worker_keys)
    ]

    with multiprocessing.Pool(processes=NUM_WORKERS) as pool:
        results = []
        for result in pool.imap_unordered(_worker, worker_args):
            wid, written, skipped, skipped_low, skipped_outside = result
            log(
                f"  Worker {wid} done — wrote {written:,} tiles "
                f"(skipped {skipped:,}; low-zoom {skipped_low:,}, outside-India {skipped_outside:,})"
            )
            results.append(result)

    total_written = sum(r[1] for r in results)
    total_skipped = sum(r[2] for r in results)
    total_skipped_low = sum(r[3] for r in results)
    total_skipped_outside = sum(r[4] for r in results)
    log(f"✓ Wrote {total_written:,} tiles into {len(all_grid_keys)} grid files", "SUCCESS")
    log(
        f"  Worker phase skipped {total_skipped:,} tiles "
        f"(low-zoom {total_skipped_low:,}, outside-India {total_skipped_outside:,})"
    )

    log("Top 10 grid cells by tile count:")
    for gk, cnt in sorted(grid_tile_count.items(), key=lambda x: -x[1])[:10]:
        path = FINAL_DIR / f"{gk}.mbtiles"
        size_kb = path.stat().st_size / 1024 if path.exists() else 0
        log(f"  {gk}.mbtiles — {cnt:,} tiles, {size_kb:,.0f} KB")

    return True


def step6_verify():
    """Verify tile counts, metadata completeness, and coordinate sanity."""
    log("=" * 70)
    log("STEP 6: Verify Grid MBTiles")
    log("=" * 70)

    grid_files    = sorted(FINAL_DIR.glob("*.mbtiles"))
    total_tiles   = 0
    errors        = 0
    REQUIRED_KEYS = {'name', 'format', 'bounds', 'center', 'minzoom', 'maxzoom'}
    sample_size   = min(5, len(grid_files))

    suspicious = [f for f in grid_files if f.stem.startswith('S') or f.stem.startswith('-')]
    if suspicious:
        log(f"⚠ {len(suspicious)} suspicious filenames: {[f.name for f in suspicious[:5]]}", "WARN")

    for i, gf in enumerate(grid_files):
        try:
            conn = sqlite3.connect(str(gf))
            cur  = conn.cursor()
            cur.execute("SELECT COUNT(*) FROM tiles")
            n = cur.fetchone()[0]
            total_tiles += n
            if i < sample_size:
                cur.execute("SELECT name, value FROM metadata")
                meta    = {r[0]: r[1] for r in cur.fetchall()}
                missing = REQUIRED_KEYS - set(meta.keys())
                if missing:
                    log(f"  {gf.name}: MISSING keys {missing}", "WARN")
                else:
                    log(f"  {gf.name}: {n:,} tiles | "
                        f"bounds={meta.get('bounds')} | "
                        f"z{meta.get('minzoom')}-{meta.get('maxzoom')}")
            conn.close()
        except Exception as e:
            log(f"  {gf.name}: ERROR — {e}", "ERROR")
            errors += 1

    log(f"Grid files:   {len(grid_files)}")
    log(f"Total tiles:  {total_tiles:,}")
    log(f"Errors:       {errors}")
    return errors == 0


def _calc_md5(path: Path) -> str:
    h = hashlib.md5()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def _calc_md5_skip_valhalla_header(path: Path) -> str:
    """
    Calculate MD5 of a .gph tile, skipping the first 272 bytes (GraphTileHeader).
    The header contains date_created which changes on every build even if the
    tile content is identical. This allows better change detection on the device.
    """
    h = hashlib.md5()
    with open(path, "rb") as f:
        f.seek(272)  # Skip GraphTileHeader
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def _calc_mbtiles_content_md5(path: Path) -> str:
    """Hash MBTiles tile content only, ignoring SQLite file layout and metadata."""
    h = hashlib.md5()
    conn = _sqlite_connect_ro(path)
    cur = conn.cursor()
    cur.execute(
        "SELECT zoom_level, tile_column, tile_row, tile_data "
        "FROM tiles ORDER BY zoom_level, tile_column, tile_row"
    )
    for zoom_level, tile_column, tile_row, tile_data in cur:
        h.update(f"{zoom_level}:{tile_column}:{tile_row}:".encode("ascii"))
        h.update(tile_data)
    conn.close()
    return h.hexdigest()


def _get_mbtiles_tile_count(path: Path) -> int:
    conn = _sqlite_connect_ro(path)
    cur = conn.cursor()
    cur.execute("SELECT COUNT(*) FROM tiles")
    count = cur.fetchone()[0]
    conn.close()
    return count


def _get_mbtiles_metadata(path: Path) -> dict:
    conn = _sqlite_connect_ro(path)
    cur = conn.cursor()
    cur.execute("SELECT name, value FROM metadata")
    metadata = {name: value for name, value in cur.fetchall()}
    conn.close()
    return metadata


def _calc_sqlite_logical_fingerprint(path: Path):
    """Hash sqlite table content deterministically (logical data, not file bytes)."""
    h = hashlib.md5()
    conn = _sqlite_connect_ro(path)
    cur = conn.cursor()
    cur.execute(
        "SELECT name, sql FROM sqlite_master "
        "WHERE type='table' AND name NOT LIKE 'sqlite_%' ORDER BY name"
    )
    table_defs = cur.fetchall()
    table_names = []
    for table_name, table_sql in table_defs:
        table_sql = (table_sql or "").strip().upper()
        # Some Valhalla timezone db tables are virtual (e.g. VirtualElementary module)
        # which may not be loadable in runtime sqlite builds used by this script.
        if table_sql.startswith("CREATE VIRTUAL TABLE"):
            continue
        table_names.append(table_name)

    total_rows = 0
    for table_name in table_names:
        h.update(f"TABLE:{table_name}\n".encode("utf-8"))
        col_rows = conn.execute(f'PRAGMA table_info("{table_name}")').fetchall()
        cols = [r[1] for r in col_rows]
        if not cols:
            continue

        order_clause = ", ".join([f'"{c}"' for c in cols])
        q = f'SELECT * FROM "{table_name}" ORDER BY {order_clause}'

        row_count = 0
        for row in conn.execute(q):
            row_count += 1
            normalized = []
            for value in row:
                if isinstance(value, bytes):
                    normalized.append(value.hex())
                else:
                    normalized.append(value)
            h.update((json.dumps(normalized, ensure_ascii=True, separators=(",", ":")) + "\n").encode("utf-8"))

        total_rows += row_count
        h.update(f"ROWS:{row_count}\n".encode("utf-8"))

    conn.close()
    return h.hexdigest(), len(table_names), total_rows


def _bounds_intersect_india(bounds_text: str) -> bool:
    try:
        west, south, east, north = [float(part) for part in bounds_text.split(",")]
    except Exception:
        return True

    return not (
        east <= INDIA_LON_MIN or
        west >= INDIA_LON_MAX or
        north <= INDIA_LAT_MIN or
        south >= INDIA_LAT_MAX
    )


def _should_publish_grid(path: Path):
    tile_count = _get_mbtiles_tile_count(path)
    if tile_count == 0:
        return False, "no tiles"

    metadata = _get_mbtiles_metadata(path)
    bounds_text = metadata.get("bounds")
    if bounds_text and not _bounds_intersect_india(bounds_text):
        return False, f"outside India bounds ({bounds_text})"

    return True, f"{tile_count:,} tiles"


def _write_md5_sidecar(path: Path, md5_value: str, label: str):
    path.write_text(f"{md5_value}  {label}\n")


def _publish_single_lz4_file(source: Path, dest: Path, content_md5: str | None = None):
    shutil.copy2(source, dest)
    if content_md5 is not None:
        _write_md5_sidecar(dest.parent / f"{dest.name}.md5", content_md5, dest.name)
    compressed_dest = dest.with_suffix(dest.suffix + ".lz4")
    if not run_cmd(["lz4", "-9", "-f", str(dest), str(compressed_dest)], f"Compressing {dest.name}"):
        return None
    dest.unlink()
    compressed_md5 = _calc_md5(compressed_dest)
    _write_md5_sidecar(dest.parent / f"{compressed_dest.name}.md5", compressed_md5, compressed_dest.name)
    return compressed_dest


def _parse_sidecar_md5(sidecar_path: Path):
    if not sidecar_path.exists():
        return None
    try:
        return sidecar_path.read_text().strip().split()[0]
    except Exception:
        return None


def _build_mbtiles_snapshot(public_export_dir: Path, nav_public_dir: Path):
    files = {}

    tiles_manifest_path = nav_public_dir / "tiles_manifest.json"
    if tiles_manifest_path.exists():
        try:
            tiles_manifest = json.loads(tiles_manifest_path.read_text())
            for filename, info in tiles_manifest.get("files", {}).items():
                content_md5 = info.get("content_md5")
                if content_md5:
                    rel_path = f"mbtiles_navigator/{filename}"
                    files[rel_path] = content_md5
        except Exception as e:
            log(f"Failed to parse tiles manifest for snapshot: {e}", "WARN")

    poi_lz4 = public_export_dir / "poi.sqlite.lz4"
    poi_md5_sidecar = public_export_dir / "poi.sqlite.md5"
    if poi_lz4.exists():
        poi_content_md5 = _parse_sidecar_md5(poi_md5_sidecar)
        if poi_content_md5:
            files["poi.sqlite.lz4"] = poi_content_md5

    # NOTE: Explicitly DO NOT track app.mbtiles in incremental snapshots.
    # app.mbtiles is rebuilt from scratch on every pipeline run with new metadata/timestamps.
    # This causes the MD5 to change 100% of the time regardless of actual OSM data changes,
    # forcing every daily patch to include the entire 1.8GB file (pointless).
    # 
    # Solution: Only weekly/monthly/yearly rollups include app.mbtiles (full rebuild).
    # Daily incremental patches only track grid tiles (mbtiles_navigator/*.mbtiles),
    # which are deterministic and detect real content changes.

    return {
        "type": PATCH_FILE_TYPE,
        "generated_at": datetime.utcnow().isoformat(),
        "run_date": datetime.utcnow().date().isoformat(),
        "files": files,
        "total_files": len(files),
    }


def _load_snapshot_by_date(run_date: str):
    snapshot_path = SNAPSHOTS_DIR / f"{run_date}.json"
    if not snapshot_path.exists():
        return None
    try:
        return json.loads(snapshot_path.read_text())
    except Exception as e:
        log(f"Failed to read snapshot {snapshot_path.name}: {e}", "WARN")
        return None


def _save_snapshot(snapshot: dict):
    SNAPSHOTS_DIR.mkdir(parents=True, exist_ok=True)
    run_date = snapshot.get("run_date")
    if not run_date:
        return False
    snapshot_path = SNAPSHOTS_DIR / f"{run_date}.json"
    snapshot_path.write_text(json.dumps(snapshot, indent=2))
    latest_path = PATCH_STATE_DIR / "latest_snapshot.json"
    latest_path.write_text(json.dumps(snapshot, indent=2))
    return True


def _list_snapshot_dates():
    if not SNAPSHOTS_DIR.exists():
        return []
    dates = []
    for snapshot_file in SNAPSHOTS_DIR.glob("*.json"):
        try:
            datetime.strptime(snapshot_file.stem, "%Y-%m-%d")
            dates.append(snapshot_file.stem)
        except ValueError:
            continue
    return sorted(set(dates))


def _diff_snapshots(base_snapshot: dict, target_snapshot: dict):
    base_files = (base_snapshot or {}).get("files", {})
    target_files = (target_snapshot or {}).get("files", {})

    changed = []
    removed = []

    for rel_path, target_md5 in target_files.items():
        if base_files.get(rel_path) != target_md5:
            changed.append(rel_path)

    for rel_path in base_files:
        if rel_path not in target_files:
            removed.append(rel_path)

    return sorted(changed), sorted(removed)


def _create_app_delta_sqlite(base_app_db: Path, target_app_db: Path, delta_db_path: Path):
    if delta_db_path.exists():
        delta_db_path.unlink()

    conn = sqlite3.connect(str(delta_db_path))
    try:
        conn.execute("PRAGMA journal_mode=OFF")
        conn.execute("PRAGMA synchronous=OFF")
        conn.execute("PRAGMA temp_store=MEMORY")

        conn.execute("CREATE TABLE patch_info (key TEXT PRIMARY KEY, value TEXT)")
        conn.execute("CREATE TABLE metadata (name TEXT PRIMARY KEY, value TEXT)")
        conn.execute("CREATE TABLE tiles_upsert (zoom_level INTEGER, tile_column INTEGER, tile_row INTEGER, tile_data BLOB)")
        conn.execute("CREATE TABLE tiles_delete (zoom_level INTEGER, tile_column INTEGER, tile_row INTEGER)")
        conn.execute("CREATE INDEX idx_tiles_upsert_key ON tiles_upsert(zoom_level, tile_column, tile_row)")
        conn.execute("CREATE INDEX idx_tiles_delete_key ON tiles_delete(zoom_level, tile_column, tile_row)")

        conn.execute("ATTACH DATABASE ? AS base", (str(base_app_db),))
        conn.execute("ATTACH DATABASE ? AS target", (str(target_app_db),))

        metadata_rows = conn.execute("SELECT name, value FROM target.metadata").fetchall()
        if metadata_rows:
            conn.executemany("INSERT INTO metadata(name, value) VALUES (?, ?)", metadata_rows)

        upsert_query = """
            SELECT t.zoom_level, t.tile_column, t.tile_row, t.tile_data
            FROM target.tiles t
            LEFT JOIN base.tiles b
              ON b.zoom_level = t.zoom_level
             AND b.tile_column = t.tile_column
             AND b.tile_row = t.tile_row
            WHERE b.tile_data IS NULL OR b.tile_data != t.tile_data
        """

        delete_query = """
            SELECT b.zoom_level, b.tile_column, b.tile_row
            FROM base.tiles b
            LEFT JOIN target.tiles t
              ON t.zoom_level = b.zoom_level
             AND t.tile_column = b.tile_column
             AND t.tile_row = b.tile_row
            WHERE t.zoom_level IS NULL
        """

        upsert_count = 0
        cursor = conn.execute(upsert_query)
        while True:
            rows = cursor.fetchmany(5000)
            if not rows:
                break
            conn.executemany(
                "INSERT INTO tiles_upsert(zoom_level, tile_column, tile_row, tile_data) VALUES (?, ?, ?, ?)",
                rows,
            )
            upsert_count += len(rows)

        delete_count = 0
        cursor = conn.execute(delete_query)
        while True:
            rows = cursor.fetchmany(5000)
            if not rows:
                break
            conn.executemany(
                "INSERT INTO tiles_delete(zoom_level, tile_column, tile_row) VALUES (?, ?, ?)",
                rows,
            )
            delete_count += len(rows)

        conn.executemany(
            "INSERT INTO patch_info(key, value) VALUES (?, ?)",
            [
                ("type", "app_mbtiles_delta"),
                ("base_db", str(base_app_db)),
                ("target_db", str(target_app_db)),
                ("upsert_count", str(upsert_count)),
                ("delete_count", str(delete_count)),
                ("generated_at", datetime.utcnow().isoformat()),
            ],
        )
        conn.commit()
        return upsert_count, delete_count
    finally:
        conn.close()


def _create_poi_delta_sqlite(base_poi_db: Path, target_poi_db: Path, delta_db_path: Path):
    if delta_db_path.exists():
        delta_db_path.unlink()

    conn = sqlite3.connect(str(delta_db_path))
    try:
        conn.execute("PRAGMA journal_mode=OFF")
        conn.execute("PRAGMA synchronous=OFF")
        conn.execute("PRAGMA temp_store=MEMORY")

        conn.execute("CREATE TABLE patch_info (key TEXT PRIMARY KEY, value TEXT)")
        conn.execute(
            """
            CREATE TABLE places_upsert (
                id TEXT PRIMARY KEY,
                name TEXT NOT NULL,
                lat REAL NOT NULL,
                lon REAL NOT NULL,
                type TEXT,
                category TEXT,
                geometry_type TEXT,
                tags TEXT
            )
            """
        )
        conn.execute("CREATE TABLE places_delete (id TEXT PRIMARY KEY)")

        conn.execute("ATTACH DATABASE ? AS base", (str(base_poi_db),))
        conn.execute("ATTACH DATABASE ? AS target", (str(target_poi_db),))

        upsert_query = """
            SELECT t.id, t.name, t.lat, t.lon, t.type, t.category, t.geometry_type, t.tags
            FROM target.places t
            LEFT JOIN base.places b ON b.id = t.id
            WHERE b.id IS NULL OR NOT (
                b.name IS t.name AND
                b.lat IS t.lat AND
                b.lon IS t.lon AND
                b.type IS t.type AND
                b.category IS t.category AND
                b.geometry_type IS t.geometry_type AND
                b.tags IS t.tags
            )
        """

        delete_query = """
            SELECT b.id
            FROM base.places b
            LEFT JOIN target.places t ON t.id = b.id
            WHERE t.id IS NULL
        """

        upsert_count = 0
        cursor = conn.execute(upsert_query)
        while True:
            rows = cursor.fetchmany(5000)
            if not rows:
                break
            conn.executemany(
                "INSERT OR REPLACE INTO places_upsert(id, name, lat, lon, type, category, geometry_type, tags) VALUES (?, ?, ?, ?, ?, ?, ?, ?)",
                rows,
            )
            upsert_count += len(rows)

        delete_count = 0
        cursor = conn.execute(delete_query)
        while True:
            rows = cursor.fetchmany(5000)
            if not rows:
                break
            conn.executemany("INSERT OR REPLACE INTO places_delete(id) VALUES (?)", rows)
            delete_count += len(rows)

        conn.executemany(
            "INSERT INTO patch_info(key, value) VALUES (?, ?)",
            [
                ("type", "poi_sqlite_delta"),
                ("base_db", str(base_poi_db)),
                ("target_db", str(target_poi_db)),
                ("upsert_count", str(upsert_count)),
                ("delete_count", str(delete_count)),
                ("generated_at", datetime.utcnow().isoformat()),
            ],
        )
        conn.commit()
        return upsert_count, delete_count
    finally:
        conn.close()


def _compress_and_sidecar(path: Path):
    lz4_path = path.with_suffix(path.suffix + ".lz4")
    if lz4_path.exists():
        lz4_path.unlink()
    if not run_cmd(["lz4", "-9", "-f", str(path), str(lz4_path)], f"Compressing artifact {path.name}", timeout=300):
        return None
    md5 = _calc_md5(lz4_path)
    _write_md5_sidecar(Path(str(lz4_path) + ".md5"), md5, lz4_path.name)
    return lz4_path


def _create_mbtiles_patch_bundle(
    schedule: str,
    base_snapshot: dict,
    target_snapshot: dict,
    changed_files: list,
    removed_files: list,
    public_export_dir: Path,
    patch_dir: Path,
    extra_artifacts: list | None = None,
    skip_changed_paths: set[str] | None = None,
):
    skip_changed_paths = skip_changed_paths or set()
    effective_changed_files = [p for p in changed_files if p not in skip_changed_paths]

    if not effective_changed_files and not removed_files and not extra_artifacts:
        return None

    target_date = target_snapshot.get("run_date")
    base_date = (base_snapshot or {}).get("run_date", "baseline")
    if not target_date:
        return None

    patch_base_name = f"patch_{PATCH_FILE_TYPE}_{schedule}_{base_date}_to_{target_date}.tar"
    patch_tar = patch_dir / patch_base_name
    patch_lz4 = patch_tar.with_suffix(patch_tar.suffix + ".lz4")

    for candidate in (patch_tar, patch_lz4, patch_dir / f"{patch_lz4.name}.md5"):
        if candidate.exists():
            candidate.unlink()

    patch_manifest = {
        "type": "mbtiles_patch",
        "file_type": PATCH_FILE_TYPE,
        "schedule": schedule,
        "from_date": base_date,
        "to_date": target_date,
        "generated_at": datetime.utcnow().isoformat(),
        "changed_files": effective_changed_files,
        "removed_files": removed_files,
        "changed_count": len(effective_changed_files),
        "removed_count": len(removed_files),
        "target_total_files": target_snapshot.get("total_files", 0),
        "extra_artifacts": extra_artifacts or [],
    }

    with NamedTemporaryFile("w", suffix=".json", delete=False) as tmp_manifest:
        tmp_manifest.write(json.dumps(patch_manifest, indent=2))
        temp_manifest_path = Path(tmp_manifest.name)

    try:
        with tarfile.open(patch_tar, "w") as tar:
            tar.add(temp_manifest_path, arcname="patch_manifest.json")

            for rel_path in effective_changed_files:
                src_path = public_export_dir / rel_path
                # If exact path doesn't exist, check for compressed version (e.g., app.mbtiles -> app.mbtiles.lz4)
                if not src_path.exists():
                    lz4_variant = Path(str(src_path) + ".lz4")
                    if lz4_variant.exists():
                        src_path = lz4_variant
                    else:
                        log(f"Patch source missing (skipping): {src_path}", "WARN")
                        continue
                tar.add(src_path, arcname=rel_path)

            for artifact in (extra_artifacts or []):
                src_path = Path(artifact.get("src_path", ""))
                arcname = artifact.get("arcname")
                if not src_path.exists() or not arcname:
                    continue
                tar.add(src_path, arcname=arcname)

        if not run_cmd(
            ["lz4", "-9", "-f", str(patch_tar), str(patch_lz4)],
            f"Compressing {schedule} MBTiles patch {patch_lz4.name}",
            timeout=600,
        ):
            if patch_tar.exists():
                patch_tar.unlink()
            return None

        if patch_tar.exists():
            patch_tar.unlink()

        patch_md5 = _calc_md5(patch_lz4)
        _write_md5_sidecar(patch_dir / f"{patch_lz4.name}.md5", patch_md5, patch_lz4.name)

        return {
            "filename": patch_lz4.name,
            "size_bytes": patch_lz4.stat().st_size,
            "size_mb": round(patch_lz4.stat().st_size / (1024 ** 2), 2),
            "md5": patch_md5,
            "from_date": base_date,
            "to_date": target_date,
            "changed_files": len(effective_changed_files),
            "removed_files": len(removed_files),
            "extra_artifacts": len(extra_artifacts or []),
            "generated_at": datetime.utcnow().isoformat(),
        }
    finally:
        try:
            temp_manifest_path.unlink(missing_ok=True)
        except Exception:
            pass
        for artifact in (extra_artifacts or []):
            try:
                src_path = Path(artifact.get("src_path", ""))
                if src_path.exists() and not artifact.get("keep_source", False):
                    src_path.unlink(missing_ok=True)
            except Exception:
                pass


def _prune_schedule_patches(schedule_map: dict, schedule: str, patch_dir: Path):
    max_count = PATCH_RETENTION_COUNT.get(schedule, 1)
    if len(schedule_map) <= max_count:
        return

    sorted_keys = sorted(schedule_map.keys())
    stale_keys = sorted_keys[: max(0, len(sorted_keys) - max_count)]

    for key in stale_keys:
        patch_info = schedule_map.pop(key, None)
        if not patch_info:
            continue
        filename = patch_info.get("filename")
        if not filename:
            continue
        for candidate in (patch_dir / filename, patch_dir / f"{filename}.md5"):
            if candidate.exists():
                candidate.unlink()


def _prune_old_snapshots(max_days=400):
    dates = _list_snapshot_dates()
    if len(dates) <= max_days:
        return
    stale_dates = dates[: len(dates) - max_days]
    for date_key in stale_dates:
        snapshot_file = SNAPSHOTS_DIR / f"{date_key}.json"
        if snapshot_file.exists():
            snapshot_file.unlink()


def generate_mbtiles_patches(
    public_export_dir: Path,
    nav_public_dir: Path,
    app_mbtiles_path: Path | None = None,
    poi_db_path: Path | None = None,
):
    patch_dir = PATCHES_PERSISTENT_DIR
    patch_dir.mkdir(parents=True, exist_ok=True)
    PATCH_STATE_DIR.mkdir(parents=True, exist_ok=True)
    SNAPSHOTS_DIR.mkdir(parents=True, exist_ok=True)

    index_path = patch_dir / "patches_index.json"
    if index_path.exists():
        try:
            patches_index = json.loads(index_path.read_text())
        except Exception:
            patches_index = {}
    else:
        patches_index = {}

    patches_index.setdefault("type", "patches_index")
    patches_index.setdefault("generated_at", datetime.utcnow().isoformat())
    patches_index.setdefault("patches", {})
    patches_index["patches"].setdefault(PATCH_FILE_TYPE, {})

    for schedule in PATCH_SCHEDULES.keys():
        patches_index["patches"][PATCH_FILE_TYPE].setdefault(schedule, {})

    current_snapshot = _build_mbtiles_snapshot(public_export_dir, nav_public_dir)
    current_date = current_snapshot["run_date"]
    latest_snapshot_path = PATCH_STATE_DIR / "latest_snapshot.json"

    previous_snapshot = None
    if latest_snapshot_path.exists():
        try:
            previous_snapshot = json.loads(latest_snapshot_path.read_text())
        except Exception as e:
            log(f"Failed to read latest snapshot: {e}", "WARN")

    if previous_snapshot and previous_snapshot.get("run_date") == current_date:
        log(f"Snapshot for {current_date} already exists; updating snapshot with latest publish", "INFO")

    _save_snapshot(current_snapshot)

    # Build incremental (daily) from previous snapshot to current snapshot
    if previous_snapshot and previous_snapshot.get("run_date") != current_date:
        changed_files, removed_files = _diff_snapshots(previous_snapshot, current_snapshot)
        incremental_extras = []
        skip_changed_paths = set()

        if app_mbtiles_path and app_mbtiles_path.exists() and APP_BASELINE_MBTILES.exists():
            delta_name = f"app_delta_{previous_snapshot.get('run_date')}_to_{current_date}.sqlite"
            delta_path = patch_dir / delta_name
            try:
                upsert_count, delete_count = _create_app_delta_sqlite(APP_BASELINE_MBTILES, app_mbtiles_path, delta_path)
                if upsert_count > 0 or delete_count > 0:
                    delta_lz4 = _compress_and_sidecar(delta_path)
                    if delta_lz4 is None:
                        delta_path.unlink(missing_ok=True)
                        raise RuntimeError("failed to compress app delta artifact")
                    incremental_extras.append(
                        {
                            "src_path": str(delta_lz4),
                            "arcname": f"app_deltas/{delta_lz4.name}",
                            "type": "app_mbtiles_delta",
                            "upserts": upsert_count,
                            "deletes": delete_count,
                            "keep_source": True,
                        }
                    )
                    delta_path.unlink(missing_ok=True)
                    log(
                        f"✓ app.mbtiles daily delta prepared ({upsert_count} upserts, {delete_count} deletes)",
                        "SUCCESS",
                    )
                else:
                    delta_path.unlink(missing_ok=True)
                    log("No app.mbtiles tile-level changes detected; app delta skipped", "INFO")
            except Exception as e:
                log(f"Failed to generate app.mbtiles delta: {e}", "WARN")
                if delta_path.exists():
                    delta_path.unlink()

        if poi_db_path and poi_db_path.exists() and POI_BASELINE_DB.exists():
            poi_delta_name = f"poi_delta_{previous_snapshot.get('run_date')}_to_{current_date}.sqlite"
            poi_delta_path = patch_dir / poi_delta_name
            try:
                upsert_count, delete_count = _create_poi_delta_sqlite(POI_BASELINE_DB, poi_db_path, poi_delta_path)
                if upsert_count > 0 or delete_count > 0:
                    poi_delta_lz4 = _compress_and_sidecar(poi_delta_path)
                    if poi_delta_lz4 is None:
                        poi_delta_path.unlink(missing_ok=True)
                        raise RuntimeError("failed to compress poi delta artifact")
                    incremental_extras.append(
                        {
                            "src_path": str(poi_delta_lz4),
                            "arcname": f"poi_deltas/{poi_delta_lz4.name}",
                            "type": "poi_sqlite_delta",
                            "upserts": upsert_count,
                            "deletes": delete_count,
                            "keep_source": True,
                        }
                    )
                    skip_changed_paths.add("poi.sqlite.lz4")
                    poi_delta_path.unlink(missing_ok=True)
                    log(
                        f"✓ poi.sqlite daily delta prepared ({upsert_count} upserts, {delete_count} deletes)",
                        "SUCCESS",
                    )
                else:
                    poi_delta_path.unlink(missing_ok=True)
                    log("No poi.sqlite row-level changes detected; POI delta skipped", "INFO")
            except Exception as e:
                log(f"Failed to generate poi.sqlite delta: {e}", "WARN")
                if poi_delta_path.exists():
                    poi_delta_path.unlink()

        patch_info = _create_mbtiles_patch_bundle(
            "incremental",
            previous_snapshot,
            current_snapshot,
            changed_files,
            removed_files,
            public_export_dir,
            patch_dir,
            extra_artifacts=incremental_extras,
            skip_changed_paths=skip_changed_paths,
        )
        if patch_info:
            patches_index["patches"][PATCH_FILE_TYPE]["incremental"][current_date] = patch_info
            log(
                f"✓ Incremental patch: {patch_info['filename']} "
                f"({patch_info['changed_files']} changed, {patch_info['removed_files']} removed, "
                f"{patch_info.get('extra_artifacts', 0)} extra artifacts)",
                "SUCCESS",
            )
        else:
            log("No daily MBTiles changes detected; incremental patch not created", "INFO")
    else:
        if not previous_snapshot:
            log("No previous snapshot found; baseline snapshot initialized (no incremental patch yet)", "INFO")

    # Build merged schedule patches using snapshot windows
    available_dates = _list_snapshot_dates()
    for schedule, window_days in PATCH_SCHEDULES.items():
        if schedule == "incremental":
            continue

        if len(available_dates) < window_days + 1:
            continue

        base_date = available_dates[-(window_days + 1)]
        target_date = current_date
        date_key = f"{base_date}_to_{target_date}"

        base_snapshot = _load_snapshot_by_date(base_date)
        target_snapshot = _load_snapshot_by_date(target_date)
        if not base_snapshot or not target_snapshot:
            continue

        changed_files, removed_files = _diff_snapshots(base_snapshot, target_snapshot)
        patch_info = _create_mbtiles_patch_bundle(
            schedule,
            base_snapshot,
            target_snapshot,
            changed_files,
            removed_files,
            public_export_dir,
            patch_dir,
            skip_changed_paths=None,
        )

        if patch_info:
            patches_index["patches"][PATCH_FILE_TYPE][schedule][date_key] = patch_info
            log(
                f"✓ {schedule} patch: {patch_info['filename']} "
                f"({patch_info['changed_files']} changed, {patch_info['removed_files']} removed)",
                "SUCCESS",
            )

    # Retention cleanup
    for schedule in PATCH_SCHEDULES.keys():
        schedule_map = patches_index["patches"][PATCH_FILE_TYPE].setdefault(schedule, {})
        _prune_schedule_patches(schedule_map, schedule, patch_dir)

    _prune_old_snapshots(max_days=400)

    patches_index["generated_at"] = datetime.utcnow().isoformat()
    patches_index["retention"] = {
        "incremental_days": PATCH_RETENTION_COUNT["incremental"],
        "weekly_windows": PATCH_RETENTION_COUNT["weekly"],
        "fortnight_windows": PATCH_RETENTION_COUNT["fortnight"],
        "monthly_windows": PATCH_RETENTION_COUNT["monthly"],
        "six_month_windows": PATCH_RETENTION_COUNT["six_month"],
        "yearly_windows": PATCH_RETENTION_COUNT["yearly"],
    }
    patches_index["notes"] = (
        "Daily incremental MBTiles patches are retained for 7 days. "
        "Incrementals include file-level navigator tile deltas, file-level poi.sqlite deltas, "
        "and app.mbtiles SQLite tile deltas "
        "(upserts/deletes from previous day when baseline exists). "
        "Merged rollups are generated for weekly, fortnight, monthly (4 weeks), "
        "six-month, and yearly windows."
    )

    index_path.write_text(json.dumps(patches_index, indent=2))
    log(f"✓ patches_index.json updated: {index_path}", "SUCCESS")

    # Mirror persisted patches into public_export for server sync
    public_patch_dir = public_export_dir / "patches"
    if public_patch_dir.exists():
        shutil.rmtree(public_patch_dir)
    shutil.copytree(patch_dir, public_patch_dir)
    log(f"✓ Patch artifacts published to: {public_patch_dir}", "SUCCESS")

    # Update app baseline after patch generation so next run can produce previous-day delta
    if app_mbtiles_path and app_mbtiles_path.exists():
        shutil.copy2(app_mbtiles_path, APP_BASELINE_MBTILES)
        log(f"✓ Updated app baseline for next incremental delta: {APP_BASELINE_MBTILES}", "SUCCESS")

    if poi_db_path and poi_db_path.exists():
        shutil.copy2(poi_db_path, POI_BASELINE_DB)
        log(f"✓ Updated POI baseline for next incremental delta: {POI_BASELINE_DB}", "SUCCESS")


def step4b_build_valhalla():
    """
    Build Valhalla routing tiles from filtered PBF.
    
    Steps:
    1. Generate valhalla config
    2. Build timezones and admin databases
    3. Build tile graph (.gph files)
    4. Create manifest of tiles
    """
    log("=" * 70)
    log("STEP 4b: Build Valhalla Routing Tiles")
    log("=" * 70)

    if VALHALLA_TILES.exists():
        existing_tiles = list(VALHALLA_TILES.glob("**/*.gph"))
        if len(existing_tiles) > 0:
            log(f"✓ Valhalla tiles already exist ({len(existing_tiles)} files) - skipping build", "SUCCESS")
            return True

    # Clean up any partial Valhalla data
    if VALHALLA_DIR.exists():
        log("Cleaning previous Valhalla directory...")
        shutil.rmtree(VALHALLA_DIR)

    VALHALLA_DIR.mkdir(parents=True, exist_ok=True)
    VALHALLA_TILES.mkdir(parents=True, exist_ok=True)
    elevation_dir = VALHALLA_DIR / "elevation"
    elevation_dir.mkdir(parents=True, exist_ok=True)

    # Build config
    log("Generating Valhalla config...")
    skip_tile_extract = WORK_DIR / "valhalla_tiles_skip.tar"
    cmd = [
        "valhalla_build_config",
        "--mjolnir-tile-dir", str(VALHALLA_TILES),
        "--mjolnir-tile-extract", str(skip_tile_extract),
        "--mjolnir-traffic-extract", str(VALHALLA_DIR / "traffic.tar"),
        "--mjolnir-admin", str(VALHALLA_DIR / "admin.sqlite"),
        "--mjolnir-landmarks", str(VALHALLA_DIR / "landmarks.sqlite"),
        "--mjolnir-timezone", str(VALHALLA_DIR / "timezones.sqlite"),
        "--mjolnir-transit-dir", str(VALHALLA_DIR / "transit"),
        "--mjolnir-transit-feeds-dir", str(VALHALLA_DIR / "transit_feeds"),
        "--additional-data-elevation", str(elevation_dir) + "/"
    ]
    try:
        log(f"$ {_format_cmd(cmd)} > {VALHALLA_CONFIG}", "CMD")
        with open(VALHALLA_CONFIG, "w") as f:
            subprocess.check_call(cmd, stdout=f, stderr=subprocess.STDOUT, timeout=120)
        log("✓ Config generated", "SUCCESS")
    except Exception as e:
        log(f"Config generation failed: {e}", "ERROR")
        return False

    # Build timezones
    log("Building timezones...")
    timezone_sqlite_path = VALHALLA_DIR / "timezones.sqlite"
    try:
        log(f"$ valhalla_build_timezones > {timezone_sqlite_path}", "CMD")
        with open(timezone_sqlite_path, "w") as f:
            result = subprocess.run(
                ["valhalla_build_timezones"],
                stdout=f,
                stderr=subprocess.PIPE,
                text=True,
                timeout=300
            )
        if result.returncode != 0:
            log(f"  Warning: timezones exit code {result.returncode}", "WARN")
        else:
            log("✓ Timezones built", "SUCCESS")

        if timezone_sqlite_path.exists():
            raw_md5 = _calc_md5(timezone_sqlite_path)
            logical_md5, tables_count, rows_count = _calc_sqlite_logical_fingerprint(timezone_sqlite_path)
            log(
                f"Timezone fingerprint: raw_md5={raw_md5}, logical_md5={logical_md5}, "
                f"tables={tables_count}, rows={rows_count}",
                "INFO",
            )

            previous_state = {}
            if TIMEZONE_HASH_STATE.exists():
                try:
                    previous_state = json.loads(TIMEZONE_HASH_STATE.read_text())
                except Exception:
                    previous_state = {}

            prev_raw = previous_state.get("raw_md5")
            prev_logical = previous_state.get("logical_md5")
            if prev_raw and prev_logical:
                if prev_logical == logical_md5 and prev_raw != raw_md5:
                    log(
                        "timezone.sqlite byte-level hash changed but logical content stayed same "
                        "(likely sqlite file-header/layout churn)",
                        "WARN",
                    )
                elif prev_logical != logical_md5:
                    log("timezone.sqlite logical content changed from previous run", "INFO")

            PATCH_STATE_DIR.mkdir(parents=True, exist_ok=True)
            TIMEZONE_HASH_STATE.write_text(
                json.dumps(
                    {
                        "raw_md5": raw_md5,
                        "logical_md5": logical_md5,
                        "tables": tables_count,
                        "rows": rows_count,
                        "generated_at": datetime.utcnow().isoformat(),
                    },
                    indent=2,
                )
            )
    except Exception as e:
        log(f"Timezones build failed: {e}", "ERROR")
        return False

    # Build admins
    log("Building admin boundaries...")
    try:
        if not run_cmd(
            ["valhalla_build_admins", "-c", str(VALHALLA_CONFIG), str(INDIA_FILTERED_PBF)],
            "Running valhalla_build_admins...",
            timeout=600,
        ):
            log("  Warning: valhalla_build_admins returned non-zero", "WARN")
        else:
            log("✓ Admins built", "SUCCESS")
    except Exception as e:
        log(f"Admins build failed: {e}", "ERROR")
        return False

    # Build tiles
    log("Building routing tiles (this may take several minutes)...")
    try:
        if not run_cmd(
            ["valhalla_build_tiles", "-c", str(VALHALLA_CONFIG), str(INDIA_FILTERED_PBF)],
            "Running valhalla_build_tiles...",
            timeout=900,
        ):
            log("ERROR: valhalla_build_tiles failed", "ERROR")
            return False
    except Exception as e:
        log(f"Tiles build failed: {e}", "ERROR")
        return False

    # Verify tiles were created
    tile_files = list(VALHALLA_TILES.glob("**/*.gph"))
    if len(tile_files) == 0:
        log("ERROR: No .gph tiles generated!", "ERROR")
        return False

    log(f"✓ Generated {len(tile_files)} routing tile files", "SUCCESS")

    # Create manifest of tiles
    log("Creating Valhalla tiles manifest...")
    manifest = {
        "type": "valhalla_routing_tiles",
        "generated_at": datetime.utcnow().isoformat(),
        "files": {}
    }
    
    total_size = 0
    for tile_path in sorted(tile_files):
        try:
            size = tile_path.stat().st_size
            # Skip first 272 bytes (GraphTileHeader with timestamps)
            # so MD5 reflects actual tile data, not metadata changes
            md5 = _calc_md5_skip_valhalla_header(tile_path)
            rel_path = tile_path.relative_to(VALHALLA_TILES)
            manifest["files"][str(rel_path)] = {
                "size_bytes": size,
                "md5": md5,
                "size_mb": round(size / (1024 ** 2), 2)
            }
            total_size += size
        except Exception as e:
            log(f"  Failed to process {tile_path}: {e}", "WARN")

    manifest["total_files"] = len(manifest["files"])
    manifest["total_size_bytes"] = total_size
    manifest["total_size_mb"] = round(total_size / (1024 ** 2), 2)

    manifest_path = VALHALLA_TILES / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2))
    log(f"✓ Manifest: {len(manifest['files'])} tiles, {manifest['total_size_mb']:.1f} MB", "SUCCESS")

    return True


def step5_archive_valhalla():
    """
    Archive Valhalla tiles directory as tar.lz4 with the manifest included.
    """
    log("=" * 70)
    log("STEP 5: Archive Valhalla Tiles")
    log("=" * 70)

    if VALHALLA_TAR_LZ4.exists():
        size_mb = VALHALLA_TAR_LZ4.stat().st_size / (1024 ** 2)
        log(f"✓ Already exists ({size_mb:.1f} MB) - skipping", "SUCCESS")
        return True

    if not VALHALLA_TILES.exists():
        log("Valhalla tiles directory does not exist - skipping archive", "WARN")
        return True

    tile_files = list(VALHALLA_TILES.glob("**/*.gph"))
    if len(tile_files) == 0:
        log("No tile files to archive - skipping", "WARN")
        return True

    log(f"Archiving {len(tile_files)} tiles with tar...")
    temp_tar = WORK_DIR / "valhalla_tiles.tar"

    try:
        # Create uncompressed tar
        with tarfile.open(temp_tar, "w") as tar:
            tar.add(VALHALLA_TILES, arcname="valhalla_tiles")

        log("Compressing with lz4...")
        if not run_cmd(
            ["lz4", "-9", "-f", str(temp_tar), str(VALHALLA_TAR_LZ4)],
            "Compressing Valhalla tar with lz4...",
            timeout=600,
        ):
            if temp_tar.exists():
                temp_tar.unlink()
            return False

        if temp_tar.exists():
            temp_tar.unlink()

        if not VALHALLA_TAR_LZ4.exists():
            log("Tar archive was not created", "ERROR")
            return False

        # Save MD5 of tar.lz4
        tar_md5 = _calc_md5(VALHALLA_TAR_LZ4)
        md5_file = Path(str(VALHALLA_TAR_LZ4) + ".md5")
        md5_file.write_text(f"{tar_md5}  valhalla_tiles.tar.lz4\n")

        size_mb = VALHALLA_TAR_LZ4.stat().st_size / (1024 ** 2)
        log(f"✓ Archived and compressed: {size_mb:.1f} MB", "SUCCESS")
        return True

    except Exception as e:
        log(f"Archive failed: {e}", "ERROR")
        return False


def step7_publish():
    """
    Publish app, POI, navigator grid tiles, and Valhalla artifacts to
    public_export, then trigger server.py blue-green deployment.
    """
    log("=" * 70)
    log("STEP 7: Publish to OTA Server")
    log("=" * 70)

    BASE_DIR      = Path("/home/navigator")
    PUBLIC_EXPORT = BASE_DIR / "public_export"
    NAV_PUBLIC    = PUBLIC_EXPORT / "mbtiles_navigator"
    VAL_PUBLIC    = PUBLIC_EXPORT / "valhalla"
    APP_PUBLIC    = PUBLIC_EXPORT
    MARKER        = BASE_DIR / "completed.done"

    PUBLIC_EXPORT.mkdir(parents=True, exist_ok=True)

    log("Publishing app.mbtiles...")
    if APP_MBTILES.exists():
        app_dest = APP_PUBLIC / "app.mbtiles"
        app_lz4 = app_dest.with_suffix(app_dest.suffix + ".lz4")
        for candidate in (app_dest, app_lz4, APP_PUBLIC / "app.mbtiles.md5", APP_PUBLIC / "app.mbtiles.lz4.md5"):
            if candidate.exists():
                candidate.unlink()
        published_app = _publish_single_lz4_file(APP_MBTILES, app_dest, _calc_mbtiles_content_md5(APP_MBTILES))
        if published_app is None:
            return False
        log(f"✓ app.mbtiles published: {published_app.name}", "SUCCESS")
    else:
        log("app.mbtiles not found - skipping", "WARN")

    log("Publishing poi.sqlite...")
    if POI_DB.exists():
        poi_dest = APP_PUBLIC / "poi.sqlite"
        poi_lz4 = poi_dest.with_suffix(poi_dest.suffix + ".lz4")
        for candidate in (poi_dest, poi_lz4, APP_PUBLIC / "poi.sqlite.md5", APP_PUBLIC / "poi.sqlite.lz4.md5"):
            if candidate.exists():
                candidate.unlink()
        poi_logical_md5, _, _ = _calc_sqlite_logical_fingerprint(POI_DB)
        published_poi = _publish_single_lz4_file(POI_DB, poi_dest, poi_logical_md5)
        if published_poi is None:
            return False
        log(f"✓ poi.sqlite published: {published_poi.name}", "SUCCESS")
    else:
        log("poi.sqlite not found - skipping", "WARN")

    # --- Publish Navigator MBTiles ---
    log("Publishing Navigator MBTiles...")
    if NAV_PUBLIC.exists():
        shutil.rmtree(NAV_PUBLIC)
    NAV_PUBLIC.mkdir(parents=True, exist_ok=True)

    grid_candidates = sorted(FINAL_DIR.glob("*.mbtiles"))
    grid_files = []
    skipped_grids = []
    for grid_path in grid_candidates:
        should_publish, reason = _should_publish_grid(grid_path)
        if should_publish:
            grid_files.append(grid_path)
        else:
            skipped_grids.append((grid_path.name, reason))

    log(f"Compressing {len(grid_files)} grid MBTiles files...")
    if skipped_grids:
        log(f"Skipping {len(skipped_grids)} empty/out-of-bounds grid files", "WARN")
        for name, reason in skipped_grids[:10]:
            log(f"  {name}: {reason}", "WARN")

    errors = 0
    tiles_manifest = {
        "type": "navigator_grid_tiles",
        "generated_at": datetime.utcnow().isoformat(),
        "files": {},
        "total_files": 0,
        "total_size_bytes": 0,
        "bbox": str(INDIA_BBOX),
    }
    for index, src in enumerate(grid_files, 1):
        tile_name = src.stem
        dst       = NAV_PUBLIC / src.name
        progress  = index / len(grid_files) * 100 if grid_files else 100.0

        try:
            # 1. Copy uncompressed file
            shutil.copy2(src, dst)

            # 2. MD5 of MBTiles content only
            uncompressed_md5 = _calc_mbtiles_content_md5(src)
            md5_sidecar = NAV_PUBLIC / f"{tile_name}.mbtiles.md5"
            md5_sidecar.write_text(f"{uncompressed_md5}  {tile_name}.mbtiles\n")

            # 3. Compress: .mbtiles → .mbtiles.lz4
            lz4_dst = NAV_PUBLIC / f"{tile_name}.mbtiles.lz4"
            if not run_cmd(
                ["lz4", "-9", "-f", str(dst), str(lz4_dst)],
                f"[{index}/{len(grid_files)} | {progress:.1f}%] Compressing {src.name}",
                timeout=300,
            ):
                errors += 1
                continue

            dst.unlink()

            # 4. MD5 of .lz4 file
            lz4_md5 = _calc_md5(lz4_dst)
            lz4_md5_sidecar = NAV_PUBLIC / f"{tile_name}.mbtiles.lz4.md5"
            lz4_md5_sidecar.write_text(f"{lz4_md5}  {tile_name}.mbtiles.lz4\n")

            orig_mb = src.stat().st_size / (1024 ** 2)
            lz4_mb  = lz4_dst.stat().st_size / (1024 ** 2)
            lz4_size_bytes = lz4_dst.stat().st_size
            tiles_manifest["files"][lz4_dst.name] = {
                "path": lz4_dst.name,
                "size_bytes": lz4_size_bytes,
                "size_mb": round(lz4_size_bytes / (1024 ** 2), 2),
                "content_md5": uncompressed_md5,
            }
            tiles_manifest["total_files"] += 1
            tiles_manifest["total_size_bytes"] += lz4_size_bytes
            log(
                f"  ✓ [{index}/{len(grid_files)} | {progress:.1f}%] {tile_name}: "
                f"{orig_mb:.1f} MB → {lz4_mb:.1f} MB ({lz4_mb / orig_mb * 100:.0f}%)"
            )

        except Exception as e:
            log(f"  {src.name}: ERROR — {e}", "ERROR")
            errors += 1

    if errors > 0:
        log(f"✗ {errors} file(s) failed compression — aborting publish", "ERROR")
        return False

    # 5. Build tiles_index.json
    index = {
        "generated_at": datetime.utcnow().isoformat(),
        "tiles": {},
        "total_tiles": 0,
        "total_size_bytes": 0,
        "bbox": str(INDIA_BBOX),
    }
    for lz4_file in sorted(NAV_PUBLIC.glob("*.mbtiles.lz4")):
        tile_name = lz4_file.name[: -len(".mbtiles.lz4")]
        md5_sidecar = NAV_PUBLIC / f"{tile_name}.mbtiles.md5"
        if md5_sidecar.exists():
            md5 = md5_sidecar.read_text().strip().split()[0]
        else:
            md5 = _calc_md5(lz4_file)
        file_size = lz4_file.stat().st_size
        index["tiles"][tile_name] = {
            "filename": lz4_file.name,
            "size_bytes": file_size,
            "size_mb": round(file_size / (1024 ** 2), 2),
            "md5": md5,
        }
        index["total_tiles"] += 1
        index["total_size_bytes"] += file_size

    index["total_size_gb"] = round(index["total_size_bytes"] / (1024 ** 3), 2)
    tiles_manifest["total_size_gb"] = round(tiles_manifest["total_size_bytes"] / (1024 ** 3), 2)

    tiles_index_path = NAV_PUBLIC / "tiles_index.json"
    tiles_index_path.write_text(json.dumps(index, indent=2))
    tiles_manifest_path = NAV_PUBLIC / "tiles_manifest.json"
    tiles_manifest_path.write_text(json.dumps(tiles_manifest, indent=2))
    log(f"✓ tiles_index.json: {index['total_tiles']} tiles, "
        f"{index['total_size_gb']:.2f} GB compressed", "SUCCESS")
    log("✓ tiles_manifest.json written with content MD5s", "SUCCESS")

    # --- Publish Valhalla ---
    log("Publishing Valhalla routing tiles...")
    if VAL_PUBLIC.exists():
        shutil.rmtree(VAL_PUBLIC)
    VAL_PUBLIC.mkdir(parents=True, exist_ok=True)

    if VALHALLA_TAR_LZ4.exists():
        try:
            # Copy tar.lz4
            shutil.copy2(VALHALLA_TAR_LZ4, VAL_PUBLIC / "valhalla_tiles.tar.lz4")
            # Copy MD5
            md5_src = Path(str(VALHALLA_TAR_LZ4) + ".md5")
            if md5_src.exists():
                shutil.copy2(md5_src, VAL_PUBLIC / "valhalla_tiles.tar.lz4.md5")
            # Copy manifest
            manifest_src = VALHALLA_TILES / "manifest.json"
            if manifest_src.exists():
                shutil.copy2(manifest_src, VAL_PUBLIC / "valhalla_tiles_manifest.json")

            # Copy support files and raw individual tile files
            for item in sorted(VALHALLA_DIR.iterdir()):
                if item.name == "valhalla_tiles_skip.tar":
                    continue
                dest = VAL_PUBLIC / item.name
                if item.is_file():
                    shutil.copy2(item, dest)
                elif item.is_dir():
                    if dest.exists():
                        shutil.rmtree(dest)
                    shutil.copytree(item, dest)
            
            tar_size_mb = (VAL_PUBLIC / "valhalla_tiles.tar.lz4").stat().st_size / (1024 ** 2)
            raw_tile_count = len(list((VAL_PUBLIC / "valhalla_tiles").glob("**/*.gph"))) if (VAL_PUBLIC / "valhalla_tiles").exists() else 0
            log(f"✓ Valhalla published: tar={tar_size_mb:.1f} MB, raw tiles={raw_tile_count}", "SUCCESS")
        except Exception as e:
            log(f"Valhalla publish failed: {e}", "WARN")
    else:
        log("Valhalla tar.lz4 not found - skipping", "WARN")

    # --- Publish MBTiles patch artifacts ---
    try:
        log("Generating MBTiles incremental/merged patches...")
        generate_mbtiles_patches(PUBLIC_EXPORT, NAV_PUBLIC, APP_MBTILES, POI_DB)
    except Exception as e:
        log(f"Patch generation failed: {e}", "ERROR")
        return False

    # 6. Drop deployment marker
    MARKER.write_text(datetime.utcnow().isoformat())
    log(f"✓ Deployment marker written: {MARKER}", "SUCCESS")
    log("  server.py will pick this up within 60 s and switch the active host", "INFO")

    return True


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    import time
    log("=" * 70)
    log("SIMPLE INDIA MBTILES PIPELINE")
    log("=" * 70)
    log(f"Workers: {NUM_WORKERS}")
    log("Starting in 3 seconds...")
    time.sleep(3)

    for i, step_func in enumerate([
        step1_download,
        step3b_generate_poi,
        step2_generate_app_mbtiles,
        step3_filter_pbf,
        step4_generate_navigator_mbtiles,
        step4b_build_valhalla,
        step5_archive_valhalla,
        step5_repack_tiles,
        step6_verify,
        step7_publish,
    ], 1):
        log("")
        log(f"[{i}/{TOTAL_STEPS}] Starting {step_func.__name__}()")
        if not step_func():
            log(f"✗ Pipeline failed at step {i}: {step_func.__name__}", "ERROR")
            sys.exit(1)
        log(f"✓ Step {i}/{TOTAL_STEPS} complete", "SUCCESS")

    log("")
    log("=" * 70)
    log("✓ PIPELINE COMPLETE!", "SUCCESS")
    log("=" * 70)
    log(f"  POI DB:            {POI_DB}")
    log(f"  App MBTiles:       {APP_MBTILES}")
    log(f"  Navigator MBTiles: {NAVIGATOR_MBTILES}")
    log(f"  Grid MBTiles:      {FINAL_DIR}/*.mbtiles")
    log(f"  Grid file count:   {len(list(FINAL_DIR.glob('*.mbtiles')))}")
    log(f"  Valhalla tiles:    {len(list(VALHALLA_TILES.glob('**/*.gph')))} .gph files")
    log(f"  Valhalla archive:  {VALHALLA_TAR_LZ4}")
    log(f"  Published to:      /home/navigator/public_export/")
    log(f"  Deployment marker: /home/navigator/completed.done  (server will pick up within 60s)")


if __name__ == "__main__":
    multiprocessing.freeze_support()
    try:
        main()
    except KeyboardInterrupt:
        log("Pipeline interrupted by user", "WARN")
        sys.exit(1)
    except Exception as e:
        log(f"Pipeline failed: {e}", "ERROR")
        import traceback
        traceback.print_exc()
        sys.exit(1)