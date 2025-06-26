import osmium
import os
import multiprocessing
from math import floor
import pickle
from tqdm import tqdm

# --- Config ---
TILE_SIZE = 0.01
OUT_DIR = "vectortiles"
CACHE_DIR = "tilecache"
CHECKPOINT_FILE = "completed_tiles.txt"
os.makedirs(CACHE_DIR, exist_ok=True)
os.makedirs(OUT_DIR, exist_ok=True)

# --- Coordinate helpers ---
def lat_index(lat): return int(floor(lat * 100))
def lon_index(lon): return int(floor(lon * 100))

# --- Checkpoint Helpers ---
def load_checkpoints():
    if os.path.exists(CHECKPOINT_FILE):
        with open(CHECKPOINT_FILE) as f:
            return set(line.strip() for line in f if line.strip())
    return set()

def is_completed(lat_idx, lon_idx):
    return f"{lat_idx},{lon_idx}" in COMPLETED_TILES

def append_checkpoint(lat_idx, lon_idx):
    with open(CHECKPOINT_FILE, "a") as f:
        f.write(f"{lat_idx},{lon_idx}\n")

# --- Pre-Pass Handler ---
class LatBandGrouper(osmium.SimpleHandler):
    def __init__(self):
        super().__init__()
        self.bands = {}

    def node(self, n):
        if not n.location.valid(): return
        band = lat_index(n.location.lat)
        self.bands.setdefault(band, []).append(('node', n))

    def way(self, w):
        if not w.nodes or not w.nodes[0].location.valid(): return
        band = lat_index(w.nodes[0].location.lat)
        self.bands.setdefault(band, []).append(('way', w))

    def save(self):
        for band, items in self.bands.items():
            path = os.path.join(CACHE_DIR, f"{band}.pkl")
            with open(path, 'wb') as f:
                pickle.dump(items, f)

# --- Worker Function ---
def process_lat_band(band_idx):
    cache_path = os.path.join(CACHE_DIR, f"{band_idx}.pkl")
    if not os.path.exists(cache_path):
        return 0

    with open(cache_path, 'rb') as f:
        objects = pickle.load(f)

    tiles = {}
    for typ, obj in objects:
        if typ == 'node':
            lat_idx = lat_index(obj.location.lat)
            lon_idx = lon_index(obj.location.lon)
        elif typ == 'way':
            loc = obj.nodes[0].location
            if not loc.valid(): continue
            lat_idx = lat_index(loc.lat)
            lon_idx = lon_index(loc.lon)
        else:
            continue

        tile_key = (lat_idx, lon_idx)
        if is_completed(*tile_key):
            continue
        tiles.setdefault(tile_key, []).append((typ, obj))

    written = 0
    for (lat_idx, lon_idx), objs in tiles.items():
        folder = os.path.join(OUT_DIR, str(lat_idx))
        os.makedirs(folder, exist_ok=True)
        path = os.path.join(folder, f"{lon_idx}.pbf")
        writer = osmium.SimpleWriter(path)
        for typ, obj in objs:
            if typ == 'node':
                writer.add_node(obj)
            elif typ == 'way':
                writer.add_way(obj)
        writer.close()
        append_checkpoint(lat_idx, lon_idx)
        written += 1

    return written

# --- Main Controller ---
def main(input_pbf, num_workers=None):
    global COMPLETED_TILES
    COMPLETED_TILES = load_checkpoints()

    if not os.listdir(CACHE_DIR):
        print("🔍 Preprocessing input PBF...")
        grouper = LatBandGrouper()
        grouper.apply_file(input_pbf, locations=True)
        grouper.save()
        print(f"✅ Saved {len(grouper.bands)} latitude bands to {CACHE_DIR}/")

    bands = [int(f[:-4]) for f in os.listdir(CACHE_DIR) if f.endswith(".pkl")]
    num_workers = num_workers or multiprocessing.cpu_count()

    print(f"🚀 Processing {len(bands)} lat bands using {num_workers} workers...")
    with multiprocessing.Pool(num_workers) as pool:
        results = list(tqdm(pool.imap_unordered(process_lat_band, bands), total=len(bands)))

    print(f"✅ Done. {sum(results)} new tiles written this run.")

# --- Entry Point ---
if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        print("Usage: python parallel_splitter_cache.py path/to/your.osm.pbf")
    else:
        main(sys.argv[1])
