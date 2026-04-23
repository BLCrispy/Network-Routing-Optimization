import xml.etree.ElementTree as ET
import sqlite3
import os
import math

OSM_FILE = "leb_cook.osm"
DB_FILE  = "leb_cook_final.db"

# Tight bounding box — trims anything outside the corridor
NORTH =  36.28
SOUTH =  36.05
EAST  = -85.45
WEST  = -86.35

# Road types to keep — excludes footpaths, service roads, etc.
KEEP_HIGHWAY = {
    "motorway", "motorway_link",
    "trunk", "trunk_link",
    "primary", "primary_link",
    "secondary", "secondary_link",
    "tertiary", "tertiary_link",
    "residential", "unclassified",
    "living_street",
}

print(f"Parsing {OSM_FILE}...")
tree = ET.parse(OSM_FILE)
root = tree.getroot()

# ── Pass 1: collect all nodes ──────────────────────────────────────────────
all_nodes = {}
for node in root.iter("node"):
    nid = int(node.get("id"))
    lat = float(node.get("lat"))
    lon = float(node.get("lon"))
    all_nodes[nid] = (lat, lon)

print(f"Total OSM nodes: {len(all_nodes)}")

# ── Pass 2: collect valid ways ─────────────────────────────────────────────
# Only keep roads of types we care about
valid_ways = []
for way in root.iter("way"):
    highway = None
    oneway  = False
    for tag in way.iter("tag"):
        k = tag.get("k")
        v = tag.get("v")
        if k == "highway":
            highway = v
        if k == "oneway" and v in ("yes", "1", "true"):
            oneway = True

    if highway not in KEEP_HIGHWAY:
        continue

    nd_refs = [int(nd.get("ref")) for nd in way.iter("nd")]
    if len(nd_refs) < 2:
        continue

    valid_ways.append((nd_refs, oneway))

print(f"Valid ways: {len(valid_ways)}")

# ── Pass 3: find nodes actually used by valid ways ─────────────────────────
used_node_ids = set()
for nd_refs, _ in valid_ways:
    used_node_ids.update(nd_refs)

# Filter to bounding box
kept_nodes = {}
for nid in used_node_ids:
    if nid not in all_nodes:
        continue
    lat, lon = all_nodes[nid]
    if SOUTH <= lat <= NORTH and WEST <= lon <= EAST:
        kept_nodes[nid] = (lat, lon)

print(f"Nodes in bounding box: {len(kept_nodes)}")

# ── Pass 4: build directed edges with haversine length ────────────────────
def haversine(lat1, lon1, lat2, lon2):
    R = 6_371_000
    lat1, lon1 = math.radians(lat1), math.radians(lon1)
    lat2, lon2 = math.radians(lat2), math.radians(lon2)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
    return R * 2 * math.asin(math.sqrt(a))

edges = []
for nd_refs, oneway in valid_ways:
    for i in range(len(nd_refs) - 1):
        u = nd_refs[i]
        v = nd_refs[i + 1]
        if u not in kept_nodes or v not in kept_nodes:
            continue
        lat1, lon1 = kept_nodes[u]
        lat2, lon2 = kept_nodes[v]
        length = haversine(lat1, lon1, lat2, lon2)
        edges.append((u, v, length))
        if not oneway:
            edges.append((v, u, length))

print(f"Directed edges: {len(edges)}")

# ── Build SQLite DB ────────────────────────────────────────────────────────
if os.path.exists(DB_FILE):
    os.remove(DB_FILE)

conn = sqlite3.connect(DB_FILE)
c    = conn.cursor()

c.execute("""
    CREATE TABLE nodes (
        id  INTEGER PRIMARY KEY,
        lat REAL NOT NULL,
        lon REAL NOT NULL
    )
""")

c.execute("""
    CREATE TABLE edges (
        u      INTEGER NOT NULL,
        v      INTEGER NOT NULL,
        length REAL    NOT NULL
    )
""")

# Insert nodes
node_rows = [(nid, lat, lon) for nid, (lat, lon) in kept_nodes.items()]
c.executemany("INSERT INTO nodes VALUES (?,?,?)", node_rows)

# Insert edges in batches
batch_size = 5000
for i in range(0, len(edges), batch_size):
    c.executemany("INSERT INTO edges VALUES (?,?,?)", edges[i:i+batch_size])

# Indices for fast spatial query
c.execute("CREATE INDEX idx_nodes_lat ON nodes(lat)")
c.execute("CREATE INDEX idx_nodes_lon ON nodes(lon)")
c.execute("CREATE INDEX idx_edges_u   ON edges(u)")
c.execute("CREATE INDEX idx_edges_v   ON edges(v)")

conn.commit()
conn.close()

db_mb = os.path.getsize(DB_FILE) / 1024 / 1024
print(f"\nDone!")
print(f"Nodes: {len(kept_nodes)}")
print(f"Edges: {len(edges)}")
print(f"DB size: {db_mb:.1f} MB")