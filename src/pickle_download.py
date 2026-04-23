import pickle
import sqlite3
import os

print("Loading slim pickle...")
with open("lebanon_cookeville_slim.pkl", "rb") as f:
    G = pickle.load(f)

print(f"Graph: {G.number_of_nodes()} nodes, {G.number_of_edges()} edges")

print("Converting to SQLite...")
db_path = "lebanon_cookeville.db"

if os.path.exists(db_path):
    os.remove(db_path)

conn = sqlite3.connect(db_path)
c    = conn.cursor()

# Create tables
c.execute("""
    CREATE TABLE nodes (
        id   INTEGER PRIMARY KEY,
        lat  REAL NOT NULL,
        lon  REAL NOT NULL
    )
""")

c.execute("""
    CREATE TABLE edges (
        u      INTEGER NOT NULL,
        v      INTEGER NOT NULL,
        length REAL NOT NULL
    )
""")

# Insert nodes in batches
print("Inserting nodes...")
node_batch = []
for node, data in G.nodes(data=True):
    node_batch.append((int(node), float(data["y"]), float(data["x"])))
    if len(node_batch) >= 5000:
        c.executemany("INSERT INTO nodes VALUES (?,?,?)", node_batch)
        node_batch = []
if node_batch:
    c.executemany("INSERT INTO nodes VALUES (?,?,?)", node_batch)

# Insert edges in batches
print("Inserting edges...")
edge_batch = []
for u, v, data in G.edges(data=True):
    edge_batch.append((int(u), int(v), float(data.get("length", 1.0))))
    if len(edge_batch) >= 5000:
        c.executemany("INSERT INTO edges VALUES (?,?,?)", edge_batch)
        edge_batch = []
if edge_batch:
    c.executemany("INSERT INTO edges VALUES (?,?,?)", edge_batch)

# Create indices for fast spatial and neighbour queries
print("Creating indices...")
c.execute("CREATE INDEX idx_nodes_lat ON nodes(lat)")
c.execute("CREATE INDEX idx_nodes_lon ON nodes(lon)")
c.execute("CREATE INDEX idx_edges_u   ON edges(u)")
c.execute("CREATE INDEX idx_edges_v   ON edges(v)")

conn.commit()
conn.close()

db_mb = os.path.getsize(db_path) / 1024 / 1024
print(f"Done! SQLite DB: {db_mb:.1f} MB")
print(f"Reduction: {90/db_mb:.1f}x smaller than slim pickle")


#-----------------
# This is for reference for slimming a pickle
#-----------------

# import osmnx as ox
# import pickle
# import os
# import networkx as nx

# print("Loading full graph...")
# with open("lebanon_cookeville.pkl", "rb") as f:
#     G = pickle.load(f)

# print(f"Original: {G.number_of_nodes()} nodes, {G.number_of_edges()} edges")
# print(f"Original file: {os.path.getsize('lebanon_cookeville.pkl')/1024/1024:.1f} MB")

# print("Stripping to minimal attributes...")

# # Keep only the largest strongly connected component
# # This removes dead ends and isolated segments that waste memory
# G = ox.truncate.largest_component(G, strongly=True)
# print(f"After largest component: {G.number_of_nodes()} nodes")

# # Build a new minimal graph with only essential attributes
# G_slim = nx.MultiDiGraph()

# # Copy graph-level CRS attribute
# G_slim.graph["crs"] = G.graph.get("crs", "epsg:4326")

# # Add nodes with only lat/lon
# for node, data in G.nodes(data=True):
#     G_slim.add_node(node, x=data["x"], y=data["y"])

# # Add edges with only length
# for u, v, key, data in G.edges(keys=True, data=True):
#     G_slim.add_edge(u, v, key=key, length=data.get("length", 1.0))

# print(f"Slim graph: {G_slim.number_of_nodes()} nodes, {G_slim.number_of_edges()} edges")

# # Save slim version
# with open("lebanon_cookeville_slim.pkl", "wb") as f:
#     pickle.dump(G_slim, f, protocol=4)

# slim_mb = os.path.getsize("lebanon_cookeville_slim.pkl") / 1024 / 1024
# print(f"Slim file: {slim_mb:.1f} MB")
# print(f"Size reduction: {200/slim_mb:.1f}x smaller")