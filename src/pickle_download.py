import osmnx as ox
import pickle
import os
import networkx as nx

print("Loading full graph...")
with open("lebanon_cookeville.pkl", "rb") as f:
    G = pickle.load(f)

print(f"Original: {G.number_of_nodes()} nodes, {G.number_of_edges()} edges")
print(f"Original file: {os.path.getsize('lebanon_cookeville.pkl')/1024/1024:.1f} MB")

print("Stripping to minimal attributes...")

# Keep only the largest strongly connected component
# This removes dead ends and isolated segments that waste memory
G = ox.truncate.largest_component(G, strongly=True)
print(f"After largest component: {G.number_of_nodes()} nodes")

# Build a new minimal graph with only essential attributes
G_slim = nx.MultiDiGraph()

# Copy graph-level CRS attribute
G_slim.graph["crs"] = G.graph.get("crs", "epsg:4326")

# Add nodes with only lat/lon
for node, data in G.nodes(data=True):
    G_slim.add_node(node, x=data["x"], y=data["y"])

# Add edges with only length
for u, v, key, data in G.edges(keys=True, data=True):
    G_slim.add_edge(u, v, key=key, length=data.get("length", 1.0))

print(f"Slim graph: {G_slim.number_of_nodes()} nodes, {G_slim.number_of_edges()} edges")

# Save slim version
with open("lebanon_cookeville_slim.pkl", "wb") as f:
    pickle.dump(G_slim, f, protocol=4)

slim_mb = os.path.getsize("lebanon_cookeville_slim.pkl") / 1024 / 1024
print(f"Slim file: {slim_mb:.1f} MB")
print(f"Size reduction: {200/slim_mb:.1f}x smaller")