import osmnx as ox
import pickle

print("Downloading full Cookeville graph...")
G = ox.graph_from_place(
    "Cookeville, Tennessee, USA",
    network_type="drive",
    simplify=True,
)
print(f"Full graph: {G.number_of_nodes()} nodes, {G.number_of_edges()} edges")

# Save as pickle
with open("cookeville_full.pkl", "wb") as f:
    pickle.dump(G, f, protocol=4)

import os
size_mb = os.path.getsize("cookeville_full.pkl") / 1024 / 1024
print(f"Saved: {size_mb:.1f} MB")