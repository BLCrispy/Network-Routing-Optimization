import osmium
import osmnx as ox
import pickle
import os

print("Converting PBF to OSM XML...")

class OSMWriter(osmium.SimpleHandler):
    def __init__(self, writer):
        super().__init__()
        self.writer = writer

    def node(self, n):
        self.writer.add_node(n)

    def way(self, w):
        self.writer.add_way(w)

    def relation(self, r):
        self.writer.add_relation(r)

writer = osmium.SimpleWriter("lebanon_cookeville.osm")
handler = OSMWriter(writer)
handler.apply_file("lebanon_cookeville.osm.pbf")
writer.close()
print("Converted to OSM XML")

print("Building graph...")
G = ox.graph_from_xml("lebanon_cookeville.osm", simplify=True)
print(f"Graph: {G.number_of_nodes()} nodes, {G.number_of_edges()} edges")

with open("lebanon_cookeville.pkl", "wb") as f:
    pickle.dump(G, f, protocol=4)

size_mb = os.path.getsize("lebanon_cookeville.pkl") / 1024 / 1024
print(f"Saved: {size_mb:.1f} MB")