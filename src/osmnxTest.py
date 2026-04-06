# Save this as a Python file (e.g., my_script.py)
import osmnx as ox

# 1. Specify the place name
place = 'New York City, NY, USA'

# 2. Download the street network data (e.g., for driving)
# This uses the graph_from_place function to fetch data from OpenStreetMap
G = ox.graph_from_place(place, network_type='drive')

# 3. Visualize the street network (optional, requires matplotlib)
# This will open a plot window or display in a Jupyter Notebook
fig, ax = ox.plot_graph(G)

# 4. You can also save the graph to disk (e.g., as a GraphML file)
ox.io.save_graphml(G, filepath=f'./{place.replace(", ", "_")}.graphml')

# You can now use the 'G' (a NetworkX MultiDiGraph object) 
# for further analysis, routing, or statistics.

