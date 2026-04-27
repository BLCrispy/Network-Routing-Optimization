# Network-Routing-Optimization
Network Routing Optimization Project for CSC2400

By: Bryce Lander, Steven Crocker, Ricky Newbold

This projects goal is to implement Dijkstra, A*, Bellman-Ford, and Ant-Colony Optimzation shortest pathing Algorithms on OSM road network MultiDiGraphs to understand how their efficiency is affected as graph scales as well as other potentially affecting graph attributes. This allows us to determine what algorithms can be useful when trying to implement Network-Routing-Optimization to find the shortest possible path. We used three implementations of this. 

IMPLEMENTATION 1
    Design:
        We implemented our algorithms on 20 different cities of random variation size to test both algorithm scalability when it comes to both run time and memory usage as well as how other graph attributes can potentially affect algorithm efficiency
    DEVICE SPECS:
        CPU:     Core i7 8700
        MEMORY:  64GB DDR4
        GPU:     GTX 1660 ti
        STORAGE: N/A

IMPLEMENTATION 2
    Design:
        We implemented our algorithms on 110 subgraphs from the USA OSM graph with a center around the middle of the country in kansas. Each subgraph was incrementally increased to test the affect of graph size has on each of the algorithms efficiency.
    DEVICE SPECS:
        CPU: Intel Core Ultra 9 185H
        MEMORY: 32GB DDR5 7467 MT/s
        GPU: Intel Arc Graphics
        Storage: 2TB M.2 NVME SSD

IMPLENTATION 3
    Design:
        We implemented our algorithms in a GPS program I created on a Raspberry Pi Zero 2 W to test algorithms efficiency and capabilities in a resource constrained envirment. This helps determine how to be able implement network routing optimization through the implementation of algorithms to do GPS Routing (A Real World Application) to be best optimized to work on lower power and resource devices such that would be for GPS Devices.
    DEVICE SPECS:
        CPU: Quad-Core 64-bit Arm Cotex-A53 CPU
        MEMORY: 512MB LPDDR2 SDRAM / 1GB SWAP microSD memory (prevent "OUT OF MEMORY" specifically in early optimization testing)
        GPU: Broadcom VideoCore IV GPU
        Storage: 32GB microSD