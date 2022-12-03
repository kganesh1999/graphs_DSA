from vertex import Vertex
from graph import Graph

if __name__ == '__main__':
    G = Graph()
    input_edges = [(0,1,5), (0,1,6), (1,2,7), (1,4,5), (1,3,6)]
    for edge in input_edges:
        s, d, w = edge
        G.addEdge(s, d, w)
    p = G.bellmanFord(0)
    print(p)
# {0: 0, 1: 4, 2: 11, 3: 17, 4: 9, 5: 22, 6: 7, 7: 8, 8: 11}