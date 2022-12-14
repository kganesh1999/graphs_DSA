from vertex import Vertex
from collections import defaultdict
import heapq

class Graph:
    def __init__(self):
        self.vertList = dict()
        self.numVertices = 0
        self.edges = []

    def addVertex(self, key):
        self.numVertices += 1
        newVertex = Vertex(key)
        self.vertList[key] = newVertex
        return newVertex

    def addEdge(self, f, t, weight=0):
        self.edges.append((f, t, weight))
        if f not in self.vertList:
            nv = self.addVertex(f)
        if t not in self.vertList:
            nv = self.addVertex(t)
        self.vertList[f].addNeighbor(self.vertList[t], weight)

    def getVertex(self, key):
        return self.vertList.get(key)
    
    def __contains__(self, key):
        return key in self.vertList

    def getVertices(self):
        return self.vertList.keys()

    def getCount(self):
        return self.numVertices

    #Function to reverse edges of directed graph
    def getTranspose(self):
        G_T = Graph()
        for edge in self.edges:
            src, dest, w = edge
            src, dest = dest, src
            G_T.addEdge(src, dest, w)
        return G_T

    def findPath(self, start, end, path=[]):
        path = path+[start]
        if start == end:
            return path
        neighbors = self.getVertex(start).getConnections()
        for node in neighbors:
            if node.id not in path:
                newpath = self.findPath(node.id, end, path)
                if newpath:
                    return newpath
                return None                

    def find(self, vertex, dsuf):
        if dsuf[vertex] == -1:
            return vertex
        return self.find(dsuf[vertex], dsuf)

    def union(self, src, dest, dsuf):
        src_par = self.find(src, dsuf)
        dest_par = self.find(dest, dsuf)
        dsuf[src_par] = dest_par

    # def isCyclic(self):
    #     dsuf = defaultdict(lambda: -1)
    #     for edge in self.edges:
    #         src, dest, _ = edge
    #         src_par = self.find(src, dsuf)
    #         dest_par = self.find(dest, dsuf)     
    #         if src_par == dest_par:
    #             return True
    #         self.union(src_par, dest_par, dsuf)
    #     return False
    
    def isCyclic(self):
        visited = set()
        def isCyclicUtil(src, parent):
            visited.add(src)
            adj_vertices = self.getVertex(src).getConnections()
            for adj_vertex in adj_vertices:
                if adj_vertex.id not in visited:
                    if isCyclicUtil(adj_vertex.id, src): 
                        return True
                elif adj_vertex.id == parent:
                    return True
            return False
        vertices = self.getVertices()
        for vertex in vertices:
            if vertex not in visited:
                if isCyclicUtil(vertex, -1):
                    return True
        return False

    #Shortest path algo - 1 
    def djikstra(self, start):
        res = [('node','cost')]
        nodes_status = defaultdict(dict)
        vertices = self.getVertices()
        min_vertex_heap = []
        for vertex in vertices:
            nodes_status[vertex]['cost'] = float('inf')
            nodes_status[vertex]['visited'] = False
            nodes_status[vertex]['prev'] = -1
            if vertex == start:
                nodes_status[vertex]['cost'] = 0
                nodes_status[vertex]['prev'] = start
        heapq.heappush(min_vertex_heap, (nodes_status[start]['cost'], start))
        while len(min_vertex_heap)!=0:
            _, current_vertex = heapq.heappop(min_vertex_heap)
            nodes_status[current_vertex]['visited'] = True
            adj_vertices = self.getVertex(current_vertex).getConnections()
            for adj_vertex in adj_vertices:
                source_cost = nodes_status[current_vertex]['cost']
                adj_cost = self.getVertex(current_vertex).getWeight(adj_vertex)
                old_cost = nodes_status[adj_vertex.id]['cost']
                new_cost = source_cost+adj_cost
                if not nodes_status[adj_vertex.id]['visited'] and new_cost < old_cost: 
                    heapq.heappush(min_vertex_heap, (new_cost, adj_vertex.id)) 
                    nodes_status[adj_vertex.id]['cost'] = source_cost+adj_cost
                    nodes_status[adj_vertex.id]['prev'] = current_vertex
        for node in nodes_status:
            res.append((node, nodes_status[node]['cost']))
        return res
    
    #Shortest path algo - 2
    def bellmanFord(self, start):
        res = [('node','cost')]
        nodes_status = defaultdict(dict)
        vertices = self.getVertices()
        for vertex in vertices:
            nodes_status[vertex]['cost'] = float('inf')
            nodes_status[vertex]['prev'] = -1
            if vertex == start:
                nodes_status[vertex]['cost'] = 0
        for i in range(self.numVertices-1):
            updated = False
            for edge in self.edges:
                src, dest, cost = edge
                src_cost = nodes_status[src]['cost']
                dest_cost = cost
                new_cost = src_cost + dest_cost
                old_cost = nodes_status[dest]['cost']
                if new_cost < old_cost:
                    updated = True
                    nodes_status[dest]['cost'] = new_cost
                    nodes_status[dest]['prev'] = src
            if not updated:
                break
        if updated:
            # Check negative weight indefinite loop         
            for edge in self.edges:
                src, dest, cost = edge
                src_cost = nodes_status[src]['cost']
                dest_cost = cost
                new_cost = src_cost + dest_cost
                old_cost = nodes_status[dest]['cost']
                if new_cost < old_cost:
                    return "Negative cycle exists"
        for node in nodes_status:
            res.append((node, nodes_status[node]['cost']))
        return res

    #Minimum Spanning Tree Algo - 1
    def primMST(self, start):
        res = [('node','src')]
        nodes_status = defaultdict(dict)
        vertices = self.getVertices()
        min_vertex_heap = []
        for vertex in vertices:
            nodes_status[vertex]['cost'] = float('inf')
            nodes_status[vertex]['visited'] = False
            nodes_status[vertex]['prev'] = -1
            if vertex == start:
                nodes_status[vertex]['cost'] = 0
        heapq.heappush(min_vertex_heap, (nodes_status[start]['cost'], start))
        while len(min_vertex_heap)!=0:
            _, current_vertex = heapq.heappop(min_vertex_heap)
            nodes_status[current_vertex]['visited'] = True
            adj_vertices = self.getVertex(current_vertex).getConnections()
            for adj_vertex in adj_vertices:
                curr_cost = self.getVertex(current_vertex).getWeight(adj_vertex)
                old_cost = nodes_status[adj_vertex.id]['cost']
                if not nodes_status[adj_vertex.id]['visited'] and curr_cost < old_cost: 
                    heapq.heappush(min_vertex_heap, (curr_cost, adj_vertex.id)) 
                    nodes_status[adj_vertex.id]['cost'] = curr_cost
                    nodes_status[adj_vertex.id]['prev'] = current_vertex
        for node in nodes_status:
            res.append((node, nodes_status[node]['prev']))
        return res

    #Minimum Spanning Tree Algo - 2
    def kruskalMST(self):
        mst_edges_inc = []
        sorted_edges = sorted(self.edges, key= lambda x: x[2])
        dsuf = defaultdict(lambda: -1)
        i, j = 0, 0
        while i < self.numVertices-1 and j < len(self.edges):
            src, dest, _ = sorted_edges[j]
            src_par = self.find(src, dsuf)
            dest_par = self.find(dest, dsuf)
            if src_par == dest_par:
                j += 1
                continue
            self.union(src, dest, dsuf)
            mst_edges_inc.append(sorted_edges[j])
            i += 1
        return mst_edges_inc

    def doDFS(self, src, visited):
            print('SCC is: ', src)
            visited.add(src)
            adj_vertices = self.getVertex(src).getConnections()
            for adj_vertex in adj_vertices:
                if adj_vertex.id not in visited:
                    self.doDFS(adj_vertex.id, visited)
    
    #Strongest Connected Component algo - 1
    def kosarajuSCC(self):
        stack = []
        visited = set()

        def fillOrder(node):
            visited.add(node)
            adj_vertices = self.getVertex(node).getConnections()
            for adj_vertex in adj_vertices:
                if adj_vertex.id not in visited:
                    fillOrder(adj_vertex.id)
            stack.append(node)               
        vertices = self.getVertices()
        for vertex in vertices:
            if vertex not in visited:
                fillOrder(vertex)
        trans_graph = self.getTranspose()
        visited = set()
        while len(stack)!=0:
            vertex = stack.pop()
            if vertex not in visited:
                trans_graph.doDFS(vertex, visited)

    #Strongest Connected Component algo - 2
    def tarjanSCC(self):
        stack = []
        nodes_status = defaultdict(dict)
        vertices = self.getVertices()
        for vertex in vertices:
            nodes_status[vertex]['discovery'] = -1
            nodes_status[vertex]['low'] = -1
            nodes_status[vertex]['instack'] = False
        
        def doDFS(src, time):
            time += 1
            nodes_status[src]['discovery'] = nodes_status[src]['low'] = time
            stack.append(src)
            nodes_status[src]['instack'] = True
            adj_vertices = self.getVertex(src).getConnections()
            for adj_vertex in adj_vertices:
                #Do DFS if forward-edge
                if nodes_status[adj_vertex.id]['discovery'] == -1:
                    doDFS(adj_vertex.id, time)
                    nodes_status[src]['low'] = min(nodes_status[src]['low'], nodes_status[adj_vertex.id]['low'])
                #Check if back-edge
                elif nodes_status[adj_vertex.id]['instack']:
                    nodes_status[adj_vertex.id]['low'] = min(nodes_status[src]['low'], nodes_status[adj_vertex.id]['discovery'])
            #If no neighbour, do backtracking
            if nodes_status[src]['discovery'] == nodes_status[src]['low']:
                print('SCC is: \n')
                while len(stack)!=0 and stack[-1] != src:
                    print(stack[-1])
                    nodes_status[stack[-1]]['instack'] = False
                    stack.pop()
                print(stack[-1])
                nodes_status[stack[-1]]['instack'] = False
                stack.pop()
        time = -1
        for vertex in vertices:
            if nodes_status[vertex]['discovery'] == -1:
                doDFS(vertex, time)

    #Topological sorting of vertex (Kahn's algo BFS based ) 
    def topologicalSort(self):
        sorted_vertices = []
        queue = []
        vertex_incoming_degree = defaultdict(lambda: 0)
        vertices = self.getVertices()
        for vertex in vertices:
            adj_vertices = self.getVertex(vertex).getConnections()
            for adj_vertex in adj_vertices:
                vertex_incoming_degree[adj_vertex.id] += 1
        for vertex in vertices:
            if vertex_incoming_degree[vertex] == 0:
                queue.append(vertex)
        count = 0
        while len(queue)!=0:
            current_vertex = queue.pop(0)
            sorted_vertices.append(current_vertex)
            count += 1
            adj_vertices = self.getVertex(current_vertex).getConnections()
            for adj_vertex in adj_vertices:
                vertex_incoming_degree[adj_vertex.id] -= 1
                if vertex_incoming_degree[adj_vertex.id] == 0:
                    queue.append(adj_vertex.id)
        if count != self.numVertices:
            return None
        return sorted_vertices

    #DFS + Stack based algorithm
    def topologicalSort(self):
        stack = []
        visited = set()
        vertices = self.getVertices()
        print(self.isCyclic())
        if self.isCyclic():
            return stack
        
        def doDFS(src):
            visited.add(src)
            adj_vertices = self.getVertex(src).getConnections()
            for adj_vertex in adj_vertices:
                if adj_vertex.id not in visited:
                    doDFS(adj_vertex.id)
            stack.append(src)

        for vertex in vertices:
            if vertex not in visited:
                doDFS(vertex)
        return stack[::-1]


