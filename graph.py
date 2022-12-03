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

    def isCyclicUtil(self, vertex, visited, parent):
        visited.add(vertex)
        neighbors = self.getVertex(vertex).getConnections()
        for node in neighbors:
            if node.id not in visited:
                if(self.isCyclicUtil(node.id, visited, vertex)) == True:
                    return True
            elif parent != node.id:
                return True
        return False

    def isCyclic(self):
        visited = set()
        nodes = self.getVertices()
        for node in nodes:
            if node not in visited:
                if(self.isCyclicUtil(node, visited, -1)) == True:
                    return True
        return False       

    def shortestPath(self, start):
        res = []
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
            adj_nodes = self.getVertex(current_vertex).getConnections()
            # temp = [adj_node.id for adj_node in adj_nodes]
            # print(temp)
            for adj_node in adj_nodes:
                source_cost = nodes_status[current_vertex]['cost']
                adj_cost = self.getVertex(current_vertex).getWeight(adj_node)
                old_cost = nodes_status[adj_node.id]['cost']
                new_cost = source_cost+adj_cost
                if not nodes_status[adj_node.id]['visited'] and new_cost < old_cost: 
                    heapq.heappush(min_vertex_heap, (new_cost, adj_node.id)) 
                    nodes_status[adj_node.id]['cost'] = source_cost+adj_cost
                    nodes_status[adj_node.id]['prev'] = current_vertex
        for node in nodes_status:
            res.append((node, nodes_status[node]['cost']))
        print(res)
    
    def bellmanFord(self, start):
        res = []
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
                    return
        for node in nodes_status:
            res.append((node, nodes_status[node]['cost']))
        return res