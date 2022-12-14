class Vertex:
    def __init__(self, key):
        self.id = key
        self.connectedTo = dict()

    def addNeighbor(self, nbr, weight=0):
        self.connectedTo[nbr] = weight
    
    def __str__(self):
        return f"{str(self.id)} connected to: {str([x.id for x in self.connectedTo])}"

    def getConnections(self):
        return self.connectedTo.keys()

    def getID(self):
        return self.id

    def getWeight(self, nbr):
        return self.connectedTo.get(nbr)