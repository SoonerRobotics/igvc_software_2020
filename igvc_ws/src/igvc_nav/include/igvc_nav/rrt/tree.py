from rtree import index

class Tree(object):
    def init(self, X):
        """
        Tree representation
        :param X: Search space
        """
        p = index.Property()
        p.dimension = X.dimension
        self.V = index.Index(interleaved=True, properties=p)
        self.V_count = 0
        self.E = {}
