import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

class Node():
    def __init__(self, origin:tuple, size:tuple) -> None:

        self.used = False        
        self.down = None
        self.right = None

        self.origin = origin
        self.x, self.y = origin
        print(origin)
        self.w, self.h = size

class Rect():
    def __init__(self, size:tuple) -> None:
        assert len(size) == 3
        self.w, self.h, self.height = size
        self.fit = None
        self.inbounds = True

class Packer():

    def __init__(self, min_w:int, min_h:int) -> None:
        self.root = Node((0,0), (min_w, min_h))
        self.bounds = (min_w, min_h)
        self.rects = []
    
    def increment_size(self, amount:int = 1) -> None:
        """ Increment the size of the bounds by +amount in both directions"""
        self.bounds = (self.bounds[0] + amount, self.bounds[1] + amount)
        self.root = Node((0,0), (self.bounds[0] + amount, self.bounds[1] + amount))

    def n_in_bounds(self) -> int:
        """ Get the number of rects that are inbound"""
        return sum([r.inbounds for r in self.rects])

    def n_outside_bounds(self) -> int:
        """ Get the number of rects that are out of bounds"""
        return len(self.rects) - self.n_in_bounds()

    def fit(self, rects:list[Rect], auto_bounds:bool=False) -> list[Rect]:
        """ 
        Fit the given rects into the bounds. if auto bounds is set to true the bounds will
        expand everytime the fit is unsuccessful in packing all rects
        """
        successful_fit = False
        self.rects = rects

        while True:
            self.rects = self._fit(rects)
            
            successful_fit = (0 == self.n_outside_bounds())

            if successful_fit or not auto_bounds:
                return self.rects
            else:
                self.increment_size()


    def _fit(self, rects:list[Rect]) -> list[Rect]:
        ...


    def find_node(self, node:Node, w:int, h:int) -> Node:
        if node.used:
            return self.find_node(node.right, w, h) or self.find_node(node.down, w, h)
        elif w <= node.w and h <= node.h:
            return node
        else:
            return None


    def split_node(self, node:Node, w:int, h:int) -> Node:
        node.used = True
        node.down = Node(origin=(node.x, node.y + h), size=(node.w, node.h - h))
        node.right = Node(origin=(node.x + w, node.y), size=(node.w - w, h))
        return node

class SimplePacker(Packer):

    def _fit(self, rects:list[Rect]) -> list[Rect]:
        for rect in rects:
            node = self.find_node(self.root, rect.w, rect.h)
            if node:
                rect.fit = self.split_node(node, rect.w, rect.h)
        
            # Set inbound variable for each rect
            if rect.fit:
                rect.inbounds = False if self.root.x+self.bounds[0] < rect.fit.x or self.root.y+self.bounds[1] < rect.fit.y else True
            else:
                rect.inbounds = False

        return rects

def plot(rects:list[Rect], figsize:tuple=(7,7)) -> None:
    """ Plot a collection of rects """
    _, ax = plt.subplots(figsize=figsize)

    ax.set_xlim([0,rects[0].fit.w])
    ax.set_ylim([0,rects[0].fit.h])
    plt.locator_params(axis="both", integer=True, tight=True)

    for r in rects:
        if not r.fit:
            continue
        draw_rect(ax, r)

    plt.show()


def draw_rect(ax, rect: Rect) -> None:
    """ Draw a single rect object """
    box = Rectangle(rect.fit.origin, rect.w, rect.h, fc='lightblue',ec='black',alpha=1.0)
    ax.add_patch(box)

if __name__ == "__main__":
    cat1_p3 = [
        (4,14,10),
        (5,2,10),
        (2,2,10),
        (9,7,10),
        (5,10,10),
        (2,5,10),
        (7,7,10),
        (3,5,10),
        (6,5,10),
        (3,2,10),
        (6,2,10),
        (4,6,10),
        (6,3,10),
        (10,3,10),
        (6,3,10),
        (6,3,10),
        (10,3,10)
    ]

    dims = cat1_p3
    size = (20, 20)
    rects = [Rect(d) for d in dims]

    p = SimplePacker(*size)

    rects = p.fit(rects, auto_bounds=True)
    plot(rects)