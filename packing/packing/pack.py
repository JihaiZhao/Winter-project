import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

class Node():
    def __init__(self, pivot:tuple, size:tuple) -> None:
        self.used = False        
        self.left = None
        self.right = None
        self.height = None

        self.pivot = pivot
        self.x, self.y, self.z = pivot
        self.w, self.l, self.h = size
        self.position = []

class Rect():
    def __init__(self, size:tuple) -> None:
        assert len(size) == 3
        self.w, self.l, self.h = size
        self.fit = None

class Packer():
    def __init__(self, w:int, l:int, h:int) -> None:
        self.root = Node((0,0,0), (w, l, h))
        self.rects = []

    def find_node(self, node:Node, w:int, l:int, h:int) -> Node:
        if node.used:
            return self.find_node(node.right, w, l, h) or self.find_node(node.left
, w, l, h)
        # or self.find_node(node.height, w, l, h)
        elif (w <= node.w and l <= node.l) and h <= node.h:
            return node
        else:
            return None

    def split_node(self, node:Node, w:int, l:int, h:int) -> Node:
        node.used = True
        node.left = Node(pivot=(node.x, node.y + l, node.z), size=(node.w, node.l - l, node.h))
        node.right = Node(pivot=(node.x + w, node.y, node.z), size=(node.w - w, l, node.h))
        node.height = Node(pivot=(node.x, node.y, node.z + h), size=(node.w, node.l, node.h-h))
        node.position = [node.x + w/2, node.y + l/2, node.z + h/2]

        return node
    
    def rotate(self, node:Node, w:int, l:int, h:int) -> Node:
        return self.find_node(node, w, l, h)
     
class SimplePacker(Packer):

    def fit(self, rects:list[Rect]) -> list[Rect]:
        for rect in rects:
            node = self.find_node(self.root, rect.w, rect.l, rect.h)
            # print(self.root.height)
            if rect.h <= self.root.h:
                if node:
                    rect.fit = self.split_node(node, rect.w, rect.l, rect.h)
                    # position = self.get_position(node)
                elif node == None:
                    node = self.rotate(self.root, rect.l, rect.w, rect.h)
                    if node:
                        sw = node.x
                        node.x = node.y
                        node.y = sw
                        sw = rect.w
                        rect.w = rect.l
                        rect.l = sw
                        # print(sw)
                        rect.fit = self.split_node(node, rect.w, rect.l, rect.h)
                    else:
                        self.root = self.root.height
                        node = self.find_node(self.root, rect.w, rect.l, rect.h)
                        if node:
                            rect.fit = self.split_node(node, rect.w, rect.l, rect.h)

        return rects


def plot(ax,rects:list[Rect]) -> None:
    """ Plot a collection of rects """
    position = []
    size = []
    plt.pause(5)
    for r in rects:
        if not r.fit:
            continue
        position.append(r.fit.pivot)
        size.append([r.w,r.l,r.h])
    for k in range(len(position)):
        pc = plotCubeAt2(position[k], size[k], edgecolor="k")
        ax.add_collection3d(pc) 
        plt.pause(1)
    plt.show()

def cuboid_data2(o, size):
    X = [[[0, 1, 0], [0, 0, 0], [1, 0, 0], [1, 1, 0]],
        [[0, 0, 0], [0, 0, 1], [1, 0, 1], [1, 0, 0]],
        [[1, 0, 1], [1, 0, 0], [1, 1, 0], [1, 1, 1]],
        [[0, 0, 1], [0, 0, 0], [0, 1, 0], [0, 1, 1]],
        [[0, 1, 0], [0, 1, 1], [1, 1, 1], [1, 1, 0]],
        [[0, 1, 1], [0, 0, 1], [1, 0, 1], [1, 1, 1]]]
    X = np.array(X).astype(float)
    for i in range(3):
        X[:,:,i] *= size[i]
    X += np.array(o)
    return X

def plotCubeAt2(positions ,sizes, **kwargs):
    g = []
    g.append(cuboid_data2(positions, sizes) )
    return Poly3DCollection(np.concatenate(g), **kwargs)

#draw the container
def set_environment(ax,sizes):
    ax.set_xlim3d(0,sizes[0])
    ax.set_ylim3d(0,sizes[1])
    ax.set_zlim3d(0,sizes[2])
    VecStart_x = [0, sizes[0], sizes[0], 0, 0, 0, 0, 0, 0, sizes[0], sizes[0], 0, sizes[0], sizes[0], sizes[0], sizes[0]]
    VecEnd_x = [sizes[0], sizes[0], 0, 0, 0, 0, 0, 0, sizes[0], sizes[0], 0, 0, sizes[0], sizes[0], sizes[0], sizes[0]]
    VecStart_y = [0, 0, 0, 0, 0, sizes[1], sizes[1], 0, sizes[1], sizes[1], sizes[1], sizes[1], 0, sizes[1], sizes[1], 0]
    VecEnd_y = [0, 0, 0, 0, sizes[1], sizes[1], 0, 0, sizes[1], sizes[1], sizes[1], sizes[1], sizes[1], sizes[1], 0, 0]
    VecStart_z = [0, 0, sizes[2], sizes[2], 0, 0, sizes[2], sizes[2], 0, 0, sizes[2], sizes[2], 0, 0, sizes[2], sizes[2]]
    VecEnd_z = [0, sizes[2], sizes[2], 0, 0, sizes[2], sizes[2], 0, 0, sizes[2], sizes[2], 0, 0, sizes[2], sizes[2], 0]

    for i in range(len(VecStart_x)):
        ax.plot([VecStart_x[i], VecEnd_x[i]], 
                [VecStart_y[i],VecEnd_y[i]],
                zs=[VecStart_z[i],VecEnd_z[i]])

def main():
    cat1_p3 = [
        (6,10,10),
        (10,5,10),
        (5,5,10),
        (5,5,10),
        (5,10,10),
        (10,6,10),
        (5,3,10),
        (3,5,10),
        (5,3,10),
        (5,5,10),
        (10,6,10),
        (6,10,10),
        (5,5,10),
        (10,5,10),
        (10,5,10),
        (5,10,10),
        (10,6,10),
        (5,5,10),
        (5,9,10),
        (3,5,10),
        (9,7,10),
        (7,7,10),
        (5,7,10)
    ]
    fig = plt.figure()
    sizes = (20,20,30)
    ax = fig.add_subplot(111, projection='3d')
    set_environment(ax,sizes)
    dims = cat1_p3
    rects = [Rect(d) for d in dims]

    p = SimplePacker(*sizes)
    rects = p.fit(rects) 
    plot(ax,rects)
    for r in rects:
        if r.fit != None:
            print(r.fit.position)
    # ,auto_bounds=True
        # cat1_p3 = input("inter")

if __name__=='__main__':
    main()