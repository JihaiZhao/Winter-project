from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib import pyplot as plt
import numpy as np
from numpy import random

class Node:
    def __init__(self, pivot, position, parent, children):
        self.pivot = pivot
        self.position = position
        self.parent = parent
        self.children = children

class packing:
    #set the size of the container
    def __init__(self, X = 50, Y = 50, Z = 100, width = [], length = [], height = []):
        self.X = X
        self.Y = Y
        self.Z = Z
        self.width = width
        self.length = length
        self.height = height
    
    #draw the container
    def set_environment(self, ax):
        ax.set_xlim3d(0,self.X)
        ax.set_ylim3d(0,self.Y)
        ax.set_zlim3d(0,self.Z)
        VecStart_x = [0, self.X, self.X, 0, 0, 0, 0, 0, 0, self.X, self.X, 0, self.X, self.X, self.X, self.X]
        VecEnd_x = [self.X, self.X, 0, 0, 0, 0, 0, 0, self.X, self.X, 0, 0, self.X, self.X, self.X, self.X]
        VecStart_y = [0, 0, 0, 0, 0, self.Y, self.Y, 0, self.Y, self.Y, self.Y, self.Y, 0, self.Y, self.Y, 0]
        VecEnd_y = [0, 0, 0, 0, self.Y, self.Y, 0, 0, self.Y, self.Y, self.Y, self.Y, self.Y, self.Y, 0, 0]
        VecStart_z = [0, 0, self.Z, self.Z, 0, 0, self.Z, self.Z, 0, 0, self.Z, self.Z, 0, 0, self.Z, self.Z]
        VecEnd_z = [0, self.Z, self.Z, 0, 0, self.Z, self.Z, 0, 0, self.Z, self.Z, 0, 0, self.Z, self.Z, 0]

        for i in range(len(VecStart_x)):
            ax.plot([VecStart_x[i], VecEnd_x[i]], 
                    [VecStart_y[i],VecEnd_y[i]],
                    zs=[VecStart_z[i],VecEnd_z[i]])

    def generate_box(self):
        for i in range(10):
            self.width.append(random.randint(5,25))
            self.length.append(random.randint(5,25))
            self.height.append(random.randint(5,25))

    def cuboid_data2(self, o, size=(1,1,1)):
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
        # print(X)
        return X

    def plotCubeAt2(self, positions,sizes=None,colors=None, **kwargs):
        g = []
        for p,s,c in zip(positions,sizes,colors):
            g.append(self.cuboid_data2(p, size=s) )
        return Poly3DCollection(np.concatenate(g),  
                                facecolors=np.repeat(colors,6), **kwargs)
    
    def near(self, node):
        point = 0
        for i in range(len(node)):
            dis = 1
            if dis < dis_near:
                dis_near = dis
                point = i
            # for j in range(len(toPack)):
            #     if toPack[i][0] == toPack[j][0]:
            #         pivot = [positions[j][0], positions[j][1], 0]
            #         self.Z -= toPack[i][2]
            #         positions.append(pivot)
            #         sizes.append(notpacked[i])
            #         # new_node = Node(pivot,sizes[i], None)
            #         # node.append(new_node)
            #     elif toPack[i][1] == toPack[j][1]:
            #         pivot = [positions[j][0], positions[j][1], 0]
            #         self.Z -= toPack[i][2]
            #         positions.append(pivot)
            #         sizes.append(notpacked[i])
            #         # new_node = Node(pivot,sizes(i),None)
            #         # node.append(new_node)                    
        return  point 
    
    def new(self, node, toPack):
        if self.X >= toPack[0][0]:
            # print(toPack[i][0])
            pivot = [positions[i-1][0]+toPack[i-1][0], 0, 0]
            self.X -= toPack[i][0]
            positions.append(pivot)
            # print(positions)
            sizes.append(notpacked[i])
            # new_node = Node(pivot,sizes(i),None)
            # node.append(new_node)
        elif self.Y >= toPack[0][1]:
            # print(toPack[i][0])
            pivot = [0, toPack[0][1]+toPack[i-1][1], 0]
            self.Y -= toPack[i][1]
            positions.append(pivot)
            # print(positions)
            sizes.append(notpacked[i])
            # new_node = Node(pivot,sizes(i),None)
            # node.append(new_node)
        else:
            break


    def generate_solution(self, ax):
        notpacked = []
        for w,l,h in zip(self.width,self.length,self.height):
            notpacked.append([w,l,h]) #x,y,z

        positions = []
        sizes = []
        toPack = notpacked[0]

        #put the first item at pivot[0,0,0]
        pivot = [0,0,0]
        positions.append(pivot)
        self.X -= toPack[0][0]
        self.Y -= toPack[0][1]
        self.Z -= toPack[0][2]
        sizes.append(notpacked[0])
        q_init = Node(pivot,sizes[0],0,None)
        node = [q_init]
    


        colors = ["crimson","crimson","crimson","crimson","crimson","crimson","crimson","crimson","crimson","crimson","crimson","crimson","crimson","crimson"]
        pc = self.plotCubeAt2(positions,sizes,colors=colors,edgecolor="k")
        print(positions)
        print(sizes)

        ax.add_collection3d(pc)    
        plt.show()




def main():

    test = packing()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    # ax.set_aspect('equal')
    test.set_environment(ax)
    test.generate_box()
    test.generate_solution(ax)

    # positions = [(0,0,0),(14,0,0)]
    # sizes = [(14,25,13), (3,13,7)]
    # colors = ["crimson","limegreen"]

    # pc = test.plotCubeAt2(positions,sizes,colors=colors,edgecolor="k")
    # ax.add_collection3d(pc)    
    # plt.show()


if __name__=='__main__':
    main()