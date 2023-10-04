import dataclasses
import numpy as np
from matplotlib import pyplot as plt
from copy import copy

@dataclasses.dataclass
class specification:
    numMeshLODLevels: int = 2
    meshBaseLODExtentHeightfieldTexels: int = 8

specs = specification()

def Point3(x, y, z):
    return np.array([x, y, z])

def gridIndex(x, y, stride):
    return y*stride + x

def remapIndices(index_array, indices):
    indices_new = np.arange(np.array(indices).shape[0])

def querryPointIndex(A, new_indices, prev_indices, index_count, points):
    hash = str(A)
    if hash in prev_indices.keys():
        index = prev_indices[hash]
    elif hash in new_indices.keys():
        index = new_indices[hash]
    else:
        index = copy(index_count)
        points.append(A)
        new_indices[hash] = index
        index_count += 1
    return index, new_indices, index_count

def addTriangle(A,B,C, indices, points, new_indices, prev_indices, index_count):
    A_idx, new_indices, index_count = querryPointIndex(A, new_indices, prev_indices, index_count, points)
    B_idx, new_indices, index_count = querryPointIndex(B, new_indices, prev_indices, index_count, points)
    C_idx, new_indices, index_count  = querryPointIndex(C, new_indices, prev_indices, index_count, points)
    indices.append(A_idx)
    indices.append(B_idx)
    indices.append(C_idx)
    
    return indices, points, new_indices, prev_indices, index_count

def buildMesh():
    builder = []
    index_count = 0
    prev_indexes = {}
    new_indexes = {}
    
    for level in range(0, specs.numMeshLODLevels):
        step = (1 << level)
        if level == 0:
            prevStep = 0
        else:
            prevStep = max(0, (1 << (level - 1)))
        halfStep = prevStep

        g = specs.meshBaseLODExtentHeightfieldTexels / 2
        L = float(level)

        # Pad by one element to hide the gap to the next level
        pad = 0
        radius = int(step * (g + pad))

        for z in range(-radius, radius, step):
            for x in range(-radius, radius, step):
                if max(abs(x + halfStep), abs(z + halfStep)) >= g * prevStep:
                    # Cleared the cutout from the previous level. Tessellate the
                    # square.

                    #   A-----B-----C
                    #   | \   |   / |
                    #   |   \ | /   |
                    #   D-----E-----F
                    #   |   / | \   |
                    #   | /   |   \ |
                    #   G-----H-----I

                    A = Point3(float(x), L, float(z))
                    C = Point3(float(x + step), L, A[-1])
                    G = Point3(A[0], L, float(z + step))
                    I = Point3(C[0], L, G[-1])

                    B = (A + C) * 0.5
                    D = (A + G) * 0.5
                    F = (C + I) * 0.5
                    H = (G + I) * 0.5

                    E = (A + I) * 0.5

                    # Stitch the border into the next level

                    if x == -radius:
                        #   A-----B-----C
                        #   | \   |   / |
                        #   |   \ | /   |
                        #   |     E-----F
                        #   |   / | \   |
                        #   | /   |   \ |
                        #   G-----H-----I
                        builder.append(E)
                        builder.append(A)
                        builder.append(G)
                    else:
                        builder.append(E)
                        builder.append(A)
                        builder.append(D)

                        builder.append(E)
                        builder.append(D)
                        builder.append(G)

                    if z == (radius - step):
                        builder.append(E)
                        builder.append(G)
                        builder.append(I)
                    else:
                        builder.append(E)
                        builder.append(G)
                        builder.append(H)
                        builder.append(E)
                        builder.append(H)
                        builder.append(I)

                    if x == (radius - step):
                        builder.append(E)
                        builder.append(I)
                        builder.append(C)
                    else:
                        builder.append(E)
                        builder.append(I)
                        builder.append(F)
                        builder.append(E)
                        builder.append(F)
                        builder.append(C)

                    if z == -radius:
                        builder.append(E)
                        builder.append(C)
                        builder.append(A)
                    else:
                        builder.append(E)
                        builder.append(C)
                        builder.append(B)
                        builder.append(E)
                        builder.append(B)
                        builder.append(A)
    return builder

#def initMesh():
#    # Cache the mesh between runs because it is expensive to generate
#    if os.exists(specs.filename):
#        add_to_stage()
#    else:
#        buildMesh()
#
#    m_gridIndex = 
#    m_gridVertex =
from mpl_toolkits.mplot3d import axes3d, Axes3D #<-- Note the capitalization! 

points = buildMesh()
points = np.array(points)
print(points)
ax = plt.figure().add_subplot(projection='3d')

#ax = plt.figure().add_subplot(projection='3d')
ax.scatter(points[:,0], points[:,1], points[:,2])
#plt.scatter(points[:,0], points[:,2])
#plt.axes("equal")
plt.show()