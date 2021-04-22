import numpy as np
from scipy.optimize import minimize
from matplotlib import pyplot
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from mpl_toolkits.mplot3d import axes3d
import scipy.interpolate
import rospkg
import rospy
import os
import pymesh
from matplotlib.pyplot import imshow
from scipy.interpolate import griddata
import math

interpolator = None
stability_poly = None
origin = [0, 0, 0]


def constraint_superior_zero(x):
    global origin, interpolator, stability_poly

    rot_x = Rx(x[0])
    local_poly = np.dot(stability_poly, rot_x.T)  # rotate points x

    rot_y = Ry(x[1])
    local_poly = np.dot(local_poly, rot_y.T)  # rotate points y

    local_poly = local_poly + origin
    local_poly = local_poly + [0, 0, x[2]]

    v = [(p[2] - interpolator(p[0], p[1])) ** 2 for p in local_poly]
    return min(v)


def Rx(theta):
    return np.array([[1, 0, 0],
                      [0, math.cos(theta), -math.sin(theta)],
                      [0, math.sin(theta), math.cos(theta)]])


def Ry(theta):
    return np.array([[math.cos(theta), 0, math.sin(theta)],
                      [0, 1, 0],
                      [-math.sin(theta), 0, math.cos(theta)]])


def objective(x):
    global origin, interpolator, stability_poly

    R = Rx(x[0])
    local_poly = np.dot(stability_poly, R.T)
    R = Ry(x[1])
    local_poly = np.dot(local_poly, R.T)

    local_poly = local_poly + origin
    local_poly = local_poly + [0, 0, x[2]]

    v = [(p[2] - interpolator(p[0], p[1])) ** 2 for p in local_poly]
    objective_v = sum(v)
    print "objective x:\t", "{:.7f}".format(x[0]), "{:.7f}".format(x[1]), "\t", objective_v, "\t", v
    return objective_v


def get_interpolator():
    global origin

    mesh = pymesh.load_mesh("/home/h3ct0r/catkin_ws_espeleo/src/espeleo_planner/espeleo_planner/test/maps/map_05_cavelike.stl")

    mesh.enable_connectivity()  # enables connectivity on mesh
    mesh.add_attribute("face_centroid")  # adds the face centroids to be accessed
    mesh.add_attribute("face_normal")  # adds the face normals to be accessed
    mesh.add_attribute("vertex_valance")

    centroids = np.concatenate([mesh.vertices, mesh.get_face_attribute("face_centroid")])

    A = np.array(centroids)
    B = np.array(origin)
    R = 2
    filtered_centroids = A[np.linalg.norm(A[:, :3] - B, axis=1) < R]

    x, y, z = zip(*filtered_centroids)
    interp_fn = scipy.interpolate.CloughTocher2DInterpolator(np.array([x, y]).T, z)

    return interp_fn, filtered_centroids


def get_stability_poly():
    global origin

    dx = 0.212  # distancia entre p1 e p2 = distancia entre p2 e p3 ...
    dy = 0.33  # distancia entre p6 e p1 = distancia entre p4 e p3
    dy_m = 0.425  # distancia entre p5 e p2 - rodas mais afastadas
    dz = 0  # altura do centro de massa

    poly = np.array([
        [dx, -(dy / 2), dz],
        [0, -(dy_m / 2), dz],
        [-dx, -(dy / 2), dz],
        [-dx, (dy / 2), dz],
        [0, (dy_m / 2), dz],
        [dx, (dy / 2), dz]
    ])

    #poly = poly + origin

    return poly


def plot(x_final):
    global interpolator, stability_poly

    fig = pyplot.figure()
    ax = pyplot.axes(projection="3d")

    x, y, z = zip(*centroids)
    xline = np.linspace(min(x), max(x), 30)
    yline = np.linspace(min(y), max(y), 30)
    xgrid, ygrid = np.meshgrid(xline, yline)
    z_interp = interpolator(xgrid, ygrid)

    ax.plot_wireframe(xgrid, ygrid, z_interp)
    ax.plot_surface(xgrid, ygrid, z_interp, alpha=0.2)
    ax.scatter3D(x, y, z, c='r')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    ax.scatter(origin[0], origin[1], origin[2], zdir='z', c='black')

    R = Rx(x_final[0])
    local_poly = np.dot(stability_poly, R.T)
    R = Ry(x_final[1])
    local_poly = np.dot(local_poly, R.T)

    local_poly = local_poly + origin
    local_poly = local_poly + [0, 0, x_final[2]]

    # R = Rx(x0[0])
    # local_poly = np.dot(stability_poly, R.T)
    # R = Ry(x0[1])
    # local_poly = np.dot(local_poly, R.T)

    ax.scatter(local_poly[:, 0], local_poly[:, 1], local_poly[:, 2], zdir='z', c='b')

    stability_poly_tuples = list([map(list, local_poly)])
    collection = Poly3DCollection(list(stability_poly_tuples), linewidths=0.5, alpha=0.2, edgecolors="b")
    face_color = [0.5, 0.5, 1]  # alternative: matplotlib.colors.rgb2hex([0.5, 0.5, 1])
    collection.set_facecolor(face_color)
    ax.add_collection3d(collection)

    for p in stability_poly_tuples[0]:
        x = p[0]
        y = p[1]
        z = p[2]

        ax.plot([x, origin[0]], [y, origin[1]], [z, origin[2]], linestyle="--", c='b', linewidth=0.5)  # points to center

    pyplot.show()


origin = [1.5, -2.0, 0]
interpolator, centroids = get_interpolator()
stability_poly = get_stability_poly()

x0 = np.zeros(3)
x0[0] = 0
x0[1] = 0
x0[2] = 0

# show initial objective
print('Initial Objective: ' + str(objective(x0)))

# optimize
b = (-2 * math.pi, 2 * math.pi)
z_bound = (-2, 2)
bnds = (b, b, z_bound)

# constraints
# ineq: it has to be non-negative
# eq: results should be zero

con1 = {'type': 'ineq', 'fun': constraint_superior_zero}  # it has to be non-negative
cons = ([con1])
solution = minimize(objective, x0, method='SLSQP', bounds=bnds, constraints=cons)
x_final = solution.x

# show final objective
print('Final Objective: ' + str(objective(x_final)))

# print solution
print('Solution')
print('x1 = ' + str(x_final[0]))
print('x2 = ' + str(x_final[1]))

# x_final = np.zeros(2)
# x_final[0] = 0.75
# x_final[1] = 0.75

plot(x_final)