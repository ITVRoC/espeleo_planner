import numpy as np
from scipy.optimize import minimize
from matplotlib import pyplot
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from mpl_toolkits.mplot3d import Axes3D
import scipy.interpolate
import rospkg
import rospy
import os
import pymesh
from matplotlib.pyplot import imshow
from scipy.interpolate import griddata
import math
import pyquaternion
import datetime

interpolator = None
stability_poly = None
origin = [0, 0, 0]
stability_poly_origin = [0, 0, 0]
stability_poly_yaw = 0.0
dz = 0.135  # altura do centro de massa
img_seq = 0
fitness_v = []


def constraint_superior_zero(x):
    global stability_poly_origin, stability_poly_yaw, interpolator, stability_poly

    R = Rxyz(x[0], x[1], stability_poly_yaw)
    local_poly = np.dot(stability_poly, R.T)

    local_poly = local_poly + stability_poly_origin
    local_poly = local_poly + [0, 0, x[2]]

    v = [(p[2] - interpolator(p[0], p[1])) for p in local_poly]
    return min(v)


def Rx(theta):
    return np.array([[1, 0, 0],
                      [0, math.cos(theta), -math.sin(theta)],
                      [0, math.sin(theta), math.cos(theta)]])


def Ry(theta):
    return np.array([[math.cos(theta), 0, math.sin(theta)],
                      [0, 1, 0],
                      [-math.sin(theta), 0, math.cos(theta)]])


def Rz(theta):
    return np.array([[math.cos(theta), -math.sin(theta), 0],
                      [math.sin(theta), math.cos(theta), 0],
                      [0, 0, 1]])


def Rxyz(x, y, z, order="ZXY"):
    """
    # Convert Euler Angles passed in a vector of Radians
    # into a rotation matrix.  The individual Euler Angles are
    # processed in the order requested.
    # https://stackoverflow.com/questions/1568568/how-to-convert-euler-angles-to-directional-vector
    :param x:
    :param y:
    :param z:
    :param order:
    :return:
    """

    Sx = math.sin(x)
    Sy = math.sin(y)
    Sz = math.sin(z)
    Cx = math.cos(x)
    Cy = math.cos(y)
    Cz = math.cos(z)

    if order == "XYZ":
        return np.array([[Cy*Cz, -Cy*Sz, Sy],
                         [Cz*Sx*Sy+Cx*Sz, Cx*Cz-Sx*Sy*Sz, -Cy*Sx],
                         [-Cx*Cz*Sy+Sx*Sz, Cz*Sx+Cx*Sy*Sz, Cx*Cy]])

    elif order == "YZX":
        return np.array([[Cy*Cz, Sx*Sy-Cx*Cy*Sz, Cx*Sy+Cy*Sx*Sz],
                        [Sz, Cx*Cz, -Cz*Sx],
                        [-Cz*Sy, Cy*Sx+Cx*Sy*Sz, Cx*Cy-Sx*Sy*Sz]])

    elif order == "ZXY":
        return np.array([[Cy*Cz-Sx*Sy*Sz, -Cx*Sz, Cz*Sy+Cy*Sx*Sz],
                        [Cz*Sx*Sy+Cy*Sz, Cx*Cz, -Cy*Cz*Sx+Sy*Sz],
                        [-Cx*Sy, Sx, Cx*Cy]])

    elif order == "ZYX":
        return np.array([[Cy*Cz, Cz*Sx*Sy-Cx*Sz, Cx*Cz*Sy+Sx*Sz],
                        [Cy*Sz, Cx*Cz+Sx*Sy*Sz, -Cz*Sx+Cx*Sy*Sz],
                        [-Sy, Cy*Sx, Cx*Cy]])

    elif order == "YXZ":
        return np.array([[Cy*Cz+Sx*Sy*Sz, Cz*Sx*Sy-Cy*Sz, Cx*Sy],
                        [Cx*Sz, Cx*Cz, -Sx],
                        [-Cz*Sy+Cy*Sx*Sz, Cy*Cz*Sx+Sy*Sz, Cx*Cy]])

    elif order == "YXZ":
        return np.array([[Cy*Cz, -Sz, Cz*Sy],
                         [Sx*Sy+Cx*Cy*Sz, Cx*Cz, -Cy*Sx+Cx*Sy*Sz],
                         [-Cx*Sy+Cy*Sx*Sz, Cz*Sx, Cx*Cy+Sx*Sy*Sz]])
    else:
        raise ValueError("Order '{}' does not match any known order".format(order))


def objective(x):
    global stability_poly_origin, stability_poly_yaw, interpolator, stability_poly, fitness_v

    R = Rxyz(x[0], x[1], stability_poly_yaw)
    local_poly = np.dot(stability_poly, R.T)

    local_poly = local_poly + stability_poly_origin
    local_poly = local_poly + [0, 0, x[2]]

    v = [(p[2] - interpolator(p[0], p[1])) for p in local_poly]
    objective_v = sum(v)
    print "objective x:\t", "{:.7f}".format(x[0]), "{:.7f}".format(x[1]), "\t", objective_v, "\t", v

    #plot(x)

    fitness_v.append(objective_v)
    return objective_v


def get_interpolator(origin):
    #mesh = pymesh.load_mesh("/home/h3ct0r/catkin_ws_espeleo/src/espeleo_planner/espeleo_planner/test/maps/map_05_cavelike.stl")
    mesh = pymesh.load_mesh("/home/h3ct0r/catkin_ws_espeleo/src/espeleo_planner/espeleo_planner/test/maps/map_01_frontiers.stl")

    mesh.enable_connectivity()  # enables connectivity on mesh
    mesh.add_attribute("face_centroid")  # adds the face centroids to be accessed
    mesh.add_attribute("face_normal")  # adds the face normals to be accessed
    mesh.add_attribute("vertex_valance")

    centroids = np.concatenate([mesh.vertices, mesh.get_face_attribute("face_centroid")])

    # filter points by a radius
    A = np.array(centroids)
    B = np.array(origin)
    R = 1.8
    filtered_centroids = A[np.linalg.norm(A[:, :3] - B, axis=1) < R]

    x, y, z = zip(*filtered_centroids)
    interp_fn = scipy.interpolate.CloughTocher2DInterpolator(np.array([x, y]).T, z)

    return interp_fn, filtered_centroids


def get_stability_poly(stability_poly_yaw):
    dx = 0.212  # distancia entre p1 e p2 = distancia entre p2 e p3 ...
    dy = 0.33  # distancia entre p6 e p1 = distancia entre p4 e p3
    dy_m = 0.425  # distancia entre p5 e p2 - rodas mais afastadas

    poly = np.array([
        [dx, -(dy / 2), 0],
        [0, -(dy_m / 2), 0],
        [-dx, -(dy / 2), 0],
        [-dx, (dy / 2), 0],
        [0, (dy_m / 2), 0],
        [dx, (dy / 2), 0]
    ])

    #poly = np.dot(poly, Rz(stability_poly_yaw).T)
    return poly


def plot_environment(show_surface=False):
    global interpolator, stability_poly, stability_poly_yaw, gz, img_seq, stability_poly_origin, origin

    fig = pyplot.figure()  # figsize=pyplot.figaspect(0.5)*1.1
    # ax = fig.axes(projection="3d")
    ax = Axes3D(fig)
    # ax.set_aspect('equal')

    x, y, z = zip(*centroids)
    xline = np.linspace(min(x), max(x), 30)
    yline = np.linspace(min(y), max(y), 30)
    xgrid, ygrid = np.meshgrid(xline, yline)
    z_interp = interpolator(xgrid, ygrid)

    ax.set_xlim3d(min(x), max(x))
    ax.set_ylim3d(min(y), max(y))
    ax.set_zlim3d(min(z), max(max(z), stability_poly_origin[2]))

    if show_surface:
        ax.plot_wireframe(xgrid, ygrid, z_interp, color="purple", linewidths=0.5)
        ax.plot_surface(xgrid, ygrid, z_interp, alpha=0.2, color="orchid")

    ax.scatter3D(x, y, z, c='r')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    pyplot.show()
    img_seq += 1
    pass


def plot(x_final, zoom=10):
    global interpolator, stability_poly, stability_poly_yaw, gz, img_seq, stability_poly_origin, origin

    fig = pyplot.figure() # figsize=pyplot.figaspect(0.5)*1.1
    #ax = fig.axes(projection="3d")
    ax = Axes3D(fig)
    #ax.set_aspect('equal')

    x, y, z = zip(*centroids)
    xline = np.linspace(min(x), max(x), 30)
    yline = np.linspace(min(y), max(y), 30)
    xgrid, ygrid = np.meshgrid(xline, yline)
    z_interp = interpolator(xgrid, ygrid)

    ax.set_xlim3d(min(x), max(x))
    ax.set_ylim3d(min(y), max(y))
    ax.set_zlim3d(min(z), max(max(z), stability_poly_origin[2]))

    # Create cubic bounding box to simulate equal aspect ratio
    # max_range = np.array([max(x) - min(x), max(y) - min(y), max(z) - min(z)]).max()
    # Xb = 0.5 * max_range * np.mgrid[-1:2:2, -1:2:2, -1:2:2][0].flatten() + 0.5 * (max(x) - min(x))
    # Yb = 0.5 * max_range * np.mgrid[-1:2:2, -1:2:2, -1:2:2][1].flatten() + 0.5 * (max(y) - min(y))
    # Zb = 0.5 * max_range * np.mgrid[-1:2:2, -1:2:2, -1:2:2][2].flatten() + 0.5 * (max(z) - min(z))
    # # Comment or uncomment following both lines to test the fake bounding box:
    # for xb, yb, zb in zip(Xb, Yb, Zb):
    #     ax.plot([xb], [yb], [zb], 'w')

    # ax.set_box_aspect((np.ptp(x), np.ptp(y), np.ptp(z))) matplotlib 3.3.0

    ax.plot_wireframe(xgrid, ygrid, z_interp, color="purple", linewidths=0.5)
    ax.plot_surface(xgrid, ygrid, z_interp, alpha=0.2, color="orchid")
    ax.scatter3D(x, y, z, c='r')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    #ax.scatter(stability_poly_origin[0], stability_poly_origin[1], stability_poly_origin[2], zdir='z', c='black')

    R = Rxyz(x_final[0], x_final[1], stability_poly_yaw)  # , order="YZX"
    local_poly = np.dot(stability_poly, R.T)

    local_poly = local_poly + stability_poly_origin
    local_poly = local_poly + [0, 0, x_final[2]]

    ax.scatter(local_poly[:, 0], local_poly[:, 1], local_poly[:, 2], zdir='z', c='b')

    stability_poly_tuples = list([map(list, local_poly)])
    collection = Poly3DCollection(list(stability_poly_tuples), linewidths=0.5, alpha=0.7, edgecolors='blue')
    face_color = [0.5, 0.5, 1]  # alternative: matplotlib.colors.rgb2hex([0.5, 0.5, 1])
    collection.set_facecolor(face_color)
    ax.add_collection3d(collection)

    cz = np.mean(local_poly[:,2])  # mean of z values

    scatter_z_interpolated = []
    for p in stability_poly_tuples[0]:
        x = p[0]
        y = p[1]
        z = p[2]

        ax.plot([x, x], [y, y], [z, interpolator(x, y)], linestyle="--", c='b', linewidth=0.4)  # points to center
        scatter_z_interpolated.append(interpolator(x, y))

    ax.scatter(local_poly[:, 0], local_poly[:, 1], scatter_z_interpolated, zdir='z', c='b', s=0.5)

    # ang_final = get_min_angle_nondynamic(local_poly)
    # min_ang_final = min(ang_final)
    # print "min_ang_final:", min_ang_final, ang_final

    # center = np.array([origin[0], origin[1], cz + 0.2])
    xf = np.array([1, 0, 0])
    yf = np.array([0, 1, 0])
    zf = np.array([0, 0, 1])
    xf_l = np.dot(R, xf)
    yf_l = np.dot(R, yf)
    zf_l = np.dot(R, zf)

    # TESTING NORMAL ESTIMATION WITH QUATERNION
    # def normalVector(obj):
    #     """ Takes a set of points, assumed to be flat, and returns a normal vector with unit length.
    #     """
    #     n = np.cross(np.array(obj[1])-np.array(obj[0]), np.array(obj[2])-np.array(obj[0]))
    #     return n/np.sqrt(np.dot(n,n))
    #
    # print "normal plane:", normalVector(local_poly)
    #
    # print "vector quiver:", zf_l[0], zf_l[1], zf_l[2]
    #
    # q1 = pyquaternion.Quaternion(matrix=R)
    # print "vector quat:", q1.vector
    #
    # print "R:", R
    # ax.quiver(stability_poly_origin[0], stability_poly_origin[1], cz, q1.vector[0], q1.vector[1], q1.vector[2], length=0.2,
    #           pivot='tail', linestyle="-", color='black')  # z from quaternion

    ax.quiver(stability_poly_origin[0], stability_poly_origin[1], cz, zf_l[0], zf_l[1], zf_l[2], length=0.2,
              pivot='tail', linestyle="-", color='blue')  # z

    # Plot robot axes:
    ax.quiver(stability_poly_origin[0], stability_poly_origin[1], stability_poly_origin[2], xf_l[0], xf_l[1], xf_l[2], length=0.3, pivot='tail',
                   linestyle="--", color='red')  # x
    ax.quiver(stability_poly_origin[0], stability_poly_origin[1], stability_poly_origin[2], yf_l[0], yf_l[1], yf_l[2], length=0.3, pivot='tail',
                   linestyle="--", color='green')  # y
    ax.quiver(stability_poly_origin[0], stability_poly_origin[1], stability_poly_origin[2], zf_l[0], zf_l[1], zf_l[2], length=0.3, pivot='tail',
                   linestyle="--", color='blue')  # z

    ax.dist = zoom

    def axisEqual3D(ax):
        extents = np.array([getattr(ax, 'get_{}lim'.format(dim))() for dim in 'xyz'])
        sz = extents[:, 1] - extents[:, 0]
        centers = np.mean(extents, axis=1)
        maxsize = max(abs(sz))
        r = maxsize / 2
        for ctr, dim in zip(centers, 'xyz'):
            getattr(ax, 'set_{}lim'.format(dim))(ctr - r, ctr + r)

    axisEqual3D(ax)

    # scaling = np.array([getattr(ax, 'get_{}lim'.format(dim))() for dim in 'xyz']);
    # ax.auto_scale_xyz(*[[np.min(scaling), np.max(scaling)]] * 3)

    pyplot.show()
    # fig.savefig('/tmp/fig_3d/plot_{}.png'.format(img_seq), dpi=fig.dpi)

    pyplot.close()
    img_seq += 1


def plot_fitness():
    global fitness_v

    timesteps = np.linspace(1, len(fitness_v), len(fitness_v))
    fig = pyplot.figure()
    ax = pyplot.axes()

    ax.plot(timesteps, fitness_v, label="Objective value")
    ax.set_xlim([0, max(timesteps) + 0.1])
    ax.set_ylim([0, max(fitness_v) + max(fitness_v) * 0.1])

    ax.grid()
    ax.set_axisbelow(True)
    ax.set_ylabel('Objective', fontsize=12)
    ax.set_xlabel('Timesteps', fontsize=12)
    ax.set_title('Pose optimization convergence', fontweight='bold', size=14)
    ax.legend(loc='upper right', fontsize=12)

    pyplot.show()
    pyplot.close()


def get_min_angle_nondynamic(local_poly):

    m = 6

    l = np.zeros((m, 3))  # variavel para receber os vetores normais aos vetores ai
    Y = np.zeros(m)  # variavel para receber os angulos de tombamento
    sigma = np.zeros(m)  # variavel para receber o sinal do angulo de tombamento
    a = np.zeros((m, 3))
    identidade = np.identity(3)

    f_g = [0, 0, -1]

    p = local_poly
    for i in range(len(a) - 1):
        a[i] = p[i + 1] - p[i]
    a[m - 1] = p[0] - p[m - 1]

    # print ("a nao normalizado: \n%s"%a)
    for i in range(len(a)):
        a[i] = a[i] / np.linalg.norm(a[i])
    # print ("a normalizado: \n%s"%a)

    for i in range(len(l) - 1):
        l[i] = np.dot((identidade - np.outer(a[i], np.transpose(a[i]))), p[i + 1])

    l[m - 1] = np.dot((identidade - np.outer(a[m - 1], np.transpose(a[m - 1]))), p[0])

    for i in range(len(sigma)):
        calc = np.dot(np.cross(l[i] / np.linalg.norm(l[i]), f_g / np.linalg.norm(f_g)), a[i])
        if calc < 0:
            sigma[i] = 1
        else:
            sigma[i] = -1

    for i in range(len(Y)):
        Y[i] = sigma[i] * np.arccos(np.dot(f_g / np.linalg.norm(f_g), l[i] / np.linalg.norm(l[i])))

    ang_final = np.rad2deg(Y)

    return ang_final


start_time = datetime.datetime.now()
#origin = [1.5, 2.0, 0]
origin = (7.76, 1.16, -0.05)
interpolator, centroids = get_interpolator(origin)

stability_poly_origin = [origin[0], origin[1], interpolator(origin[0], [1])[0] + 2]
stability_poly_yaw = 1.75
print "stability_poly_origin:", stability_poly_origin
stability_poly = get_stability_poly(stability_poly_yaw)

x0 = np.zeros(3)
x0[0] = 0
x0[1] = 0
x0[2] = 0

#plot(x0)

# show initial objective
print('Initial Objective: ' + str(objective(x0)))

# optimize
b = (-math.pi, math.pi)
z_bound = (-2, 2)
bnds = (b, b, z_bound)

# constraints
# ineq: it has to be non-negative
# eq: results should be zero

con1 = {'type': 'ineq', 'fun': constraint_superior_zero}  # it has to be non-negative
cons = ([con1])
solution = minimize(objective, x0, method='SLSQP', bounds=bnds, constraints=cons)
x_final = solution.x

end_time = datetime.datetime.now()
delta = end_time - start_time
delta_millis = int(delta.total_seconds() * 1000)  # milliseconds
print "total time millis:{}".format(delta_millis)

# show final objective
print('Final Objective: ' + str(objective(x_final)))

# print solution
print('Solution')
print('x1 = ' + str(x_final[0]))
print('x2 = ' + str(x_final[1]))

# plot_environment(show_surface=False)
plot_environment(show_surface=True)

plot(x_final)
# for i in xrange(18, 3, -1):
#     plot(x_final, zoom=i * 1/2.0)

#plot_fitness()
