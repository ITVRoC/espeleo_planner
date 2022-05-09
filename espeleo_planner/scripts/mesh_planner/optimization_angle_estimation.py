#!/usr/bin/env python

import numpy as np
from scipy.optimize import minimize
import scipy.interpolate
import os
import open3d as o3d
import math
import datetime
from matplotlib import pyplot
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from mpl_toolkits.mplot3d import Axes3D
from . import mesh_helper


class OptimizationAngleEstimation:

    def __init__(self, mesh_path, show_GUI=False):

        if mesh_path is None or not os.path.isfile(mesh_path):
            raise ValueError("mesh_path is not valid: {}".format(mesh_path))

        if show_GUI:
            pass

        print("mesh_path:", mesh_path)

        self.mesh = o3d.io.read_triangle_mesh(mesh_path)

        self.stability_poly = self.get_stability_poly()
        self.stability_poly_yaw = 0
        self.stability_poly_origin = None
        self.interp_fn = None
        self.filtered_centroids = None

        # optimization bounds
        self.b = (-math.pi, math.pi)
        self.z_bound = (-2, 2)

        self.last_good_objective_x = None

        self.cache_dict = {}

    @staticmethod
    def get_stability_poly():
        dx = 0.212  # distance between p1 and p2 = p2 and p3
        dy = 0.33  # distance between p1 and p6 = p4 and p3
        dy_m = 0.425  # distance between p5 and p2

        poly = np.array([
            [dx, -(dy / 2), 0],
            [0, -(dy_m / 2), 0],
            [-dx, -(dy / 2), 0],
            [-dx, (dy / 2), 0],
            [0, (dy_m / 2), 0],
            [dx, (dy / 2), 0]
        ])

        return poly

    def constraint_superior_zero(self, x):
        if x.tobytes() not in self.cache_dict:
            R = mesh_helper.Rxyz(x[0], x[1], self.stability_poly_yaw)
            local_poly = np.dot(self.stability_poly, R.T)

            local_poly = local_poly + self.stability_poly_origin
            local_poly = local_poly + [0, 0, x[2]]

            v = [(p[2] - self.interp_fn(p[0], p[1])) for p in local_poly]

            self.cache_dict[x.tobytes()] = {
                'sum': sum(v),
                'min': min(v)
            }

        return self.cache_dict[x.tobytes()]['min']

        # R = mesh_helper.Rxyz(x[0], x[1], self.stability_poly_yaw)
        # local_poly = np.dot(self.stability_poly, R.T)
        #
        # local_poly = local_poly + self.stability_poly_origin
        # local_poly = local_poly + [0, 0, x[2]]
        #
        # v = [(p[2] - self.interp_fn(p[0], p[1])) for p in local_poly]
        # return min(v)

    def objective(self, x):
        if x.tobytes() not in self.cache_dict:
            R = mesh_helper.Rxyz(x[0], x[1], self.stability_poly_yaw)
            local_poly = np.dot(self.stability_poly, R.T)

            local_poly = local_poly + self.stability_poly_origin
            local_poly = local_poly + [0, 0, x[2]]

            #start_time = datetime.datetime.now()
            v = [(p[2] - self.interp_fn(p[0], p[1])) for p in local_poly]

            self.cache_dict[x.tobytes()] = {
                'sum': sum(v),
                'min': min(v)
            }

        objective_v = self.cache_dict[x.tobytes()]['sum']

        #delta_intepolator_millis = int((datetime.datetime.now() - start_time).total_seconds() * 1000)
        #print "objective interpol time millis:{}".format(delta_intepolator_millis)

        #print "objective x:\t", "{:.7f}".format(x[0]), "{:.7f}".format(x[1]), "{:.7f}".format(
        #    x[2]), "\t", objective_v, "\t", v, "\t x:", x

        if not math.isnan(objective_v): #and self.constraint_superior_zero(x):
            self.last_good_objective_x = x
        else:
            #if math.isnan(objective_v):
            raise ValueError("The optimizator used nan values for x")

        return objective_v

    def generate_interpolator(self, start_pos):
        mesh_centroids = np.asarray(self.mesh.vertices)

        # filter points by a radius
        A = np.array(mesh_centroids)
        B = np.array(start_pos)
        R = 1.8
        filtered_centroids = A[np.linalg.norm(A[:, :3] - B, axis=1) < R]

        x, y, z = list(zip(*filtered_centroids))
        interp_fn = scipy.interpolate.CloughTocher2DInterpolator(np.array([x, y]).T, z)

        self.interp_fn = interp_fn
        self.filtered_centroids = filtered_centroids

    def estimate_pose(self, start_pos, z_distance=2, stability_poly_yaw=0):
        start_time = datetime.datetime.now()

        #print("start_pos:", start_pos)

        self.stability_poly_origin = [start_pos[0], start_pos[1], start_pos[2] + z_distance]
        self.stability_poly_yaw = stability_poly_yaw


        self.generate_interpolator(start_pos)
        end_time = datetime.datetime.now()
        delta_intepolator_millis = int((end_time - start_time).total_seconds() * 1000)

        # optimize starting from x0
        x0 = np.zeros(3)

        # constraints
        # ineq: it has to be non-negative
        # eq: results should be zero
        con1 = {'type': 'ineq', 'fun': self.constraint_superior_zero}  # it has to be non-negative
        cons = ([con1])

        try:
            solution = minimize(self.objective, x0,
                                method='SLSQP',
                                bounds=(self.b, self.b, self.z_bound),
                                constraints=cons,
                                options={'maxiter': 100, "ftol":1e-02})
            x_final = solution.x
            if math.isnan(x_final[0]) or math.isnan(x_final[1]) or math.isnan(x_final[2]):
                x_final = self.last_good_objective_x

        except ValueError as ve:
            #print("Value error catched:", ve)
            x_final = self.last_good_objective_x

        self.cache_dict = {}

        end_time = datetime.datetime.now()
        delta = end_time - start_time
        delta_millis = int(delta.total_seconds() * 1000)  # milliseconds
        #print "total time millis:{} (interpolator: {})".format(delta_millis, delta_intepolator_millis)

        # show final objective
        #print('Final Objective: ' + str(self.objective(x_final)))

        # print solution
        # print('Solution')
        # print('x1 = ' + str(x_final[0]))
        # print('x2 = ' + str(x_final[1]))
        # print('x3 = ' + str(x_final[2]))

        #self.plot(x_final)

        R = mesh_helper.Rxyz(x_final[0], x_final[1], self.stability_poly_yaw)

        zf = np.array([0, 0, 1])
        zf_l = np.dot(R, zf)

        return [zf_l[0], zf_l[1], zf_l[2]]

    def plot(self, x_final, zoom=10):
        fig = pyplot.figure()  # figsize=pyplot.figaspect(0.5)*1.1
        # ax = fig.axes(projection="3d")
        ax = Axes3D(fig)
        # ax.set_aspect('equal')

        x, y, z = list(zip(*self.filtered_centroids))
        xline = np.linspace(min(x), max(x), 30)
        yline = np.linspace(min(y), max(y), 30)
        xgrid, ygrid = np.meshgrid(xline, yline)
        z_interp = self.interp_fn(xgrid, ygrid)

        ax.set_xlim3d(min(x), max(x))
        ax.set_ylim3d(min(y), max(y))
        ax.set_zlim3d(min(z), max(max(z), self.stability_poly_origin[2]))

        ax.plot_wireframe(xgrid, ygrid, z_interp, color="purple", linewidths=0.5)
        ax.plot_surface(xgrid, ygrid, z_interp, alpha=0.2, color="orchid")
        ax.scatter3D(x, y, z, c='r')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        R = mesh_helper.Rxyz(x_final[0], x_final[1], self.stability_poly_yaw)  # , order="YZX"
        local_poly = np.dot(self.stability_poly, R.T)

        local_poly = local_poly + self.stability_poly_origin
        local_poly = local_poly + [0, 0, x_final[2]]

        ax.scatter(local_poly[:, 0], local_poly[:, 1], local_poly[:, 2], zdir='z', c='b')

        stability_poly_tuples = list([list(map(list, local_poly))])
        collection = Poly3DCollection(list(stability_poly_tuples), linewidths=0.5, alpha=0.7, edgecolors='blue')
        face_color = [0.5, 0.5, 1]  # alternative: matplotlib.colors.rgb2hex([0.5, 0.5, 1])
        collection.set_facecolor(face_color)
        ax.add_collection3d(collection)

        cz = np.mean(local_poly[:, 2])  # mean of z values

        scatter_z_interpolated = []
        for p in stability_poly_tuples[0]:
            x = p[0]
            y = p[1]
            z = p[2]

            ax.plot([x, x], [y, y], [z, self.interp_fn(x, y)], linestyle="--", c='b', linewidth=0.4)  # points to center
            scatter_z_interpolated.append(self.interp_fn(x, y))

        ax.scatter(local_poly[:, 0], local_poly[:, 1], scatter_z_interpolated, zdir='z', c='b', s=0.5)

        xf = np.array([1, 0, 0])
        yf = np.array([0, 1, 0])
        zf = np.array([0, 0, 1])
        xf_l = np.dot(R, xf)
        yf_l = np.dot(R, yf)
        zf_l = np.dot(R, zf)

        ax.quiver(self.stability_poly_origin[0], self.stability_poly_origin[1], cz, zf_l[0], zf_l[1], zf_l[2],
                  length=0.2,
                  pivot='tail', linestyle="-", color='blue')  # z

        # Plot robot axes:
        ax.quiver(self.stability_poly_origin[0], self.stability_poly_origin[1], self.stability_poly_origin[2], xf_l[0],
                  xf_l[1], xf_l[2], length=0.3, pivot='tail',
                  linestyle="--", color='red')  # x
        ax.quiver(self.stability_poly_origin[0], self.stability_poly_origin[1], self.stability_poly_origin[2], yf_l[0],
                  yf_l[1], yf_l[2], length=0.3, pivot='tail',
                  linestyle="--", color='green')  # y
        ax.quiver(self.stability_poly_origin[0], self.stability_poly_origin[1], self.stability_poly_origin[2], zf_l[0],
                  zf_l[1], zf_l[2], length=0.3, pivot='tail',
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

        pyplot.show()
        # fig.savefig('/tmp/fig_3d/plot_{}.png'.format(img_seq), dpi=fig.dpi)

        pyplot.close()