import numpy as np
import matplotlib.pyplot as plt

#colors = ["white", (0.011, 0.317, 0.988), (0.937, 0.023, 0.317), (0.552, 0.682, 0.968), (0.968, 0.552, 0.686)]
#labels = ["Normal angle only", "Optimization", "Pybullet", "Optimization (selective)", "Pybullet (selective)"]

colors = ["white", (0.011, 0.317, 0.988), (0.937, 0.023, 0.317)]
labels = ["Normal angle only", "Optimization", "Pybullet"]

# colors = [(0.011, 0.317, 0.988), (0.937, 0.023, 0.317), (0.552, 0.682, 0.968), (0.968, 0.552, 0.686)]
# labels = ["Optimization", "Pybullet", "Optimization (selective)", "Pybullet (selective)"]

f = lambda m,c: plt.plot([],[],marker=m, color=c, ls="none")[0]
handles = [f("s", colors[i]) for i in range(len(colors))]
legend = plt.legend(handles, labels, loc=3, framealpha=1, frameon=True)

def export_legend(legend, filename="legend.png", expand=[-5,-5, 5, 5]):
    fig  = legend.figure
    fig.canvas.draw()
    bbox  = legend.get_window_extent()
    bbox = bbox.from_extents(*(bbox.extents + np.array(expand)))
    bbox = bbox.transformed(fig.dpi_scale_trans.inverted())
    fig.savefig(filename, dpi="figure", bbox_inches=bbox)

export_legend(legend, "legend_w_o_p.pdf")
plt.show()