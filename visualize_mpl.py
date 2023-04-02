import numpy as np
import matplotlib.pyplot as plt
from constants import Constants

class Visualize(Constants):
    def __init__(self) -> None:
        super(Visualize, self).__init__()
    
    def _plot_2d_trajectory(self,trajectories):
        lines = ['b-', 'ro']
        fig = plt.figure(figsize=(15, 8), constrained_layout=True)
        ax1 = fig.add_subplot(2, 1, 1)
        ax2 = fig.add_subplot(2, 1, 2)
        for i in range(len(trajectories)):
            traj = trajectories[i]
            line = lines[1] if (i+1) % 2 == 0 else lines[0]
            label = "Input Trajectory" if (i) % 2 == 0 else "Predicted Trajectory"
            xs = []
            ys = []
            zs = []
            for xy in traj:
                xs.append(xy[0])
                ys.append(xy[1])
                zs.append(xy[2])

            ax1.plot([0, self.L], [0, 0], 'k-', linewidth=8)
            ax1.plot([0, self.L], [self.SH, self.SH], 'k--', linewidth=2)
            ax1.plot([self.L, self.L], [0, self.SH], 'k-', linewidth=4)
            ax1.plot(xs, zs, line, linewidth=3, label = label)
            if (i+1) % 2 != 0:
                ax1.plot([self.L], [zs[-1]], 'r*', markersize=20)
            ax1.set_xlabel('Pitch', fontsize = 16)
            ax1.set_ylabel('Height', fontsize = 16)
            ax1.set_xlim((-0.3, self.L + 0.3))
            ax1.set_ylim((-0.1, 2.5 + 0.1))

            ax2.plot([0, self.L], [self.RCD, self.RCD], 'k-', linewidth=8)
            ax2.plot([0, self.L], [-self.RCD, -self.RCD], 'k-', linewidth=8)
            ax2.plot([0, self.L], [self.SW/2, self.SW/2], 'k--', linewidth=2)
            ax2.plot([0, self.L], [-self.SW/2, -self.SW/2], 'k--', linewidth=2)
            ax2.plot([self.L, self.L], [-self.SW/2, self.SW/2], 'k-', linewidth=4)
            ax2.plot(xs, ys, line, linewidth=3)
            if (i+1) % 2 != 0:
                ax2.plot([self.L], [ys[-1]], 'r*', markersize=20)
            ax2.set_xlabel('Pitch', fontsize = 16)
            ax2.set_ylabel('Width', fontsize = 16)
            ax2.set_xlim((-0.3, self.L + 0.3))
            ax2.set_ylim((-0.1 - self.RCD, self.RCD + 0.1))
        # fig.legend(fontsize = 16, frameon =False)
        return fig
    
    def _plot_3d_trajectory(self,traj, orientation = 90):
        xs = []
        ys = []
        zs = []
        for xy in traj:
            xs.append(xy[0])
            ys.append(xy[1])
            zs.append(xy[2])
        fig = plt.figure(figsize=(12,12), constrained_layout=True)
        orientations = [90, 120, 180, 210]
        # orientations = [orientation]
        for i in range(len(orientations)):
            ax = fig.add_subplot(2,2,i+1, projection='3d')
            ax.plot([self.L, self.L], [-self.SW/2, -self.SW/2], [0, self.SH], 'k-', lw=3)
            ax.plot([self.L, self.L], [self.SW/2, self.SW/2], [0, self.SH], 'k-', lw=3)
            ax.plot([self.L, self.L], [0, 0], [0, self.SH], 'k-', lw=3)
            ax.plot([self.L, self.L], [-self.SW/2, self.SW/2], [self.SH, self.SH], 'k-', lw=3)

            ax.plot(xs, ys, zs, 'r-', lw=4)
            ax.plot([self.L], [ys[-1]], [zs[-1]], 'r*', markersize=20)

            xp = np.arange(0, self.L, 0.1)
            yp = np.arange(-self.RCD, self.RCD, 0.1)
            Xp, Yp = np.meshgrid(xp, yp)
            Zp = Xp*0 + Yp*0
            ax.plot_surface(Xp, Yp, Zp, edgecolor='green')
            ax.set_xlim3d(-0.1, 0.1+self.L)
            ax.set_ylim3d((-0.1 - self.RCD, self.RCD + 0.1))
            ax.set_zlim3d((0, 2.5))
            ax.view_init(0, orientations[i])
        return fig
            
    def plot(self,trajectories, plot3d=False):
        if plot3d:
            self._plot_3d_trajectory(trajectories[0])
        else:
            self._plot_2d_trajectory(trajectories)
    
    def show_plot(self):
        plt.show()
    