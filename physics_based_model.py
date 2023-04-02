import numpy as np
import matplotlib.pyplot as plt
from argparse import ArgumentParser
from constants import Constants
import os
from visualize_mpl import Visualize


class PhysicalEulerModel(Constants):
    def __init__(self) -> None:
        super(PhysicalEulerModel, self).__init__()

    def get_position(self, release, theta, LDS, SHD, AL):
        xs = np.zeros(3, dtype=np.float64)
        xs[0] = AL*np.sin(release)
        xs[1] = LDS + AL*np.cos(release)*np.sin(theta)
        xs[2] = SHD + AL*np.cos(release)*np.cos(theta)
        return xs

    def get_velocity(self,release, theta, speed):
        vs = np.zeros(3, dtype=np.float64)
        vs[0] = speed*np.cos(release)
        vs[1] = -1.0*speed*np.sin(release)*np.sin(theta)
        vs[2] = -1.0*speed*np.sin(release)*np.cos(theta)
        return vs

    def get_drag_and_side_force_coeff(self, speed, new_ball=False, shallow_transit=True):
        mph_to_mps = 0.00044704
        speed = speed/mph_to_mps
        if(new_ball):
            if(shallow_transit):
                if (speed <= 81):
                    Cd, Cs = 0.5, 0.3
                elif (speed >= 100):
                    Cd, Cs = 0.3, 0.0
                else:
                    Cd = 0.5 - (0.2*(speed - 81)/19)
                    Cs = 0.3 - (0.3*(speed - 81)/19)
            else:
                if (speed <= 81):
                    Cd, Cs = 0.5, 0.3
                elif (speed >= 86):
                    Cd, Cs = 0.3, 0.0
                else:
                    Cd = 0.5 - (0.2*(speed - 81)/5)
                    Cs = 0.3 - (0.3*(speed - 81)/5)
        else:
            if(shallow_transit):
                if (speed <= 63):
                    Cd, Cs = 0.5, 0.3
                elif (speed >= 81):
                        Cd, Cs = 0.3, -0.2
                else:
                    Cd = 0.5 - (0.2*(speed - 63)/18)
                    Cs = 0.3 - (0.5*(speed - 63)/18)
            else:
                if (speed <= 63):
                    Cd, Cs = 0.5, 0.3
                elif (speed >= 68):
                    Cd, Cs = 0.3, -0.2
                else:
                    Cd = 0.5 - (0.2*(speed - 63)/5)
                    Cs = 0.3 - (0.5*(speed - 63)/5) 
        return Cd, Cs
            
    def f_traj(self, u, v, w, windspeed, new_ball=False, shallow_transit=True):
        fs = np.zeros(3)
        v_norm = np.sqrt((u - windspeed[0])**2 + (v - windspeed[1])**2 + (w - windspeed[2])**2)
        Cd, Cs = self.get_drag_and_side_force_coeff(v_norm, new_ball, shallow_transit)
        fs[0] = -v_norm*self.C*Cd*(u - windspeed[0])
        fs[1] = -v_norm*(-self.C*Cd*(v - windspeed[1]) + self.C*Cs*(u - windspeed[0]))
        fs[2] = -v_norm*self.C*Cd*(w - windspeed[2]) - self.g
        return fs

    def step_Euler(self, xs, vs, windspeed, dt, new_ball=False, shallow_transit=True):
        vs = vs + dt*self.f_traj(*vs, windspeed, new_ball, shallow_transit)
        xs = xs + dt*vs
        return xs, vs
    
    def get_ball_trajectory(self,speed, release, theta, windspeed, bounce,
                    LDS, SHD, AL, new_ball=False, shallow_transit=True,
                    dt=0.005, history=True):

        speed = speed*self.kmph2mps
        release = release*self.deg2rad
        theta = theta*self.deg2rad
        windspeed = [i*self.kmph2mps for i in windspeed]

        xs = self.get_position(release, theta, LDS, SHD, AL)
        vs = self.get_velocity(release, theta, speed)
        itr = 0
        traj = []
        if history:
            traj.append(xs)
        first_bounce = 1.0
        while xs[0] < self.L and itr <= self.max_iter:
            itr += 1
            xs, vs = self.step_Euler(xs, vs, windspeed, dt, new_ball, shallow_transit)
            if xs[2] <= 0.0 and first_bounce:
                vs[2] *= -bounce
                first_bounce = 0.0
            if history:
                traj.append(xs)

        if history:
            return np.array(traj)
        else:
            return xs
      
      
if __name__ == "__main__":
    parser = ArgumentParser(description="Input form CMD")
    parser.add_argument('-s', '--speed', type=float, default=140.0)
    parser.add_argument('-r', '--release', type=float, default=5.0)
    parser.add_argument('-t', '--theta', type=float, default=10.0)
    parser.add_argument('-b', '--bounce', type=float, default=0.8)
    parser.add_argument('-lds', '--lateral_dist', type=float, default=0.2)
    parser.add_argument('-shd', '--shoulder_ht', type=float, default=1.5)
    parser.add_argument('-al', '--arm_len', type=float, default=0.8)
    parser.add_argument('-wx', '--wind_x', type=float, default=0.0)
    parser.add_argument('-wy', '--wind_y', type=float, default=0.0)
    parser.add_argument('-wz', '--wind_z', type=float, default=0.0)
    parser.add_argument('-nb', '--new_ball', type=bool, default=False)
    parser.add_argument('-sh_tr', '--shallow_transit', type=bool, default=True)
    parser.add_argument('-ep', '--end_point', type=bool, default=False)
    parser.add_argument('-v', '--visualize', type=bool, default=False)
    parser.add_argument('-dt', '--delta_t', type=float, default=0.005)
    parser.add_argument("-o", "--directory", default=None)
    
    args = parser.parse_args()
    
    phyModel = PhysicalEulerModel()
    traj = phyModel.get_ball_trajectory(speed=args.speed, 
                                          release=args.release,
                                          theta=args.theta,
                                          windspeed=[args.wind_x, args.wind_y, args.wind_z],
                                          bounce=args.bounce,
                                          LDS=args.lateral_dist,
                                          SHD=args.shoulder_ht,
                                          AL=args.arm_len,
                                          new_ball=args.new_ball,
                                          shallow_transit=args.shallow_transit,
                                          dt=args.delta_t,
                                          history=not(args.end_point))
    
    vis = Visualize()
    fig2d = vis.plot([traj,traj])
    fig3d = vis.plot([traj], plot3d=True)
    
    if args.visualize:
        vis.show_plot()

    if args.directory is not None:
        dir_path = os.path.join(os.getcwd(),args.directory)
        if os.path.exists(dir_path):
            raise Exception("The current directory already exists. Choose a different directory!")
        else:
            os.mkdir(dir_path)
        np.savez(os.path.join(dir_path,"traj.npz"), traj, args)
        fig2d = vis.plot([traj,traj])
        plt.savefig(os.path.join(dir_path,"2D_Plot.png"))
        fig3d = vis.plot([traj], plot3d=True)
        plt.savefig(os.path.join(dir_path,"3D_Plot.png"))
        