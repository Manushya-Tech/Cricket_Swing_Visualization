from math import pi
class Constants:
    def __init__(self) -> None:
        self.g = 9.81  # m/s^2
        self.m = 0.16  # kg
        self.r = 0.036  # m
        self.pi = pi
        self.A = self.pi*self.r*self.r  # m^2
        self.rho = 1.225  # kg/m^3
        self.C = 0.5*self.rho*self.A/self.m  # 1/m
        self.L = 20.12 - 1.22  # m; pitch length - crease length
        self.RCD = 1.83  # m, Return crease distance from stumps
        self.SH = 0.71  # m; stumps height
        self.SW = 0.2286  # m; stumps width
        self.MR = 2.5  # m; maxm vertical release
        self.deg2rad = self.pi/180.0
        self.kmph2mps = 0.277778
        self.max_iter = 2000
        