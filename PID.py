from .filter_lpf import *

class PID:
    def __init__(self):
        self.dt = 1.0
        self.Kp = 0.0
        self.Ki = 0.0
        self.Kd = 0.0
        self.fc = 9999999999999.9
        self.u_max = 0.0
        self.e = 0.0
        self.e_pre = 0.0
        self.I = 0.0
        self.D = 0.0
        self.u = 0.0
        self.start = True
        self.lpf = Filter_LPF()
        self.lpf.set_param(self.dt, self.fc)
    
    def init(self, dt_, Kp_, Ki_, Kd_, fc_, u_max_):
        self.dt = dt_
        self.Kp = Kp_
        self.Ki = Ki_
        self.Kd = Kd_
        self.fc = fc_
        self.u_max = u_max_
        self.e = 0.0
        self.e_pre = 0.0
        self.I = 0.0
        self.D = 0.0
        self.u = 0.0
        self.start = True
        self.lpf.set_param(self.dt, self.fc)
    
    def set_param(self, dt_, Kp_, Ki_, Kd_, fc_, u_max_):
        self.dt = dt_
        self.Kp = Kp_
        self.Ki = Ki_
        self.Kd = Kd_
        self.fc = fc_
        self.u_max = u_max_
        self.lpf.set_param(self.dt, self.fc)

    def reset(self):
        self.dt = 1.0
        self.Kp = 0.0
        self.Ki = 0.0
        self.Kd = 0.0
        self.fc = 9999999999999.9
        self.u_max = 0.0
        self.e = 0.0
        self.e_pre = 0.0
        self.I = 0.0
        self.D = 0.0
        self.u = 0.0
        self.start = True
        self.lpf.set_param(self.dt, self.fc)

    def calculate(self, x0, x):
        self.e = x0 - x

        self.I = self.I + self.e*self.dt

        self.I = self.integral_windup_1(self.I, self.u_max)
        # self.I = self.I - self.integral_windup_2(self.e, -0.1, 0.1) * self.e * self.dt

        if(self.start==True):
            self.D = 0.0
            self.start = False
        else:
            self.D = (self.e - self.e_pre) / self.dt
            self.e_pre = self.e
        self.D = self.lpf.calculate(self.D)

        self.u = self.Kp*self.e + self.Ki*self.I + self.Kd*self.D
        self.u = self.saturate(self.u, -self.u_max, self.u_max)
        return self.u
    
    def merge(self, e_, I_):
        self.e_pre = e_
        self.I = I_
    
    def saturate(self, x, x_min, x_max):
        if(x<x_min):
            return x_min
        elif(x>x_max):
            return x_max
        else:
            return x
    
    # saturate the integral term
    def integral_windup_1(self, I_, I_max_):
        return self.saturate(I_, -I_max_, I_max_)
    
    # stop the integration
    def integral_windup_2(self, e_, e_toler_min, e_toler_max):
        if((e_ >= e_toler_min) and (e_ <= e_toler_max)):
            return 1
        else:
            return 0
    
    def deadband_1(self, x, d):
        if(abs(x) >= 0.5*d):
            return x
        else:
            return 0.0
    
    def deadband_inv_1(self, x, d):
        if(abs(x) >= 0.5*d):
            return x
        elif(abs(x)<0.01*d): # x < (d*p)/100
            return 50.0*x # y = (50/p)x
    
    def sign(self, x):
        if(x==0):
            return 0.0
        elif(x<0):
            return -1.0
        else:
            return 1.0

if __name__ == "__main__":
    test = PID()