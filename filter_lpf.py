import math

class Filter_LPF:
    def __init__(self):
        self.dt = 0.0
        self.fc = 9999999999999.0
        self.tau = 1.0/(2.0*math.pi*self.fc)
        self.alpha = self.dt / (self.dt + self.tau)
        self.y_pre = 0.0
        self.start = True
    
    def set_param(self, dt_, fc_):
        self.dt = dt_
        self.fc = fc_
        self.tau = 1.0/(2.0*math.pi*self.fc)
        self.alpha = self.dt / (self.dt + self.tau)
    
    def calculate(self, x):
        y = 0.0
        if(self.start==True):
            y = x
            self.y_pre = y
            self.start = False
        else:
            y = self.alpha*x + (1.0 - self.alpha)*self.y_pre
            self.y_pre = y
        return y
    
    def reset(self):
        self.y_pre = 0.0
        self.start = True

if __name__ == "__main__":
    test = Filter_LPF()