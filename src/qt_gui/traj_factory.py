import numpy as np
#
# a collection of ready-made trajectories
#
import pat3.trajectory_1D as p_t1d
import pat3.vehicles.rotorcraft.multirotor_trajectory as p_mt
import pat3.vehicles.rotorcraft.multirotor_trajectory_dev as p_mt_dev

class Traj1(p_mt.Circle):
    name, desc = 'circle1', 'circle r=2 v=4, constant heading and height'
    def __init__(self): p_mt.Circle.__init__(self, [0, 0, 1.5], r=2., v=4., psit=p_t1d.CstOne(0))

class Traj2(p_mt.Circle):
    name, desc = 'circle2', 'circle r=2 v=4, constant heading, sine height'
    def __init__(self): p_mt.Circle.__init__(self, [0, 0, 1.5], r=2., v=2., psit=p_t1d.CstOne(0), zt=p_t1d.SinOne(c=2, a=0.5, om=4))

class Traj3(p_mt.SmoothBackAndForth):
    name, desc = 'smooth_back_and_forth', 'smooth back and forth'
    def __init__(self):
        super().__init__(Y0=[-1, 0, 1.5, 0], Y1=[1, 0, 2.5, 0])
        
class Traj4(p_mt.CircleWithIntro):
    name, desc = 'circle_with_intro', 'circle with intro'
    def __init__(self):
        super().__init__(Y0=[0, 0, 1.5, 0], c=[0, 0, 2.5],
                         r=2., v=3., dt_intro=5., dt_stay=0.5, psit=p_t1d.CstOne(0.))


class Donut0(p_mt.Trajectory):
    name, desc = 'donut', 'quad4d rebooted: donut'
    def __init__(self, c=[0, 0, -2.], r=1., r2=1., v=4., psi=None, duration=80.):
        self.c, self.r, self.r2, self.v = np.asarray(c), r, r2, v # center, radius, velocity
        self.omega1, self.omega2 = 1, 0.1 #self.v/self.r
        self.t0, self.duration = 0, duration

    def reset(self, t0):
        self.t0 = t0

    def get(self, t):
        dt = t-self.t0
        alpha, beta = self.omega1*dt, self.omega2*dt
        rca, rsa = np.abs(self.r)*np.cos(alpha), np.abs(self.r)*np.sin(alpha)
        cb, sb = np.cos(beta), np.sin(beta)
        c1 = self.c + [-self.r2*sb, self.r2*cb, 0]
        A = np.array([[cb, -sb, 0],[sb, cb, 0],[0, 0, 1]])
        B = np.array([0, rsa, rca])
        Yc = np.zeros((5,4))
        Yc[0,:p_mt._z+1] = c1 + A@B
        #cbd, sbd = self.omega2
        c1d = [-self.omega2*self.r2*cb, -self.omega2*self.r2*sb, 0]
        Ad = self.omega2 * np.array([[-sb, -cb, 0],[cb, -sb, 0],[0, 0, 1]])
        Bd = self.omega1 * np.array([0, rca, -rsa])
        Yc[1,:p_mt._z+1] = c1d + Ad@B + A@Bd
        return Yc.T

class Donut1(p_mt.CompositeTraj):
    name, desc = 'donut_with_intro', 'quad4d rebooted: donut with intro'
    def __init__(self):
        Y0 = [0., 0, -1.5, 0.]
        d1 = Donut0(r=0.7, r2=1., duration=61.)
        Y1 = d1.get(0)#[:,0]
        Y2 = d1.get(d1.duration)
        steps = [p_mt.SmoothLine(Y0, Y1, duration=2.),
                 d1,
                 p_mt.SmoothLine(Y2, Y0, duration=2.),
                 p_mt.Cst(Y0, duration=1.)]
        super().__init__(steps)




        

class Traj17(p_mt.Trajectory):
    name, desc = 'sphere0', 'sphere0'
    def __init__(self, c=[0, 0, 3.5], r=2.5, v=2., psi=None):
        self.c, self.r, self.v = np.asarray(c), r, v # center, radius, velocity
        self.omega1, self.omega2 = 1, 0.1 #self.v/self.r
        self.t0, self.duration = 0, 80

        
    def reset(self, t0):
        self.t0 = t0

    def get(self, t):
        dt = t-self.t0
        alpha = self.omega1*(dt)# + self.alpha0
        beta = self.omega2*(dt)# + self.alpha0
        rca, rsa = np.abs(self.r)*np.cos(alpha), np.abs(self.r)*np.sin(alpha) 
        cb, sb = np.cos(beta), np.sin(beta)
        Yc = np.zeros((5,4))
        #Yc[0,:pmt._psi] = self.c[:pmt._psi] + [rca*cb, rsa, rca*sb]
        A = np.array([[cb, 0, -sb],[0, 1, 0],[sb, 0, cb]])
        B = np.array([rca, rsa, 0])
        Yc[0,:p_mt._z+1] = self.c + A@B
        alpha_d, beta_d = self.omega1, self.omega2
        Ad = -beta_d*np.array([[sb, 0, cb],[0, 0, 0],[-cb, 0, sb]])
        Bd =  alpha_d * np.array([-rsa, rca, 0])
        Yc[1,:p_mt._z+1] = Ad@B+A@Bd
        alpha_dd, beta_dd = 0, 0
        Add = -beta_dd*np.array([[sb, 0, cb],[0, 0, 0],[-cb, 0, sb]])-beta_d**2*np.array([[cb, 0, -sb],[0, 0, 0],[sb, 0, cb]])
        Bdd = alpha_dd*np.array([-rsa, rca, 0])-alpha_d**2*np.array([rca, rsa, 0])
        Yc[2,:p_mt._z+1] = Add@B + 2*Ad@Bd + A@Bdd
        
        # pointing center
        #Yc[0,pmt._psi] = np.arctan2(Yc[0,pmt._y], Yc[0,pmt._x])
        om3 = 0.25
        #Yc[0,pmt._psi] = om3*dt
        #Yc[1,pmt._psi] = om3
        Yc[0,p_mt._psi] =         np.sin(om3*dt)
        Yc[1,p_mt._psi] =  om3   *np.cos(om3*dt)
        Yc[2,p_mt._psi] = -om3**2*np.sin(om3*dt)
        Yc[3,p_mt._psi] = -om3**3*np.cos(om3*dt)
        Yc[4,p_mt._psi] =  om3**4*np.sin(om3*dt)
        
        return Yc.T


        
#
# Space indexed examples
#

        
class Traj42(p_mt_dev.SpaceIndexedTraj):
    name, desc = 'ex_si1', 'Space indexed waypoint trajectory example 1'
    def __init__(self, wps=None, dyn_pts=None):
        wps = wps if wps is not None else [[0.2,0, 1],[2.,3., 2], [2.,-3., 3], [-2.,-3., 4], [-2.,3., 3], [-0.2, 0., 2]]
        dyn_pts = dyn_pts if dyn_pts is not None else [[0,0],[1., 0], [4.5,0.1], [7.5,0.7], [10.,0.9], [14., 1.], [15,1.]]
        #dyn_pts = [[0,0],[1., 0], [3.,0.1], [5.,0.7], [7.,0.9], [9., 1.], [10,1.]]
        self.wps = np.array(wps)
        self.wp_traj = p_mt_dev.SpaceWaypoints(self.wps)
        self.dyn_ctl_pts = np.array(dyn_pts)
        self.dyn_segments = [p_t1d.AffOne(self.dyn_ctl_pts[i], self.dyn_ctl_pts[i+1]) for i in range(len(self.dyn_ctl_pts)-1)]
        self.dyn_traj = p_t1d.SmoothedCompositeOne(self.dyn_segments, eps=0.75)
        self.traj = p_mt_dev.SpaceIndexedTraj(self.wp_traj, self.dyn_traj)
        #super().__init__(self.wp_traj, self.dyn_smoothed)
        self.duration = self.traj.duration

    def has_waypoints(self): return True
        
    def set_waypoints(self, waypoints):
        self.wps = waypoints
        self.wp_traj = p_mt_dev.SpaceWaypoints(self.wps)
        self.traj = p_mt_dev.SpaceIndexedTraj(self.wp_traj, self.dyn_traj)

    def get_waypoints(self): return self.wps
        
    def has_dyn_ctl_pts(self): return True
    def set_dynamic(self, dyn_ctl_pts):
        self.dyn_ctl_pts = dyn_ctl_pts
        self.dyn_segments = [p_t1d.AffOne(self.dyn_ctl_pts[i], self.dyn_ctl_pts[i+1]) for i in range(len(self.dyn_ctl_pts)-1)]
        self.dyn_traj = p_t1d.SmoothedCompositeOne(self.dyn_segments, eps=0.75)
        self.traj = p_mt_dev.SpaceIndexedTraj(self.wp_traj, self.dyn_traj)
        self.duration = self.traj.duration
    def get_dyn_ctl_pts(self): return self.dyn_ctl_pts
        
    def get(self, t):
        return self.traj.get(t)

# optimized version of the above for duration with max vel
class Traj43(Traj42):
    name, desc = 'ex_si2', 'Space indexed waypoint trajectory example 2'
    def __init__(self):
        super().__init__()
        self.dt_acc, self.dl_acc, self.dt_cruise = 5.06857143, 0.29494083, 4.73810948
        self.duration = 2*self.dt_acc + self.dt_cruise
        self.dyn_traj = p_t1d.SmoothStopStopCstVel(self.dt_acc, self.dl_acc, self.dt_cruise)
        self.dyn_ctl_pts = np.array([[0, 0], [self.dt_acc, self.dl_acc], [self.dt_acc+self.dt_cruise, 1-self.dl_acc], [self.duration, 1.]])
        self.traj = p_mt_dev.SpaceIndexedTraj(self.wp_traj, self.dyn_traj)

    def set_dynamic(self, dyn_ctl_pts):
        pass # TODO

class Traj44(Traj42):
    name, desc = 'ex_si3', 'Space indexed waypoint slalom'
    def __init__(self):
        wps = np.array([[0, -3, 1.5],
                        [2, -2, 2.5],
                        [0, -1, 1.2],
                        [2,  0, 2.5],
                        [0,  1, 1.5],
                        [2,  2, 2.5],
                        [0,  3, 1.5]])
        dyn_pts = [[0,0],[1., 0], [2.,0.1], [3.,0.2], [5.,0.7], [7.,0.8], [9., 1.], [10,1.]]
        super().__init__(wps, dyn_pts)

    
class TrajFactory:
    trajectories = {}

    @staticmethod
    def register(T):
        TrajFactory.trajectories[T.name] = T
    
TrajFactory.register(Traj1)
TrajFactory.register(Traj2)
TrajFactory.register(Traj3)
TrajFactory.register(Traj4)
TrajFactory.register(Donut0)
TrajFactory.register(Donut1)
TrajFactory.register(Traj17)
TrajFactory.register(Traj42)
TrajFactory.register(Traj43)
TrajFactory.register(Traj44)
