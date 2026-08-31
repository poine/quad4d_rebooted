import numpy as np
#
# a collection of ready-made trajectories
#
import pat3.trajectory_1D as p_t1d
import pat3.vehicles.rotorcraft.multirotor_trajectory as p_mt
import pat3.vehicles.rotorcraft.multirotor_trajectory_dev as p_mt_dev

class ClosedLoop(p_mt.CompositeTraj):
    """Wrap a trajectory whose start != end so it loops without a jump.
    Appends a smooth min-snap segment from the end state back to the
    start state; SmoothLine matches the full flat output (position AND
    derivatives) at both ends, so the join is velocity/accel-continuous."""
    def __init__(self, traj, return_duration=8.):  # 4s dove ~5m down at ~2.3 m/s, through own downwash (vortex ring risk)
        Y_end   = traj.get(traj.duration)
        Y_start = traj.get(0.)
        return_seg = p_mt.SmoothLine(Y_end, Y_start, duration=return_duration)
        p_mt.CompositeTraj.__init__(self, [traj, return_seg])


class Traj1(p_mt.Circle):
    name, desc = 'circle north', 'circle r=2 v=2, constant heading and height'
    def __init__(self): p_mt.Circle.__init__(self, [0, 0, 1.5], r=2., v=2., psit=p_t1d.CstOne(0))

class Traj2(p_mt.Circle):
    name, desc = 'circle center', 'circle r=2 v=2, facing center, constant height'
    def __init__(self):
        r, v, alpha0 = 2., 2., 0; om = v/r; psit = p_t1d.AffineOne(om, alpha0+np.sign(r)*np.pi)
        p_mt.Circle.__init__(self, [0, 0, 1.5], r=r, v=v, alpha0=alpha0, psit=psit)

class Traj3(p_mt.Circle):
    name, desc = 'circle front', 'circle r=2 v=2, facing forward, constant height'
    def __init__(self):
        r, v, alpha0 = -2., 2., 0; om = v/r; psit = p_t1d.AffineOne(om, alpha0+np.sign(r)*np.pi/2)
        p_mt.Circle.__init__(self, [0, 0, 1.5], r=r, v=v, alpha0=alpha0, psit=psit)

class Traj4(p_mt.Circle):
    name, desc = 'circle zsine', 'circle r=2 v=2, constant heading, sine height'
    def __init__(self): p_mt.Circle.__init__(self, [0, 0, 1.5], r=2., v=2., psit=p_t1d.CstOne(0), zt=p_t1d.SinOne(c=2, a=0.5, om=4))

class Traj5(p_mt.SmoothBackAndForth):
    name, desc = 'smooth_back_and_forth', 'smooth back and forth'
    def __init__(self):
        super().__init__(Y0=[-1, 0, 1.5, 0], Y1=[1, 0, 2.5, 0])

class Traj6(p_mt.CircleWithIntro):
    name, desc = 'circle_with_intro', 'circle with intro'
    def __init__(self):
        super().__init__(Y0=[0, 0, 1.5, 0], c=[0, 0, 2.5],
                         #r=2., v=1., dt_intro=5., dt_stay=0.5, psit=p_t1d.CstOne(0.))
                         r=2., v=2., dt_intro=5., dt_stay=0.5, psit=p_t1d.CstOne(0.))

class Traj61(ClosedLoop):
    name, desc = 'circle_with_intro1', 'circle with intro, closed loop'
    def __init__(self):
        # ClosedLoop: without it the looping show teleported the reference
        # back to Y0 at wrap; with 3 drones whose Y0s are <1m apart, all
        # references converged at once and the avoidance blew up
        super().__init__(p_mt.CircleWithIntro(Y0=[-0.5, -0.5, 1., 0], c=[-1, 0, 2.],
                         r=2., v=1., dt_intro=5., dt_stay=5., psit=p_t1d.CstOne(0.)))
class Traj62(ClosedLoop):
    name, desc = 'circle_with_intro2', 'circle with intro, closed loop'
    def __init__(self):
        super().__init__(p_mt.CircleWithIntro(Y0=[0., 0., 1.5, 0], c=[0, 0, 2.5],
                         r=2., v=1., dt_intro=5., dt_stay=5., psit=p_t1d.CstOne(0.)))
class Traj63(ClosedLoop):
    name, desc = 'circle_with_intro3', 'circle with intro, closed loop'
    def __init__(self):
        super().__init__(p_mt.CircleWithIntro(Y0=[0.5, 0.5, 2., 0], c=[1, 0, 3.],
                         r=2., v=1., dt_intro=5., dt_stay=5., psit=p_t1d.CstOne(0.)))
        
class Traj7(p_mt.Oval):
    name, desc = 'oval', 'oval'
    def __init__(self):
        super().__init__(l=2, r=1.5, v=2., z=2)

# collision avoidance
class Traj81(p_mt.SmoothBackAndForth):
    name, desc = 'smooth_back_and_forth1', 'smooth back and forth north/south'
    def __init__(self):
        super().__init__(Y0=[-2, 0, 2.5, 0], Y1=[2, 0, 2.5, 0], dt_move=4.)
        
class Traj82(p_mt.SmoothBackAndForth):
    name, desc = 'smooth_back_and_forth2', 'smooth back and forth west/east'
    def __init__(self):
        super().__init__(Y0=[0, -2, 2.5, 0], Y1=[0, 2, 2.5, 0], dt_move=4.)
        
        
class Donut0(p_mt.Trajectory):
    name, desc = 'donut', 'quad4d rebooted: donut'
    def __init__(self, c=[0, 0, 3.], r=1., r2=1., v=4., psi=None, duration=80.):
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
        Y0 = [0., 0, 1.5, 0.]
        d1 = Donut0(r=0.7, r2=1., duration=61.)
        Y1 = d1.get(0)#[:,0]
        Y2 = d1.get(d1.duration)
        steps = [p_mt.SmoothLine(Y0, Y1, duration=2.),
                 d1,
                 p_mt.SmoothLine(Y2, Y0, duration=2.),
                 p_mt.Cst(Y0, duration=1.)]
        super().__init__(steps)


class cercle_back_and_forth(p_mt.CompositeTraj):
    name, desc = 'cercle_back_and_forth', 'cicle followed by back and forth'
    def __init__(self):
        fig1 = Traj62()
        fig2 = Traj81()

        Y_fin_fig1 = fig1.get(fig1.duration)
        Y_debut_fig2 = fig2.get(0)

        steps = [fig1, 
            p_mt.SmoothLine(Y_fin_fig1, Y_debut_fig2, duration=5.), 
            fig2
            ]
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
    name, desc = 'space indexed race track 1', 'Space indexed waypoint trajectory example 1'
    def __init__(self, wps=None, dyn_pts=None, bc="periodic"):
        wps = wps if wps is not None else [[0.2,0, 1],[2.,2.6, 2], [2.,-2.6, 3], [-2.,-2.6, 4], [-2.,2.6, 3], [0.2, 0., 1]]
        dyn_pts = dyn_pts if dyn_pts is not None else [[0,0],[2., 0], [9.,0.1], [15.,0.7], [20.,0.9], [28., 1.], [30,1.]]
        self.wps = np.array(wps)
        self.wp_traj = p_mt_dev.SpaceWaypoints2(self.wps, bc=bc)
        self.dyn_ctl_pts = np.array(dyn_pts)
        self.dyn_segments = [p_t1d.AffOne(self.dyn_ctl_pts[i], self.dyn_ctl_pts[i+1]) for i in range(len(self.dyn_ctl_pts)-1)]
        self.dyn_traj = p_t1d.SmoothedCompositeOne(self.dyn_segments, eps=0.75)
        self.traj = p_mt_dev.SpaceIndexedTraj(self.wp_traj, self.dyn_traj)
        self.duration = dyn_pts[-1][0]

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
    name, desc = 'space indexed race track 2', 'Space indexed waypoint trajectory example 2'
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
    name, desc = 'space indexed slalon', 'Space indexed waypoint slalom, closed loop'
    def __init__(self):
        wps = np.array([[ 0. , -2.6, 1.5],
                        [ 2. , -1.8, 2.5],
                        [ 0. , -1. , 1.2],
                        [ 2. ,  0. , 2.5],
                        [ 0. ,  1. , 1.5],
                        [ 2. ,  1.8, 2.5],
                        [ 0. ,  2.6, 1.5],
                        [-1.5,  1.3, 2. ],
                        [-1.5, -1.3, 2. ],
                        [ 0. , -2.6, 1.5]])
        dyn_pts = [[0,0],[2., 0], [6.,0.15], [20.,0.85], [26., 1.], [28,1.]]
        super().__init__(wps, dyn_pts)


class Traj45(p_mt_dev.SpaceIndexedTraj):
    name, desc = 'space indexed figure of height', 'Space indexed waypoint trajectory example 1'
    def __init__(self, wps=None):
        wps = wps or [[1.4,1.4,2], [0,0,2], [-1.4,-1.4,2], [-1,-3,2], [1,-3,2], [1.4,-1.4,2], [0,0,2], [-1.4,1.4,2], [-1,3,2], [1,3,2], [1.4,1.4,2]]
        self.wps = np.array(wps)
        self.wp_traj = p_mt_dev.SpaceWaypoints2(self.wps, bc="periodic")
        self.duration = 30
        self.dyn_traj = p_t1d.AffOne((0,0),(self.duration,1))
        self.traj = p_mt_dev.SpaceIndexedTraj(self.wp_traj, self.dyn_traj)
        super().__init__(self.wp_traj, self.dyn_traj)

    def has_waypoints(self): return True
    def get_waypoints(self): return self.wps

class Traj46(Traj45):
    name, desc = 'space indexed figure of height2', 'Space indexed waypoint trajectory example 2'
    def __init__(self):
        wps = [[1.4,-1.4,1], [0,0,4], [-1.4,1.4,2], [-3,1,2], [-3,-1,2], [-1.4,-1.4,2], [0,0,4], [1.4,1.4,2], [3,1,2], [3,-1,2], [1.4,-1.4,1]]
        super().__init__(wps)

class Traj47(Traj45):
    name, desc = 'space indexed figure of height3', 'Space indexed waypoint trajectory example 3'
    def __init__(self):
        z = 2.
        wps = [[1.4,1.4,4], [0,0,z], [-1.4,-1.4,z], [-3,-1,z], [-3,1,z], [-1.4,1.4,z], [0,0,z], [1.4,-1.4,z], [3,-1,z], [3,1,z], [1.4,1.4,4]]
        super().__init__(wps)

class Traj47flat(Traj45):
    name, desc = 'space indexed figure of height3 flat', 'figure-of-eight, flat z=2, conflicts at centre'
    def __init__(self):
        z = 2.
        wps = [[-1.4,1.4,z], [0,0,z], [1.4,-1.4,z], [3,-1,z], [3,1,z], [1.4,1.4,z], [0,0,z], [-1.4,-1.4,z], [-3,-1,z], [-3,1,z], [-1.4,1.4,z]]
        super().__init__(wps)

class Traj48(Traj45):
    name, desc = 'space indexed oval', 'Space indexed waypoint example 4'
    def __init__(self):
        x1, x2, y1, y2, z = 0.8, 1.4, 1.2, 3., 2.
        wps = [[x2,y1,z], [x2,-y1,z], [x1,-y2,z], [-x1,-y2,z], [-x2,-y1,z], [-x2,y1,z], [-x1,y2,z], [x1,y2,z], [x2,y1,z]]
        super().__init__(wps)

class Traj49(Traj45):
    name, desc = 'space indexed oval2', 'Space indexed waypoint example 5'
    def __init__(self):
        x1, x2, y1, y2, z = 1.2, 3., 0.8, 1.4, 2.
        wps = [[x1,y2,z], [-x1,y2,z], [-x2,y1,z], [-x2,-y1,z], [-x1,-y2,z], [x1,-y2,z], [x2,-y1,z], [x2,y1,z], [x1,y2,z]]
        super().__init__(wps)


class Traj50(Traj45):
    name, desc = 'space indexed gate race1', 'Space indexed waypoint example 6'
    def __init__(self):
        p1 = [-2, 3, 1] # start
        p2 = [ 0.5, 2.5, 2.5]
        p3 = [ 1.9, 1.5, 3]
        p4 = [ 2, 0, 3] # g1
        
        p5 = [ 0, -2, 3] # g2
        p6 = [-1, -2.2, 2.5] # 
        p7 = [ 0, -2, 2] # g3
        
        p8 = [ 2, 1, 2] # g4
        p9 = [ 0.5, 2.5, 1.5] # 
        p10 = [ -2, 3, 1] # end
        
        wps = [p1, p2, p3, p4, p5, p6, p7, p8, p9, p10]
        super().__init__(wps)

#Let's try a queue leu leu showcase
class QueueLeuLeu(p_mt_dev.SpaceIndexedTraj):
    name, desc = 'queue leu leu', 'Space indexed waypoint example 7'
    def __init__(self, wps=None, delay=0.0, phase=0.0):
        if wps is None:
            wps = [[-2, 3, 1], [0.5, 2.5, 2.5], [1.9, 1.5, 3], [2, 0, 3],
               [0, -2, 3], [-1, -2.2, 2.5], [0, -2, 2], [2, 1, 2],
               [0.5, 2.5, 1.5], [-2, 3, 1]]
        self.wps = np.array(wps)
        self.wp_traj = p_mt_dev.SpaceWaypoints2(self.wps, bc="periodic")
        self.duration = 20.0
        if phase > 0.0:
            self.dyn_traj = p_t1d.AffOne((0, phase), (self.duration, phase + 1.))
        elif delay > 0.0:
            dyn_pts = [[0,0],[delay, 0], [delay + self.duration, 1.]]
            dyn_segments = [p_t1d.AffOne(dyn_pts[i], dyn_pts[i+1]) for i in range(len(dyn_pts)-1)]
            self.dyn_traj = p_t1d.SmoothedCompositeOne(dyn_segments, eps=0.01)
        else:
            self.dyn_traj = p_t1d.AffOne((0,0),(self.duration,1))

        super().__init__(self.wp_traj, self.dyn_traj)

    def has_waypoints(self): return True
    def get_waypoints(self): return self.wps

class QueueLeuLeu1(QueueLeuLeu):
    name, desc = 'queue leu leu 1', 'Course with delay1'
    def __init__(self):
        super().__init__(delay=0.0)

class QueueLeuLeu2(QueueLeuLeu):
    name, desc = 'queue leu leu 2', 'Course with delay2'
    def __init__(self):
        wps_2 = [[-2, 2, 1], [0.5, 2.5, 2.5], [1.9, 1.5, 3], [2, 0, 3],
               [0, -2, 3], [-1, -2.2, 2.5], [0, -2, 2], [2, 1, 2],
               [0.5, 2.5, 1.5], [-2, 2, 1]]
        super().__init__(wps=wps_2, delay=0.0)

class QueueLeuLeu3(QueueLeuLeu):
    name, desc = 'queue leu leu 3', 'Course, 0.3 phase offset (was delay=6)'
    def __init__(self):
        super().__init__(phase=0.7)
       

# Dans traj_factory.py

class CercleSafe1(p_mt.Circle):
    name, desc = 'cercle safe 1', 'Rayon 1m'
    def __init__(self):
        p_mt.Circle.__init__(self, [0, 0, 1.5], r=1., v=1., psit=p_t1d.CstOne(0))

class CercleSafe2(p_mt.Circle):
    name, desc = 'cercle safe 2', 'Rayon 2.5m'
    def __init__(self):
        p_mt.Circle.__init__(self, [0, 0, 1.5], r=2.5, v=2., psit=p_t1d.CstOne(0))

class CercleSafe3(p_mt.Circle):
    name, desc = 'cercle safe 3', 'Rayon 3m, z=2.7'
    def __init__(self):
        p_mt.Circle.__init__(self, [0, 0, 2.7], r=3., v=2.4, psit=p_t1d.CstOne(0))


# Circle in the LEFT / RIGHT half of the cage: same radius, height and phase,
# so the two drones keep a constant ~3.4m gap -> safe by spatial separation
class CircleLeft(p_mt.Circle):
    name, desc = 'circle left', 'circle r=1 at x=-1.7, z=2'
    def __init__(self):
        p_mt.Circle.__init__(self, [-1.7, 0, 2.], r=1., v=1.5, psit=p_t1d.CstOne(0))

class CircleRight(p_mt.Circle):
    name, desc = 'circle right', 'circle r=1 at x=+1.7, z=2'
    def __init__(self):
        p_mt.Circle.__init__(self, [1.7, 0, 2.], r=1., v=1.5, psit=p_t1d.CstOne(0))

# Circle high (r=2, z=3), paired with a low lissajous for a safe split-height,
class RingHigh(p_mt.Circle):
    name, desc = 'ring high', 'circle r=2, z=3.0'
    def __init__(self):
        p_mt.Circle.__init__(self, [0, 0, 3.0], r=2., v=1.5, psit=p_t1d.CstOne(0))



class SpiraleA(p_mt.Circle):
    name, desc = 'spirale a', 'spirale 1/3 : r=2 v=2, 120 deg, z sinus 2->4m'
    def __init__(self):
        r, v, a0 = 2., 2., 0.;           om = v/r
        p_mt.Circle.__init__(self, [0,0,3.], r=r, v=v, alpha0=a0,
                             psit=p_t1d.CstOne(0), zt=p_t1d.SinOne(c=3., a=1., om=om))  
class SpiraleB(p_mt.Circle):
    name, desc = 'spirale b', 'spirale 2/3 : r=2 v=2, 120 deg, z sinus 2->4m'
    def __init__(self):
        r, v, a0 = 2., 2., 2*np.pi/3;    om = v/r
        p_mt.Circle.__init__(self, [0,0,3.], r=r, v=v, alpha0=a0,
                             psit=p_t1d.CstOne(0), zt=p_t1d.SinOne(c=3., a=1., om=om))  
class SpiraleC(p_mt.Circle):
    name, desc = 'spirale c', 'spirale 3/3 : r=2 v=2, 120 deg, z sinus 2->4m'
    def __init__(self):
        r, v, a0 = 2., 2., 4*np.pi/3;    om = v/r
        p_mt.Circle.__init__(self, [0,0,3.], r=r, v=v, alpha0=a0,
                             psit=p_t1d.CstOne(0), zt=p_t1d.SinOne(c=3., a=1., om=om))  




#
# Show trajectories (collision-free by construction, smooth, indoor-safe)
#


class ShowRosetteA(p_mt.Circle):
    name, desc = 'show rosette a', 'rosette 1/3: r=2 v=2, facing center, phase 0'
    def __init__(self):
        r, v, a0 = 2., 2., 0.;           om = v/r
        p_mt.Circle.__init__(self, [0,0,2.], r=r, v=v, alpha0=a0, psit=p_t1d.AffineOne(om, a0+np.pi))
class ShowRosetteB(p_mt.Circle):
    name, desc = 'show rosette b', 'rosette 2/3: r=2 v=2, facing center, phase 120'
    def __init__(self):
        r, v, a0 = 2., 2., 2*np.pi/3;    om = v/r
        p_mt.Circle.__init__(self, [0,0,2.], r=r, v=v, alpha0=a0, psit=p_t1d.AffineOne(om, a0+np.pi))
class ShowRosetteC(p_mt.Circle):
    name, desc = 'show rosette c', 'rosette 3/3: r=2 v=2, facing center, phase 240'
    def __init__(self):
        r, v, a0 = 2., 2., 4*np.pi/3;    om = v/r
        p_mt.Circle.__init__(self, [0,0,2.], r=r, v=v, alpha0=a0, psit=p_t1d.AffineOne(om, a0+np.pi))


class ShowTornadoInner(p_mt.Circle):
    name, desc = 'show tornado inner', 'concentric ring r=1.5 z=1.5 v=1.7'
    def __init__(self): p_mt.Circle.__init__(self, [0,0,1.5], r=1.5, v=1.7, psit=p_t1d.CstOne(0))
class ShowTornadoMid(p_mt.Circle):
    name, desc = 'show tornado mid', 'concentric ring r=2.5 z=3.0 v=2.0'
    def __init__(self): p_mt.Circle.__init__(self, [0,0,3.0], r=2.5, v=2.0, psit=p_t1d.CstOne(0))
class ShowTornadoOuter(p_mt.Circle):
    name, desc = 'show tornado outer', 'concentric ring r=3 z=4.5 v=2.5'
    def __init__(self): p_mt.Circle.__init__(self, [0,0,4.5], r=3., v=2.5, psit=p_t1d.CstOne(0))


class ShowTwinLow(p_mt.Circle):
    name, desc = 'show twin ring low', 'r=2 v=2 z=1.5 CCW'
    def __init__(self): p_mt.Circle.__init__(self, [0,0,1.5], r= 2., v=2., psit=p_t1d.CstOne(0))
class ShowTwinHigh(p_mt.Circle):
    name, desc = 'show twin ring high', 'r=2 v=2 z=4.0 CW'
    def __init__(self): p_mt.Circle.__init__(self, [0,0,4.0], r=-2., v=2., psit=p_t1d.CstOne(0))


class ShowPulseA(p_mt.Circle):
    name, desc = 'show pulse a', 'pulsing ring 1/3, sine height'
    def __init__(self):
        r, v, a0 = 2., 2., 0.;           om = v/r
        p_mt.Circle.__init__(self, [0,0,2.], r=r, v=v, alpha0=a0,
                             psit=p_t1d.AffineOne(om, a0+np.pi), zt=p_t1d.SinOne(c=2., a=0.5, om=1.0))  # z bob softened 1.5->1.0
class ShowPulseB(p_mt.Circle):
    name, desc = 'show pulse b', 'pulsing ring 2/3, sine height'
    def __init__(self):
        r, v, a0 = 2., 2., 2*np.pi/3;    om = v/r
        p_mt.Circle.__init__(self, [0,0,2.], r=r, v=v, alpha0=a0,
                             psit=p_t1d.AffineOne(om, a0+np.pi), zt=p_t1d.SinOne(c=2., a=0.5, om=1.0))  # z bob softened 1.5->1.0
class ShowPulseC(p_mt.Circle):
    name, desc = 'show pulse c', 'pulsing ring 3/3, sine height'
    def __init__(self):
        r, v, a0 = 2., 2., 4*np.pi/3;    om = v/r
        p_mt.Circle.__init__(self, [0,0,2.], r=r, v=v, alpha0=a0,
                             psit=p_t1d.AffineOne(om, a0+np.pi), zt=p_t1d.SinOne(c=2., a=0.5, om=1.0))  # z bob softened 1.5->1.0


# Oval stack: two ovals at different heights (1.2m gap) and speeds.
class ShowOvalLow(p_mt.Oval):
    name, desc = 'show oval low', 'oval l=1.5 r=1.5 v=2.0 z=1.8'
    def __init__(self): super().__init__(l=1.5, r=1.5, v=2.0, z=1.8)
class ShowOvalHigh(p_mt.Oval):
    name, desc = 'show oval high', 'oval l=1.5 r=1.5 v=2.0 z=3.0'
    def __init__(self): super().__init__(l=1.5, r=1.5, v=2.0, z=3.0)


class ShowLissajous(p_mt.Trajectory):
    name, desc = 'show lissajous', 'analytic 3:2 lissajous, solo showpiece'
    def __init__(self, A=2.5, B=2.5, a=3, b=2, om=0.28, z=2., delta=np.pi/2):  
        self.A, self.B, self.a, self.b, self.om = A, B, a, b, om
        self.z, self.delta = z, delta
        self.t0, self.duration = 0., 2*np.pi/om  
    def reset(self, t0): self.t0 = t0
    def get(self, t):
        dt = t - self.t0
        wa, wb = self.a*self.om, self.b*self.om
        pa, pb = wa*dt + self.delta, wb*dt
        sa, ca = np.sin(pa), np.cos(pa)
        sb, cb = np.sin(pb), np.cos(pb)
        Yc = np.zeros((5,4))
        Yc[0,p_mt._x], Yc[1,p_mt._x] =  self.A*sa,        self.A*wa*ca
        Yc[2,p_mt._x], Yc[3,p_mt._x] = -self.A*wa**2*sa, -self.A*wa**3*ca
        Yc[4,p_mt._x]                =  self.A*wa**4*sa
        Yc[0,p_mt._y], Yc[1,p_mt._y] =  self.B*sb,        self.B*wb*cb
        Yc[2,p_mt._y], Yc[3,p_mt._y] = -self.B*wb**2*sb, -self.B*wb**3*cb
        Yc[4,p_mt._y]                =  self.B*wb**4*sb
        Yc[0,p_mt._z] = self.z
        return Yc.T

class ShowLissajousLow(ShowLissajous):
    name, desc = 'show lissajous low', 'analytic 3:2 lissajous, z=1.5'
    def __init__(self): super().__init__(z=1.5)

class ShowStar(Traj45):
    name, desc = 'show star', '5-branch star (rounded by spline), space indexed'
    def __init__(self):
        R, r, z = 2.5, 1.0, 2.
        wps = [[ (R if k%2==0 else r)*np.cos(np.pi/2 + k*np.pi/5),
                 (R if k%2==0 else r)*np.sin(np.pi/2 + k*np.pi/5), z ] for k in range(10)]
        wps.append(wps[0])  # close the loop (periodic bc, like Traj45)
        super().__init__(wps)

# --- analytic showpieces built as sums of sinusoids ---------------------
# Any curve written as a sum of sinusoids has ALL its time-derivatives in
# closed form, so the flat output (pos..snap) is exact -> clean diff-flatness
# and good tracking. _harm returns [f, f', f'', f''', f''''] for one axis.
def _harm(dt, terms):
        out = np.zeros(5)
    for amp, om, ph, kind in terms:
        th = om * dt + ph
        if kind == 'c':          # d^n/dt^n [cos] cycles cos,-sin,-cos,sin,...
            vals = (np.cos(th), -np.sin(th), -np.cos(th), np.sin(th), np.cos(th))
        else:                    # sin
            vals = (np.sin(th),  np.cos(th), -np.sin(th), -np.cos(th), np.sin(th))
        p = 1.0
        for n in range(5):
            out[n] += amp * p * vals[n]
            p *= om              # amp * om^n * vals[n]
    return out
 
 
# Spirograph / epicyclic rosette: sum of two rotating vectors. w2 = -k*w1 (k
# integer) so it closes after one base turn; petal count set by k. Hypnotic,
# analytic, solo (no inter-drone conflict).
class ShowSpirograph(p_mt.Trajectory):
    name, desc = 'show spirograph', 'epicyclic rosette (two superposed rotations), solo'
    def __init__(self, R=1.6, a=0.9, k=4, om=0.28, z=2.5, phase=0.):
        self.R, self.a, self.k, self.om, self.z, self.phase = R, a, k, om, z, phase
        self.t0, self.duration = 0., 2*np.pi/om
    def reset(self, t0): self.t0 = t0
    def get(self, t):
        dt = t - self.t0
        w1, w2 = self.om, -self.k*self.om
        Yc = np.zeros((5, 4))
        Yc[:, p_mt._x] = _harm(dt, [(self.R, w1, self.phase, 'c'), (self.a, w2, self.phase, 'c')])
        Yc[:, p_mt._y] = _harm(dt, [(self.R, w1, self.phase, 's'), (self.a, w2, self.phase, 's')])
        Yc[:, p_mt._z] = _harm(dt, [(self.z, 0., 0., 'c')])
        return Yc.T
 
# three stacked rosettes (different petal counts), height-separated >= margin
# so the trio is safe by construction (a rosette "tower").
class ShowSpirographLow(ShowSpirograph):
    name, desc = 'show spirograph low', 'rosette k=3, z=1.5'
    def __init__(self): super().__init__(k=3, om=0.26, z=1.5)
class ShowSpirographMid(ShowSpirograph):
    name, desc = 'show spirograph mid', 'rosette k=4, z=3.0'
    def __init__(self): super().__init__(k=4, om=0.26, z=3.0)
class ShowSpirographHigh(ShowSpirograph):
    name, desc = 'show spirograph high', 'rosette k=5, z=4.5'
    def __init__(self): super().__init__(k=5, om=0.26, z=4.5)
 
 
# (p,q) torus knot: product-to-sum turns cos(q.)cos(p.) etc. into sums of
# sinusoids, so it's analytic like the rest. Looks genuinely 3D. Solo.
class ShowKnot(p_mt.Trajectory):
    name, desc = 'show knot', '(2,3) torus knot, solo 3D showpiece'
    def __init__(self, Rmaj=1.5, r=0.7, p=2, q=3, om=0.30, zc=3.0):
        self.Rmaj, self.r, self.p, self.q, self.om, self.zc = Rmaj, r, p, q, om, zc
        self.t0, self.duration = 0., 2*np.pi/om
    def reset(self, t0): self.t0 = t0
    def get(self, t):
        dt = t - self.t0
        w, r2 = self.om, self.r/2.
        wp, wpq, wmq, wq = self.p*w, (self.p+self.q)*w, (self.p-self.q)*w, self.q*w
        Yc = np.zeros((5, 4))
        Yc[:, p_mt._x] = _harm(dt, [(self.Rmaj, wp, 0., 'c'), (r2, wpq, 0., 'c'), (r2, wmq, 0., 'c')])
        Yc[:, p_mt._y] = _harm(dt, [(self.Rmaj, wp, 0., 's'), (r2, wpq, 0., 's'), (r2, wmq, 0., 's')])
        Yc[:, p_mt._z] = _harm(dt, [(self.zc, 0., 0., 'c'), (self.r, wq, 0., 's')])
        return Yc.T
 
 
# Fountain: each drone flies the SAME vertical circle (radius R at horizontal
# offset rho0, azimuth phi), so the trio at phi = 0/120/240 stays 120 deg apart
# at every instant -> pairwise distance = sqrt(3)*s(t) >= sqrt(3)*(rho0-R),
# safe by construction. Constant speed R*om. A blooming 3-arm fountain.
class _Fountain(p_mt.Trajectory):
    def __init__(self, phi=0., rho0=2.0, R=1.0, z0=2.5, om=0.9):
        self.cphi, self.sphi = np.cos(phi), np.sin(phi)
        self.rho0, self.R, self.z0, self.om = rho0, R, z0, om
        self.t0, self.duration = 0., 2*np.pi/om
    def reset(self, t0): self.t0 = t0
    def get(self, t):
        dt, w = t - self.t0, self.om
        Yc = np.zeros((5, 4))
        Yc[:, p_mt._x] = _harm(dt, [(self.rho0*self.cphi, 0., 0., 'c'), (self.R*self.cphi, w, 0., 'c')])
        Yc[:, p_mt._y] = _harm(dt, [(self.rho0*self.sphi, 0., 0., 'c'), (self.R*self.sphi, w, 0., 'c')])
        Yc[:, p_mt._z] = _harm(dt, [(self.z0, 0., 0., 'c'), (self.R, w, 0., 's')])
        return Yc.T
class FountainA(_Fountain):
    name, desc = 'fountain a', 'fountain arc, azimuth 0'
    def __init__(self): super().__init__(phi=0.)
class FountainB(_Fountain):
    name, desc = 'fountain b', 'fountain arc, azimuth 120'
    def __init__(self): super().__init__(phi=2*np.pi/3)
class FountainC(_Fountain):
    name, desc = 'fountain c', 'fountain arc, azimuth 240'
    def __init__(self): super().__init__(phi=4*np.pi/3)
class FountainOpp(_Fountain):   # paired with 'fountain a' for a 2-drone version
    name, desc = 'fountain opp', 'fountain arc, azimuth 180'
    def __init__(self): super().__init__(phi=np.pi)
 
 
# Morphing formation: each drone oscillates between a 'line' point L and a
# 'triangle' point T with beta(t) = (1-cos om t)/2, so pos = mid + amp*cos(om t)
# (a pure sinusoid per axis). L and T are chosen so the min pairwise distance
# over the whole morph stays ~1.9 m -> safe by construction.
class _Morph(p_mt.Trajectory):
    def __init__(self, L, T, z=2.0, om=0.6):
        self.L, self.T, self.z, self.om = np.asarray(L, float), np.asarray(T, float), z, om
        self.t0, self.duration = 0., 2*np.pi/om
    def reset(self, t0): self.t0 = t0
    def get(self, t):
        dt, w = t - self.t0, self.om
        mid = 0.5*(self.L + self.T)          # centre of the oscillation
        amp = -0.5*(self.T - self.L)         # coefficient of cos(w t)
        Yc = np.zeros((5, 4))
        Yc[:, p_mt._x] = _harm(dt, [(mid[0], 0., 0., 'c'), (amp[0], w, 0., 'c')])
        Yc[:, p_mt._y] = _harm(dt, [(mid[1], 0., 0., 'c'), (amp[1], w, 0., 'c')])
        Yc[:, p_mt._z] = _harm(dt, [(self.z, 0., 0., 'c')])
        return Yc.T
_R_MORPH = 1.6
class MorphA(_Morph):
    name, desc = 'morph a', 'line<->triangle morph, drone A'
    def __init__(self):
        super().__init__(L=[-2., 0.], T=[_R_MORPH*np.cos(np.radians(210)), _R_MORPH*np.sin(np.radians(210))])
class MorphB(_Morph):
    name, desc = 'morph b', 'line<->triangle morph, drone B'
    def __init__(self): super().__init__(L=[0., 0.], T=[0., _R_MORPH])
class MorphC(_Morph):
    name, desc = 'morph c', 'line<->triangle morph, drone C'
    def __init__(self):
        super().__init__(L=[2., 0.], T=[_R_MORPH*np.cos(np.radians(330)), _R_MORPH*np.sin(np.radians(330))])
 



class ConflitTriA(p_mt.SmoothBackAndForth):
    name, desc = 'conflit tri a', 'coin->centre->coin, 0 deg'
    def __init__(self): super().__init__(Y0=[ 3.0, 0.0,2.5,0], Y1=[0,0,2.5,0], dt_move=4.)
class ConflitTriB(p_mt.SmoothBackAndForth):
    name, desc = 'conflit tri b', 'coin->centre->coin, 120 deg'
    def __init__(self): super().__init__(Y0=[-1.5, 2.6,2.5,0], Y1=[0,0,2.5,0], dt_move=4.)
class ConflitTriC(p_mt.SmoothBackAndForth):
    name, desc = 'conflit tri c', 'coin->centre->coin, 240 deg'
    def __init__(self): super().__init__(Y0=[-1.5,-2.6,2.5,0], Y1=[0,0,2.5,0], dt_move=4.)



class ScaraRace(p_mt.CompositeTraj):
    name, desc = 'scara race', 'type SCARA : dashs horizontaux rapides + arrets nets adoucis'
    def __init__(self):
        z, psi = 2.5, 0.
        t_move, t_dwell = 2.2, 0.4   
        coins = [[-2,-2,z,psi], [2,-2,z,psi], [2,2,z,psi], [-2,2,z,psi]]
        coins.append(coins[0])                  #referme la boucle 
        steps = []
        for i in range(len(coins)-1):
            steps.append(p_mt.SmoothLine(coins[i], coins[i+1], duration=t_move))  
            steps.append(p_mt.Cst(coins[i+1], duration=t_dwell))                  
        super().__init__(steps)



def _spirale_montante(a0):
    # N tours a `spacing` m d'ecart vertical ; depart bas pour rester sous
    # le plafond : montee totale = spacing*N, sommet = z_min + spacing*N
    r, v, N, spacing, z_min = 2., 2., 3, 1.8, 1.0
    om = v/r;  om_z = om/(2*N)
    a = spacing*N/2.        # amplitude pour l'ecart par tour voulu
    c = z_min + a           # bas de l'oscillation a z_min (sommet a z_min+2a)
    return p_mt.Circle([0,0,c], r=r, v=v, alpha0=a0,
                       psit=p_t1d.CstOne(0), zt=p_t1d.SinOne(c=c, a=a, om=om_z))

class SpiraleMontanteA(ClosedLoop):
    name, desc = 'spirale montante a', 'helice 1/3 : r=2 v=2, 120 deg, monte en 3 tours puis redescend en douceur'
    def __init__(self): super().__init__(_spirale_montante(0.))
class SpiraleMontanteB(ClosedLoop):
    name, desc = 'spirale montante b', 'helice 2/3 : r=2 v=2, 120 deg, monte en 3 tours puis redescend en douceur'
    def __init__(self): super().__init__(_spirale_montante(2*np.pi/3))
class SpiraleMontanteC(ClosedLoop):
    name, desc = 'spirale montante c', 'helice 3/3 : r=2 v=2, 120 deg, monte en 3 tours puis redescend en douceur'
    def __init__(self): super().__init__(_spirale_montante(4*np.pi/3))


class _Flower(p_mt.Trajectory):
    def __init__(self, a0, Rc=1.85, Ra=1.15, om=0.9, omr=0.3, z=2.5):
        self.a0, self.Rc, self.Ra, self.om, self.omr, self.z = a0, Rc, Ra, om, omr, z
        self.t0, self.duration = 0., 2*np.pi/omr   # one full breath
    def reset(self, t0): self.t0 = t0
    def get(self, t):
        dt = t - self.t0
        w, wr = self.om, self.omr
        phi = w*dt + self.a0
        c, s = np.cos(phi), np.sin(phi)
        R  =  self.Rc - self.Ra*np.cos(wr*dt)
        R1 =  self.Ra*wr    *np.sin(wr*dt)
        R2 =  self.Ra*wr**2 *np.cos(wr*dt)
        R3 = -self.Ra*wr**3 *np.sin(wr*dt)
        R4 = -self.Ra*wr**4 *np.cos(wr*dt)
        Yc = np.zeros((5, 4))
        Yc[0, p_mt._x], Yc[0, p_mt._y], Yc[0, p_mt._z] = R*c, R*s, self.z
        Yc[1, p_mt._x] = R1*c - R*w*s
        Yc[1, p_mt._y] = R1*s + R*w*c
        Yc[2, p_mt._x] = (R2 - R*w**2)*c - 2*R1*w*s
        Yc[2, p_mt._y] = (R2 - R*w**2)*s + 2*R1*w*c
        Yc[3, p_mt._x] = (R3 - 3*R1*w**2)*c + (-3*R2*w + R*w**3)*s
        Yc[3, p_mt._y] = (R3 - 3*R1*w**2)*s + ( 3*R2*w - R*w**3)*c
        Yc[4, p_mt._x] = (R4 - 6*R2*w**2 + R*w**4)*c + (-4*R3*w + 4*R1*w**3)*s
        Yc[4, p_mt._y] = (R4 - 6*R2*w**2 + R*w**4)*s + ( 4*R3*w - 4*R1*w**3)*c
        return Yc.T
 
class FlowerA(_Flower):
    name, desc = 'flower a', 'blooming flower 1/3 (3 drones, breathing)'
    def __init__(self): super().__init__(0.)
class FlowerB(_Flower):
    name, desc = 'flower b', 'blooming flower 2/3'
    def __init__(self): super().__init__(2*np.pi/3)
class FlowerC(_Flower):
    name, desc = 'flower c', 'blooming flower 3/3'
    def __init__(self): super().__init__(4*np.pi/3)
 
 
# --- Mexican wave: 3 drones stay put in a row (fixed x, y) and bob up/down
#     in z with a phase offset -> a travelling "ola". Fixed >2m apart -> safe.
class _VerticalBob(p_mt.Trajectory):
    def __init__(self, x0, y0, zc, za, om, phi):
        self.x0, self.y0, self.zc, self.za, self.om, self.phi = x0, y0, zc, za, om, phi
        self.t0, self.duration = 0., 2*np.pi/om   # one full bob
    def reset(self, t0): self.t0 = t0
    def get(self, t):
        dt = t - self.t0
        a = self.om*dt + self.phi
        s, c = np.sin(a), np.cos(a)
        Yc = np.zeros((5, 4))
        Yc[0, p_mt._x], Yc[0, p_mt._y] = self.x0, self.y0
        Yc[0, p_mt._z] = self.zc + self.za*s
        Yc[1, p_mt._z] =  self.za*self.om    *c
        Yc[2, p_mt._z] = -self.za*self.om**2 *s
        Yc[3, p_mt._z] = -self.za*self.om**3 *c
        Yc[4, p_mt._z] =  self.za*self.om**4 *s
        return Yc.T
 
class WaveLeft(_VerticalBob):
    name, desc = 'wave left', 'vertical bob at x=-2.5'
    def __init__(self): super().__init__(-2.5, 0., 2.5, 1.2, 1.0, 0.)
class WaveMid(_VerticalBob):
    name, desc = 'wave mid', 'vertical bob at x=0'
    def __init__(self): super().__init__(0., 0., 2.5, 1.2, 1.0, 2*np.pi/3)
class WaveRight(_VerticalBob):
    name, desc = 'wave right', 'vertical bob at x=+2.5'
    def __init__(self): super().__init__(2.5, 0., 2.5, 1.2, 1.0, 4*np.pi/3)
 
 

def _helix_strand(a0, z_up=True):
    r, v, N, spacing, z_min = 2., 2., 3, 1.8, 1.0
    om = v/r;  om_z = om/(2*N)
    a = spacing*N/2.;  c = z_min + a
    return p_mt.Circle([0, 0, c], r=r, v=v, alpha0=a0, psit=p_t1d.CstOne(0),
                       zt=p_t1d.SinOne(c=c, a=(a if z_up else -a), om=om_z))
 
class DnaA(ClosedLoop):
    name, desc = 'dna strand a', 'helix strand, 0 deg, z up'
    def __init__(self): super().__init__(_helix_strand(0., z_up=True))
class DnaB(ClosedLoop):
    name, desc = 'dna strand b', 'helix strand, 180 deg, z opposite (weaves)'
    def __init__(self): super().__init__(_helix_strand(np.pi, z_up=False))
 
 
# --- Cascade: the same circle at 3 heights, 120deg apart 
class CascadeLow(p_mt.Circle):
    name, desc = 'cascade low', 'circle r=2 z=1.5'
    def __init__(self): p_mt.Circle.__init__(self, [0, 0, 1.5], r=2., v=1.5, alpha0=0.,        psit=p_t1d.CstOne(0))
class CascadeMid(p_mt.Circle):
    name, desc = 'cascade mid', 'circle r=2 z=3.0, +120 deg'
    def __init__(self): p_mt.Circle.__init__(self, [0, 0, 3.0], r=2., v=1.5, alpha0=2*np.pi/3, psit=p_t1d.CstOne(0))
class CascadeHigh(p_mt.Circle):
    name, desc = 'cascade high', 'circle r=2 z=4.5, +240 deg'
    def __init__(self): p_mt.Circle.__init__(self, [0, 0, 4.5], r=2., v=1.5, alpha0=4*np.pi/3, psit=p_t1d.CstOne(0))
 



class TrajFactory:
    _chapters = {}
    _trajectories = {}

    @staticmethod
    def chapters(): return TrajFactory._chapters
    
    @staticmethod
    def get(name, chapter=None):
        if chapter is None: return TrajFactory._trajectories[name]
        else: return TrajFactory._chapters[chapter][name]
    
    @staticmethod
    def register(T, chapter=None):
        if chapter is not None:
            try: TrajFactory._chapters[chapter]
            except KeyError:
                TrajFactory._chapters[chapter] = {}
            TrajFactory._chapters[chapter][T.name] = T
        TrajFactory._trajectories[T.name] = T
    
TrajFactory.register(Traj1, 'circles')
TrajFactory.register(Traj2, 'circles')
TrajFactory.register(Traj3, 'circles')
TrajFactory.register(Traj4, 'circles')
TrajFactory.register(Traj5, 'misc')
TrajFactory.register(Traj6, 'misc')
TrajFactory.register(Traj61, 'test_voliere')
TrajFactory.register(Traj62, 'test_voliere')
TrajFactory.register(Traj63, 'test_voliere')

TrajFactory.register(Traj7, 'misc')

TrajFactory.register(Traj81, 'test_optim')
TrajFactory.register(Traj82, 'test_optim')


TrajFactory.register(Donut0, 'misc')
TrajFactory.register(Donut1, 'misc')
TrajFactory.register(Traj17, 'misc')
TrajFactory.register(Traj42, 'space index')
TrajFactory.register(Traj43, 'space index')
TrajFactory.register(Traj44, 'space index')
TrajFactory.register(Traj45, 'space index')
TrajFactory.register(Traj46, 'space index')
TrajFactory.register(Traj47, 'space index')
TrajFactory.register(Traj47flat, 'space index')
TrajFactory.register(Traj48, 'space index')
TrajFactory.register(Traj49, 'space index')
TrajFactory.register(Traj50, 'space index')


TrajFactory.register(cercle_back_and_forth, 'showcase 1')

TrajFactory.register(QueueLeuLeu1, 'Poursuite')
TrajFactory.register(QueueLeuLeu2, 'Poursuite')
TrajFactory.register(QueueLeuLeu3, 'Poursuite')

TrajFactory.register(CercleSafe1, 'safe_test')
TrajFactory.register(CercleSafe2, 'safe_test')
TrajFactory.register(CercleSafe3, 'safe_test')
TrajFactory.register(CircleLeft, 'safe_test')
TrajFactory.register(CircleRight, 'safe_test')
TrajFactory.register(RingHigh, 'safe_test')

TrajFactory.register(ShowRosetteA, 'show')
TrajFactory.register(ShowRosetteB, 'show')
TrajFactory.register(ShowRosetteC, 'show')

TrajFactory.register(ShowTornadoInner, 'show')
TrajFactory.register(ShowTornadoMid, 'show')
TrajFactory.register(ShowTornadoOuter, 'show')

TrajFactory.register(ShowTwinLow, 'show')
TrajFactory.register(ShowTwinHigh, 'show')

TrajFactory.register(ShowPulseA, 'show')
TrajFactory.register(ShowPulseB, 'show')
TrajFactory.register(ShowPulseC, 'show')

TrajFactory.register(ShowOvalLow, 'show')
TrajFactory.register(ShowOvalHigh, 'show')

TrajFactory.register(ShowLissajous, 'show')
TrajFactory.register(ShowLissajousLow, 'show')
TrajFactory.register(ShowStar, 'show')

TrajFactory.register(ShowSpirograph, 'show')
TrajFactory.register(ShowSpirographLow, 'show')
TrajFactory.register(ShowSpirographMid, 'show')
TrajFactory.register(ShowSpirographHigh, 'show')
TrajFactory.register(ShowKnot, 'show')
 
TrajFactory.register(FountainA, 'show')
TrajFactory.register(FountainB, 'show')
TrajFactory.register(FountainC, 'show')
TrajFactory.register(FountainOpp, 'show')
TrajFactory.register(MorphA, 'show')
TrajFactory.register(MorphB, 'show')
TrajFactory.register(MorphC, 'show')

TrajFactory.register(SpiraleA, 'show')
TrajFactory.register(SpiraleB, 'show')
TrajFactory.register(SpiraleC, 'show')

TrajFactory.register(ScaraRace, 'show')

TrajFactory.register(SpiraleMontanteA, 'show')
TrajFactory.register(SpiraleMontanteB, 'show')
TrajFactory.register(SpiraleMontanteC, 'show')

TrajFactory.register(FlowerA, 'show')
TrajFactory.register(FlowerB, 'show')
TrajFactory.register(FlowerC, 'show')
 
TrajFactory.register(WaveLeft, 'show')
TrajFactory.register(WaveMid, 'show')
TrajFactory.register(WaveRight, 'show')
 
TrajFactory.register(DnaA, 'show')
TrajFactory.register(DnaB, 'show')
 
TrajFactory.register(CascadeLow, 'show')
TrajFactory.register(CascadeMid, 'show')
TrajFactory.register(CascadeHigh, 'show')
 

TrajFactory.register(ConflitTriA, 'Conflicts')
TrajFactory.register(ConflitTriB, 'Conflicts')
TrajFactory.register(ConflitTriC, 'Conflicts')
