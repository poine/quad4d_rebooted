import numpy as np

import pat3.trajectory_1D as p_t1d
import pat3.vehicles.rotorcraft.multirotor_trajectory as p_mt
import pat3.vehicles.rotorcraft.multirotor_trajectory_dev as p_mt_dev

class Traj0(p_mt.Circle):
    name, desc = 'circle1', 'circle r=2 v=4'
    def __init__(self): p_mt.Circle.__init__(self, [0, 0, 1.5], r=2., v=4., psit=None)#p_mt.AffineOne(1, 0))

class Traj1(p_mt.SmoothBackAndForth):
    name, desc = 'smooth_back_and_forth', 'smooth back and forth'

class Traj2(p_mt.CircleWithIntro):
    name, desc = 'circle_with_intro', 'circle with intro'
    def __init__(self):
        super().__init__(Y0=[0, 0, 1.5, 0], c=[0, 0, 2.5],
                         r=2., v=3., dt_intro=5., dt_stay=0.5, psit=p_mt.CstOne(0.))

class Traj42(p_mt_dev.SpaceIndexedTraj):
    name, desc = 'ex_si0', 'Space indexed waypoint trajectory example'
    def __init__(self):
        self.wps = np.array([[0.2,0, 1],[2.,3., 2], [2.,-3., 3], [-2.,-3., 4], [-2.,3., 3], [-0.2, 0., 2]])
        self.wp_traj = p_mt_dev.SpaceWaypoints(self.wps)
        self.dyn_ctl_pts = np.array([[0,0],[1., 0], [3.,0.1], [5.,0.7], [7.,0.9], [9., 1.], [10,1.]])
        self.dyn_segments = [p_t1d.AffOne(self.dyn_ctl_pts[i], self.dyn_ctl_pts[i+1]) for i in range(len(self.dyn_ctl_pts)-1)]
        self.dyn_traj = p_t1d.SmoothedCompositeOne(self.dyn_segments, eps=0.75)
        self.traj = p_mt_dev.SpaceIndexedTraj(self.wp_traj, self.dyn_traj)
        #super().__init__(self.wp_traj, self.dyn_smoothed)
        self.duration = self.traj.duration

    def set_waypoints(self, waypoints):
        self.wps = waypoints
        self.wp_traj = p_mt_dev.SpaceWaypoints(self.wps)
        self.traj = p_mt_dev.SpaceIndexedTraj(self.wp_traj, self.dyn_traj)

    def get_waypoints(self): return self.wps
        
    def set_dynamic(self, dyn_ctl_pts):
        self.dyn_ctl_pts = dyn_ctl_pts
        self.dyn_segments = [p_t1d.AffOne(self.dyn_ctl_pts[i], self.dyn_ctl_pts[i+1]) for i in range(len(self.dyn_ctl_pts)-1)]
        self.dyn_traj = p_t1d.SmoothedCompositeOne(self.dyn_segments, eps=0.75)
        self.traj = p_mt_dev.SpaceIndexedTraj(self.wp_traj, self.dyn_traj)
        self.duration = self.traj.duration

    def get_dyn_ctl_pts(self): return self.dyn_ctl_pts
        
    def get(self, t):
        return self.traj.get(t)

        
class TrajFactory:
    trajectories = {}

    @staticmethod
    def register(T):
        TrajFactory.trajectories[T.name] = T
    
TrajFactory.register(Traj0)
TrajFactory.register(Traj1)
TrajFactory.register(Traj2)
TrajFactory.register(Traj42)
