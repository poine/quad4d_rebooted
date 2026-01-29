import numpy as np
import yaml

import pat3.algebra as p_al, pat3.trajectory_1D as p_t1d
import pat3.vehicles.rotorcraft.multirotor_trajectory as p_tm
import pat3.vehicles.rotorcraft.multirotor_trajectory_dev as p_tmdev
import pat3.vehicles.rotorcraft.multirotor_fdm as p_mfdm
import pat3.vehicles.rotorcraft.multirotor_control as p_mctl


import traj_factory
        

class Model:
    def __init__(self, load_id='ex_si1'):
        #self.trajectory = Trajectory('./traj_exemple.yaml') # maybe someday....
        self.load_from_factory(load_id)
        
        self.wps = self.trajectory.wps 
        self.wp_traj = self.trajectory.wp_traj

        self.set_dynamics(self.trajectory.dyn_ctl_pts)
        self.fdm = p_mfdm.MR_FDM()

        self.extends = ((-5, 5), (-5, 5), (0, 8.))
        self.extends = ((-5, 5), (-4, 4), (0, 8.))

       
    def load_from_factory(self, which):
        self.trajectory = traj_factory.TrajFactory.trajectories[which]()
        print(f'model loaded {which} ({self.trajectory.desc}, {self.trajectory.duration}s)')

    def get_trajectory(self): return self.trajectory
        
        
    def get_output_at(self, t):
        return self.trajectory.get(t)

    def get_state_at(self, t):
        Y = self.get_output_at(t)
        X, U, Xd = p_mctl.DiffFlatness.state_and_cmd_of_flat_output(None, Y, self.fdm.P)
        pos, vel = X[p_mfdm.sv_slice_pos], np.linalg.norm(X[p_mfdm.sv_slice_vel])
        #euler = p_al.euler_of_quat(X[p_mfdm.sv_slice_quat])
        rmat = p_al.rmat_of_quat(X[p_mfdm.sv_slice_quat])
        return pos, vel, rmat
    
    def set_dynamics(self, dyn_ctl_pts):
        self.trajectory.set_dynamic(dyn_ctl_pts)

    def set_waypoints(self, wps):
        self.trajectory.set_waypoints(wps)

    def get_waypoints(self):
        # do we put get_waypoints in base trajectory class ?
        try: wps = self.trajectory.get_waypoints()
        except AttributeError: wps = np.array([[0,0,0]])
        return wps
        
    # TODO: cache sampling

    def sample_geometry(self, npts=1000):
        ls = np.linspace(0, 1, npts)
        return ls, np.array([self.trajectory.wp_traj.get(l) for l in ls])

    def sample_dynamics(self, npts=1000):
        time = np.linspace(0, self.trajectory.duration, npts)
        return time, np.array([self.trajectory.dyn_traj.get(t) for t in time])
        
    def sample_output(self, npts=1000):
        time = np.linspace(0, self.trajectory.duration, npts)
        return time, np.array([self.trajectory.get(t) for t in time])

    def sample_state(self, npts=1000):
        time, Ys = self.sample_output(npts)
        Xs = np.array([p_mctl.DiffFlatness.state_and_cmd_of_flat_output(None, Y, self.fdm.P)[0] for Y in Ys])
        pos, vel = Xs[:, p_mfdm.sv_slice_pos], np.linalg.norm(Xs[:, p_mfdm.sv_slice_vel], axis=1)
        euler = np.array([p_al.euler_of_quat(q) for q in Xs[:, p_mfdm.sv_slice_quat]])
        return time, pos, vel, euler
