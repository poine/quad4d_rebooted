import logging, yaml
import numpy as np

import pat3.algebra as p_al, pat3.trajectory_1D as p_t1d
import pat3.vehicles.rotorcraft.multirotor_trajectory as p_tm
import pat3.vehicles.rotorcraft.multirotor_trajectory_dev as p_tmdev
import pat3.vehicles.rotorcraft.multirotor_fdm as p_mfdm
import pat3.vehicles.rotorcraft.multirotor_control as p_mctl

import traj_factory, misc_utils as mu
        
logger = logging.getLogger(__name__)

class Model:
    def __init__(self, load_fact_id=None, trajectory=None):

        self.trajectories = []
        
        if trajectory is not None:
            self.trajectory.append(trajectory)
        if load_fact_id is not None:
            self.load_from_factory(load_fact_id)
        self.fdm = p_mfdm.MR_FDM()
        self.extends = ((-5, 5), (-5, 5), (0, 8.))
        self.safe_extends = ((-4, 4), (-4, 4), (1, 7.))
       
    def load_from_factory(self, name, chapter=None, idx=None):
        trajectory = traj_factory.TrajFactory.get(name, chapter)()
        if idx is not None and idx<len(self.trajectories):
            self.trajectories[idx]=trajectory
        else:
            self.trajectories.append(trajectory)
            idx = len(self.trajectories)-1
        logger.info(f'loaded {name} ({trajectory.desc}, {trajectory.duration:.2f}s) in slot {idx}')
        return idx

    def set_trajectory(self, trajectory, idx=0): self.trajectories[idx] = trajectory
    def get_trajectory(self, idx=0): return self.trajectories[idx]
    def trajectory_nb(self): return len(self.trajectories)
    def trajectory_duration(self, idx=None):
        if idx is not None: return self.trajectories[idx].duration
        else:
            return np.max([_t.duration for _t in self.trajectories])
    
    def get_traj_output_at(self, t, idx=0): return self.trajectories[idx].get(t)

    def get_traj_pose_at(self, t, idx=0):
        Yenu = self.get_traj_output_at(t, idx)
        Yned = mu.Yenu2ned(Yenu)
        Xned, U, Xd = p_mctl.DiffFlatness.state_and_cmd_of_flat_output(Yned, self.fdm.P)
        Tenu2fru = mu.Tenu2fru_of_Xned(Xned)
        return Tenu2fru
    
    def set_dynamics(self, dyn_ctl_pts, idx=0):
        self.trajectories[idx].set_dynamic(dyn_ctl_pts)

    def set_waypoints(self, wps, idx=0):
        self.trajectories[idx].set_waypoints(wps)

    def get_waypoints(self, idx=0):
        return self.get_trajectory(idx).get_waypoints()
        
    # TODO: cache sampling

    def sample_traj_geometry(self, idx=0, npts=1000):
        traj = self.get_trajectory(idx)
        if traj.is_space_indexed():
            ls = np.linspace(0, 1, npts)
            return ls, np.array([traj.wp_traj.get(l) for l in ls])
        else: return [],[] # MAYBE return None instead?

    def sample_traj_dynamics(self, idx=0, npts=1000):
        traj = self.get_trajectory(idx)
        time = np.linspace(0, traj.duration, npts)
        return time, np.array([traj.dyn_traj.get(t) for t in time])
        
    def sample_traj_output(self, idx=0, npts=1000):
        traj = self.get_trajectory(idx)
        time = np.linspace(0, traj.duration, npts)
        return time, np.array([traj.get(t) for t in time])

    def sample_traj_state(self, idx=0, npts=1000):
        time, Yenus = self.sample_traj_output(idx, npts) # enu
        pos, vel = Yenus[:,:3,0], Yenus[:,:3,1]
        Yned = [mu.Yenu2ned(Y) for Y in Yenus]
        #Yned2 = mu.Yenu2ned(Yenus) # fuck.... not properly vectorized
        #np.allclose(Yned, Yned2)
        #breakpoint()
        Xs_ned = np.array([p_mctl.DiffFlatness.state_and_cmd_of_flat_output(Y, self.fdm.P)[0] for Y in Yned])
        eulers_ned = np.array([p_al.euler_of_quat(q) for q in Xs_ned[:, p_mfdm.sv_slice_quat]])
        eulers_enu = mu.euler_enu_of_ned(eulers_ned)
        rvel_frd = Xs_ned[:, p_mfdm.sv_slice_rvel]
        rvel_flu = np.zeros_like(eulers_ned)
        rvel_flu[:,0] = -rvel_frd[:,0]
        rvel_flu[:,1] = -rvel_frd[:,1]
        rvel_flu[:,2] = -rvel_frd[:,2]
        return time, pos, vel, eulers_enu, rvel_flu
