import logging, yaml
import numpy as np

import pat3.algebra as p_al, pat3.trajectory_1D as p_t1d
import pat3.vehicles.rotorcraft.multirotor_trajectory as p_tm
import pat3.vehicles.rotorcraft.multirotor_trajectory_dev as p_tmdev
import pat3.vehicles.rotorcraft.multirotor_fdm as p_mfdm
import pat3.vehicles.rotorcraft.multirotor_control as p_mctl

import traj_factory, misc_utils as mu

from conflict_detector import ConflictDetector

logger = logging.getLogger(__name__)

class Arena:
    def __init__(self, cfg_file=None):
        self.gates = []
        if cfg_file is not None: self.load_cfg(cfg_file)
        else:
            self.extends = ((-5, 5), (-5, 5), (0, 8.))
            self.safe_extends = ((-4, 4), (-4, 4), (1, 7.))
        
    def add_gate(self, name, Tw2g, dim, texture):
        self.gates.append({'name':name, 'pose':Tw2g, 'dim': dim, 'texture':texture})

    def load_cfg(self, filename):
        with open(filename, "r") as file:
            config = yaml.safe_load(file)
        for key in config:
            if key=='extends':
                self.extends = np.array(config[key])
            if key=='safe_extends':
                self.safe_extends = np.array(config[key])
            elif key=='gates':
                for g_name in config[key]:
                    cg = config[key][g_name]
                    Tw2g = mu.T_of_t_euler(cg['pos'], cg['rot'], degrees=True)
                    self.add_gate(g_name, Tw2g, cg['dim'], cg['text'])
        
class Model:
    def __init__(self, traj_fact_id=None, trajectory=None, arena_cfg=None):
        self.trajectories = []
        
        if trajectory is not None:
            self.trajectories.append(trajectory)
        if traj_fact_id is not None:
            self.load_from_factory(traj_fact_id)
        self.fdm = p_mfdm.MR_FDM()
        self.arena = Arena(arena_cfg)
        self.conflict_detector = ConflictDetector(safety_distance=1.2)
       
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
            
    def detect_conflicts(self, safety_distance=1.0, step=0.1):
        """ duration = self.trajectory_duration()
        conflicts = []
        drones_indices = range(self.trajectory_nb())

        for t in np.arange(0, duration, step):
            positions = {}
            for idx in drones_indices:
                pos = self.get_traj_output_at(t, idx)[:3, 0]
                positions[idx] = pos
            for i in range(len(drones_indices)):
                for j in range(i + 1, len(drones_indices)):
                    idA, idB = drones_indices[i], drones_indices[j]
                    if np.linalg.norm(positions[idA] - positions[idB]) < safety_distance:
                        conflicts.append({'t': t, 'idA': idA, 'idB': idB})
        return conflicts """

        det = self.conflict_detector
        det.safety_distance, det.time_step = safety_distance, step
        drones_dict = {idx: self.get_trajectory(idx) for idx in range(self.trajectory_nb())}
        _ok, pairs = det.check_scenario(drones_dict)
        return [{'t': t, 'idA': a, 'idB': b} for (a, b, t) in pairs]

    def resolve_conflicts(self, safety_distance=1.0, step=0.1, delay_increment=2.0):
        #first idea: ddecaller le depart des drones pour eviter les conflits
        max_attempts = 10
        for attempts in range(max_attempts):
            conflicts = self.detect_conflicts(safety_distance, step)
            if not conflicts:
                logger.info("safe scenario")
                return True
            conflicts.sort(key=lambda x: x['t'])
            first_conflict = conflicts[0] 

            t_conflict = first_conflict['t']
            idA = first_conflict['idA']
            idB = first_conflict['idB']
            traj= self.trajectories[idB]

            if isinstance(traj, DelayedTrajectory):
                new_delay = traj.delay + delay_increment
                self.trajectories[idB] = DelayedTrajectory(traj.original_traj, new_delay)
            else:
                self.trajectories[idB] = DelayedTrajectory(traj, delay_increment)
        logger.error("Can't resolve all the conflicts after 10 attempts")
        return False

class DelayedTrajectory:
    
    def __init__(self, original_traj, delay):
        self.original_traj = original_traj
        self.delay = delay
        self.duration = getattr(original_traj, 'duration', 20.0) + delay
        
    def get(self, t):
        if t < self.delay:
            return self.original_traj.get(0)
        return self.original_traj.get(t - self.delay)

    def __getattr__(self, name):
        return getattr(self.original_traj, name)
