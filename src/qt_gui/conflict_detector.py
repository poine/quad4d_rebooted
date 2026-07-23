

import numpy as np


class ConflictDetector:
    def __init__(self, safety_distance=1.2, time_step=0.1):
        self.safety_distance = safety_distance
        self.time_step = time_step

    def check_scenario(self, drones_dict, duration=None):
        if not drones_dict or len(drones_dict) < 2:
            return True, []  # No conflict here since it's only one drone or none

        if duration is None: 
            duration = ([getattr(traj, 'duration', 21.0) for traj in drones_dict.values()])
            max_duration = max(duration)

        conflicts = []
        drones_ids = list(drones_dict.keys())

        for t in np.arange(0, max_duration, self.time_step):
            positions = {}
            for d_id in drones_ids:
                traj = drones_dict[d_id]
                pos_3d = traj.get(t)[:3, 0]
                positions[d_id] = np.array(pos_3d)

            for i in range(len(drones_ids)):
                for j in range(i + 1, len(drones_ids)):
                    id_A = drones_ids[i]
                    id_B = drones_ids[j]
                    distance = np.linalg.norm(positions[id_A] - positions[id_B])
                    if distance < self.safety_distance:
                        conflicts.append((id_A, id_B, t))

        sans_conflict = (len(conflicts) == 0)
        return sans_conflict, conflicts


    
