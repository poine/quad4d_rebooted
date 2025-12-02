#! /usr/bin/env python3

import math, numpy as np, matplotlib.pyplot as plt

import pat3.vehicles.rotorcraft.multirotor_trajectory as trj
import pat3.vehicles.rotorcraft.multirotor_trajectory_dev as trj_dev

def main():
    #/home/poine/work/pat/src/pat3/test/rotorcraft/test_05_space_index.py

    wps = [[0, 0, 0], [1, 1, 0.5], [0, 2, 0], [1, 3, 0.5], [0, 4, 0]]
    straj = trj_dev.SpaceWaypoints(wps)

    lambdas = time = np.arange(0, 1., 0.01)

    Ys = np.array([ straj.get(_l) for _l in lambdas])

    plt.plot(lambdas, Ys[:,0,0])
    #breakpoint()
    plt.show()
    
if __name__ == "__main__":
    np.set_printoptions(linewidth=500)
    main()
