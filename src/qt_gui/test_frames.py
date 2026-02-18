#!/usr/bin/env python3

import sys, signal, numpy as np, matplotlib.pyplot as plt

from PySide6 import QtCore, QtWidgets, QtGui
import pat3.utils as p3_u, pat3.algebra as p3_al
import pat3.vehicles.rotorcraft.multirotor_trajectory as p3_mt
import pat3.vehicles.rotorcraft.multirotor_fdm as p3_mfdm
import pat3.vehicles.rotorcraft.multirotor_control as p3_mctl

import view_three_d, model, misc_utils as mu

#
#  Make sure we're not screwing frames (enu vs ned)
#

# body is body_fru (hypothese)
# so roll positive to left, pitch positive down, yaw positive left

def TofXned(Xned):
    T = np.eye(4)
    x, y, z = Xned[p3_mfdm.sv_slice_pos]
    T[:3,3] = y, x, -z
    rmat_ned2frd = p3_al.rmat_of_quat(Xned[p3_mfdm.sv_slice_quat])
    rmat_enu2ned = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
    rmat_frd2flu = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
    rmat_enu2flu = rmat_frd2flu@rmat_ned2frd@rmat_enu2ned
    rmat_enu2flu = rmat_enu2flu.T #np.linalg.inv(rmat_enu2flu) # same bug ?? rmat_enu2flu is really rmat_flu2enu ???
    T[:3,:3] = rmat_enu2flu
    
    return T

def pose_enu_from_ned(pos_ned, euler_ned2frd):   # works
    print(f'euler ned2frd: {np.rad2deg(euler_ned2frd)}')
    rmat_ned2frd = p3_al.rmat_of_euler(euler_ned2frd)
    rmat_enu2ned = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
    rmat_frd2flu = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
    rmat_enu2flu = rmat_frd2flu@rmat_ned2frd@rmat_enu2ned
    rmat_enu2flu = np.linalg.inv(rmat_enu2flu) # !!!! rmat_enu2flu is really rmat_flu2enu ???
    pos_enu = [pos_ned[1], pos_ned[0], -pos_ned[2]]
    T_enu2flu = np.eye(4)
    T_enu2flu[:3,3] = pos_enu
    T_enu2flu[:3,:3] = rmat_enu2flu
    return T_enu2flu
    
def pose_enu_from_Y_enu():
    Yenu = np.zeros((p3_mt._ylen, p3_mt._nder))
    Yenu[:p3_mt._z+1,0] = [0., 0., 1.] # up 1m
    #Yenu[:p3_mt._z+1,1] = [10., 0., 0.] # flying forward
    #Yenu[:p3_mt._z+1,1] = [0., 10., 0.] # flying left
    Yenu[p3_mt._psi,0] = np.pi/4 # flying north east
    print(f'Yenu\n{Yenu}')
    Yned = mu.Yenu2ned(Yenu)
    print(f'Yned\n{Yned}')
    fdm = p3_mfdm.MR_FDM()
    Xned, Ufrd, Xdned = p3_mctl.DiffFlatness.state_and_cmd_of_flat_output(Yned, fdm.P)
    print(f'Xned pos {Xned[p3_mfdm.sv_slice_pos]} m')
    print(f'Xned vel {Xned[p3_mfdm.sv_slice_vel]} m/s')
    print(f'Xned quat {Xned[p3_mfdm.sv_slice_quat]} ')
    print(f'Xned euler {np.rad2deg(p3_al.euler_of_quat(Xned[p3_mfdm.sv_slice_quat]))} deg')
    print(f'Xned rvel {np.rad2deg(Xned[p3_mfdm.sv_slice_rvel])} deg/s')
    Tenu2flu = TofXned(Xned)
    return Tenu2flu

def check_eulers():
    # Is this true ?
    # phi_enu = -phi_ned
    # theta_enu = -theta_ned
    # psi_enu = pi/4-psi_ned
    #
    eulers_ned2frd = np.deg2rad([10, 11, 12])
    rmat_ned2frd = p3_al.rmat_of_euler(eulers_ned2frd)

def check_psi_normalization(): # OK
    #psis = np.array([0, np.pi/4, np.pi/2, np.pi, 3*np.pi/2, 2*np.pi, 5*np.pi/2])
    psis = np.linspace(-5*np.pi, 5*np.pi, 100)

    psis1 = p3_u.norm_mpi_pi(psis)
    plt.plot(psis, '.')
    plt.plot(psis1, '.')
    plt.show()
    
if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    if 0:
        app = QtWidgets.QApplication([])
        #mod = model.Model()
        tdv = view_three_d.ThreeDWidget(model=None)
        tdv.setCameraPosition(distance=7)
        tdv.set_item_visible('frames', True)
        tdv.show_quad(True)
        tdv.resize(800, 600)
        tdv.show()

    if 0:
        position_ned = [0.5, 1, -0.2]
        euler_ned2frd = np.deg2rad([20, 0, 0])
        T_enu2flu = pose_enu_from_ned(position_ned, euler_ned2frd)
        tdv.set_quad_pose(T_enu2flu)
    if 0:
        Tenu2flu = pose_enu_from_Y_enu()
        tdv.set_quad_pose2(Tenu2flu)
    if 0:
        check_eulers()
    if 1:
        check_psi_normalization()
        
    sys.exit(app.exec())
