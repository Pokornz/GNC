from scipy import linalg as la
import matplotlib.pyplot as pl
import numpy as np
import quadrotor as quad
import quadlog
import animation as ani
from time import time as tm

# Quadrotor
m = 0.65 # Kg
l = 0.23 # m
Jxx = 7.5e-3 # Kg/m^2
Jyy = Jxx
Jzz = 1.3e-2
Jxy = 0
Jxz = 0
Jyz = 0
J = np.array([[Jxx, Jxy, Jxz], \
              [Jxy, Jyy, Jyz], \
              [Jxz, Jyz, Jzz]])
CDl = 9e-3
CDr = 9e-4
kt = 3.13e-5  # Ns^2
km = 7.5e-7   # Ns^2
kw = 1/0.18   # rad/s

# Initial conditions
att_0 = np.array([0.0, 0.0, 0.0])
pqr_0 = np.array([0.0, 0.0, 0.0])
xyz1_0 = np.array([1.0, 1.2, 0.0])
xyz2_0 = np.array([1.2, 2.0, 0.0])
xyz3_0 = np.array([-1.1, 2.6, 0.0])
# xyz1_0 = np.array([0.0, 0.0, 0.0])
# xyz2_0 = np.array([0.0, 0.0, 0.0])
# xyz3_0 = np.array([0.0, 0.0, 0.0])
v_ned_0 = np.array([0.0, 0.0, 0.0])
w_0 = np.array([0.0, 0.0, 0.0, 0.0])

# Formation information
B = np.array([[-1, 0],
              [ 0, 1],
              [ 1,-1]])
D_e = np.array([[2],
                [2]])
D_n = np.array([[ 3.464],
                [-3.464]])

# Simulation parameters
tf = 400
dt = 1e-2
time = np.linspace(0, tf, tf/dt)
it = 0
frames = 1000

# Setting quads
q1 = quad.quadrotor(1, dt, m, l, J, CDl, CDr, kt, km, kw, \
        att_0, pqr_0, xyz1_0, v_ned_0, w_0, B, D_e, D_n)

q2 = quad.quadrotor(2, dt, m, l, J, CDl, CDr, kt, km, kw, \
        att_0, pqr_0, xyz2_0, v_ned_0, w_0, B, D_e, D_n)

q3 = quad.quadrotor(3, dt, m, l, J, CDl, CDr, kt, km, kw, \
        att_0, pqr_0, xyz3_0, v_ned_0, w_0, B, D_e, D_n)



# Data log
q1_log = quadlog.quadlog(time)
q2_log = quadlog.quadlog(time)
q3_log = quadlog.quadlog(time)

# Plots
quadcolor = ['r', 'g', 'b']
pl.close("all")
pl.ion()
fig = pl.figure(0)
axis3d = fig.add_subplot(111, projection='3d')

init_area = 5
s = 2

# Desired altitude and heading
v_2D_d = np.array([0, 0])
alt_d = -4
# q1.yaw_d =  0
timer = tm()
for t in time:

    # Simulation
    # bias1 = np.array([q1.c_q[2][0], q1.c_q[2][0]])
    # bias2 = np.array([q2.c_q[2][0], q2.c_q[2][0]])
    # bias3 = np.array([q3.c_q[2][0], q3.c_q[2][0]])
    #
    # q1.set_v_2D_alt_lya(v_2D_d - bias1, alt_d)
    # q2.set_v_2D_alt_lya(v_2D_d - bias2, alt_d)
    # q3.set_v_2D_alt_lya(v_2D_d - bias3, alt_d)
    n_all = np.array([q1.xyz[0], q2.xyz[0], q3.xyz[0]])
    e_all = np.array([q1.xyz[1], q2.xyz[1], q3.xyz[1]])
    
    p_n = np.array([[q1.xyz[0]],
                    [q2.xyz[0]],
                    [q3.xyz[0]]])
    p_e = np.array([[q1.xyz[1]],
                    [q2.xyz[1]],
                    [q3.xyz[1]]])
    
    
    # bias = np.array([q1.c_q[2][0], q1.c_q[2][0]])
    # q1.set_v_2D_alt_lya(v_2D_d - bias,alt_d)
    q1.set_auto_formation(p_n, p_e, alt_d)
    q2.set_auto_formation(p_n, p_e, alt_d)
    q3.set_auto_formation(p_n, p_e, alt_d)

    q1.step(dt)
    q2.step(dt)
    q3.step(dt)

    # Animation
    if it%frames == 0:

        pl.figure(0)
        axis3d.cla()
        ani.draw3d(axis3d, q1.xyz, q1.Rot_bn(), quadcolor[0])
        ani.draw3d(axis3d, q2.xyz, q2.Rot_bn(), quadcolor[1])
        ani.draw3d(axis3d, q3.xyz, q3.Rot_bn(), quadcolor[2])
        axis3d.set_xlim(-5, 5)
        axis3d.set_ylim(-5, 5)
        axis3d.set_zlim(0, 10)
        axis3d.set_xlabel('South [m]')
        axis3d.set_ylabel('East [m]')
        axis3d.set_zlabel('Up [m]')
        axis3d.set_title("Time %.3f s" %t)
        # pl.pause(0.001)
        pl.draw()
        #namepic = '%i'%it
        #digits = len(str(it))
        #for j in range(0, 5-digits):
        #    namepic = '0' + namepic
        #pl.savefig("./images/%s.png"%namepic)
        
        pl.figure(1)
        pl.clf()
        for i in range(len(quadcolor)):
            pl.plot(e_all[i], n_all[i], 'o' + quadcolor[i])
        # ani.draw2d(1, X, fc, quadcolor)
        # ani.draw_edges(1, X, fc, -1)
        pl.plot(np.array([q1.xyz[1], q2.xyz[1], q3.xyz[1],q1.xyz[1]]),np.array([q1.xyz[0], q2.xyz[0], q3.xyz[0],q1.xyz[0]]), 'k--', lw=2)
        pl.xlabel('South [m]')
        pl.ylabel('West [m]')
        pl.title('2D Map')
        pl.axis('equal')
        pl.axis([-s*init_area, s*init_area, -s*init_area, s*init_area])
        # pl.xlim(-s*init_area, s*init_area)
        # pl.ylim(-s*init_area, s*init_area)
        pl.grid()
        pl.pause(0.001)
        pl.draw()

    q1p = np.array([q1.xyz[0], q1.xyz[1]])
    q2p = np.array([q2.xyz[0], q2.xyz[1]])
    q3p = np.array([q3.xyz[0], q3.xyz[1]])
    
    # Log
    q1_log.xyz_h[it, :] = q1.xyz
    q1_log.d[it] = np.linalg.norm(q2p - q1p)
    q1_log.w_h[it, :] = q1.w
    q1_log.v_ned_h[it, :] = q1.dB
    q1_log.b_h[it] = q1.c_q[2][0]

    q2_log.xyz_h[it, :] = q2.xyz
    q2_log.d[it] = np.linalg.norm(q3p - q2p)
    q2_log.w_h[it, :] = q2.w
    q2_log.v_ned_h[it, :] = q2.dB
    q2_log.b_h[it] = q2.c_q[2][0]

    q3_log.xyz_h[it, :] = q3.xyz
    q3_log.d[it] = np.linalg.norm(q1p - q3p)
    q3_log.w_h[it, :] = q3.w
    q3_log.v_ned_h[it, :] = q3.dB
    q3_log.b_h[it] = q3.c_q[2][0]


    it+=1
    
    # Stop if crash
    if (q1.crashed == 1 or q2.crashed == 1 or q3.crashed == 1):
        break
print(tm()-timer)
pl.figure(1)
pl.clf()
for i in range(len(quadcolor)):
    pl.plot(e_all[i], n_all[i], 'o' + quadcolor[i])
pl.plot(q1_log.xyz_h[:, 1], q1_log.xyz_h[:, 0], label="q1", color=quadcolor[0])
pl.plot(q2_log.xyz_h[:, 1], q2_log.xyz_h[:, 0], label="q2", color=quadcolor[1])
pl.plot(q3_log.xyz_h[:, 1], q3_log.xyz_h[:, 0], label="q3", color=quadcolor[2])
pl.plot(np.array([q1.xyz[1], q2.xyz[1], q3.xyz[1], q1.xyz[1]]), np.array([q1.xyz[0], q2.xyz[0], q3.xyz[0], q1.xyz[0]]),
        'k--', lw=2)
pl.xlabel("South")
pl.ylabel("West")
pl.title("2D Position [m]")
pl.axis('equal')
pl.axis([-s*init_area, s*init_area, -s*init_area, s*init_area])
pl.grid()
pl.legend()

pl.figure(2)
pl.plot(time, q1_log.v_ned_h[:, 0], label="q1",color=quadcolor[0])
pl.plot(time, q2_log.v_ned_h[:, 0], label="q2",color=quadcolor[1])
pl.plot(time, q3_log.v_ned_h[:, 0], label="q3",color=quadcolor[2])
pl.xlabel("Time [s]")
pl.ylabel("Derivation of the estimated bias [m/s^2]")
pl.grid()
pl.legend()

pl.figure(3)
pl.plot(time, q1_log.d[:], label="$||q_2 - q_1||$", color='y')
pl.plot(time, q2_log.d[:], label="$||q_3 - q_2||$", color='c')
pl.plot(time, q3_log.d[:], label="$||q_1 - q_3||$", color='m')
pl.xlabel("Time [s]")
pl.ylabel("Triangle sides lengths [m]")
pl.grid()
pl.legend(loc=2)

pl.figure(4)
pl.plot(time, q1_log.b_h, label="bias q1", color=quadcolor[0])
pl.plot(time, q2_log.b_h, label="bias q2", color=quadcolor[1])
pl.plot(time, q3_log.b_h, label="bias q3", color=quadcolor[2])
pl.xlabel("Time [s]")
pl.ylabel("Estimated bias [m/s]")
pl.grid()
pl.legend()


print(np.linalg.norm(q2p-q1p))
print(np.linalg.norm(q3p-q2p))
print(np.linalg.norm(q1p-q3p))

pl.pause(0)
