# import class_robot

import numpy as np
import time
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R


import sim
import numpy as np

class Robot:
    def __init__(self, frame_name, motor_names=None, propellers_names=None, client_id=0):
        # If there is an existing connection
        if client_id:
            self.client_id = client_id
        else:
            self.client_id = self.open_connection()

        self.motors = self._get_handlers(motor_names)
        self.propellers = self._get_handlers(propellers_names)

        # Robot frame
        self.frame = self._get_handler(frame_name)
        # self._cal_design_matrix()
        # if self.motors:
        #     self.inv_A = np.linalg.pinv(self.A)

    def open_connection(self):
        sim.simxFinish(-1)  # just in case, close all opened connections
        self.client_id = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to CoppeliaSim

        if self.client_id != -1:
            print('Robot connected')
        else:
            print('Connection failed')
        return self.client_id

    def close_connection(self):
        # self.actuate([0]*len(self.motors))
        # Before closing the connection of CoppeliaSim,
        # make sure that the last command sent out had time to arrive.
        sim.simxGetPingTime(self.client_id)
        sim.simxFinish(self.client_id)  # Now close the connection of CoppeliaSim:
        print('Connection closed')

    def is_connected(self):
        c, result = sim.simxGetPingTime(self.client_id)
        # Return true if the robot is connected
        return result > 0

    def _get_handler(self, name):
        err_code, handler = sim.simxGetObjectHandle(self.client_id, name, sim.simx_opmode_blocking)
        if err_code != 0:
            print("ERROR: CANNOT GET HANDLER FOR OBJECT '{}'".format(name))
        return handler

    def _get_handlers(self, names):
        handlers = []
        if names is not None:
            for name in names:
                handler = self._get_handler(name)
                handlers.append(handler)

        return handlers

    def send_motor_velocities(self, vels):
        for motor, vel in zip(self.motors, vels):
            err_code = sim.simxSetJointTargetVelocity(self.client_id,
                                                      motor, vel, sim.simx_opmode_streaming)

            if err_code != 0:
                print("ERROR: CANNOT SET MOTOR {} WITH VELOCITY {}".format(motor, vel))

    def set_position(self, position, relative_object=-1):
        # By default, get the position wrt the reference frame
        if relative_object != -1:
            relative_object = self._get_handler(relative_object)
        err_code = sim.simxSetObjectPosition(self.client_id, self.frame,
                                             relative_object, position, sim.simx_opmode_oneshot)
        if err_code != 0:
            print("ERROR: CANNOT SET POSITION W.R.T. {} TO {}".format(relative_object, position))

    def set_orientation(self, orientation, relative_object=-1):
        # By default, get the position wrt the reference frame
        if relative_object != -1:
            relative_object = self._get_handler(relative_object)
        err_code = sim.simxSetObjectOrientation(self.client_id, self.frame,
                                                relative_object, orientation, sim.simx_opmode_oneshot)
        if err_code != 0:
            print("ERROR: CANNOT SET ORIENTATION W.R.T. {} TO {}".format(relative_object, orientation))

    def sim_time(self):
        return sim.simxGetLastCmdTime(self.client_id)

    def get_position(self, relative_object=-1):
        # Get position relative to an object, -1 for global frame
        if relative_object != -1:
            relative_object = self._get_handler(relative_object)
        res, position = sim.simxGetObjectPosition(self.client_id, self.frame, relative_object, sim.simx_opmode_streaming)
        return np.array(position)

    def get_orientation(self, relative_object=-1):
        # Get orientation relative to an object, -1 for global frame
        if relative_object != -1:
            relative_object = self._get_handler(relative_object)
        res, euler = sim.simxGetObjectOrientation(self.client_id, self.frame, relative_object, sim.simx_opmode_streaming)
        return np.array(euler)


    def set_orientation_motors(self, quat):
        # Get orientation relative to an object, -1 for global frame

        for i in range(len(self.motors)):
            err_code = sim.simxSetObjectOrientation(self.client_id, self.motors[i], self.frame, quat[i],
                                                    sim.simx_opmode_blocking)
            err_code = sim.simxSetObjectOrientation(self.client_id, self.propellers[i], self.frame, quat[i],
                                                    sim.simx_opmode_blocking)
        if err_code != 0:
            print("ERROR: CANNOT SET ORIENTATION W.R.T. {} TO {}")

    def set_pos_motors(self, position):
        # Get orientation relative to an object, -1 for global frame

        for i in range(len(self.motors)):
            err_code = sim.simxSetObjectPosition(self.client_id, self.motors[i], self.frame, position[i],
                                                    sim.simx_opmode_blocking)
            err_code = sim.simxSetObjectPosition(self.client_id, self.propellers[i], self.frame, position[i],
                                                 sim.simx_opmode_blocking)
        if err_code != 0:
            print("ERROR: CANNOT SET ORIENTATION W.R.T. {} TO {}")

    def get_quaternion_motors(self):
        # Get orientation relative to an object, -1 for global frame
        quaternion = np.zeros((len(motors),4))
        for i in range(len(self.motors)):
            res, quaternion[i] = sim.simxGetObjectQuaternion(self.client_id, self.motors[i], self.frame, sim.simx_opmode_blocking)
        return np.array(quaternion)

    def get_pos_motors(self):
        # Get orientation relative to an object, -1 for global frame
        pos = np.zeros((len(motors),3))
        for i in range(len(self.motors)):
            res, pos[i] = sim.simxGetObjectPosition(self.client_id, self.motors[i], self.frame, sim.simx_opmode_blocking)
        return np.array(pos)

    def get_quaternion(self, relative_object=-1):
        # Get orientation in quaternion relative to an object, -1 for global frame
        if relative_object != -1:
            relative_object = self._get_handler(relative_object)
        res, quat = sim.simxGetObjectQuaternion(self.client_id, self.frame, relative_object, sim.simx_opmode_streaming)
        return np.array(quat)

    def get_velocity(self, relative_object=-1):
        # Get velocity relative to an object, -1 for global frame
        if relative_object != -1:
            relative_object = self._get_handler(relative_object)
        res, velocity, omega = sim.simxGetObjectVelocity(self.client_id, self.frame, sim.simx_opmode_streaming)
        return np.array(velocity), np.array(omega)

    def get_object_position(self, object_name):
        # Get Object position in the world frame
        err_code, object_h = sim.simxGetObjectHandle(self.client_id, object_name, sim.simx_opmode_streaming)
        res, position = sim.simxGetObjectPosition(self.client_id, object_h, -1, sim.simx_opmode_streaming)
        return np.array(position)

    def get_object_relative_position(self, object_name):
        # Get Object position in the robot frame
        err_code, object_h = sim.simxGetObjectHandle(self.client_id, object_name, sim.simx_opmode_streaming)
        res, position = sim.simxGetObjectPosition(self.client_id, object_h, self.frame, sim.simx_opmode_streaming)
        return np.array(position)

    def set_signal(self, signal, value):
        return sim.simxSetFloatSignal(self.client_id, signal, value, sim.simx_opmode_oneshot)

    def actuate(self, u):
        thruster_names = ['f{}'.format(i+1) for i in range(len(self.motors))]
        for fi, ui in zip(thruster_names, u):
            self.set_signal(fi, ui)


def vee_map(skew_s):
    return np.array([skew_s[2, 1], skew_s[0, 2], skew_s[1, 0]])

def null_space(A, rcond=None):
    u, s, vh = np.linalg.svd(A, full_matrices=True)
    M, N = u.shape[0], vh.shape[1]
    if rcond is None:
        rcond = np.finfo(s.dtype).eps * max(M, N)
    tol = np.amax(s) * rcond
    num = np.sum(s > tol, dtype=int)
    Q = vh[num:, :].T.conj()
    return Q

def quat2rot(quat):
    # Covert a quaternion into a full three-dimensional rotation matrix.
    # Extract the values from quat
    q0 = quat[3]
    q1 = quat[0]
    q2 = quat[1]
    q3 = quat[2]

    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)

    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)

    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1

    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])

    return rot_matrix

def trajectory(t):
    xd = np.array([0., 0.,0.,
                   0., 0.,0.,
                   0., 0.,0.])
#     xd = np.array([5 + 5*np.sin(t/2), 5 + 5*np.cos(t/2), np.pi/2*np.cos(t/2),
#                 5*np.cos(t/2)/2, -5*np.sin(t/2)/2    , -np.pi/2*np.sin(t/2)/2,
#                -5*np.sin(t/2)/4, -5*np.cos(t/2)/4    , -np.pi/2*np.cos(t/2)/4])
    return xd



motors=['propeller{:01d}'.format(i+1) for i in range(8)]

propellers = ['propeller_respondable{:01d}'.format(i + 1) for i in range(8)]

r = Robot('MultiRotor', motor_names=motors, propellers_names=propellers)
d = Robot('DesiredBox')

RR = np.array([[-np.pi/8, np.pi/8, 0],
               [-np.pi/8, -np.pi/8, 0],
               [np.pi/8, -np.pi/8, 0],
               [np.pi/8, np.pi/8, 0],
               [-np.pi/16, np.pi/16, 0],
               [-np.pi/16, -np.pi/16, 0],
               [np.pi/16, -np.pi/16, 0],
               [np.pi/16, np.pi/16, 0]])

pp = np.array([[ 0.22,        -0.22,         0.1],
             [-0.22,        -0.22,         0.1],
             [-0.22,         0.22,         0.1],
             [ 0.22,         0.22,         0.1],
             [ 0.22,        -0.22,        -0.1],
             [-0.22,        -0.22,        -0.1],
             [-0.22,         0.22,        -0.1],
             [ 0.22,         0.22,        -0.1]])

r.set_orientation_motors(RR)
r.set_pos_motors(pp)

print(r.get_quaternion_motors())
print(r.get_pos_motors())

design_matrix = np.zeros([6, len(motors)])
null_basis = np.zeros([6, 1])

km_over_kf = 0.0
A_matrix = np.zeros([6, len(motors)])

RR = r.get_quaternion_motors()

for i in range(len(motors)):
    # print(res)
    # euler_i = R.from_euler('xyz', RR[i])
    # euler_i = quat2rot(RR[i])
    R_i = quat2rot(RR[i])
    x_i = np.array(pp[i])
    f_i = R_i.dot(np.array([0, 0, 1]))
    tau_i = np.cross(x_i, f_i)
    A_matrix[:, i] = np.concatenate([f_i, tau_i])

if np.linalg.matrix_rank(design_matrix) == 6:
    print("Fully actuated!")
    ns = null_space(design_matrix)
    ns_std = np.inf
    for i in range(ns.shape[1]):
        if (ns[:, i] > 0).all():
            if ns_std == np.inf:
                print("Omnidirectional with unidirectional motors")
            if np.std(ns[:, i]) < ns_std:
                null_basis = ns[:, i]
                ns_std = np.std(ns[:, i])

inv_A = np.linalg.pinv(A_matrix)
ns = null_basis
print('Rank is:', np.linalg.matrix_rank(A_matrix))

m = 8.8  # kg mass of the block plus the mass of the prop
g = 9.81
inertia = 0.1

x_0 = np.array([0., 0., 0., 0., 0., 0.])


log_p = []
log_R = []
log_time = []
log_u = []
log_rpy = []
log_th = []
log_tor = []

try:
    cap_R_i = 5.0
    e_R_i = np.array([0.0, 0.0, 0.0])

    simulation_start = time.time()
    while True:
        time_start = time.time()
        d.set_position([0, 0, 1])
        d.set_orientation([20*np.pi/180,0,0])
        # d.set_orientation(np.array([time_start/10.0, 0, 0]))
        # Robot state
        p = r.get_position()
        v, omega = r.get_velocity()
        rpy = r.get_orientation()
        # quaternion = R.from_quat(r.get_quaternion())
        R_c = quat2rot(r.get_quaternion())
        # quaternion = R.from_quat(d.get_quaternion())
        R_d = quat2rot(d.get_quaternion())

        # Desired state
        p_d = d.get_position()
        v_d, omega_d = d.get_velocity()
        rpy_d = d.get_orientation()

        # errors
        epos = p_d - p
        evel = v_d - v

        e_angle = rpy_d - rpy
        e_omega = omega_d - omega

        kp1 = np.array([1, 1, 1])
        kd1 = np.array([1, 1, 1])

        kp2 = np.array([3, 3, 3])
        kd2 = np.array([4, 4, 4])

        u_f = kp1 * epos.T + kd1 * evel.T + np.array([0., 0., m * g])

        u_tao = kp2 * e_angle.T + kd2 * e_omega.T

        uT = np.hstack((R_c.T.dot(u_f), inertia * u_tao))


        #     print(uT)

        a = inv_A.dot(uT)

        r.actuate(a.tolist())

        log_p.append(epos)
        # log_R.append(eR)
        log_rpy.append(e_angle)
        log_u.append(uT)
        log_time.append(time.time() - simulation_start)
        # print(time.time() - time_start)
        while time.time() - time_start < 0.05:
            time.sleep(0.001)

except KeyboardInterrupt:
    r.close_connection()


r.close_connection()

log_p = np.array(log_p)
log_R = np.array(log_R)
log_rpy = np.array(log_rpy)
log_u = np.array(log_u)
log_th = np.array(log_th)
log_tor = np.array(log_tor)
log_time = np.array(log_time)

fig = plt.figure()
ax1 = fig.add_subplot(311)
ax1.grid()
ax1.set_xlabel('time (s)')
ax1.set_ylabel('position error (m)')
ax1.set_title('position error v.s. time')
ax1.plot(log_time, log_p[:, 0], label='$e_x$')
ax1.plot(log_time, log_p[:, 1], label='$e_y$')
ax1.plot(log_time, log_p[:, 2], label='$e_z$')
ax1.legend()

ax2 = fig.add_subplot(312)
ax2.grid()
ax2.set_xlabel('time (s)')
ax2.set_title('orientation error v.s. time')
ax2.plot(log_time, log_rpy[:, 0], label='$e_{roll}$')
ax2.plot(log_time, log_rpy[:, 1], label='$e_{pitch}$')
ax2.plot(log_time, log_rpy[:, 2], label='$e_{yaw}$')
ax2.legend()
#
# ax3 = fig.add_subplot(234)
# ax3.grid()
# ax3.set_xlabel('time (s)')
# ax3.set_title('input forces v.s. time')
# for i in range(len(r.motors)):
#     ax3.plot(log_time, log_u[:, i], label='$u_{}$'.format(i+1))
# ax3.legend()
#
# ax4 = fig.add_subplot(235)
# ax4.grid()
# ax4.set_xlabel('time (s)')
# ax4.set_title('rpy error v.s. time')
# ax4.plot(log_time, log_rpy[:, 0], label='$e_{roll}$')
# ax4.plot(log_time, log_rpy[:, 1], label='$e_{pitch}$')
# ax4.plot(log_time, log_rpy[:, 2], label='$e_{yaw}$')
# ax4.legend()
#
# ax5 = fig.add_subplot(233)
# ax5.grid()
# ax5.set_xlabel('time (s)')
# ax5.set_title('des thrust in W v.s. time')
# ax5.plot(log_time, log_th[:, 0], label='$f_x$')
# ax5.plot(log_time, log_th[:, 1], label='$f_y$')
# ax5.plot(log_time, log_th[:, 2], label='$f_z$')
# ax5.legend()
#
# ax4 = fig.add_subplot(236)
# ax4.grid()
# ax4.set_xlabel('time (s)')
# ax4.set_title('des torque in B v.s. time')
# ax4.plot(log_time, log_tor[:, 0], label='$tau_{roll}$')
# ax4.plot(log_time, log_tor[:, 1], label='$tau_{pitch}$')
# ax4.plot(log_time, log_tor[:, 2], label='$tau_{yaw}$')
# ax4.legend()

plt.show()