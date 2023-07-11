import numpy as np
from PyQt5.QtCore import QThread, pyqtSignal, QRunnable
import time
import socket
from pyquaternion import Quaternion
import imufusion # for attitude calculation
from madgwickahrs import MadgwickAHRS

## this code definitely works for udp comms with unity
# import socket
#
# UDP_IP = "127.0.0.1"
# UDP_PORT = 11000
# MESSAGE = b"hello from PyCharm IMUs!"
#
# print("UDP target IP: %s" % UDP_IP)
# print("UDP target port: %s" % UDP_PORT)
# print("message: %s" % MESSAGE)
#
# sock = socket.socket(socket.AF_INET,  # Internet
#                  socket.SOCK_DGRAM)  # UDP
# sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
###############################

class Thread_UDPUnity(QThread):
    # IMU_Data_Slot = pyqtSignal(list)
    def __init__(self):
        super(Thread_UDPUnity, self).__init__()
        # udp address and port number
        # self.udp_addr = ('192.168.100.2', 56688)
        self.udp_addr = ('127.0.0.1', 11000)
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        #########
        # # testing with imufusion package
        # Instantiate algorithms
        self.sample_rate = 100
        self.offset = imufusion.Offset(self.sample_rate)
        self.ahrs = imufusion.Ahrs()
        self.ahrs.settings = imufusion.Settings(imufusion.CONVENTION_NWU,  # convention
                                           0.5,  # gain
                                           10,  # acceleration rejection
                                           20,  # magnetic rejection
                                           5 * self.sample_rate)  # rejection timeout = 5 seconds
        self.delta_time = 0.01  # no timestamp so assuming delta is 0.01
        ########
        # # test using madgwick package
        # self.madgwickahrs = MadgwickAHRS(sampleperiod=1/100)

    def setIdentity(self, text):
        self.identity = text

    def IMUtoPRY(self, IMU_Data):
        self.Pitch = IMU_Data[1]
        self.Roll = IMU_Data[2]
        self.Yaw = IMU_Data[3]

    def attitude_calc(self, IMU_Data):
        self.accelerometer = np.asarray(IMU_Data[1:4])
        self.gyroscope = np.asarray(IMU_Data[4:7])
        self.magnetometer = np.asarray(IMU_Data[7:10])
        # perform correction and update values
        self.gyroscope = self.offset.update(self.gyroscope)
        self.ahrs.update(self.gyroscope, self.accelerometer, self.magnetometer, self.delta_time)


    def run(self):
        # print(self.ahrs.quaternion.x)
        # print(self.ahrs.quaternion.y)
        # print(self.ahrs.quaternion.z)
        # print(self.ahrs.quaternion.w)
        self.udp_data='X' + format(float(self.ahrs.quaternion.x), '.4f') + 'Y' + format(float(self.ahrs.quaternion.y), '.4f') + 'Z' + format(float(self.ahrs.quaternion.z), '.4f') + 'W' + format(float(self.ahrs.quaternion.w), '.4f') + 'E'
        self.udp_socket.sendto(self.udp_data.encode('utf-8'), self.udp_addr)
        # # already extracted the correct IMU data
        # # Give the attitude angles of head under our frame. Frame axes orientation: X-forward, Y-left, Z-upward
        # self.Pitch_rad = self.Pitch / 180 * np.pi
        # self.Roll_rad = self.Roll / 180 * np.pi
        # self.Yaw_rad = self.Yaw / 180 * np.pi
        # # Orientation matrix under our frame, deduced by attitude angles
        # self.T_Nr2Br = np.array([[np.cos(self.Pitch_rad) * np.cos(self.Yaw_rad),
        #                              np.cos(self.Pitch_rad) * np.sin(self.Yaw_rad),
        #                              -np.sin(self.Pitch_rad)],
        #                             [-np.cos(self.Roll_rad) * np.sin(self.Yaw_rad) + np.sin(self.Roll_rad) * np.sin(
        #                                 self.Pitch_rad) * np.cos(self.Yaw_rad),
        #                              np.cos(self.Roll_rad) * np.cos(self.Yaw_rad) + np.sin(self.Roll_rad) * np.sin(
        #                                  self.Pitch_rad) * np.sin(self.Yaw_rad),
        #                              np.sin(self.Roll_rad) * np.cos(self.Pitch_rad)],
        #                             [np.sin(self.Roll_rad) * np.sin(self.Yaw_rad) + np.cos(self.Roll_rad) * np.sin(
        #                                 self.Pitch_rad) * np.cos(self.Yaw_rad),
        #                              -np.sin(self.Roll_rad) * np.cos(self.Yaw_rad) + np.cos(self.Roll_rad) * np.sin(
        #                                  self.Pitch_rad) * np.sin(self.Yaw_rad),
        #                              np.cos(self.Roll_rad) * np.cos(self.Pitch_rad)]])
        # # Rotation matrix from our frame (X-forward, Y-left, Z-upward) to that used in Unity
        # self.T_Nl2Nr = np.array([[0, 0, 1], [-1, 0, 0], [0, 1, 0]])
        # self.T_Br2Bl = np.array([[0, -1, 0], [0, 0, 1], [1, 0, 0]])
        # # Orientation matrix under Unity frame
        # self.T_Nl2Bl = self.T_Br2Bl.dot(self.T_Nr2Br.dot(self.T_Nl2Nr))
        # # Orientation represented in quaternions
        # self.qw = ((self.T_Nl2Bl[0][0] + self.T_Nl2Bl[1][1] + self.T_Nl2Bl[2][2] + 1)**0.5)/2
        # self.qx = (self.T_Nl2Bl[2][1] - self.T_Nl2Bl[1][2]) / (4 * self.qw)
        # self.qy = (self.T_Nl2Bl[0][2] - self.T_Nl2Bl[2][0]) / (4 * self.qw)
        # self.qz = (self.T_Nl2Bl[1][0] - self.T_Nl2Bl[0][1]) / (4 * self.qw)
        #
        # self.qw_Unity = -self.qw
        # self.qx_Unity = self.qx
        # self.qy_Unity = self.qy
        # self.qz_Unity = self.qz
        #
        # # Send the quaternions to Unity
        # self.udp_data='X' + format(float(self.qx_Unity), '.4f') + 'Y' + format(float(self.qy_Unity), '.4f') + 'Z' + format(float(self.qz_Unity), '.4f') + 'W' + format(float(self.qw_Unity), '.4f') + 'E'
        # self.udp_socket.sendto(self.udp_data.encode('utf-8'), self.udp_addr)