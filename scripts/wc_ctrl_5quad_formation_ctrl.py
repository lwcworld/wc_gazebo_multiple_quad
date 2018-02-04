#!/usr/bin/env python
import numpy as np
import math
import rospy
from std_msgs.msg import Header

from tf.transformations import quaternion_from_euler, euler_from_quaternion

from threading import Thread, Event

# msgs
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import OverrideRCIn
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, Quaternion, Pose, PoseWithCovarianceStamped, TwistStamped
from sensor_msgs.msg import NavSatFix
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

# about QTGUI
import sys
from PyQt5 import QtWidgets
from PyQt5 import uic
from PyQt5 import QtGui
from PyQt5 import QtCore
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import QGraphicsScene, QGraphicsView, QGraphicsPixmapItem

# filter
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import sigma_points

def local_position_callback_1(msg):
    # print(msg)
    agent1.position = msg

def local_position_callback_2(msg):
    agent2.position = msg
    agent2.position.pose.position.x = agent2.position.pose.position.x + 1
    agent2.position.pose.position.y = agent2.position.pose.position.y + 1

def local_position_callback_3(msg):
    agent3.position = msg
    agent3.position.pose.position.x = agent3.position.pose.position.x + 1
    agent3.position.pose.position.y = agent3.position.pose.position.y - 1

def local_position_callback_4(msg):
    agent4.position = msg
    agent4.position.pose.position.x = agent4.position.pose.position.x + 2
    agent4.position.pose.position.y = agent4.position.pose.position.y + 2

def local_position_callback_5(msg):
    agent5.position = msg
    agent5.position.pose.position.x = agent5.position.pose.position.x + 2
    agent5.position.pose.position.y = agent5.position.pose.position.y - 2

def local_velocity_callback_1(msg):
    agent1.velocity = msg
    agent1.vel_x = agent1.velocity.twist.linear.x
    agent1.vel_y = agent1.velocity.twist.linear.y
    agent1.vel_z = agent1.velocity.twist.linear.z

def local_velocity_callback_2(msg):
    agent2.velocity = msg
    agent2.vel_x = agent2.velocity.twist.linear.x
    agent2.vel_y = agent2.velocity.twist.linear.y
    agent2.vel_z = agent2.velocity.twist.linear.z

def local_velocity_callback_3(msg):
    agent3.velocity = msg
    agent3.vel_x = agent3.velocity.twist.linear.x
    agent3.vel_y = agent3.velocity.twist.linear.y
    agent3.vel_z = agent3.velocity.twist.linear.z

def local_velocity_callback_4(msg):
    agent4.velocity = msg
    agent4.vel_x = agent4.velocity.twist.linear.x
    agent4.vel_y = agent4.velocity.twist.linear.y
    agent4.vel_z = agent4.velocity.twist.linear.z

def local_velocity_callback_5(msg):
    agent5.velocity = msg
    agent5.vel_x = agent5.velocity.twist.linear.x
    agent5.vel_y = agent5.velocity.twist.linear.y
    agent5.vel_z = agent5.velocity.twist.linear.z

# data
class Data_agent(object):
    def __init__(self, idx_uav):
        if idx_uav == 1:
            self.arm  = rospy.ServiceProxy('/uav1/mavros/cmd/arming', CommandBool)
            self.mode = rospy.ServiceProxy('/uav1/mavros/set_mode', SetMode)
            self.pub_att = rospy.Publisher('/uav1/mavros/setpoint_attitude/attitude', PoseStamped, queue_size=10)
            self.pub_thr = rospy.Publisher('/uav1/mavros/setpoint_attitude/att_throttle', Float64, queue_size=10)
            rospy.Subscriber("/uav1/mavros/local_position/pose", PoseStamped, local_position_callback_1)
            rospy.Subscriber("/uav1/mavros/local_position/velocity", TwistStamped, local_velocity_callback_1)
            self.des_x = 0
            self.des_y = 0
            self.des_z = 3

            self.formation_heading = 0
            self.formation_velocity = 0

        elif idx_uav == 2:
            self.arm  = rospy.ServiceProxy('/uav2/mavros/cmd/arming', CommandBool)
            self.mode = rospy.ServiceProxy('/uav2/mavros/set_mode', SetMode)
            self.pub_att = rospy.Publisher('/uav2/mavros/setpoint_attitude/attitude', PoseStamped, queue_size=10)
            self.pub_thr = rospy.Publisher('/uav2/mavros/setpoint_attitude/att_throttle', Float64, queue_size=10)
            rospy.Subscriber("/uav2/mavros/local_position/pose", PoseStamped, local_position_callback_2)
            rospy.Subscriber("/uav2/mavros/local_position/velocity", TwistStamped, local_velocity_callback_2)
            self.des_x = 1
            self.des_y = 1
            self.des_z = 3

        elif idx_uav == 3:
            self.arm  = rospy.ServiceProxy('/uav3/mavros/cmd/arming', CommandBool)
            self.mode = rospy.ServiceProxy('/uav3/mavros/set_mode', SetMode)
            self.pub_att = rospy.Publisher('/uav3/mavros/setpoint_attitude/attitude', PoseStamped, queue_size=10)
            self.pub_thr = rospy.Publisher('/uav3/mavros/setpoint_attitude/att_throttle', Float64, queue_size=10)
            rospy.Subscriber("/uav3/mavros/local_position/pose", PoseStamped, local_position_callback_3)
            rospy.Subscriber("/uav3/mavros/local_position/velocity", TwistStamped, local_velocity_callback_3)
            self.des_x = 1
            self.des_y = -1
            self.des_z = 3

        elif idx_uav == 4:
            self.arm  = rospy.ServiceProxy('/uav4/mavros/cmd/arming', CommandBool)
            self.mode = rospy.ServiceProxy('/uav4/mavros/set_mode', SetMode)
            self.pub_att = rospy.Publisher('/uav4/mavros/setpoint_attitude/attitude', PoseStamped, queue_size=10)
            self.pub_thr = rospy.Publisher('/uav4/mavros/setpoint_attitude/att_throttle', Float64, queue_size=10)
            rospy.Subscriber("/uav4/mavros/local_position/pose", PoseStamped, local_position_callback_4)
            rospy.Subscriber("/uav4/mavros/local_position/velocity", TwistStamped, local_velocity_callback_4)
            self.des_x = 2
            self.des_y = 2
            self.des_z = 3

        elif idx_uav == 5:
            self.arm  = rospy.ServiceProxy('/uav5/mavros/cmd/arming', CommandBool)
            self.mode = rospy.ServiceProxy('/uav5/mavros/set_mode', SetMode)
            self.pub_att = rospy.Publisher('/uav5/mavros/setpoint_attitude/attitude', PoseStamped, queue_size=10)
            self.pub_thr = rospy.Publisher('/uav5/mavros/setpoint_attitude/att_throttle', Float64, queue_size=10)
            rospy.Subscriber("/uav5/mavros/local_position/pose", PoseStamped, local_position_callback_5)
            rospy.Subscriber("/uav5/mavros/local_position/velocity", TwistStamped, local_velocity_callback_5)
            self.des_x = 2
            self.des_y = -2
            self.des_z = 3

        self.roll_cmd = 0
        self.pitch_cmd = 0
        self.yaw_cmd = 0
        self.throttle_cmd = 0

        self.position = PoseStamped()
        self.velocity = TwistStamped()
        self.vel_x = 0
        self.vel_y = 0
        self.vel_z = 0

class Data_FDI(object):
    def __init__(self):
        self.FDI_on = False

        self.m = 2
        self.n = 5

        m = self.m
        n = self.n

        ## Graph Theory
        # Adjacency Matrix
        self.AdjM = np.ones((n, n)) - np.eye(n);

        # Degree Matrix
        self.DegM = (n - 1) * np.eye(n);

        # Laplacian matrix
        self.L = self.DegM - self.AdjM;

    def get_sensor_data(self):
        GPS_Meas = np.array([agent1.position.pose.position.x, agent1.position.pose.position.y,
                      agent2.position.pose.position.x, agent2.position.pose.position.y,
                      agent3.position.pose.position.x, agent3.position.pose.position.y,
                      agent4.position.pose.position.x, agent4.position.pose.position.y,
                      agent5.position.pose.position.x, agent5.position.pose.position.y])

        ACC_Meas = np.array([agent1.vel_x, agent1.vel_y,
                      agent2.vel_x, agent2.vel_y,
                      agent3.vel_x, agent3.vel_y,
                      agent4.vel_x, agent4.vel_y,
                      agent5.vel_x, agent5.vel_y])

        z = np.concatenate((GPS_Meas,ACC_Meas),axis=0)
        z = np.array(z)

        return z

    def filter_init(self):
        ## UKF filter parameters
        n=self.n
        m=self.m
        dt = 0.01 #
        self.dim_x = 20 #
        self.dim_z = 20 #

        #unscented points
        self.points = sigma_points.SimplexSigmaPoints(n=self.dim_x) #


        # initial cov
        Pos_cov = 10*np.eye(m*n)
        Vel_cov = np.eye(m*n)
        P_up = np.concatenate((Pos_cov, 0*np.eye(self.m*self.n)),axis=1)
        P_down = np.concatenate((0*np.eye(self.m*self.n), Vel_cov), axis=1)
        P = np.concatenate((P_up,P_down),axis=0)

        # initial process cov
        Q = np.eye(self.m*self.n*2)*(dt**2.)

        #meas cov
        GPS_cov = 10*np.eye(m*n)
        ACC_cov = np.eye(m*n)
        R_up = np.concatenate((GPS_cov, 0*np.eye(m*n)),axis=1)
        R_down = np.concatenate((0*np.eye(m*n), ACC_cov), axis=1)
        R = np.concatenate((R_up,R_down),axis=0)
        R = np.array(R) #

        self.filter = UKF(dim_x=self.dim_x, dim_z=self.dim_z, dt=dt, hx=self.hx, fx=self.fx, points=self.points)
        # self.filter.x = X
        self.filter.P = P
        self.filter.Q = Q
        self.filter.R = R

    def fx(self, x, dt):
        """ state transition function for sstate [downrange, vel, altitude]"""
        x_out = self.rk4(dyn=self.dyn, x=x, dt=dt)

        return x_out


    def dyn(self,x):
        n = self.n
        m = self.m
        F1_up = np.concatenate((0*np.eye(m), np.eye(m)),axis=1)
        F1_down = np.concatenate((0*np.eye(m), 0*np.eye(m)), axis=1)
        F1 = np.concatenate((F1_up,F1_down),axis=0)
        dx = np.dot(np.kron(F1,np.eye(n)),x)

        return dx

    def rk4(self, dyn, x, dt):
        k1 = dyn(x)*dt
        k2 = dyn(x + k1*0.5)*dt
        k3 = dyn(x + k2*0.5)*dt
        k4 = dyn(x + k3)*dt

        x = x + (k1 + 2 * (k2 + k3) + k4) / 6.0

        return x

    def hx(self, x):
        n = self.n
        m = self.m
        return np.dot(np.eye(n*m*2),x)

# define agent (global variable)
agent1 = Data_agent(idx_uav=1)
agent2 = Data_agent(idx_uav=2)
agent3 = Data_agent(idx_uav=3)
agent4 = Data_agent(idx_uav=4)
agent5 = Data_agent(idx_uav=5)

FDI = Data_FDI()

class PX4_GUI(QtWidgets.QDialog):
    def __init__(self, parent=None):
        QtWidgets.QDialog.__init__(self, parent)
        self.ui = uic.loadUi("gui_window/gui_5_quad_formation_ctrl.ui", self)
        self.ui.show()

        self.srv_reset = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        agent1.OT = Offboard_thread(idx_uav=1)
        agent2.OT = Offboard_thread(idx_uav=2)
        agent3.OT = Offboard_thread(idx_uav=3)
        agent4.OT = Offboard_thread(idx_uav=4)
        agent5.OT = Offboard_thread(idx_uav=5)

        FDI.OT = FDI_thread()



        self.slider_roll_1 = self.horizontalSlider_roll_1
        self.slider_pitch_1 = self.verticalSlider_pitch_1
        self.slider_yaw_1 = self.horizontalSlider_yaw_1
        self.slider_throttle_1 = self.verticalSlider_throttle_1

        self.slider_roll_2 = self.horizontalSlider_roll_2
        self.slider_pitch_2 = self.verticalSlider_pitch_2
        self.slider_yaw_2 = self.horizontalSlider_yaw_2
        self.slider_throttle_2 = self.verticalSlider_throttle_2

        self.slider_roll_3 = self.horizontalSlider_roll_3
        self.slider_pitch_3 = self.verticalSlider_pitch_3
        self.slider_yaw_3 = self.horizontalSlider_yaw_3
        self.slider_throttle_3 = self.verticalSlider_throttle_3

        self.slider_des_x_1 = self.horizontalSlider_des_x_1
        self.slider_des_y_1 = self.horizontalSlider_des_y_1
        self.slider_des_z_1 = self.horizontalSlider_des_z_1

        self.slider_des_x_2 = self.horizontalSlider_des_x_2
        self.slider_des_y_2 = self.horizontalSlider_des_y_2
        self.slider_des_z_2 = self.horizontalSlider_des_z_2

        self.slider_des_x_3 = self.horizontalSlider_des_x_3
        self.slider_des_y_3 = self.horizontalSlider_des_y_3
        self.slider_des_z_3 = self.horizontalSlider_des_z_3

        self.text_des_x_1 = self.plainTextEdit_des_x_1
        self.text_des_y_1 = self.plainTextEdit_des_y_1
        self.text_des_z_1 = self.plainTextEdit_des_z_1

        self.text_des_x_2 = self.plainTextEdit_des_x_2
        self.text_des_y_2 = self.plainTextEdit_des_y_2
        self.text_des_z_2 = self.plainTextEdit_des_z_2

        self.text_des_x_3 = self.plainTextEdit_des_x_3
        self.text_des_y_3 = self.plainTextEdit_des_y_3
        self.text_des_z_3 = self.plainTextEdit_des_z_3

        self.text_state_x_1 = self.plainTextEdit_state_x_1
        self.text_state_y_1 = self.plainTextEdit_state_y_1
        self.text_state_z_1 = self.plainTextEdit_state_z_1

        self.text_state_x_2 = self.plainTextEdit_state_x_2
        self.text_state_y_2 = self.plainTextEdit_state_y_2
        self.text_state_z_2 = self.plainTextEdit_state_z_2

        self.text_state_x_3 = self.plainTextEdit_state_x_3
        self.text_state_y_3 = self.plainTextEdit_state_y_3
        self.text_state_z_3 = self.plainTextEdit_state_z_3

        self.slider_formation_heading = self.horizontalSlider_heading
        self.slider_formation_velocity = self.horizontalSlider_velocity

        self.scene = QGraphicsScene()

        self.waypoint_run = 0

        # timer for periodic update of GUI
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(100)  # Throw event timeout with an interval of 1000 milliseconds
        self.timer.timeout.connect(self.update_ctrl)  # each time timer counts a second, call self.blink
        self.color_flag = True

        self.timer_start()

    def timer_start(self):
        self.timer.start()

    def timer_stop(self):
        self.timer.stop()

    def update_ctrl(self):
        t = rospy.get_time()

        agent1.formation_heading = agent1.formation_heading + 0.1*self.slider_formation_heading.value()

        self.slider_roll_1.setValue(agent1.roll_cmd*100+50)
        self.slider_pitch_1.setValue(agent1.pitch_cmd*100+50)
        self.slider_yaw_1.setValue(agent1.yaw_cmd*100+50)
        self.slider_throttle_1.setValue(agent1.throttle_cmd*100+50)

        self.slider_roll_2.setValue(agent2.roll_cmd*100+50)
        self.slider_pitch_2.setValue(agent2.pitch_cmd*100+50)
        self.slider_yaw_2.setValue(agent2.yaw_cmd*100+50)
        self.slider_throttle_2.setValue(agent2.throttle_cmd*100+50)

        self.slider_roll_3.setValue(agent3.roll_cmd*100+50)
        self.slider_pitch_3.setValue(agent3.pitch_cmd*100+50)
        self.slider_yaw_3.setValue(agent3.yaw_cmd*100+50)
        self.slider_throttle_3.setValue(agent3.throttle_cmd*100+50)

        self.text_des_x_1.setPlainText(str("{0:.2f}".format(agent1.des_x)))
        self.text_des_y_1.setPlainText(str("{0:.2f}".format(agent1.des_y)))
        self.text_des_z_1.setPlainText(str("{0:.2f}".format(agent1.des_z)))

        self.text_des_x_2.setPlainText(str("{0:.2f}".format(agent2.des_x)))
        self.text_des_y_2.setPlainText(str("{0:.2f}".format(agent2.des_y)))
        self.text_des_z_2.setPlainText(str("{0:.2f}".format(agent2.des_z)))

        self.text_des_x_3.setPlainText(str("{0:.2f}".format(agent3.des_x)))
        self.text_des_y_3.setPlainText(str("{0:.2f}".format(agent3.des_y)))
        self.text_des_z_3.setPlainText(str("{0:.2f}".format(agent3.des_z)))

        self.text_state_x_1.setPlainText(str("{0:.2f}".format(agent1.position.pose.position.x)))
        self.text_state_y_1.setPlainText(str("{0:.2f}".format(agent1.position.pose.position.y)))
        self.text_state_z_1.setPlainText(str("{0:.2f}".format(agent1.position.pose.position.z)))

        self.text_state_x_2.setPlainText(str("{0:.2f}".format(agent2.position.pose.position.x)))
        self.text_state_y_2.setPlainText(str("{0:.2f}".format(agent2.position.pose.position.y)))
        self.text_state_z_2.setPlainText(str("{0:.2f}".format(agent2.position.pose.position.z)))

        self.text_state_x_3.setPlainText(str("{0:.2f}".format(agent3.position.pose.position.x)))
        self.text_state_y_3.setPlainText(str("{0:.2f}".format(agent3.position.pose.position.y)))
        self.text_state_z_3.setPlainText(str("{0:.2f}".format(agent3.position.pose.position.z)))

        self.plainTextEdit_formation_heading.setPlainText(str("{0:.2f}".format(self.slider_formation_heading.value())))
        self.plainTextEdit_formation_velocity.setPlainText(str("{0:.2f}".format(agent1.formation_velocity)))

        # self.tableWidget.setItem(5, 0, QtWidgets.QTableWidgetItem(str(["{0:.2f}".format(agent1.position.pose.position.x), "{0:.2f}".format(agent1.position.pose.position.y), "{0:.2f}".format(agent1.position.pose.position.z)])))

        pixmap = QtGui.QPixmap()
        pixmap.load('catkin_ws/src/wc_gazebo/scripts/camera_image.jpeg')
        item = QGraphicsPixmapItem(pixmap)
        self.scene.addItem(item)

    # UAV_1
    @pyqtSlot()
    def slot1(self):  # pushButton_arm
        self.tableWidget.setItem(0, 0, QtWidgets.QTableWidgetItem("armed"))
        agent1.arm(True)

    @pyqtSlot()
    def slot2(self): # pushButton_disarm
        self.tableWidget.setItem(0, 0, QtWidgets.QTableWidgetItem("disarmed"))
        agent1.arm(False)

    @pyqtSlot()
    def slot3(self): # click offboard radio button
        # !! should check there is periodic ctrl command
        check = agent1.mode(custom_mode="OFFBOARD")
        # if check.success == True:
        self.tableWidget.setItem(1, 0, QtWidgets.QTableWidgetItem("offboard"))

    @pyqtSlot()
    def slot4(self): # click stabilize radio button
        # check = self.mode(custom_mode="STABILIZED")
        check = agent1.mode(custom_mode='MANUAL')

        # if check.success == True:
        self.tableWidget.setItem(1, 0, QtWidgets.QTableWidgetItem("stabilize"))

    @pyqtSlot() ##
    def slot5(self): # click offboard thread on
        self.tableWidget.setItem(3, 0, QtWidgets.QTableWidgetItem("on"))
        agent1.OT.start()

    @pyqtSlot() ##
    def slot6(self):  # click offboard thread off
        self.tableWidget.setItem(3, 0, QtWidgets.QTableWidgetItem("off"))
        agent1.OT.myExit()

    @pyqtSlot() ##
    def slot7(self):  # click offboard thread suspend
        self.tableWidget.setItem(3, 0, QtWidgets.QTableWidgetItem("suspend"))
        agent1.OT.mySuspend()

    @pyqtSlot() ##
    def slot8(self):  # click offboard thread resume
        self.tableWidget.setItem(3, 0, QtWidgets.QTableWidgetItem("on"))
        # OT = Offboard_thread()
        agent1.OT.myResume()

    @pyqtSlot()
    def slot13(self): # click offboard input as joystick
        self.tableWidget.setItem(2, 0, QtWidgets.QTableWidgetItem("joystick"))
        self.ctrl_type = 'joystick'

    @pyqtSlot()
    def slot14(self): # click offboard input as autonomous
        self.tableWidget.setItem(2, 0, QtWidgets.QTableWidgetItem("auto"))
        self.ctrl_type = 'auto'
        print(self.tableWidget.item(2,0).text())
        print(self.tableWidget.item(2,0).text() == 'ff')

    @pyqtSlot()
    def slot15(self): # horizontal slider(desired pos (x))
        agent1.des_x = self.slider_des_x_1.value()

    @pyqtSlot()
    def slot16(self): # horizontal slider(desired pos (y))
        agent1.des_y = self.slider_des_y_1.value()

    @pyqtSlot()
    def slot17(self): # horizontal slider(desired pos (z))
        agent1.des_z = self.slider_des_z_1.value()

    @pyqtSlot()
    def slot18(self): # run waypoint flight (set waypoint trajectory)
        self.traj = traj_gen()
        self.traj.coeff_x = self.traj.calc_coeff(self.traj.wp[0, :], self.traj.T, self.traj.S)
        self.traj.coeff_y = self.traj.calc_coeff(self.traj.wp[1, :], self.traj.T, self.traj.S)
        self.traj.coeff_z = self.traj.calc_coeff(self.traj.wp[2, :], self.traj.T, self.traj.S)
        self.waypoint_run=1

    ### UAV_2
    @pyqtSlot()
    def slot21(self):  # pushButton_arm
        self.tableWidget_2.setItem(0, 0, QtWidgets.QTableWidgetItem("armed"))
        agent2.arm(True)

    @pyqtSlot()
    def slot22(self): # pushButton_disarm
        self.tableWidget_2.setItem(0, 0, QtWidgets.QTableWidgetItem("disarmed"))
        agent2.arm(False)

    @pyqtSlot()
    def slot23(self): # click offboard radio button
        # !! should check there is periodic ctrl command
        check = agent2.mode(custom_mode="OFFBOARD")
        # if check.success == True:
        self.tableWidget_2.setItem(1, 0, QtWidgets.QTableWidgetItem("offboard"))

    @pyqtSlot()
    def slot24(self): # click stabilize radio button
        # check = self.mode(custom_mode="STABILIZED")
        check = agent2.mode(custom_mode='MANUAL')
        # if check.success == True:
        self.tableWidget_2.setItem(1, 0, QtWidgets.QTableWidgetItem("stabilize"))

    @pyqtSlot() ##
    def slot25(self): # click offboard thread on
        self.tableWidget_2.setItem(3, 0, QtWidgets.QTableWidgetItem("on"))
        agent2.OT.start()

    @pyqtSlot() ##
    def slot26(self):  # click offboard thread off
        self.tableWidget_2.setItem(3, 0, QtWidgets.QTableWidgetItem("off"))
        agent2.OT.myExit()

    @pyqtSlot() ##
    def slot27(self):  # click offboard thread suspend
        self.tableWidget_2.setItem(3, 0, QtWidgets.QTableWidgetItem("suspend"))
        agent2.OT.mySuspend()

    @pyqtSlot() ##
    def slot28(self):  # click offboard thread resume
        self.tableWidget_2.setItem(3, 0, QtWidgets.QTableWidgetItem("on"))
        agent2.OT.myResume()

    @pyqtSlot()
    def slot29(self): # click offboard input as joystick
        self.tableWidget_2.setItem(2, 0, QtWidgets.QTableWidgetItem("joystick"))
        self.ctrl_type = 'joystick'

    @pyqtSlot()
    def slot30(self): # click offboard input as autonomous
        self.tableWidget_2.setItem(2, 0, QtWidgets.QTableWidgetItem("auto"))
        self.ctrl_type = 'auto'
        print(self.tableWidget.item(2,0).text())
        print(self.tableWidget.item(2,0).text() == 'ff')

    @pyqtSlot()
    def slot31(self): # horizontal slider(desired pos (x))
        agent2.des_x = self.slider_des_x_2.value()

    @pyqtSlot()
    def slot32(self): # horizontal slider(desired pos (y))
        agent2.des_y = self.slider_des_y_2.value()

    @pyqtSlot()
    def slot33(self): # horizontal slider(desired pos (z))
        agent2.des_z = self.slider_des_z_2.value()

    @pyqtSlot()
    def slot34(self): # run waypoint flight (set waypoint trajectory)
        self.traj = traj_gen()
        self.traj.coeff_x = self.traj.calc_coeff(self.traj.wp[0, :], self.traj.T, self.traj.S)
        self.traj.coeff_y = self.traj.calc_coeff(self.traj.wp[1, :], self.traj.T, self.traj.S)
        self.traj.coeff_z = self.traj.calc_coeff(self.traj.wp[2, :], self.traj.T, self.traj.S)
        self.waypoint_run=1

    ### UAV_3
    @pyqtSlot()
    def slot41(self):  # pushButton_arm
        self.tableWidget_3.setItem(0, 0, QtWidgets.QTableWidgetItem("armed"))
        agent3.arm(True)
        agent5.arm(True)

    @pyqtSlot()
    def slot42(self): # pushButton_disarm
        self.tableWidget_3.setItem(0, 0, QtWidgets.QTableWidgetItem("disarmed"))
        agent3.arm(False)

    @pyqtSlot()
    def slot43(self): # click offboard radio button
        # !! should check there is periodic ctrl command
        check = agent3.mode(custom_mode="OFFBOARD")
        # if check.success == True:
        self.tableWidget_3.setItem(1, 0, QtWidgets.QTableWidgetItem("offboard"))

    @pyqtSlot()
    def slot44(self): # click stabilize radio button
        # check = self.mode(custom_mode = "STABILIZED")
        check = agent3.mode(custom_mode = 'MANUAL')
        # if check.success == True:
        self.tableWidget_3.setItem(1, 0, QtWidgets.QTableWidgetItem("stabilize"))

    @pyqtSlot() ##
    def slot45(self): # click offboard thread on
        self.tableWidget_3.setItem(3, 0, QtWidgets.QTableWidgetItem("on"))
        agent3.OT.start()

    @pyqtSlot() ##
    def slot46(self):  # click offboard thread off
        self.tableWidget_3.setItem(3, 0, QtWidgets.QTableWidgetItem("off"))
        agent3.OT.myExit()

    @pyqtSlot() ##
    def slot47(self):  # click offboard thread suspend
        self.tableWidget_3.setItem(3, 0, QtWidgets.QTableWidgetItem("suspend"))
        agent3.OT.mySuspend()

    @pyqtSlot() ##
    def slot48(self):  # click offboard thread resume
        self.tableWidget_3.setItem(3, 0, QtWidgets.QTableWidgetItem("on"))
        agent3.OT.myResume()

    @pyqtSlot()
    def slot49(self): # click offboard input as joystick
        self.tableWidget_3.setItem(2, 0, QtWidgets.QTableWidgetItem("joystick"))
        self.ctrl_type = 'joystick'

    @pyqtSlot()
    def slot50(self): # click offboard input as autonomous
        self.tableWidget_3.setItem(2, 0, QtWidgets.QTableWidgetItem("auto"))
        self.ctrl_type = 'auto'
        print(self.tableWidget.item(2,0).text())
        print(self.tableWidget.item(2,0).text() == 'ff')

    @pyqtSlot()
    def slot51(self): # horizontal slider(desired pos (x))
        agent3.des_x = self.slider_des_x_3.value()

    @pyqtSlot()
    def slot52(self): # horizontal slider(desired pos (y))
        agent3.des_y = self.slider_des_y_3.value()

    @pyqtSlot()
    def slot53(self): # horizontal slider(desired pos (z))
        agent3.des_z = self.slider_des_z_3.value()

    @pyqtSlot()
    def slot54(self): # run waypoint flight (set waypoint trajectory)
        self.traj = traj_gen()
        self.traj.coeff_x = self.traj.calc_coeff(self.traj.wp[0, :], self.traj.T, self.traj.S)
        self.traj.coeff_y = self.traj.calc_coeff(self.traj.wp[1, :], self.traj.T, self.traj.S)
        self.traj.coeff_z = self.traj.calc_coeff(self.traj.wp[2, :], self.traj.T, self.traj.S)
        self.waypoint_run=1

    ### all UAVs
    @pyqtSlot()
    def slot501(self):  # pushButton_arm
        agent1.arm(True)
        agent2.arm(True)
        agent3.arm(True)
        agent4.arm(True)
        agent5.arm(True)

    @pyqtSlot()
    def slot502(self): # pushButton_disarm
        agent1.arm(False)
        agent2.arm(False)
        agent3.arm(False)
        agent4.arm(False)
        agent5.arm(False)

    @pyqtSlot()
    def slot503(self): # click offboard radio button
        agent1.mode(custom_mode="OFFBOARD")
        agent2.mode(custom_mode="OFFBOARD")
        agent3.mode(custom_mode="OFFBOARD")
        agent4.mode(custom_mode="OFFBOARD")
        agent5.mode(custom_mode="OFFBOARD")

    @pyqtSlot()
    def slot504(self): # click stabilize radio button
        agent1.mode(custom_mode='MANUAL')
        agent2.mode(custom_mode='MANUAL')
        agent3.mode(custom_mode='MANUAL')
        agent4.mode(custom_mode='MANUAL')
        agent5.mode(custom_mode='MANUAL')

    @pyqtSlot() ##
    def slot505(self): # click offboard thread on
        agent1.OT.start()
        agent2.OT.start()
        agent3.OT.start()
        agent4.OT.start()
        agent5.OT.start()

        FDI.filter_init()
        FDI.OT.start()
        FDI.FDI_on = True

    @pyqtSlot() ##
    def slot506(self):  # click offboard thread off
        agent1.OT.myExit()
        agent2.OT.myExit()
        agent3.OT.myExit()
        agent4.OT.myExit()
        agent5.OT.myExit()

    @pyqtSlot() ##
    def slot507(self):  # click offboard thread suspend
        agent1.OT.mySuspend()
        agent2.OT.mySuspend()
        agent3.OT.mySuspend()
        agent4.OT.mySuspend()
        agent5.OT.mySuspend()

    @pyqtSlot() ##
    def slot508(self):  # click offboard thread resume
        agent1.OT.myResume()
        agent2.OT.myResume()
        agent3.OT.myResume()
        agent4.OT.myResume()
        agent5.OT.myResume()

    @pyqtSlot()
    def slot509(self): # click offboard input as joystick
        self.tableWidget_3.setItem(2, 0, QtWidgets.QTableWidgetItem("joystick"))
        self.ctrl_type = 'joystick'
    #
    # @pyqtSlot()
    # def slot510(self):  # horizontal slider(formation heading)
    #     1
    #     # agent1.formation_heading = self.slider_formation_heading.value()

    @pyqtSlot()
    def slot511(self):  # horizontal slider(formation velocity)
        agent1.formation_velocity = self.slider_formation_velocity.value()

    @pyqtSlot()
    def slot50(self): # click offboard input as autonomous
        self.tableWidget_3.setItem(2, 0, QtWidgets.QTableWidgetItem("auto"))
        self.ctrl_type = 'auto'
        print(self.tableWidget.item(2,0).text())
        print(self.tableWidget.item(2,0).text() == 'ff')

    @pyqtSlot()
    def slot51(self): # horizontal slider(desired pos (x))
        agent3.des_x = self.slider_des_x_3.value()

    @pyqtSlot()
    def slot52(self): # horizontal slider(desired pos (y))
        agent3.des_y = self.slider_des_y_3.value()

    @pyqtSlot()
    def slot53(self): # horizontal slider(desired pos (z))
        agent3.des_z = self.slider_des_z_3.value()

    @pyqtSlot()
    def slot54(self): # run waypoint flight (set waypoint trajectory)
        self.traj = traj_gen()
        self.traj.coeff_x = self.traj.calc_coeff(self.traj.wp[0, :], self.traj.T, self.traj.S)
        self.traj.coeff_y = self.traj.calc_coeff(self.traj.wp[1, :], self.traj.T, self.traj.S)
        self.traj.coeff_z = self.traj.calc_coeff(self.traj.wp[2, :], self.traj.T, self.traj.S)
        self.waypoint_run=1


    # reset all
    @pyqtSlot()
    def slot19(self): # reset simulation
        # before reset, set all the desired pos/command to 0
        agent1.des_x = 0
        agent1.des_y = 0
        agent1.des_z = 0
        agent1.roll_cmd = 0
        agent1.pitch_cmd = 0
        agent1.yaw_cmd = 0
        agent1.throttle_cmd = 0.066

        pose = Pose()
        pose.position.x = 0
        pose.position.y = 0
        pose.position.z = 2
        q = quaternion_from_euler(0,0,0)
        pose.orientation = Quaternion(*q)
        state = ModelState()
        state.model_name = "iris"
        state.pose = pose
        self.srv_reset(state)

class traj_gen():
    def __init__(self):
        self.tn = rospy.get_time()         # time now
        self.wp = np.matrix('0 1 2 3 2 0;\
                           0 2 4 5 2 0;\
                           0 4 5 4 2 1')   # wp_z
        self.T = np.matrix('3 3 3 3 3')    # Time to go for each path
        self.S = np.matrix('0 3 6 9 12')
        self.S = self.S + self.tn
        self.ts = self.S[0,0]
        self.dt = 0.01
        self.tf = self.ts + np.sum(self.T)

        self.n_wp = np.size(self.wp, 1)  # col size
        self.n_p = self.n_wp - 1

        self.coeff_x = np.zeros((self.n_p, 8))
        self.coeff_y = np.zeros((self.n_p, 8))
        self.coeff_z = np.zeros((self.n_p, 8))

    def calc_coeff(self, wp, T, S):
        n_wp = np.size(wp, 1)  # col size
        n_p = n_wp - 1

        wp = wp.astype(float)
        T = T.astype(float)
        S = S.astype(float)

        a = np.zeros((8 * n_p, 1))
        a = a.astype(float)
        A = np.zeros((8 * n_p, 8 * n_p))
        A = A.astype(float)
        b = np.zeros((8 * n_p, 1))
        b = b.astype(float)

        idx_row = 0

        # waypoint constraint (2n constraint) => n polynomial & 2 constraints (0,S)
        for i in range(n_p):
            A[idx_row, 8 * i:8 * (i + 1)] = [1, 0, 0, 0, 0, 0, 0, 0]
            A[idx_row + 1, 8 * i:8 * (i + 1)] = [1, 1, 1, 1, 1, 1, 1, 1]
            # print(wp)
            b[idx_row, 0] = wp[0, i]
            b[idx_row + 1, 0] = wp[0, i + 1]
            idx_row = idx_row + 2

        # derivative 1-3 are aero at endpoint(6 constraint)
        # derivative 1
        A[idx_row, 0:8] = [0, 1, 0, 0, 0, 0, 0, 0] / T[0, 0]
        A[idx_row + 1, 8 * (n_p - 1):8 * n_p] = [0, 1, 2, 3, 4, 5, 6, 7] / T[0, n_p - 1]
        idx_row = idx_row + 2;
        # derivative 2
        A[idx_row, 0:8] = [0, 0, 2, 0, 0, 0, 0, 0] / T[0, 0] ** 2
        A[idx_row + 1, 8 * (n_p - 1):8 * n_p] = [0, 0, 2, 6, 12, 20, 30, 42] / T[0, n_p - 1] ** 2
        idx_row = idx_row + 2;
        # derivative 3
        A[idx_row, 0:8] = [0, 0, 0, 6, 0, 0, 0, 0] / T[0, 0] ** 3
        A[idx_row + 1, 8 * (n_p - 1):8 * n_p] = [0, 0, 0, 6, 24, 60, 120, 210] / T[0, n_p - 1] ** 3
        idx_row = idx_row + 2;

        # derivative 1-6(6n-6 constraints)
        for i in range(n_p - 1):
            A[idx_row + 6 * i + 0, 8 * i:8 * (i + 1)] = [0, 1, 2, 3, 4, 5, 6, 7] / T[0, i]
            A[idx_row + 6 * i + 0, 8 * (i + 1):8 * (i + 2)] = [0, -1, 0, 0, 0, 0, 0, 0] / T[0, i + 1]  # dev-1
            A[idx_row + 6 * i + 1, 8 * i:8 * (i + 1)] = [0, 0, 2, 6, 12, 20, 30, 42] / T[0, i] ** 2
            A[idx_row + 6 * i + 1, 8 * (i + 1):8 * (i + 2)] = [0, 0, -2, 0, 0, 0, 0, 0] / T[0, i + 1] ** 2  # dev-2
            A[idx_row + 6 * i + 2, 8 * i:8 * (i + 1)] = [0, 0, 0, 6, 24, 60, 120, 210] / T[0, i] ** 3
            A[idx_row + 6 * i + 2, 8 * (i + 1):8 * (i + 2)] = [0, 0, 0, -6, 0, 0, 0, 0] / T[0, i + 1] ** 3  # dev-3
            A[idx_row + 6 * i + 3, 8 * i:8 * (i + 1)] = [0, 0, 0, 0, 24, 120, 360, 840] / T[0, i] ** 4
            A[idx_row + 6 * i + 3, 8 * (i + 1):8 * (i + 2)] = [0, 0, 0, 0, -24, 0, 0, 0] / T[0, i + 1] ** 4  # dev-4
            A[idx_row + 6 * i + 4, 8 * i:8 * (i + 1)] = [0, 0, 0, 0, 0, 120, 720, 2520] / T[0, i] ** 5
            A[idx_row + 6 * i + 4, 8 * (i + 1):8 * (i + 2)] = [0, 0, 0, 0, 0, -120, 0, 0] / T[0, i + 1] ** 5  # dev-5
            A[idx_row + 6 * i + 5, 8 * i:8 * (i + 1)] = [0, 0, 0, 0, 0, 0, 720, 5040] / T[0, i] ** 6
            A[idx_row + 6 * i + 5, 8 * (i + 1):8 * (i + 2)] = [0, 0, 0, 0, 0, 0, -720, 0] / T[0, i + 1] ** 6  # dev-6

        a = np.dot(np.linalg.inv(A), b)
        a_reshape = np.transpose(a.reshape(n_p, 8))
        return a_reshape

    def des_t(self, t, coeff, S, T):
        bound_low = (S <= t)
        bound_high = (S + T >= t)
        p_i = np.where(bound_low == bound_high)  # which polynomial to use
        a = coeff[:, p_i[1][0]]
        Tgo = T[0, p_i[1][0]]
        Si = S[0, p_i[1][0]]

        x = (t - Si) / Tgo
        pos = a[0] + a[1]*x + a[2]*x**2 + a[3]*x**3 + a[4]*x**4 + a[5]*x**5 + a[6]*x**6 + a[7]*x**7

        return pos

class FDI_thread(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.rate = rospy.Rate(10) # 5Hz

        self.time_prev = rospy.get_time()

        self.__suspend = False
        self.__exit = False
        self.daemon = True

    def run(self):
        while not rospy.is_shutdown():
            try:
                if self.__suspend == True:
                    continue

                # FDI
                self.time_now = rospy.get_time()
                dt = self.time_now - self.time_prev

                if FDI.FDI_on == True:
                    # get sensor data
                    z = FDI.get_sensor_data()

                    FDI.filter.predict(dt = dt)
                    FDI.filter.update(z=z)



                    # FDI
                    P = FDI.filter.P
                    R = FDI.filter.R
                    Sig_1_up = np.concatenate((P[0:2,0:2],0*np.eye(2)),axis=1) + np.concatenate((R[0:2,0:2],0*np.eye(2)),axis=1)
                    Sig_1_down = np.concatenate((0*np.eye(2),P[10:12,10:12]),axis=1) + np.concatenate((0*np.eye(2),R[10:12,10:12]),axis=1)
                    Sig_1 = np.concatenate((Sig_1_up,Sig_1_down),axis=0)

                    Sig_2_up = np.concatenate((P[2:4,2:4],0*np.eye(2)),axis=1) + np.concatenate((R[2:4,2:4],0*np.eye(2)),axis=1)
                    Sig_2_down = np.concatenate((0*np.eye(2),P[12:14,12:14]),axis=1) + np.concatenate((0*np.eye(2),R[12:14,12:14]),axis=1)
                    Sig_2 = np.concatenate((Sig_2_up,Sig_2_down),axis=0)

                    Sig_3_up = np.concatenate((P[4:6,4:6],0*np.eye(2)),axis=1) + np.concatenate((R[4:6,4:6],0*np.eye(2)),axis=1)
                    Sig_3_down = np.concatenate((0*np.eye(2),P[14:16,14:16]),axis=1) + np.concatenate((0*np.eye(2),R[14:16,14:16]),axis=1)
                    Sig_3 = np.concatenate((Sig_3_up,Sig_3_down),axis=0)

                    Sig_4_up = np.concatenate((P[6:8,6:8],0*np.eye(2)),axis=1) + np.concatenate((R[6:8,6:8],0*np.eye(2)),axis=1)
                    Sig_4_down = np.concatenate((0*np.eye(2),P[16:18,16:18]),axis=1) + np.concatenate((0*np.eye(2),R[16:18,16:18]),axis=1)
                    Sig_4 = np.concatenate((Sig_4_up,Sig_4_down),axis=0)

                    Sig_5_up = np.concatenate((P[8:10,8:10],0*np.eye(2)),axis=1) + np.concatenate((R[8:10,8:10],0*np.eye(2)),axis=1)
                    Sig_5_down = np.concatenate((0*np.eye(2),P[18:20,18:20]),axis=1) + np.concatenate((0*np.eye(2),R[18:20,18:20]),axis=1)
                    Sig_5 = np.concatenate((Sig_5_up,Sig_5_down),axis=0)

                    # calc res
                    res = FDI.filter.y
                    res_1 = np.concatenate((res[0:2],res[10:12]),axis=0)
                    res_2 = np.concatenate((res[2:4],res[12:14]),axis=0)
                    res_3 = np.concatenate((res[4:6],res[14:16]),axis=0)
                    res_4 = np.concatenate((res[6:8],res[16:18]),axis=0)
                    res_5 = np.concatenate((res[8:10],res[18:20]),axis=0)


                    # calc h
                    h = [None]*5
                    h[0] = np.dot(np.dot(res_1, np.linalg.inv(Sig_1)), res_1)
                    h[1] = np.dot(np.dot(res_2, np.linalg.inv(Sig_2)), res_2)
                    h[2] = np.dot(np.dot(res_3, np.linalg.inv(Sig_3)), res_3)
                    h[3] = np.dot(np.dot(res_4, np.linalg.inv(Sig_4)), res_4)
                    h[4] = np.dot(np.dot(res_5, np.linalg.inv(Sig_5)), res_5)

                    # print(h)

                self.time_prev = self.time_now

                self.rate.sleep()

                ### Exit ###
                if self.__exit:
                    break

            except rospy.ROSInterruptException:
                pass

    def mySuspend(self):
        self.__suspend = True

    def myResume(self):
        self.__suspend = False

    def myExit(self):
        self.__exit = True

class Offboard_thread(Thread):
    def __init__(self, idx_uav):
        Thread.__init__(self)
        self.idx_uav = idx_uav
        self.rate = rospy.Rate(20) # 5Hz

        self.ctrl = setpoint_att()

        self.__suspend = False
        self.__exit = False
        self.daemon = True

    def run(self):
        while not rospy.is_shutdown():
            try:
                if self.__suspend == True:
                    continue

                # if self.tableWidget.item(2,0).text() == 'auto':
                #     self.ctrl.calc_cmd_att_thr_auto()
                # elif self.tableWidget.item(2,0).text() == 'joystick':

                # self.ctrl.calc_cmd_att_thr_joy()
                self.ctrl.calc_cmd_att_thr_auto(self.idx_uav)
                self.ctrl.talk(self.idx_uav)

                self.rate.sleep()

                ### Exit ###
                if self.__exit:
                    break

            except rospy.ROSInterruptException:
                pass

    def mySuspend(self):
        self.__suspend = True

    def myResume(self):
        self.__suspend = False

    def myExit(self):
        self.__exit = True

class setpoint_att(object):
    def __init__(self):
        # self.pub_att = rospy.Publisher('/uav3/mavros/setpoint_attitude/attitude', PoseStamped, queue_size=10)
        # self.pub_thr = rospy.Publisher('/uav3/mavros/setpoint_attitude/att_throttle', Float64, queue_size=10)
        self.cmd_att = PoseStamped()
        self.cmd_thr = Float64()

        self.count = 1
        self.cmd_att.header.stamp.secs = rospy.get_time()
        self.cmd_att.header.seq = self.count

        self.roll = 0 # [-1, 1]
        self.pitch = 0 # [-1, 1]
        self.yaw = 0 # [-1, 1]
        self.throttle = 0 # [0, 1]

        self.state_x = 0
        self.state_y = 0
        self.state_z = 0

        self.state_phi = 0
        self.state_theta = 0
        self.state_psi = 0

        self.des_x = 0 #
        self.des_y = 0 #
        self.des_z = 0 #
        self.des_vx = 0
        self.des_vy = 0
        self.des_vz = 0
        self.des_ax = 0
        self.des_ay = 0
        self.des_az = 0

        self.des_yaw = 0 #
        self.des_yawdot = 0 #

        self.Kp_x = 20
        self.Kv_x = 5
        self.Kp_y = 20
        self.Kv_y = 5
        self.Kp_z = 2.5
        self.Kv_z = 1.8

        self.Kp_phi = 0.1
        self.Kv_phi = 0
        self.Kp_theta = 0.1

        self.Kv_theta = 0
        self.Kp_psi = 0.1
        self.Kv_psi = 0

        self.m = 0.0565
        self.g = 9.81

    def calc_cmd_att_thr_joy(self): # offboard control using joystic
        self.roll = agent1.roll_cmd # [-1, 1]
        self.pitch = agent1.pitch_cmd # [-1, 1]
        self.yaw = agent1.yaw_cmd # [-1, 1]
        self.throttle = agent1.throttle_cmd + 0.5 # [0, 1]

        self.cmd_att.header.stamp = rospy.Time.now()

        q = quaternion_from_euler(self.roll,self.pitch,self.yaw)
        self.cmd_att.header.seq = self.count
        self.cmd_att.pose.orientation = Quaternion(*q)
        self.cmd_thr.data = self.throttle

        self.count += 1

        return (self.cmd_att, self.cmd_thr)

    def calc_cmd_att_thr_auto(self, idx_uav): # offboard control autonomously (roll/pitch/yaw/throttle)
        # -----------------------------------------------------------------
        t_prev = self.cmd_att.header.stamp.secs
        self.cmd_att.header.stamp.secs = rospy.get_time()
        t_now = self.cmd_att.header.stamp.secs

        dt = t_now - t_prev
        if dt == 0 :
            dt =0.05

        if idx_uav == 1:
            agent = agent1
        elif idx_uav == 2:
            agent = agent2
        elif idx_uav == 3:
            agent = agent3
        elif idx_uav == 4:
            agent = agent4
        elif idx_uav == 5:
            agent = agent5

        # -----------------------------------------------------------------
        self.des_x = agent.des_x  #
        self.des_y = agent.des_y  #
        self.des_z = agent.des_z  #

        des_x_prev = self.des_x
        des_y_prev = self.des_y
        des_z_prev = self.des_z
        des_vx_prev = self.des_vx
        des_vy_prev = self.des_vy
        des_vz_prev = self.des_vz

        self.des_vx = (self.des_x - des_x_prev) / dt
        self.des_vy = (self.des_y - des_y_prev) / dt
        self.des_vz = (self.des_z - des_z_prev) / dt
        self.des_ax = (self.des_vx - des_vx_prev) / dt
        self.des_ay = (self.des_vy - des_vy_prev) / dt
        self.des_az = (self.des_vz - des_vz_prev) / dt

        state_phi_prev = self.state_phi
        state_theta_prev = self.state_theta
        state_psi_prev = self.state_psi
        (self.state_phi, self.state_theta, self.state_psi) = euler_from_quaternion([agent.position.pose.orientation.x, agent.position.pose.orientation.y, agent.position.pose.orientation.z, agent.position.pose.orientation.w])
        state_vphi = (self.state_phi - state_phi_prev) / dt
        state_vtheta = (self.state_theta - state_theta_prev) / dt
        state_vpsi = (self.state_psi - state_psi_prev) / dt

        state_x_prev = self.state_x
        state_y_prev = self.state_y
        state_z_prev = self.state_z

        self.state_x = agent.position.pose.position.x
        self.state_y = agent.position.pose.position.y
        self.state_z = agent.position.pose.position.z



        # state_vx = (self.state_x - state_x_prev) / dt
        # state_vy = (self.state_y - state_y_prev) / dt
        # state_vz = (self.state_z - state_z_prev) / dt
        #
        # if idx_uav == 1:
        #     agent1.vel_x = state_vx
        #     agent1.vel_y = state_vy
        #     agent1.vel_z = state_vz
        # elif idx_uav == 2:
        #     agent2.vel_x = state_vx
        #     agent2.vel_y = state_vy
        #     agent2.vel_z = state_vz
        # elif idx_uav == 3:
        #     agent3.vel_x = state_vx
        #     agent3.vel_y = state_vy
        #     agent3.vel_z = state_vz
        # elif idx_uav == 4:
        #     agent4.vel_x = state_vx
        #     agent4.vel_y = state_vy
        #     agent4.vel_z = state_vz
        # elif idx_uav == 5:
        #     agent5.vel_x = state_vx
        #     agent5.vel_y = state_vy
        #     agent5.vel_z = state_vz

        if idx_uav == 1:
            state_vx = agent1.vel_x
            state_vy = agent1.vel_y
            state_vz = agent1.vel_z
        elif idx_uav == 2:
            state_vx = agent2.vel_x
            state_vy = agent2.vel_y
            state_vz = agent2.vel_z
        elif idx_uav == 3:
            state_vx = agent3.vel_x
            state_vy = agent3.vel_y
            state_vz = agent3.vel_z
        elif idx_uav == 4:
            state_vx = agent4.vel_x
            state_vy = agent4.vel_y
            state_vz = agent4.vel_z
        elif idx_uav == 5:
            state_vx = agent5.vel_x
            state_vy = agent5.vel_y
            state_vz = agent5.vel_z

        # err_x = self.des_x - self.state_x
        # err_y = self.des_y - self.state_y
        err_z = self.des_z - self.state_z

        # err_vx = self.des_vx - state_vx
        # err_vy = self.des_vy - state_vy
        err_vz = self.des_vz - state_vz

        # -----------------------------------------------------------------
        n = FDI.n  # number of uavs
        m = FDI.m  # number of dimension

        # Laplacian matrix
        L = FDI.L

        # gain
        kap1 = 3
        kap2 = 3
        kap3 = 3

        ## Measurement Matrix
        Hx = np.concatenate((np.eye(m), 0 * np.eye(m)), axis=1)
        Hv = np.concatenate((0 * np.eye(m), np.eye(m)), axis=1)
        Hxv = np.concatenate((np.kron(np.eye(n), Hx), np.kron(np.eye(n), Hv)), axis=0)

        V = agent1.formation_velocity
        th = agent1.formation_heading * math.pi/180
        Xd = np.array([0, 0, V * np.cos(th), V * np.sin(th),
                       -3 * np.cos(math.pi/4 - th), 3 * np.sin(math.pi/4 - th), V * np.cos(th), V * np.sin(th),
                       -3 * np.cos(math.pi/4 + th), -3 * np.sin(math.pi/4 + th), V * np.cos(th), V * np.sin(th),
                       -5 * np.cos(math.pi / 4 - th), 5 * np.sin(math.pi / 4 - th), V * np.cos(th), V * np.sin(th),
                       -5 * np.cos(math.pi / 4 + th), -5 * np.sin(math.pi / 4 + th), V * np.cos(th), V * np.sin(th)])

        X = np.array([agent1.position.pose.position.x, agent1.position.pose.position.y, agent1.vel_x, agent1.vel_y,
                      agent2.position.pose.position.x, agent2.position.pose.position.y, agent2.vel_x, agent2.vel_y,
                      agent3.position.pose.position.x, agent3.position.pose.position.y, agent3.vel_x, agent3.vel_y,
                      agent4.position.pose.position.x, agent4.position.pose.position.y, agent4.vel_x, agent4.vel_y,
                      agent5.position.pose.position.x, agent5.position.pose.position.y, agent5.vel_x, agent5.vel_y])


        temp = np.concatenate((kap1 * L, kap2 * L + kap3 * np.eye(n)), axis=1)
        U = np.dot(np.dot(-np.kron(temp, np.eye(m)), Hxv), np.transpose(X - Xd))

        if idx_uav == 1:
            acc_comm_x = U[0]
            acc_comm_y = U[1]
        elif idx_uav == 2:
            acc_comm_x = U[2]
            acc_comm_y = U[3]
        elif idx_uav == 3:
            acc_comm_x = U[4]
            acc_comm_y = U[5]
        elif idx_uav == 4:
            acc_comm_x = U[6]
            acc_comm_y = U[7]
        elif idx_uav == 5:
            acc_comm_x = U[8]
            acc_comm_y = U[9]

        # =================================================================
        # acc_comm_x = self.des_ax + self.Kv_x * err_vx + self.Kp_x * err_x
        # acc_comm_y = self.des_ay + self.Kv_y * err_vy + self.Kp_y * err_y
        acc_comm_z = self.des_az + self.Kv_z * err_vz + self.Kp_z * err_z

        u1 = self.m * (self.g + acc_comm_z)

        des_phi = 1/self.g * (acc_comm_x*math.sin(self.des_yaw) - acc_comm_y*math.cos(self.des_yaw))
        des_theta = 1/self.g * (acc_comm_x*math.cos(self.des_yaw) + acc_comm_y*math.sin(self.des_yaw))


        u_phi = self.Kp_phi*(des_phi-self.state_phi) + self.Kv_phi*(0-state_vphi)
        u_theta = self.Kp_theta*(des_theta-self.state_theta) + self.Kv_theta*(0-state_vtheta)
        u_psi = self.Kp_psi*(self.des_yaw-self.state_psi) + self.Kv_psi*(self.des_yawdot - state_vpsi)

        u1 = np.maximum(u1, 0)
        u1 = np.minimum(u1, 0.7)
        u_phi = np.maximum(u_phi,-0.6)
        u_phi = np.minimum(u_phi,0.6)
        u_theta = np.maximum(u_theta,-0.6)
        u_theta = np.minimum(u_theta,0.6)
        u_psi = np.maximum(u_psi,-0.6)
        u_psi = np.minimum(u_psi,0.6)

        q = quaternion_from_euler(u_phi,u_theta,u_psi)
        self.cmd_att.header.seq = self.count
        self.cmd_att.pose.orientation = Quaternion(*q)
        self.cmd_thr.data = u1

        if idx_uav == 1:
            agent1.roll_cmd = u_phi
            agent1.pitch_cmd = u_theta
            agent1.yaw_cmd = u_psi
            agent1.throttle_cmd = u1-0.5
        elif idx_uav == 2:
            agent2.roll_cmd = u_phi
            agent2.pitch_cmd = u_theta
            agent2.yaw_cmd = u_psi
            agent2.throttle_cmd = u1-0.5
        elif idx_uav == 3:
            agent3.roll_cmd = u_phi
            agent3.pitch_cmd = u_theta
            agent3.yaw_cmd = u_psi
            agent3.throttle_cmd = u1-0.5
        elif idx_uav == 4:
            agent4.roll_cmd = u_phi
            agent4.pitch_cmd = u_theta
            agent4.yaw_cmd = u_psi
            agent4.throttle_cmd = u1-0.5
        elif idx_uav == 5:
            agent5.roll_cmd = u_phi
            agent5.pitch_cmd = u_theta
            agent5.yaw_cmd = u_psi
            agent5.throttle_cmd = u1-0.5

        return (self.cmd_att, self.cmd_thr)

    def talk(self, idx_uav):
        # print(self.theta)
        self.cmd_att.pose.position.z = 10.0
        if idx_uav == 1:
            agent1.pub_att.publish(self.cmd_att)
            agent1.pub_thr.publish(self.cmd_thr)
        elif idx_uav == 2:
            agent2.pub_att.publish(self.cmd_att)
            agent2.pub_thr.publish(self.cmd_thr)
        elif idx_uav == 3:
            agent3.pub_att.publish(self.cmd_att)
            agent3.pub_thr.publish(self.cmd_thr)
        elif idx_uav == 4:
            agent4.pub_att.publish(self.cmd_att)
            agent4.pub_thr.publish(self.cmd_thr)
        elif idx_uav == 5:
            agent5.pub_att.publish(self.cmd_att)
            agent5.pub_thr.publish(self.cmd_thr)
        # rospy.loginfo("[cmd req] att : %s, %s, %s, %s", self.roll, self.pitch, self.yaw, self.throttle)

if __name__ == '__main__':
    rospy.init_node('wc_ctrl_quad')
    # rospy.Subscriber("/uav3/mavros/local_position/pose", PoseStamped, local_position_callback)

    # joy = Joystick_thread()
    # joy.start()

    app = QtWidgets.QApplication(sys.argv)
    W = PX4_GUI()
    app.exec_()