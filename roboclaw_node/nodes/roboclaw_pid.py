#!/usr/bin/env python

from math import pi, cos, sin

import numpy as np
import diagnostic_msgs
import diagnostic_updater
import roboclaw_driver.roboclaw_driver_new as roboclaw
import rospy
import tf
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
import sys

__author__ = "bwbazemore@uga.edu (Brad Bazemore)"

status1, enc1, crc1 = None, None, None
status2, enc2, crc2 = None, None, None
status3, enc3, crc3 = None, None, None
status4, enc4, crc4 = None, None, None
X = np.array([0.00,0.00,0.00])  
get_cmd_vel=0
real_sp_m1=0
real_sp_m2=0
real_sp_m3 =0
real_sp_m4 = 0
wheel_pwm_err_prev=np.array([0.00,0.00,0.00,0.00])
wheel_pwm_int_err=np.array([0.00,0.00,0.00,0.00])
U =np.array([0.00,0.00,0.00,0.00])
# version = 0x80

# TODO need to find some better was of handling OSerror 11 or preventing it, any ideas?
class EncoderOdom:
    def __init__(self, ticks_per_meter, base_width, base_length, wheel_radius):
        # rospy.logwarn(sys.version)
        self.TICKS_PER_METER = ticks_per_meter
        self.BASE_WIDTH = base_width
        self.BASE_LENGTH = base_length
        self.WHEEL_RADIUS = wheel_radius
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.cur_x = 0.0
        self.cur_y = 0.0
        self.cur_theta = 0.0
        self.last_enc_front_left = 0
        self.last_enc_front_right = 0
        self.last_enc_back_left = 0
        self.last_enc_back_right = 0
        self.last_enc_time = rospy.Time.now()

    @staticmethod
    def normalize_angle(angle):
        while angle > pi:
            angle -= 2.0 * pi
        while angle < -pi:
            angle += 2.0 * pi
        return angle

    def wheel_vel_update(self, enc_front_left, enc_front_right, enc_back_left, enc_back_right):
        lx = self.BASE_LENGTH
        ly = self.BASE_WIDTH
        r = self.WHEEL_RADIUS

        front_left_ticks = enc_front_left - self.last_enc_front_left
        front_right_ticks = enc_front_right - self.last_enc_front_right
        back_left_ticks = enc_back_left - self.last_enc_back_left
        back_right_ticks = enc_back_right - self.last_enc_back_right
        self.last_enc_front_left = enc_front_left
        self.last_enc_front_right = enc_front_right
        self.last_enc_back_left = enc_back_left
        self.last_enc_back_right = enc_back_right

        dist_front_left = front_left_ticks*6.28 / self.TICKS_PER_METER
        dist_front_right = front_right_ticks*6.28 / self.TICKS_PER_METER
        dist_back_left = back_left_ticks*6.28 / self.TICKS_PER_METER
        dist_back_right = back_right_ticks*6.28 / self.TICKS_PER_METER


        # rospy.logwarn("front_left_ticks %f"%front_left_ticks)
        # rospy.logwarn("dist_front_left %f"%dist_front_left)
        # rospy.logwarn("dist_front_right %f"%dist_front_right)
        # rospy.logwarn("dist_back_left %f"%dist_back_left)
        # rospy.logwarn("dist_back_right %f"%dist_back_right)

    
        # rospy.logwarn("self.TICKS_PER_METER %d"%self.TICKS_PER_METER)


        # rospy.logwarn("enc_front_left %d"%enc_front_left)
        # rospy.logwarn("enc_front_right %d"%enc_front_right)
        # rospy.logwarn("enc_back_left %d"%enc_back_left)
        # rospy.logwarn("enc_back_right %d"%enc_back_right)

        # dist = (dist_right + dist_left) / 2.0

        current_time = rospy.Time.now()
        d_time = (current_time - self.last_enc_time).to_sec()
        self.last_enc_time = current_time

        W = np.array([dist_front_left/d_time,dist_front_right/d_time,dist_back_left/d_time,dist_back_right/d_time])
        # rospy.logwarn(W)

        return W, d_time

    def update(self, enc_front_left, enc_front_right, enc_back_left, enc_back_right):
        lx = self.BASE_LENGTH
        ly = self.BASE_WIDTH
        r = self.WHEEL_RADIUS
        # rospy.logwarn("before - before cur_th: %f" %self.cur_theta) 
        ######to be modified t oreturn vel_x, vel_y, vel_theta
        # rospy.logwarn("enc_front_left %f"%enc_front_left)
        # rospy.logwarn("self.last_enc_front_left %f"%self.last_enc_front_left)

        [W, d_time] = self.wheel_vel_update(enc_front_left, enc_front_right, enc_back_left, enc_back_right)
        
        B = np.array([[1.00, 1.00, 1.00, 1.00],[-1.00, 1.00, 1.00, -1.00 ],[1.00/(lx+ly), -1.00/(lx+ly), 1.00/(lx+ly), -1.00/(lx+ly)]])

        V = np.dot(B*(r/4), W)  ####no r i think

        self.cur_x += V[0]*d_time
        # rospy.logwarn("self.cur_x %f"%self.cur_x)
        self.cur_y += V[1]*d_time
        self.cur_theta += V[2]*d_time

        vel_x = V[0]
        vel_y = V[1]
        vel_theta = V[2]
        # rospy.logwarn("curent wheel velocities %f %f %f %f " %(W[0],W[1],W[2],W[3]))

        # rospy.logwarn("robot vel_x %f"%vel_x)

        return vel_x, vel_y, vel_theta, W, d_time

    def publish_odom(self, cur_x, cur_y, cur_theta, vx,vy, vth):
        # quat = tf.transformations.quaternion_from_euler(0, 0, cur_theta)
        roll=0
        pitch =0
        yaw = cur_theta
        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)
        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)
        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)

        quat = Quaternion()
        quat.w = cy * cp * cr + sy * sp * sr
        quat.x = cy * cp * sr - sy * sp * cr
        quat.y = sy * cp * sr + cy * sp * cr
        quat.z = sy * cp * cr - cy * sp * sr
        
        current_time = rospy.Time.now()
        # br = tf.TransformBroadcaster()
        # br.sendTransform((cur_x, cur_y, 0),
        #                  tf.transformations.quaternion_from_euler(0, 0, -cur_theta),
        #                  current_time,
        #                  "summit_base_footprint",
        #                  "odom")

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'

        odom.pose.pose.position.x = cur_x
        odom.pose.pose.position.y = cur_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quat

        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[14] = 99999
        odom.pose.covariance[21] = 99999
        odom.pose.covariance[28] = 99999
        odom.pose.covariance[35] = 0.01

        # odom.child_frame_id = 'summit_base_footprint'
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth
        odom.twist.covariance = odom.pose.covariance
        self.odom_pub.publish(odom)

    def update_publish(self, enc_front_left, enc_front_right, enc_back_left, enc_back_right):
        # 2106 per 0.1 seconds is max speed, error in the 16th bit is 32768
        # TODO lets find a better way to deal with this error
        if abs(enc_front_left - self.last_enc_front_left) > 700000:
            rospy.logerr("Ignoring front left encoder jump: cur %d, last %d" % (enc_front_left, self.last_enc_front_left))
        elif abs(enc_back_right - self.last_enc_back_right) > 700000:
            rospy.logerr("Ignoring back right encoder jump: cur %d, last %d" % (enc_back_right, self.last_enc_back_right))
        elif abs(enc_front_right - self.last_enc_front_right) > 700000:
            rospy.logerr("Ignoring front right encoder jump: cur %d, last %d" % (enc_front_right, self.last_enc_front_right))
        elif abs(enc_back_left - self.last_enc_back_left) > 700000:
            rospy.logerr("Ignoring back left encoder jump: cur %d, last %d" % (enc_back_left, self.last_enc_back_left))
        else:
            vel_x, vel_y, vel_theta, W, d_time = self.update(enc_front_left, enc_front_right, enc_back_left, enc_back_right)###changed
            self.publish_odom(self.cur_x, self.cur_y, self.cur_theta, vel_x,vel_y, vel_theta)
        return W, d_time


class Node:
    def __init__(self):
        global p1,p2
        self.ERRORS = {0x0000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "Normal"),
                       0x0001: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 over current"),
                       0x0002: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 over current"),
                       0x0004: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Emergency Stop"),
                       0x0008: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature1"),
                       0x0010: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature2"),
                       0x0020: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Main batt voltage high"),
                       0x0040: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage high"),
                       0x0080: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage low"),
                       0x0100: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 driver fault"),
                       0x0200: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 driver fault"),
                       0x0400: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage high"),
                       0x0800: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage low"),
                       0x1000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature1"),
                       0x2000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature2"),
                       0x4000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M1 home"),
                       0x8000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M2 home")}

        rospy.init_node("roboclaw_node_f")
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Connecting to roboclaw")
        dev_name_front = rospy.get_param("~dev_front", "/dev/ttyACM0")
        dev_name_back = rospy.get_param("~dev_back", "/dev/ttyACM1")
        baud_rate = int(rospy.get_param("~baud", "38400"))
        
        rospy.logwarn("after dev name")
        self.address_front = int(rospy.get_param("~address_front", "128"))
        self.address_back = int(rospy.get_param("~address_back", "129"))


        #check wheather the addresses are in range
        if self.address_front > 0x87 or self.address_front < 0x80 or self.address_back > 0x87 or self.address_back < 0x80:
            rospy.logfatal("Address out of range")
            rospy.signal_shutdown("Address out of range")
        
        # TODO need someway to check if address is correct
        try:
            p1 = roboclaw.Open(dev_name_front, baud_rate)
        except Exception as e:
            rospy.logfatal("Could not connect to front Roboclaw")
            rospy.logdebug(e)
            rospy.signal_shutdown("Could not connect to front Roboclaw")

        try:
            p2 = roboclaw.Open(dev_name_back, baud_rate)
        except Exception as e:
            rospy.logfatal("Could not connect to back Roboclaw")
            rospy.logdebug(e)
            rospy.signal_shutdown("Could not connect to back Roboclaw")

        #run diagnostics 
        # self.updater = diagnostic_updater.Updater() ###check later may be do it for both the motors
        # self.updater.setHardwareID("Roboclaw")
        # self.updater.add(diagnostic_updater.
        #                  FunctionDiagnosticTask("Vitals", self.check_vitals))

        #check if you can get the version of the roboclaw...mosly you dont
        try:
            version = roboclaw.ReadVersion(self.address_front,p1)
        except Exception as e:
            rospy.logwarn("Problem getting front roboclaw version")
            rospy.logdebug(e)
            pass

        if not version[0]:
            rospy.logwarn("Could not get version from front roboclaw")
        else:
            rospy.logdebug(repr(version[1]))

        try:
            version = roboclaw.ReadVersion(self.address_back,p2)
        except Exception as e:
            rospy.logwarn("Problem getting back roboclaw version")
            rospy.logdebug(e)
            pass

        if not version[0]:
            rospy.logwarn("Could not get version from back roboclaw")
        else:
            rospy.logdebug(repr(version[1]))



        # roboclaw.SpeedM1M2(self.address_front, 0, 0,p1)
        roboclaw.ResetEncoders(self.address_front,p1)

        # roboclaw.SpeedM1M2(self.address_back, 0, 0,p2)
        roboclaw.ResetEncoders(self.address_back,p2)

        self.MAX_SPEED = float(rospy.get_param("~max_speed", "1.1"))
        self.TICKS_PER_METER = float(rospy.get_param("~ticks_per_meter", "35818"))
        self.BASE_WIDTH = float(rospy.get_param("~base_width", "0.1762"))
        self.BASE_LENGTH = float(rospy.get_param("~base_length", "0.2485"))
        self.WHEEL_RADIUS = float(rospy.get_param("~wheel_radius", "0.0635"))

        self.encodm = EncoderOdom(self.TICKS_PER_METER, self.BASE_WIDTH,self.BASE_LENGTH,self.WHEEL_RADIUS)
        self.last_set_speed_time = rospy.get_rostime()

        rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)

        # rospy.sleep(1)

        rospy.logdebug("dev_front %s dev_back %s", dev_name_front, dev_name_back)
        rospy.logdebug("baud %d", baud_rate)
        rospy.logdebug("address_front %d address_back %d", self.address_front, self.address_back)
        rospy.logdebug("max_speed %f", self.MAX_SPEED)
        rospy.logdebug("ticks_per_meter %f", self.TICKS_PER_METER)
        rospy.logdebug("base_width %f base_length %f", self.BASE_WIDTH, self.BASE_LENGTH)

    def run(self):
        global p1,p2,get_cmd_vel,U, wheel_pwm_err_prev, wheel_pwm_int_err
        rospy.loginfo("Starting motor drive")
        r_time = rospy.Rate(100)
        max_possible_speed=28.19 #- angular vel - rad/s
        while not rospy.is_shutdown():

            if (rospy.get_rostime() - self.last_set_speed_time).to_sec() > 1:
                rospy.loginfo("Did not get command for 1 second, stopping")
                try:
                    roboclaw.ForwardM1(self.address_front, 0,p1)
                    roboclaw.ForwardM2(self.address_front, 0,p1)
                    roboclaw.ForwardM1(self.address_back, 0,p2)
                    roboclaw.ForwardM2(self.address_back, 0,p2)
                except OSError as e:
                    rospy.logerr("Could not stop")
                    rospy.logdebug(e)

            # TODO need find solution to the OSError11 looks like sync problem with serial
            

            try:
                status1, enc1, crc1 = roboclaw.ReadEncM1(self.address_front,p1)
            except ValueError:
                pass
            except OSError as e:
                rospy.logwarn("ReadEncM1 OSError: %d", e.errno)
                rospy.logdebug(e)

            

            try:
                status2, enc2, crc2 = roboclaw.ReadEncM2(self.address_front,p1)
            except ValueError:
                pass
            except OSError as e:
                rospy.logwarn("ReadEncM2 OSError: %d", e.errno)
                rospy.logdebug(e)

            try:
                status3, enc3, crc3 = roboclaw.ReadEncM1(self.address_back,p2)
            except ValueError:
                pass
            except OSError as e:
                rospy.logwarn("ReadEncM3 OSError: %d", e.errno)
                rospy.logdebug(e)

            

            try:
                status4, enc4, crc4 = roboclaw.ReadEncM2(self.address_back,p2)
            except ValueError:
                pass
            except OSError as e:
                rospy.logwarn("ReadEncM4 OSError: %d", e.errno)
                rospy.logdebug(e)

            if ('enc1' in vars()) and ('enc2' in vars()) and ('enc3' in vars()) and ('enc4' in vars()):
                rospy.logdebug(" Encoders %d %d %d %d" % (enc1, enc2,enc3,enc4))
                W, d_time = self.encodm.update_publish(enc1, enc2, enc3, enc4)

                # self.updater.update()
            rospy.logwarn("wheel velocities %f %f %f %f " %(U[0],U[1],U[2],U[3]))
            rospy.logwarn("W is %f %f %f %f " %(W[0],W[1],W[2],W[3]))
            rospy.logwarn("d_time is %f" %d_time)
            # if(get_cmd_vel==1):
            try:
                if U[0] is 0 and U[1] is 0 and U[2] is 0 and U[3] is 0:
                    roboclaw.ForwardBackwardM1(self.address_front, 63,p1)
                    roboclaw.ForwardBackwardM2(self.address_front, 63,p1)
                    roboclaw.ForwardBackwardM1(self.address_back, 63,p2)
                    roboclaw.ForwardBackwardM2(self.address_back, 63,p2)
                else:
                    wheel_pwm_err = U-W
                    wheel_pwm_deriv_err = (wheel_pwm_err-wheel_pwm_err_prev)/d_time
                    wheel_pwm_int_err = wheel_pwm_int_err + wheel_pwm_err * d_time
                    
                    rospy.logwarn(wheel_pwm_err)
                    Kp_err = 0.5
                    Kd_err = 0.001
                    Ki_err = 0.01
                    wheel_pwm_acc = Kp_err*wheel_pwm_err+Kd_err*wheel_pwm_deriv_err + Ki_err*wheel_pwm_int_err

                    wheel_pwm_err_prev = wheel_pwm_err
                    rospy.logwarn(wheel_pwm_acc)
                    real_sp_m1 = ((wheel_pwm_acc[0]/max_possible_speed)*63)+64
                    real_sp_m2 = ((wheel_pwm_acc[1]/max_possible_speed)*63)+64
                    real_sp_m3 = ((wheel_pwm_acc[2]/max_possible_speed)*63)+64
                    real_sp_m4 = ((wheel_pwm_acc[3]/max_possible_speed)*63)+64
                    # real_sp_m1 = ((U[0]/max_possible_speed)*63)+64
                    # real_sp_m2 = ((U[1]/max_possible_speed)*63)+64
                    # real_sp_m3 = ((U[2]/max_possible_speed)*63)+64
                    # real_sp_m4 = ((U[3]/max_possible_speed)*63)+64
                    roboclaw.ForwardBackwardM1(self.address_front, int(real_sp_m1),p1)
                    roboclaw.ForwardBackwardM2(self.address_front, int(real_sp_m2),p1)
                    roboclaw.ForwardBackwardM1(self.address_back, int(real_sp_m3),p2)
                    roboclaw.ForwardBackwardM2(self.address_back, int(real_sp_m4),p2)
            except OSError as e:
                rospy.logwarn("SpeedM1M2 OSError: %d", e.errno)
                rospy.logdebug(e)
                # get_cmd_vel=0

            r_time.sleep()


    def cmd_vel_callback(self, twist):
        global p1,p2, U, get_cmd_vel
        self.last_set_speed_time = rospy.get_rostime()

        max_linear_speed = 1.79
        max_angular_speed = 0.7
        max_possible_speed=28.19 #angular velocity- rad/s
        
        lx = self.BASE_LENGTH
        ly = self.BASE_WIDTH
        r = self.WHEEL_RADIUS

        linear_x = twist.linear.x
        linear_y = twist.linear.y
        angular_z = twist.angular.z

        if linear_x > max_linear_speed:
            linear_x = max_linear_speed
        if linear_x < -max_linear_speed:
            linear_x = -max_linear_speed

        if linear_y > max_linear_speed:
            linear_y = max_linear_speed
        if linear_y < -max_linear_speed:
            linear_y = -max_linear_speed

        if angular_z > max_angular_speed:
            angular_z = max_angular_speed
        if angular_z < -max_angular_speed:
            angular_z = -max_angular_speed

        
        
        
        #######inverse kinematic Equations
        
        B_inv = np.array([[1.00, -1.00, -(lx+ly)],[1.00, 1.00, (lx+ly)] ,[1.00, 1.00, -(lx+ly)], [1.00, -1.00, (lx+ly)]])
        
        X[0]=linear_x
        X[1]=linear_y
        X[2]=angular_z
        
        U_inv =X
        
        #wheel velocities 
        U = np.dot(B_inv,U_inv)*(1/r)

        
        
        get_cmd_vel=1
        # try:
        #     if U[0] is 0 and U[1] is 0 and U[2] is 0 and U[3] is 0:
        #         roboclaw.ForwardBackwardM1(self.address_front, 63,p1)
        #         roboclaw.ForwardBackwardM2(self.address_front, 63,p1)
        #         roboclaw.ForwardBackwardM1(self.address_back, 63,p2)
        #         roboclaw.ForwardBackwardM2(self.address_back, 63,p2)
        #     else:
        #         real_sp_m1 = ((U[0]/max_possible_speed)*63)+64
        #         real_sp_m2 = ((U[1]/max_possible_speed)*63)+64
        #         real_sp_m3 = ((U[2]/max_possible_speed)*63)+64
        #         real_sp_m4 = ((U[3]/max_possible_speed)*63)+64
        #         roboclaw.ForwardBackwardM1(self.address_front, int(real_sp_m1),p1)
        #         roboclaw.ForwardBackwardM2(self.address_front, int(real_sp_m2),p1)
        #         roboclaw.ForwardBackwardM1(self.address_back, int(real_sp_m3),p2)
        #         roboclaw.ForwardBackwardM2(self.address_back, int(real_sp_m4),p2)
        # except OSError as e:
        #     rospy.logwarn("SpeedM1M2 OSError: %d", e.errno)
        #     rospy.logdebug(e)

    # TODO: Need to make this work when more than one error is raised
    def check_vitals(self, stat):
        global p1,p2
        try:
            status_front = roboclaw.ReadError(self.address_front,p1)[1]
            status_back = roboclaw.ReadError(self.address_back,p2)[1]
        except OSError as e:
            rospy.logwarn("Diagnostics OSError: %d", e.errno)
            rospy.logdebug(e)
            return
        state, message = self.ERRORS[status_front]
        stat.summary(state, message)
        state, message = self.ERRORS[status_back]
        stat.summary(state, message)
        try:
            stat.add("front Main Batt V:", float(roboclaw.ReadMainBatteryVoltage(self.address_front,p1)[1] / 10))
            stat.add("front Logic Batt V:", float(roboclaw.ReadLogicBatteryVoltage(self.address_front,p1)[1] / 10))
            stat.add("front Temp1 C:", float(roboclaw.ReadTemp(self.address_front,p1)[1] / 10))
            stat.add("front Temp2 C:", float(roboclaw.ReadTemp2(self.address_front,p1)[1] / 10))

            stat.add("back Main Batt V:", float(roboclaw.ReadMainBatteryVoltage(self.address_back,p2)[1] / 10))
            stat.add("back Logic Batt V:", float(roboclaw.ReadLogicBatteryVoltage(self.address_back,p2)[1] / 10))
            stat.add("back Temp1 C:", float(roboclaw.ReadTemp(self.address_back,p2)[1] / 10))
            stat.add("back Temp2 C:", float(roboclaw.ReadTemp2(self.address_back,p2)[1] / 10))
        except OSError as e:
            rospy.logwarn("Diagnostics OSError: %d", e.errno)
            rospy.logdebug(e)
        return stat

    # TODO: need clean shutdown so motors stop even if new msgs are arriving
    def shutdown(self):
        global p1,p2
        rospy.loginfo("Shutting down")
        try:
            roboclaw.ForwardBackwardM1(self.address_front, 63,p1)
            roboclaw.ForwardBackwardM2(self.address_front, 63,p1)
            roboclaw.ForwardBackwardM1(self.address_back, 63,p2)
            roboclaw.ForwardBackwardM2(self.address_back, 63,p2)
        except OSError:
            rospy.logerr("Shutdown did not work trying again")
            try:
                roboclaw.ForwardBackwardM1(self.address_front, 63,p1)
                roboclaw.ForwardBackwardM2(self.address_front, 63,p1)
                roboclaw.ForwardBackwardM1(self.address_back, 63,p2)
                roboclaw.ForwardBackwardM2(self.address_back, 63,p2)
            except OSError as e:
                rospy.logerr("Could not shutdown motors!!!!")
                rospy.logdebug(e)


if __name__ == "__main__":
    try:
        node = Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting")
